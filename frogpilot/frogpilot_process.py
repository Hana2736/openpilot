#!/usr/bin/env python3
import datetime
import json
import time

from cereal import messaging
from openpilot.common.realtime import DT_MDL, Priority, Ratekeeper, config_realtime_process
from openpilot.common.time import system_time_valid

from openpilot.frogpilot.assets.model_manager import MODEL_DOWNLOAD_ALL_PARAM, MODEL_DOWNLOAD_PARAM, ModelManager
from openpilot.frogpilot.assets.theme_manager import THEME_COMPONENT_PARAMS, ThemeManager
from openpilot.frogpilot.common.frogpilot_functions import backup_toggles
from openpilot.frogpilot.common.frogpilot_utilities import capture_report, flash_panda, is_url_pingable, lock_doors, run_thread_with_lock, update_maps, update_openpilot
from openpilot.frogpilot.common.frogpilot_variables import ERROR_LOGS_PATH, FrogPilotVariables, get_frogpilot_toggles, params, params_cache, params_memory
from openpilot.frogpilot.common.lat_delay_collect import (
  apply_pending as apply_lat_delay,
  reset_table as reset_lat_delay,
  restore_idle_status as restore_lat_delay_idle_status,
  restore_preview_status as restore_lat_delay_preview_status,
  run_collect as run_lat_delay_collect,
  run_fit as run_lat_delay_fit,
)
from openpilot.frogpilot.common.long_collect import restore_idle_status as restore_long_idle_status, run_collect as run_long_collect
from openpilot.frogpilot.common.long_delay_collect import (
  apply_pending as apply_long_delay,
  reset_table as reset_long_delay,
  restore_idle_status as restore_long_delay_idle_status,
  restore_preview_status as restore_long_delay_preview_status,
  run_fit as run_long_delay_fit,
)
from openpilot.frogpilot.common.long_static_collect import (
  apply_pending as apply_long_static,
  reset_table as reset_long_static,
  restore_idle_status as restore_long_static_idle_status,
  restore_preview_status as restore_long_static_preview_status,
  run_fit as run_long_static_fit,
)
from openpilot.frogpilot.common.lat_openloop_collect import (
  apply_pending as apply_lat_openloop,
  reset_table as reset_lat_openloop,
  restore_idle_status as restore_lat_openloop_idle_status,
  restore_preview_status as restore_lat_openloop_preview_status,
  run_collect as run_lat_openloop_collect,
  run_fit as run_lat_openloop_fit,
)
from openpilot.frogpilot.common.torque_autotune import (
  apply_pending,
  restore_preview_status,
  run_autotune,
  run_collect as run_closed_loop_collect,
)
from openpilot.frogpilot.controls.frogpilot_planner import FrogPilotPlanner
from openpilot.frogpilot.system.frogpilot_stats import send_stats
from openpilot.frogpilot.system.frogpilot_tracking import FrogPilotTracking

ASSET_CHECK_RATE = (1 / DT_MDL)

def assets_checks(model_manager, theme_manager, frogpilot_toggles):
  if params_memory.get_bool(MODEL_DOWNLOAD_ALL_PARAM):
    run_thread_with_lock("download_all_models", model_manager.download_all_models)
  elif params_memory.get_bool("UpdateTinygrad"):
    run_thread_with_lock("update_tinygrad", model_manager.update_tinygrad)
  else:
    model_to_download = params_memory.get(MODEL_DOWNLOAD_PARAM, encoding="utf-8")
    if model_to_download:
      run_thread_with_lock("download_model", model_manager.download_model, (model_to_download,))

  if params_memory.get_bool("FlashPanda"):
    run_thread_with_lock("flash_panda", flash_panda)

  report_data = json.loads(params_memory.get("IssueReported", encoding="utf-8") or "{}")
  if report_data:
    capture_report(report_data["DiscordUser"], report_data["Issue"], vars(frogpilot_toggles))
    params_memory.remove("IssueReported")

  for asset_type, asset_param in THEME_COMPONENT_PARAMS.items():
    asset_to_download = params_memory.get(asset_param, encoding="utf-8")
    if asset_to_download:
      run_thread_with_lock("download_theme", theme_manager.download_theme, (asset_type, asset_to_download, asset_param, frogpilot_toggles))

def _stack_status(msg: str) -> None:
  params_memory.put("MazdaTuneStackStatus", msg)


def run_collect_all() -> None:
  """Sequentially ingest every Mazda rolling store.  Each step's per-feature
  status param is updated by the underlying collector; this function only
  writes the combined MazdaTuneStackStatus so the top-of-panel button shows
  a single progress line for the whole stack."""
  steps = (
    ("Long (static + delay)", run_long_collect),
    ("Lat delay",             run_lat_delay_collect),
    ("Closed-loop lat (BCD)", run_closed_loop_collect),
    ("Open-loop lat (A)",     run_lat_openloop_collect),
  )
  for i, (label, fn) in enumerate(steps, 1):
    _stack_status(f"Collect|Step {i}/{len(steps)}: {label}|"
                  f"Running collector {i} of {len(steps)} — see individual button for live progress.")
    try:
      fn()
    except Exception as e:
      _stack_status(f"Refused|{label} crashed|{label} collector raised: {e!r}. "
                    f"Other steps not run.  See individual buttons for details.")
      return
  _stack_status(f"Done|All {len(steps)} collectors ran|"
                f"Collected: long (static + delay), lat delay, closed-loop lat (b/c/d), "
                f"open-loop lat (a). Each store's own status button shows sample counts. "
                f"Now tap Fit All — open-loop lat cross-fits a (open-loop) + b/c/d (closed-loop).")


def run_fit_all() -> None:
  """Sequentially run every Mazda fitter.  Each writes its own Pending +
  per-feature status; this function only writes the combined stack status.
  Apply remains per-feature so the user can sanity-check each before
  committing."""
  steps = (
    ("Long delay",        run_long_delay_fit),
    ("Long static map",   run_long_static_fit),
    ("Lat delay",         run_lat_delay_fit),
    ("Open-loop lat",     run_lat_openloop_fit),
  )
  for i, (label, fn) in enumerate(steps, 1):
    _stack_status(f"Fit|Step {i}/{len(steps)}: {label}|"
                  f"Running fitter {i} of {len(steps)} — see individual button for the preview.")
    try:
      fn()
    except Exception as e:
      _stack_status(f"Refused|{label} crashed|{label} fitter raised: {e!r}. "
                    f"Other steps not run.  See individual buttons for details.")
      return
  _stack_status(f"Done|All {len(steps)} fitters ran|"
                f"Previewed: long delay, long static map, lat delay, open-loop lat. "
                f"Tap each individual Apply to commit; this stack button does NOT auto-apply.")


def autotune_check(started):
  # Auto-Tune is heavy (reads many rlogs) so it only runs while parked.
  if started:
    return
  restore_preview_status()
  restore_long_idle_status()
  if params_memory.get_bool("MazdaAutoTuneApply"):
    params_memory.remove("MazdaAutoTuneApply")
    apply_pending()
  if params_memory.get_bool("MazdaAutoTune"):
    params_memory.remove("MazdaAutoTune")
    params_memory.put("AutoTuneStatus", "Queued...")
    run_thread_with_lock("mazda_autotune", run_autotune)
  if params_memory.get_bool("LongAutoTuneCollect"):
    params_memory.remove("LongAutoTuneCollect")
    params_memory.put("LongAutoTuneStatus", "Queued...")
    run_thread_with_lock("mazda_long_collect", run_long_collect)
  restore_lat_delay_preview_status()
  restore_lat_delay_idle_status()
  if params_memory.get_bool("LatDelayReset"):
    params_memory.remove("LatDelayReset")
    reset_lat_delay()
  if params_memory.get_bool("LatDelayApply"):
    params_memory.remove("LatDelayApply")
    apply_lat_delay()
  if params_memory.get_bool("LatDelayFit"):
    params_memory.remove("LatDelayFit")
    params_memory.put("LatDelayStatus", "Fitting...")
    run_thread_with_lock("mazda_lat_delay", run_lat_delay_fit)
  if params_memory.get_bool("LatDelayCollect"):
    params_memory.remove("LatDelayCollect")
    params_memory.put("LatDelayStatus", "Queued...")
    run_thread_with_lock("mazda_lat_delay", run_lat_delay_collect)
  restore_long_delay_preview_status()
  restore_long_delay_idle_status()
  if params_memory.get_bool("LongDelayReset"):
    params_memory.remove("LongDelayReset")
    reset_long_delay()
  if params_memory.get_bool("LongDelayApply"):
    params_memory.remove("LongDelayApply")
    apply_long_delay()
  if params_memory.get_bool("LongDelayFit"):
    params_memory.remove("LongDelayFit")
    params_memory.put("LongDelayStatus", "Fitting...")
    run_thread_with_lock("mazda_long_delay", run_long_delay_fit)
  restore_long_static_preview_status()
  restore_long_static_idle_status()
  if params_memory.get_bool("LongStaticReset"):
    params_memory.remove("LongStaticReset")
    reset_long_static()
  if params_memory.get_bool("LongStaticApply"):
    params_memory.remove("LongStaticApply")
    apply_long_static()
  if params_memory.get_bool("LongStaticFit"):
    params_memory.remove("LongStaticFit")
    params_memory.put("LongStaticStatus", "Fitting...")
    run_thread_with_lock("mazda_long_static", run_long_static_fit)
  restore_lat_openloop_preview_status()
  restore_lat_openloop_idle_status()
  if params_memory.get_bool("LatOpenLoopReset"):
    params_memory.remove("LatOpenLoopReset")
    reset_lat_openloop()
  if params_memory.get_bool("LatOpenLoopApply"):
    params_memory.remove("LatOpenLoopApply")
    apply_lat_openloop()
  if params_memory.get_bool("LatOpenLoopFit"):
    params_memory.remove("LatOpenLoopFit")
    params_memory.put("LatOpenLoopStatus", "Fitting...")
    run_thread_with_lock("mazda_lat_openloop", run_lat_openloop_fit)
  if params_memory.get_bool("LatOpenLoopCollect"):
    params_memory.remove("LatOpenLoopCollect")
    params_memory.put("LatOpenLoopStatus", "Queued...")
    run_thread_with_lock("mazda_lat_openloop", run_lat_openloop_collect)
  if params_memory.get_bool("MazdaCollectAll"):
    params_memory.remove("MazdaCollectAll")
    params_memory.put("MazdaTuneStackStatus", "Queued...")
    run_thread_with_lock("mazda_tune_stack", run_collect_all)
  if params_memory.get_bool("MazdaFitAll"):
    params_memory.remove("MazdaFitAll")
    params_memory.put("MazdaTuneStackStatus", "Queued...")
    run_thread_with_lock("mazda_tune_stack", run_fit_all)


def update_checks(model_manager, now, theme_manager, frogpilot_toggles, boot_run=False):
  while not (is_url_pingable("https://github.com") or is_url_pingable("https://gitlab.com")):
    time.sleep(60)

  model_manager.update_models(boot_run)
  theme_manager.update_themes(frogpilot_toggles, boot_run)

  run_thread_with_lock("update_maps", update_maps, (now,))

  if frogpilot_toggles.automatic_updates:
    run_thread_with_lock("update_openpilot", update_openpilot)

  time.sleep(1)

def frogpilot_thread():
  rate_keeper = Ratekeeper(1 / DT_MDL, None)

  config_realtime_process(5, Priority.CTRL_LOW)

  frogpilot_toggles = get_frogpilot_toggles()

  error_log = ERROR_LOGS_PATH / "error.txt"
  if error_log.is_file():
    error_log.unlink()

  frogpilot_variables = FrogPilotVariables()
  model_manager = ModelManager()
  theme_manager = ThemeManager()

  toggles_last_updated = datetime.datetime.now(datetime.timezone.utc)

  pm = messaging.PubMaster(["frogpilotPlan"])
  sm = messaging.SubMaster(["carControl", "carState", "controlsState", "deviceState", "driverMonitoringState",
                            "liveLocationKalman", "liveParameters", "managerState", "modelV2", "onroadEvents",
                            "pandaStates", "radarState", "frogpilotCarState", "frogpilotControlsState",
                            "frogpilotModelV2", "frogpilotNavigation", "frogpilotOnroadEvents"],
                            poll="modelV2", ignore_avg_freq=["frogpilotRadarState"])

  run_update_checks = False
  started_previously = False
  time_validated = False
  toggles_updated = False

  while True:
    sm.update()

    now = datetime.datetime.now(datetime.timezone.utc)

    started = sm["deviceState"].started

    if not started and started_previously:
      run_update_checks = True

      frogpilot_variables.update(theme_manager.holiday_theme, started)
      frogpilot_toggles = get_frogpilot_toggles()

      if frogpilot_toggles.lock_doors_timer:
        run_thread_with_lock("lock_doors", lock_doors, (frogpilot_toggles.lock_doors_timer, sm), report=False)

      if frogpilot_toggles.random_themes:
        theme_manager.update_active_theme(time_validated, frogpilot_toggles, randomize_theme=True)

      if time_validated:
        send_stats()

    elif started and not started_previously:
      if error_log.is_file():
        error_log.unlink()

      frogpilot_planner = FrogPilotPlanner(error_log, theme_manager)
      frogpilot_tracking = FrogPilotTracking(frogpilot_planner, frogpilot_toggles)

    if started and sm.updated["modelV2"]:
      frogpilot_planner.update(now, time_validated, sm, frogpilot_toggles)
      frogpilot_planner.publish(theme_manager.theme_updated, toggles_updated, sm, pm, frogpilot_toggles)

      frogpilot_tracking.update(now, time_validated, sm, frogpilot_toggles)
    elif not started:
      frogpilot_plan_send = messaging.new_message("frogpilotPlan")
      frogpilot_plan_send.frogpilotPlan.themeUpdated = theme_manager.theme_updated or params_memory.get_bool("UseActiveTheme")
      frogpilot_plan_send.frogpilotPlan.togglesUpdated = toggles_updated
      pm.send("frogpilotPlan", frogpilot_plan_send)

    started_previously = started

    if rate_keeper.frame % ASSET_CHECK_RATE == 0:
      assets_checks(model_manager, theme_manager, frogpilot_toggles)
      autotune_check(started)

    if params_memory.get_bool("FrogPilotTogglesUpdated") or theme_manager.theme_updated:
      previous_holiday_themes = frogpilot_toggles.holiday_themes
      previous_random_themes = frogpilot_toggles.random_themes

      frogpilot_variables.update(theme_manager.holiday_theme, started)
      frogpilot_toggles = get_frogpilot_toggles()

      randomize_theme = frogpilot_toggles.holiday_themes != previous_holiday_themes
      randomize_theme |= frogpilot_toggles.random_themes != previous_random_themes

      theme_manager.theme_updated = False
      theme_manager.update_active_theme(time_validated, frogpilot_toggles, randomize_theme=randomize_theme)

      if time_validated:
        run_thread_with_lock("backup_toggles", backup_toggles, (params_cache,), report=False)

      toggles_last_updated = now

    toggles_updated = (now - toggles_last_updated).total_seconds() <= 1

    run_update_checks |= params_memory.get_bool("ManualUpdateInitiated")
    run_update_checks |= now.second == 0 and (now.minute % 60 == 0 or (now.minute % 5 == 0 and frogpilot_toggles.frogs_go_moo))
    run_update_checks &= time_validated

    if run_update_checks:
      theme_manager.update_active_theme(time_validated, frogpilot_toggles)
      run_thread_with_lock("update_checks", update_checks, (model_manager, now, theme_manager, frogpilot_toggles))

      run_update_checks = False
    elif not time_validated:
      time_validated = system_time_valid()
      if not time_validated:
        continue

      theme_manager.update_active_theme(time_validated, frogpilot_toggles)
      run_thread_with_lock("update_checks", update_checks, (model_manager, now, theme_manager, frogpilot_toggles, True))

    rate_keeper.keep_time()

def main():
  frogpilot_thread()

if __name__ == "__main__":
  main()
