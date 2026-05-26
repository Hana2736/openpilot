#!/usr/bin/env python3
# Mazda longitudinal delay rolling-store fitter + applier (throttle/brake split).
#
# Reads delay_samples.f32 (schema v3, 5 cols: v_ego, lag_s, ncc, rpm,
# mean_accel_desired) that long_collect.run_collect() populates.  No new
# collector here.
#
# Why two tables: mazda/interface.py:137 already notes "gas is 0.25s and
# brake looks like 0.5".  When we xcorr against a mix of both, every
# highway bin comes back bimodal (clusters at ~0.02-0.18s and ~0.36-0.58s)
# and IQR blows past any sane gate.  Splitting by sign(accel_desired)
# resolves the bimodality and lets each actuator be characterized in
# isolation.  At consume time, whichever way the MPC is currently pushing
# picks the appropriate table.
#
#   1. fit_store / run_fit: split samples by sign of mean_accel_desired,
#      aggregate per-speed-bin median lag for each side, gate independently,
#      build two breakpoint tables, stash in LongDelayPending.
#   2. apply_pending: commit pending -> LongDelayTableThrottle +
#      LongDelayTableBrake.  longcontrol and longitudinal_planner pick
#      based on sign(planned accel) at runtime.
#   3. reset_table: clears both tables -> consumers fall back to scalar.
#
# Safety nets (consumers run every MPC tick):
#   - Per-bin gates: n>=MIN_BIN_N, IQR_width<=MAX_IQR_S, ncc_med>=MIN_NCC.
#   - Per-side: need >=2 monotonic breakpoints or that table is dropped
#     (consumer falls back to scalar on that side).
#   - At least ONE side must produce a valid table or the whole fit refuses.
#   - Table-level clip [MIN_DELAY_S, MAX_DELAY_S] at fit AND apply.
#   - Don't shape-gate (per feedback_fit_shape_gates).

import json

import numpy as np

from openpilot.common.params import Params
from openpilot.frogpilot.common.long_autotune import DEFAULT_BINS_MS
from openpilot.frogpilot.common.long_collect import (
  DELAY_SAMPLE_BYTES, DELAY_SAMPLES_PATH, load_delay_store,
)

STATUS_PARAM = "LongDelayStatus"
PENDING_PARAM = "LongDelayPending"
APPLIED_PARAM_THROTTLE = "LongDelayTableThrottle"
APPLIED_PARAM_BRAKE = "LongDelayTableBrake"
# Legacy single-table param (pre-split).  Cleared on apply/reset so the
# consumer never reads a stale mixed-delay table after upgrade.
LEGACY_APPLIED_PARAM = "LongDelayTable"

# Per-bin data-quality gates.  After splitting throttle/brake, each side
# has roughly half the samples - so MIN_BIN_N is lower than the mixed-store
# threshold.  IQR can stay tight because each side should be unimodal.
MIN_BIN_N = 4
MAX_IQR_S = 0.30
MIN_BIN_NCC = 0.75

# Drop ambiguous-direction windows (near-zero mean command) - they
# contaminate either bucket.  Tuned to filter coasting/maintain-speed
# windows where the MPC barely touches either actuator.
MIN_CMD_MAG = 0.10  # m/s^2

MIN_DELAY_S = 0.05
MAX_DELAY_S = 0.80

# Plant-measured per-side defaults for Mazda, used by frogpilot_variables
# when no fitted table is present. Source: mazda/interface.py:137 comment
# "gas is 0.25s and brake looks like 0.5". Flat 2-point tables (constant
# vs v_ego) until enough Experimental-mode samples accrue to refine into
# per-speed curves. Fitted tables override these on apply.
DEFAULT_TABLE_THROTTLE = ((0.0, 0.25), (40.0, 0.25))
DEFAULT_TABLE_BRAKE = ((0.0, 0.50), (40.0, 0.50))

BIN_CENTERS_MS = tuple(0.5 * (lo + hi) for lo, hi in zip(DEFAULT_BINS_MS[:-1], DEFAULT_BINS_MS[1:]))


params = Params()


def _set_status(msg: str) -> None:
  Params("/dev/shm/params").put(STATUS_PARAM, msg)


# --- per-side fit ---------------------------------------------------------

def _fit_side(v: np.ndarray, lag: np.ndarray, ncc: np.ndarray) -> dict:
  """Returns {'breakpoints': [(v, delay)], 'n': total, 'per_bin': [...]}.
  breakpoints can be empty if no bin passed - caller checks len() >= 2.
  """
  per_bin = []
  for i in range(len(DEFAULT_BINS_MS) - 1):
    lo, hi = DEFAULT_BINS_MS[i], DEFAULT_BINS_MS[i + 1]
    m = (v >= lo) & (v < hi)
    if not m.any():
      per_bin.append(None)
      continue
    lags = lag[m]
    nccs = ncc[m]
    iqr_lo = float(np.percentile(lags, 25))
    iqr_hi = float(np.percentile(lags, 75))
    entry = {
      "lo": float(lo), "hi": float(hi),
      "n": int(m.sum()),
      "median": float(np.median(lags)),
      "iqr_lo": iqr_lo, "iqr_hi": iqr_hi,
      "iqr_w": iqr_hi - iqr_lo,
      "ncc_med": float(np.median(nccs)),
    }
    entry["ok"] = (entry["n"] >= MIN_BIN_N
                   and entry["iqr_w"] <= MAX_IQR_S
                   and entry["ncc_med"] >= MIN_BIN_NCC)
    per_bin.append(entry)

  bps = []
  for entry, v_center in zip(per_bin, BIN_CENTERS_MS):
    if entry is None or not entry["ok"]:
      continue
    bps.append((float(v_center), float(np.clip(entry["median"],
                                               MIN_DELAY_S, MAX_DELAY_S))))
  return {"breakpoints": bps, "n": int(v.size), "per_bin": per_bin}


def fit_store() -> dict | None:
  """Returns {'throttle': {...}, 'brake': {...}, 'n': total} or None if
  data store empty / both sides produce <2 breakpoints.
  """
  data = load_delay_store()
  if data.shape[0] == 0:
    return None
  v, lag, ncc, _rpm, mean_a = data[:, 0], data[:, 1], data[:, 2], data[:, 3], data[:, 4]

  # Split by command sign, dropping near-zero windows (coast/maintain-speed
  # have ambiguous direction and would contaminate either bucket).
  thr_mask = mean_a >= MIN_CMD_MAG
  brk_mask = mean_a <= -MIN_CMD_MAG

  throttle = _fit_side(v[thr_mask], lag[thr_mask], ncc[thr_mask])
  brake = _fit_side(v[brk_mask], lag[brk_mask], ncc[brk_mask])

  if len(throttle["breakpoints"]) < 2 and len(brake["breakpoints"]) < 2:
    return None

  return {"throttle": throttle, "brake": brake, "n": int(data.shape[0])}


# --- status helpers --------------------------------------------------------

def _short(bps):
  return "→".join(f"{d:.2f}" for _, d in bps) if bps else "none"


def _per_bin_str(per_bin):
  return "  ".join(
    f"{e['lo']:g}-{e['hi']:g}: med={e['median']:.3f}s n={e['n']} iqr={e['iqr_w']:.2f} "
    f"ncc={e['ncc_med']:.2f} {'OK' if e['ok'] else 'FAIL'}"
    for e in per_bin if e is not None
  )


def _idle_status() -> str:
  try:
    n_samples = DELAY_SAMPLES_PATH.stat().st_size // DELAY_SAMPLE_BYTES
  except FileNotFoundError:
    n_samples = 0
  applied = []
  for label, p in [("throttle", APPLIED_PARAM_THROTTLE), ("brake", APPLIED_PARAM_BRAKE)]:
    raw = params.get(p)
    if not raw:
      continue
    try:
      bps = json.loads(raw).get("breakpoints", [])
      applied.append(f"{label}: " + ",".join(f"{v:.1f}→{d:.3f}s" for v, d in bps))
    except Exception:
      pass
  table_str = ("  applied: " + " | ".join(applied)) if applied else ""
  return (f"Idle|{n_samples} delay samples|"
          f"Delay store: {n_samples} xcorr windows.  Tap Fit to build per-speed "
          f"throttle/brake delay tables from the rolling store.{table_str}")


def restore_idle_status() -> None:
  if Params("/dev/shm/params").get(STATUS_PARAM):
    return
  _set_status(_idle_status())


def _preview_status(fit: dict) -> str:
  thr_bps = fit["throttle"]["breakpoints"]
  brk_bps = fit["brake"]["breakpoints"]
  return (f"Preview|Preview ready — tap Apply (thr {_short(thr_bps)} / brk {_short(brk_bps)})|"
          f"THROTTLE bps: " + (", ".join(f"{v:.1f}m/s→{d:.3f}s" for v, d in thr_bps)
                               if thr_bps else "(insufficient)") + "  ·  "
          f"BRAKE bps: " + (", ".join(f"{v:.1f}m/s→{d:.3f}s" for v, d in brk_bps)
                            if brk_bps else "(insufficient)") + "  ·  "
          f"{fit['n']} total samples  ·  "
          f"throttle per-bin: " + _per_bin_str(fit["throttle"]["per_bin"]) + "  ·  "
          f"brake per-bin: " + _per_bin_str(fit["brake"]["per_bin"]))


def restore_preview_status() -> None:
  if Params("/dev/shm/params").get(STATUS_PARAM):
    return
  raw = params.get(PENDING_PARAM)
  if not raw:
    return
  try:
    fit = json.loads(raw)
    _set_status(_preview_status(fit))
  except Exception:
    pass


# --- main entries ----------------------------------------------------------

def run_fit() -> None:
  fit = fit_store()
  if fit is None:
    _set_status("Refused|Need more data|"
                "Long-delay fit refused: neither throttle nor brake side has "
                "≥2 bins passing the per-bin gates (n≥4, IQR≤0.30s, NCC≥0.75). "
                "Drive more on Experimental mode and retry.")
    return
  params.put(PENDING_PARAM, json.dumps(fit))
  _set_status(_preview_status(fit))


def _validate_bps(bps):
  cleaned = [(float(v), float(np.clip(d, MIN_DELAY_S, MAX_DELAY_S))) for v, d in bps]
  if len(cleaned) < 2:
    return None
  vs = [v for v, _ in cleaned]
  if not all(b > a for a, b in zip(vs[:-1], vs[1:])):
    return None
  return cleaned


def apply_pending() -> None:
  raw = params.get(PENDING_PARAM)
  if not raw:
    _set_status("No previewed long-delay table to apply. Run Fit first.")
    return
  try:
    fit = json.loads(raw)
    thr_bps = _validate_bps(fit.get("throttle", {}).get("breakpoints", []))
    brk_bps = _validate_bps(fit.get("brake", {}).get("breakpoints", []))
  except Exception:
    params.remove(PENDING_PARAM)
    _set_status("Previewed long-delay table was invalid. Re-run Fit.")
    return

  if thr_bps is None and brk_bps is None:
    params.remove(PENDING_PARAM)
    _set_status("Neither side has ≥2 monotonic breakpoints to apply.")
    return

  # Pre-split cleanup: a leftover LongDelayTable would still be loaded by
  # frogpilot_variables and confuse the new per-side consumer.
  params.remove(LEGACY_APPLIED_PARAM)

  applied = []
  if thr_bps is not None:
    params.put(APPLIED_PARAM_THROTTLE, json.dumps({"breakpoints": thr_bps}))
    applied.append(f"throttle ({_short(thr_bps)})")
  else:
    params.remove(APPLIED_PARAM_THROTTLE)
  if brk_bps is not None:
    params.put(APPLIED_PARAM_BRAKE, json.dumps({"breakpoints": brk_bps}))
    applied.append(f"brake ({_short(brk_bps)})")
  else:
    params.remove(APPLIED_PARAM_BRAKE)
  params.remove(PENDING_PARAM)

  detail_parts = []
  if thr_bps is not None:
    detail_parts.append("THROTTLE: " + ", ".join(f"{v:.1f}m/s→{d:.3f}s" for v, d in thr_bps))
  if brk_bps is not None:
    detail_parts.append("BRAKE: " + ", ".join(f"{v:.1f}m/s→{d:.3f}s" for v, d in brk_bps))
  _set_status(f"Done|Applied {' + '.join(applied)} — reboot to use|"
              f"Long-delay tables applied. Consumers will interp by current "
              f"v_ego using sign(planned accel) to pick:  "
              + "  ·  ".join(detail_parts)
              + ".  Reboot for clean startup.")


def reset_table() -> None:
  params.remove(APPLIED_PARAM_THROTTLE)
  params.remove(APPLIED_PARAM_BRAKE)
  params.remove(LEGACY_APPLIED_PARAM)
  params.remove(PENDING_PARAM)
  _set_status(_idle_status())
