#!/usr/bin/env python3
# Mazda Gen2 longitudinal static-map fitter + applier.
#
# Reads samples.f32 (5 cols v_ego, can_cmd, a_ego, pitch, rpm) that
# long_collect.run_collect() populates.  Per-speed-bin piecewise-affine-
# with-deadband fit on pitch-corrected aEgo, builds a table that
# carcontroller uses to invert the plant instead of the global
# accel_scale=200, accel_offset=2000 affine.
#
# Why this matters: the stock affine is a single linear map across all
# speeds.  Real plant response varies with speed (Mazda's onboard ACC
# applies different control at different regimes), and has a deadzone
# around CAN center 2000 where small commands produce zero achieved
# acceleration.  Per-bin (dz_lo, dz_hi, s_pos, s_neg) captures both.
#
#   1. fit_store / run_fit: per-bin pitch-corrected pwl-deadband fit,
#      data-quality-gated (n_pos+n_neg samples per side, R² per side),
#      stashes in LongStaticPending.
#   2. apply_pending: commits to LongStaticMapTable.  carcontroller
#      inverts at runtime per (target_accel, v_ego).
#   3. reset_table: clears LongStaticMapTable -> consumers fall back to
#      the stock global affine.
#
# Safety nets (carcontroller writes the CAN integer that drives the car):
#   - Per-bin gates: n_pos/n_neg >= MIN_PER_SIDE, R² >= MIN_R2.
#   - Per-bin slope clipped to [SLOPE_LO, SLOPE_HI] (matches collector).
#   - Per-bin dz clipped to [ACC_CMD_CENTER - DZ_MAX_COUNTS, +DZ_MAX_COUNTS].
#   - Need >=2 monotonic bins or fit/apply refuses.
#   - carcontroller hard-clips the output to [CMD_VALID_LO, CMD_VALID_HI]
#     regardless of what the table says (defense in depth).
#   - Default off until Apply.
#   - Don't shape-gate (per feedback_fit_shape_gates).

import json

import numpy as np

from openpilot.common.params import Params
from openpilot.frogpilot.common.long_autotune import (
  ACC_CMD_CENTER, CMD_VALID_HI, CMD_VALID_LO, DEFAULT_BINS_MS, DZ_MAX_COUNTS,
  MIN_PER_SIDE, SLOPE_HI, SLOPE_LO, _fit_pwl_deadband,
)
from openpilot.frogpilot.common.long_collect import (
  SAMPLE_BYTES, SAMPLES_PATH, load_store,
)

STATUS_PARAM = "LongStaticStatus"
PENDING_PARAM = "LongStaticPending"
APPLIED_PARAM = "LongStaticMapTable"

# Per-bin data-quality gates.  MIN_R2 is loose - real plant data is noisy
# and we'd rather fit a messy bin than throw it out, since the consumer
# always clamps to CMD_VALID_* anyway.
MIN_R2 = 0.30

GRAVITY = 9.81  # m/s² for pitch correction: a_ego_pwt = a_ego + g*sin(pitch)

BIN_CENTERS_MS = tuple(0.5 * (lo + hi) for lo, hi in zip(DEFAULT_BINS_MS[:-1], DEFAULT_BINS_MS[1:]))


params = Params()


def _set_status(msg: str) -> None:
  Params("/dev/shm/params").put(STATUS_PARAM, msg)


# --- fit -------------------------------------------------------------------

def fit_store() -> dict | None:
  """Returns {'breakpoints': [(v_center, dz_lo, dz_hi, s_pos, s_neg), ...],
  'n': total, 'per_bin': [...]} or None if <2 bins pass gates.
  """
  data = load_store()
  if data.shape[0] == 0:
    return None
  v = data[:, 0]
  cc = data[:, 1]
  ae_raw = data[:, 2]
  ph = data[:, 3]
  # Pitch correction: aEgo as measured includes the gravity component on
  # hills.  a_ego_pwt = a_ego + g*sin(pitch) gives the powertrain-only
  # contribution, which is what the plant map actually represents.
  ae = ae_raw + GRAVITY * np.sin(ph)

  per_bin = []
  for i in range(len(DEFAULT_BINS_MS) - 1):
    lo, hi = DEFAULT_BINS_MS[i], DEFAULT_BINS_MS[i + 1]
    m = (v >= lo) & (v < hi)
    if not m.any():
      per_bin.append(None)
      continue
    fit = _fit_pwl_deadband(cc[m], ae[m])
    entry = {
      "lo": float(lo), "hi": float(hi),
      "n": int(m.sum()),
    }
    if fit is None:
      entry["ok"] = False
      entry["reason"] = "deadband fit gates failed (insufficient per-side samples or slope out of range)"
      per_bin.append(entry)
      continue
    entry.update({k: float(v) if isinstance(v, (int, float)) else v for k, v in fit.items()})
    entry["ok"] = (entry["r2_pos"] >= MIN_R2 and entry["r2_neg"] >= MIN_R2
                   and entry["n_pos"] >= MIN_PER_SIDE
                   and entry["n_neg"] >= MIN_PER_SIDE)
    per_bin.append(entry)

  bps = []
  for entry, v_center in zip(per_bin, BIN_CENTERS_MS):
    if entry is None or not entry["ok"]:
      continue
    dz_lo = float(np.clip(entry["dz_lo"], ACC_CMD_CENTER - DZ_MAX_COUNTS, ACC_CMD_CENTER + DZ_MAX_COUNTS))
    dz_hi = float(np.clip(entry["dz_hi"], ACC_CMD_CENTER - DZ_MAX_COUNTS, ACC_CMD_CENTER + DZ_MAX_COUNTS))
    s_pos = float(np.clip(entry["s_pos"], SLOPE_LO, SLOPE_HI))
    s_neg = float(np.clip(entry["s_neg"], SLOPE_LO, SLOPE_HI))
    bps.append((float(v_center), dz_lo, dz_hi, s_pos, s_neg))
  if len(bps) < 2:
    return None

  return {"breakpoints": bps, "n": int(data.shape[0]), "per_bin": per_bin}


# --- status helpers --------------------------------------------------------

def _idle_status() -> str:
  try:
    n_samples = SAMPLES_PATH.stat().st_size // SAMPLE_BYTES
  except FileNotFoundError:
    n_samples = 0
  raw = params.get(APPLIED_PARAM)
  table_str = ""
  if raw:
    try:
      bps = json.loads(raw).get("breakpoints", [])
      table_str = "  applied: " + " | ".join(
        f"{v:.0f}m/s dz=[{dz_lo:.0f},{dz_hi:.0f}] s+={s_pos*100:.2f} s-={s_neg*100:.2f}"
        for v, dz_lo, dz_hi, s_pos, s_neg in bps
      )
    except Exception:
      pass
  return (f"Idle|{n_samples} static samples|"
          f"Static store: {n_samples} steady-state windows.  Tap Fit to build "
          f"per-speed pwl-deadband table for carcontroller inverse.{table_str}")


def restore_idle_status() -> None:
  if Params("/dev/shm/params").get(STATUS_PARAM):
    return
  _set_status(_idle_status())


def _preview_status(fit: dict) -> str:
  bps = fit["breakpoints"]
  short_summary = ",".join(f"{v:.0f}m/s" for v, *_ in bps)
  per_bin_str = "  ".join(
    f"{e['lo']:g}-{e['hi']:g}: " + (
      f"dz=[{e['dz_lo']:.0f},{e['dz_hi']:.0f}] s+={e.get('s_pos',0)*100:.2f}/100ct (n={e.get('n_pos',0)}, R²={e.get('r2_pos',0):+.2f}) "
      f"s-={e.get('s_neg',0)*100:.2f}/100ct (n={e.get('n_neg',0)}, R²={e.get('r2_neg',0):+.2f}) "
      f"{'OK' if e['ok'] else 'FAIL'}"
      if "dz_lo" in e else f"n={e['n']} FAIL: {e.get('reason','')}"
    )
    for e in fit["per_bin"] if e is not None
  )
  return (f"Preview|Preview ready — tap Apply ({short_summary})|"
          f"Per-bin breakpoints — " + ", ".join(
            f"{v:.0f}m/s: dz=[{dz_lo:.0f},{dz_hi:.0f}] s+={s_pos*100:.2f}/100ct s-={s_neg*100:.2f}/100ct"
            for v, dz_lo, dz_hi, s_pos, s_neg in bps
          ) + f"  ·  {fit['n']} total static samples  ·  per-bin: " + per_bin_str)


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
                "Static-map fit refused: fewer than 2 speed bins passed "
                "per-bin gates (n_pos/n_neg ≥ 8, R² ≥ 0.30 each side). "
                "Keep collecting; the long collector accumulates every Collect tap.")
    return
  params.put(PENDING_PARAM, json.dumps(fit))
  _set_status(_preview_status(fit))


def _validate_bps(bps):
  cleaned = []
  for entry in bps:
    if len(entry) != 5:
      return None
    v, dz_lo, dz_hi, s_pos, s_neg = entry
    cleaned.append((
      float(v),
      float(np.clip(dz_lo, ACC_CMD_CENTER - DZ_MAX_COUNTS, ACC_CMD_CENTER + DZ_MAX_COUNTS)),
      float(np.clip(dz_hi, ACC_CMD_CENTER - DZ_MAX_COUNTS, ACC_CMD_CENTER + DZ_MAX_COUNTS)),
      float(np.clip(s_pos, SLOPE_LO, SLOPE_HI)),
      float(np.clip(s_neg, SLOPE_LO, SLOPE_HI)),
    ))
  if len(cleaned) < 2:
    return None
  vs = [v for v, *_ in cleaned]
  if not all(b > a for a, b in zip(vs[:-1], vs[1:])):
    return None
  return cleaned


def apply_pending() -> None:
  raw = params.get(PENDING_PARAM)
  if not raw:
    _set_status("No previewed long-static table to apply. Run Fit first.")
    return
  try:
    fit = json.loads(raw)
    bps = _validate_bps(fit.get("breakpoints", []))
    if bps is None:
      raise ValueError("invalid breakpoints (need ≥2 strictly-increasing in v_ego)")
  except Exception:
    params.remove(PENDING_PARAM)
    _set_status("Previewed long-static table was invalid. Re-run Fit.")
    return

  params.put(APPLIED_PARAM, json.dumps({"breakpoints": bps}))
  params.remove(PENDING_PARAM)
  short_summary = ",".join(f"{v:.0f}m/s" for v, *_ in bps)
  _set_status(f"Done|Applied ({short_summary}) — reboot to use|"
              f"Static-map table applied. carcontroller will invert plant "
              f"per (target_accel, v_ego) using per-bin (dz, s_pos, s_neg) "
              f"with safety clip to [{CMD_VALID_LO:.0f},{CMD_VALID_HI:.0f}] "
              f"CAN counts.  Reboot for clean startup.")


def reset_table() -> None:
  params.remove(APPLIED_PARAM)
  params.remove(PENDING_PARAM)
  _set_status(_idle_status())
