#!/usr/bin/env python3
# Mazda longitudinal delay rolling-store fitter + applier.
#
# Mirrors lat_delay_collect.py but consumes the existing delay store that
# long_collect.run_collect() already populates (delay_samples.f32, 4 cols:
# v_ego, lag_s, ncc, rpm). No new collector here - we just fit + apply.
#
#   1. fit_store / run_fit: aggregate per-speed-bin median lag, gate on data
#      quality (n, IQR, NCC), build a breakpoint table, stash in
#      LongDelayPending.
#   2. apply_pending: commit pending -> LongDelayTable. longcontrol.py and
#      longitudinal_planner.py interp this by current v_ego at runtime,
#      replacing the single toggle.longitudinalActuatorDelay scalar.
#   3. reset_table: clear LongDelayTable -> consumers fall back to the
#      single scalar.
#
# Why this matters: the hardcoded longitudinalActuatorDelay=0.35s in
# selfdrive/car/mazda/interface.py:137 is a single global value. Our
# delay store shows ~0.20s at highway, looser at low speed. A single
# value over-anticipates one regime and under-anticipates the other.
# Per-speed table fixes the systemic mismatch.
#
# Safety nets (consumers run every MPC tick):
#   - Per-bin gates: n>=MIN_BIN_N, IQR_width<=MAX_IQR_S, ncc_med>=MIN_NCC.
#   - Table-level clip [MIN_DELAY_S, MAX_DELAY_S] at fit AND apply.
#   - Need >=2 monotonic breakpoints or fit/apply refuses.
#   - Default off: consumers only use the table when toggle.use_long_delay_table
#     is True, which only flips when LongDelayTable parses cleanly.
#   - Don't shape-gate (per feedback_fit_shape_gates): reject on data
#     quality, not on the result shape. Other hardware could legitimately
#     have a flat or surprising delay profile.

import json

import numpy as np

from openpilot.common.params import Params
from openpilot.frogpilot.common.long_autotune import DEFAULT_BINS_MS
from openpilot.frogpilot.common.long_collect import (
  DELAY_SAMPLE_BYTES, DELAY_SAMPLES_PATH, load_delay_store,
)

STATUS_PARAM = "LongDelayStatus"
PENDING_PARAM = "LongDelayPending"
APPLIED_PARAM = "LongDelayTable"

# Per-bin data-quality gates. Long delay store is sparser than lateral
# (~97 samples vs hundreds), so n threshold is lower; IQR is tighter
# because real long delay variance is smaller than steering delay.
MIN_BIN_N = 5
MAX_IQR_S = 0.30
MIN_BIN_NCC = 0.75

# Whole-table sanity clip: every breakpoint pinned to this range.
# MPC-relevant: 0.05s is essentially "act now"; 0.80s is the worst
# car/bus delay we'd ever expect.
MIN_DELAY_S = 0.05
MAX_DELAY_S = 0.80

BIN_CENTERS_MS = tuple(0.5 * (lo + hi) for lo, hi in zip(DEFAULT_BINS_MS[:-1], DEFAULT_BINS_MS[1:]))


params = Params()


def _set_status(msg: str) -> None:
  Params("/dev/shm/params").put(STATUS_PARAM, msg)


# --- fit -------------------------------------------------------------------

def fit_store() -> dict | None:
  """Returns {'breakpoints': [(v, delay), ...], 'n': total, 'per_bin': [...]}
  or None if no bin passes gates / fewer than 2 bins usable.
  """
  data = load_delay_store()
  if data.shape[0] == 0:
    return None
  v = data[:, 0]
  lag = data[:, 1]
  ncc = data[:, 2]

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
  if len(bps) < 2:
    return None

  return {"breakpoints": bps, "n": int(data.shape[0]), "per_bin": per_bin}


# --- status helpers --------------------------------------------------------

def _idle_status() -> str:
  try:
    n_samples = DELAY_SAMPLES_PATH.stat().st_size // DELAY_SAMPLE_BYTES
  except FileNotFoundError:
    n_samples = 0
  applied = params.get(APPLIED_PARAM)
  table_str = ""
  if applied:
    try:
      t = json.loads(applied)
      table_str = "  applied: " + ",".join(f"{v:.1f}m/s→{d:.3f}s"
                                            for v, d in t.get("breakpoints", []))
    except Exception:
      pass
  return (f"Idle|{n_samples} delay samples|"
          f"Delay store: {n_samples} xcorr windows.  Tap Fit to build a per-speed "
          f"long-delay table from the rolling store.{table_str}")


def restore_idle_status() -> None:
  if Params("/dev/shm/params").get(STATUS_PARAM):
    return
  _set_status(_idle_status())


def _preview_status(fit: dict) -> str:
  bps = fit["breakpoints"]
  short = "→".join(f"{d:.2f}" for _, d in bps)
  full_bins = "  ".join(
    f"{e['lo']:g}-{e['hi']:g}: med={e['median']:.3f}s n={e['n']} iqr={e['iqr_w']:.2f} "
    f"ncc={e['ncc_med']:.2f} {'OK' if e['ok'] else 'FAIL'}"
    for e in fit["per_bin"] if e is not None
  )
  return (f"Preview|Preview ready — tap Apply ({short})|"
          f"Bps: " + ", ".join(f"{v:.1f}m/s→{d:.3f}s" for v, d in bps)
          + f"  ·  {fit['n']} total samples  ·  per-bin: " + full_bins)


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
                "Long-delay fit refused: too few samples per bin, or per-bin "
                "IQR/NCC gates failed. Drive more on Experimental mode and retry.")
    return
  params.put(PENDING_PARAM, json.dumps(fit))
  _set_status(_preview_status(fit))


def apply_pending() -> None:
  raw = params.get(PENDING_PARAM)
  if not raw:
    _set_status("No previewed long-delay table to apply. Run Fit first.")
    return
  try:
    fit = json.loads(raw)
    bps = fit["breakpoints"]
    bps = [(float(v), float(np.clip(d, MIN_DELAY_S, MAX_DELAY_S))) for v, d in bps]
    if len(bps) < 2:
      raise ValueError("need ≥2 breakpoints")
    vs = [v for v, _ in bps]
    if not all(b > a for a, b in zip(vs[:-1], vs[1:])):
      raise ValueError("breakpoints must be strictly increasing in v_ego")
  except Exception:
    params.remove(PENDING_PARAM)
    _set_status("Previewed long-delay table was invalid. Re-run Fit.")
    return

  params.put(APPLIED_PARAM, json.dumps({"breakpoints": bps}))
  params.remove(PENDING_PARAM)
  short = "→".join(f"{d:.2f}" for _, d in bps)
  _set_status(f"Done|Applied ({short}) — reboot to use|"
              f"Long-delay table applied. The long planner and controller will "
              f"interp longitudinalActuatorDelay by current v_ego using: "
              + ", ".join(f"{v:.1f}m/s→{d:.3f}s" for v, d in bps)
              + ".  Reboot for clean startup.")


def reset_table() -> None:
  params.remove(APPLIED_PARAM)
  params.remove(PENDING_PARAM)
  _set_status(_idle_status())
