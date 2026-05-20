#!/usr/bin/env python3
# Independent lateral actuator delay diagnostic for Mazda Gen2.
#
# Background: openpilot's lagd.py learns lateral delay via masked normalized
# cross-correlation between commanded torque and observed lat_accel. That
# learner is LTI (linear time-invariant) by assumption. On a car with
# nonlinear EPS, speed-dependent assist, or - critically - a controller that
# is itself oscillating in the data, the cross-correlation peak can land on
# the OSCILLATION half-period instead of the true delay, giving a biased
# estimate that the closed-loop system never escapes.
#
# This script reproduces lagd's method on the same rlog data but ALSO runs
# the cross-correlation with ping-pong-contaminated windows excluded, AND
# bins the result by speed. Three numbers per speed bin let us tell the
# stories apart:
#   - filtered ≈ unfiltered → lagd was probably right, ping-pong is elsewhere
#   - filtered < unfiltered → lagd was biased high by ping-pong; trust the
#     filtered number
#   - both vary significantly across bins → real delay is speed-dependent
#     and a single global value is fundamentally wrong (mazda's variable-
#     assist EPS is a plausible cause)
#
# Read-only, SSH-only:
#   python3 -m openpilot.frogpilot.common.lag_diagnostic [--limit-logs N]

import argparse
import sys
import time
from pathlib import Path

import numpy as np

from cereal import log as capnp_log
from openpilot.frogpilot.common.torque_autotune import _ffill, _read_rlog_bytes, find_rlogs

# --- defaults --------------------------------------------------------------

DEFAULT_BINS_MS = (8.0, 14.0, 20.0, 28.0, 40.0)  # match long_autotune above the brake-overboost zone

GRID_HZ = 50.0
GRID_DT = 1.0 / GRID_HZ

MAX_LAG_S = 0.8                # tightened from lagd's 1.0 (boundary artifacts otherwise)
MIN_VEGO = 8.0                 # lower than lagd's 15.0 so the 8-14 bin gets samples
MIN_SIGNAL_STD = 0.05          # m/s² - window needs meaningful lat-accel command activity

WINDOW_LEN_S = 6.0             # cross-correlate over 6s chunks (300 samples)
WINDOW_HOP_S = 2.0             # 4s overlap between chunks
MIN_NCC = 0.70                 # accept a per-window estimate if NCC >= 0.70 (looser than lagd's 0.95)

# Ping-pong detector: bandpass commanded torque between ~0.5-2.0 Hz via
# subtracted moving averages, then count "energy" relative to total. Anything
# above OSC_RATIO_THRESHOLD is flagged as oscillation-contaminated.
OSC_LO_HZ = 0.5
OSC_HI_HZ = 2.0
OSC_WINDOW_S = 2.0
OSC_RATIO_THRESHOLD = 0.35     # bandpass²/total² > 0.35 → ping-pong


# --- helpers ---------------------------------------------------------------

def _moving_mean(x: np.ndarray, w: int) -> np.ndarray:
  """Centered moving mean, NaN-safe."""
  n = x.shape[0]
  if w <= 1 or n < w:
    return x.copy()
  x = np.where(np.isnan(x), 0.0, x)
  c = np.concatenate(([0.0], np.cumsum(x)))
  out = (c[w:] - c[:-w]) / w
  # Center: pad start/end with edge values so length matches.
  pad_lo = w // 2
  pad_hi = n - out.shape[0] - pad_lo
  return np.concatenate([np.full(pad_lo, out[0]), out, np.full(pad_hi, out[-1])])


def _oscillation_mask(output: np.ndarray) -> np.ndarray:
  """Per-sample True where commanded torque has 0.5-2 Hz energy ratio above threshold."""
  if output.size < int(OSC_WINDOW_S * GRID_HZ):
    return np.zeros_like(output, dtype=bool)
  hp_w = max(2, int(1.0 / (OSC_LO_HZ * GRID_DT)))   # ~2s window cuts <0.5 Hz
  lp_w = max(2, int(1.0 / (OSC_HI_HZ * GRID_DT)))   # ~0.5s window cuts >2 Hz
  hp = output - _moving_mean(output, hp_w)
  bp = _moving_mean(hp, lp_w)
  energy_w = int(OSC_WINDOW_S * GRID_HZ)
  bp_pow = _moving_mean(bp * bp, energy_w)
  tot_pow = _moving_mean(output * output, energy_w) + 1e-9
  return (bp_pow / tot_pow) > OSC_RATIO_THRESHOLD


def _xcorr_lag(desired: np.ndarray, actual: np.ndarray) -> tuple[float, float] | None:
  """Cross-correlate DESIRED lat_accel against ACTUAL lat_accel at lags ∈ [0, MAX_LAG_S].

  Both inputs in m/s². Matches lagd.py's signal convention: a lag of L
  means actual[t] best matches desired[t-L], i.e. actual trails desired
  by L seconds. Returns (peak_lag_s, peak_ncc) or None.

  Refuses a peak that lands at the boundary of the search range (lag=0
  or lag=MAX) - those are usually noise artifacts, not real peaks.
  """
  n = desired.size
  if n < int(0.5 * GRID_HZ):
    return None
  x = desired - desired.mean()
  y = actual - actual.mean()
  sx, sy = x.std(), y.std()
  if sx < 1e-6 or sy < 1e-6:
    return None
  x /= sx
  y /= sy

  max_lag = int(MAX_LAG_S * GRID_HZ)
  max_lag = min(max_lag, n - 1)
  lags = np.arange(0, max_lag + 1)
  ncc = np.empty(lags.size, dtype=np.float64)
  for i, k in enumerate(lags):
    ncc[i] = float((x[: n - k] * y[k:]).sum() / (n - k))
  peak = int(np.argmax(ncc))
  if peak == 0 or peak == lags.size - 1:
    return None  # boundary - not a real peak
  return float(peak / GRID_HZ), float(ncc[peak])


# --- rlog extractor --------------------------------------------------------

def _extract_lateral(path: Path):
  """Returns dict with uniform 50 Hz arrays for one rlog, or None on read failure."""
  try:
    dat = _read_rlog_bytes(path)
  except Exception:
    return None

  events = []
  try:
    reader = iter(capnp_log.Event.read_multiple_bytes(dat))
    while True:
      try:
        events.append(next(reader))
      except StopIteration:
        break
      except Exception:
        break  # salvage what we have
  except Exception:
    return None
  if not events:
    return None
  events.sort(key=lambda e: e.logMonoTime)

  n = len(events)
  t_evt = np.empty(n, dtype=np.float64)
  cols = {k: np.full(n, np.nan, dtype=np.float64) for k in
          ("desired_curvature", "actual_lat_accel", "v_ego",
           "steering_pressed", "lat_active")}

  for i, ev in enumerate(events):
    t_evt[i] = ev.logMonoTime / 1e9
    try:
      which = ev.which()
      if which == "carState":
        cs = ev.carState
        cols["v_ego"][i] = cs.vEgo
        cols["steering_pressed"][i] = 1.0 if cs.steeringPressed else 0.0
      elif which == "carControl":
        cols["lat_active"][i] = 1.0 if ev.carControl.latActive else 0.0
      elif which == "controlsState":
        # lagd's signal convention: desiredCurvature * v² is what the planner
        # asked for; torqueState.actualLateralAccel is what we observed.
        cs = ev.controlsState
        cols["desired_curvature"][i] = cs.desiredCurvature
        lcs = cs.lateralControlState
        if lcs.which() == "torqueState":
          cols["actual_lat_accel"][i] = lcs.torqueState.actualLateralAccel
    except Exception:
      continue

  # Bound the grid to the carState timespan (skip pre-drive initData outliers).
  cs_idx = np.where(np.isfinite(cols["v_ego"]))[0]
  if cs_idx.size == 0:
    return None
  t0, t1 = float(t_evt[cs_idx[0]]), float(t_evt[cs_idx[-1]])
  if t1 - t0 < 5.0:
    return None

  for k in cols:
    cols[k] = _ffill(cols[k])

  ng = int((t1 - t0) / GRID_DT) + 1
  t_grid = t0 + np.arange(ng) * GRID_DT
  idx = np.searchsorted(t_evt, t_grid, side="right") - 1
  idx = np.clip(idx, 0, n - 1)

  return {k: v[idx] for k, v in cols.items()} | {"t": t_grid}


# --- per-bin analysis ------------------------------------------------------

def _process_log(rec, per_bin):
  """Sweep a record into per-bin lists of cross-correlation peaks."""
  v_ego = rec["v_ego"]
  steering = rec["steering_pressed"]
  lat_act = rec["lat_active"]
  # lagd-style signals: desired = curvature * v_ego², actual = torqueState lat_accel
  desired = rec["desired_curvature"] * v_ego * v_ego
  actual = rec["actual_lat_accel"]
  n = desired.size

  valid = (np.isfinite(desired) & np.isfinite(actual) & np.isfinite(v_ego)
           & (lat_act > 0.5) & (steering < 0.5) & (v_ego >= MIN_VEGO))
  if not valid.any():
    return

  # Detect ping-pong from the DESIRED signal - if the planner itself is
  # oscillating, that's the signature we want to exclude.
  osc = _oscillation_mask(desired)
  valid_filt = valid & ~osc

  win = int(WINDOW_LEN_S * GRID_HZ)
  hop = int(WINDOW_HOP_S * GRID_HZ)

  i = 0
  while i + win <= n:
    sl = slice(i, i + win)
    if valid[sl].all():
      v_med = float(np.median(v_ego[sl]))
      bin_idx = _find_bin(v_med)
      if bin_idx is not None and desired[sl].std() >= MIN_SIGNAL_STD:
        r_unfilt = _xcorr_lag(desired[sl], actual[sl])
        if r_unfilt is not None and r_unfilt[1] >= MIN_NCC:
          per_bin[bin_idx]["unfilt"].append(r_unfilt)
        if valid_filt[sl].all():
          r_filt = _xcorr_lag(desired[sl], actual[sl])
          if r_filt is not None and r_filt[1] >= MIN_NCC:
            per_bin[bin_idx]["filt"].append(r_filt)
    i += hop


def _find_bin(v: float):
  for i in range(len(DEFAULT_BINS_MS) - 1):
    if DEFAULT_BINS_MS[i] <= v < DEFAULT_BINS_MS[i + 1]:
      return i
  return None


def _summary(rows):
  lags = np.array([r[0] for r in rows])
  nccs = np.array([r[1] for r in rows])
  if lags.size == 0:
    return None
  return {
    "n": lags.size,
    "median": float(np.median(lags)),
    "iqr_lo": float(np.percentile(lags, 25)),
    "iqr_hi": float(np.percentile(lags, 75)),
    "ncc_med": float(np.median(nccs)),
  }


# --- main ------------------------------------------------------------------

def main():
  ap = argparse.ArgumentParser(description="Mazda Gen2 lateral delay diagnostic")
  ap.add_argument("--limit-logs", type=int, default=0)
  ap.add_argument("--skip-tail", type=int, default=10,
                  help="skip last N rlogs (parked junk in konik dir)")
  args = ap.parse_args()

  rlogs = find_rlogs()
  if args.skip_tail > 0:
    rlogs = rlogs[:-args.skip_tail]
  if args.limit_logs > 0:
    rlogs = rlogs[-args.limit_logs:]
  if not rlogs:
    sys.exit("no driving rlogs found")

  print(f"Scanning {len(rlogs)} rlogs for lateral-delay samples...")
  print(f"(Filter: vEgo>={MIN_VEGO} m/s, latActive, no driver override, "
        f"desired_lat_accel_std>={MIN_SIGNAL_STD}; windows: {WINDOW_LEN_S}s/"
        f"{WINDOW_HOP_S}s; max_lag={MAX_LAG_S}s; NCC≥{MIN_NCC}.)")
  print(f"(Ping-pong filter: bandpass {OSC_LO_HZ}-{OSC_HI_HZ} Hz energy ratio > "
        f"{OSC_RATIO_THRESHOLD} on desired_lat_accel.)\n")

  per_bin = [{"unfilt": [], "filt": []} for _ in range(len(DEFAULT_BINS_MS) - 1)]
  t_start = time.time()

  for i, path in enumerate(rlogs, 1):
    rec = _extract_lateral(path)
    if rec is None:
      print(f"  [{i:3d}/{len(rlogs)}] {path.parent.name}: unreadable")
      continue
    _process_log(rec, per_bin)
    totals = [(len(b["unfilt"]), len(b["filt"])) for b in per_bin]
    print(f"  [{i:3d}/{len(rlogs)}] {path.parent.name}: per-bin (unfilt/filt) {totals}")

  print(f"\nProcessed in {time.time() - t_start:.1f}s")
  print("\nPer-bin lateral delay (filtered = ping-pong windows excluded):")
  print("  bin (m/s)   | unfilt (lagd-like)              | filt (ping-pong-clean)")
  print("  ----------- | ------------------------------- | ------------------------------")
  for i in range(len(DEFAULT_BINS_MS) - 1):
    lo, hi = DEFAULT_BINS_MS[i], DEFAULT_BINS_MS[i + 1]
    u = _summary(per_bin[i]["unfilt"])
    f = _summary(per_bin[i]["filt"])

    def fmt(s):
      if s is None:
        return "n=  0   --"
      return (f"n={s['n']:4d}  med={s['median']:.3f}s  "
              f"IQR=[{s['iqr_lo']:.3f},{s['iqr_hi']:.3f}]  ncc={s['ncc_med']:.2f}")

    print(f"  {lo:4g}-{hi:<5g} | {fmt(u):<32s} | {fmt(f)}")

  print("\nHow to read this:")
  print(" - 'unfilt' is what lagd-style cross-correlation produces on your data.")
  print(" - 'filt' is the same method with detected ping-pong windows excluded.")
  print(" - filt < unfilt across bins → lagd is biased high; trust 'filt'.")
  print(" - filt ≈ unfilt → ping-pong isn't the bias source; lagd's number is real.")
  print(" - large variation across bins → real delay is speed-dependent; the")
  print("   single global value can't represent your car correctly.")


if __name__ == "__main__":
  main()
