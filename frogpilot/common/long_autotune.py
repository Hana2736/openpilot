#!/usr/bin/env python3
# Mazda Gen2 longitudinal "static-map" diagnostic.
#
# The current Gen2 long path is one affine map:
#
#     raw_acc_output = int(target_accel * accel_scale + accel_offset)
#
# (selfdrive/car/mazda/values.py + carcontroller.py), with a low-speed brake
# overboost on top. Openpilot does NOT own actuator dynamics here - Mazda's own
# ACC closed loop interprets the CAN setpoint we send. So the right ID problem
# is: what is the steady-state forward map "CAN ACCEL_CMD on the bus -> achieved
# aEgo", parameterized by vEgo? Once we have that, the carcontroller can use the
# inverse instead of the global affine that spends time inside Mazda's own
# deadband.
#
# We read the ACTUAL integer that was on the bus (Mazda msg 544 "ACC",
# signal ACCEL_CMD = bits 16|12@0+, big-endian unsigned), not
# carControl.actuators.accel. The plant (Mazda's onboard controller + throttle/
# brake response) doesn't care whether that integer came from Mazda's own ACC
# algorithm, from openpilot's affine, or from a blend - so logs collected with
# BlendedACC=ON or stock ACC are all fair game. That's a much wider data set
# than waiting for a pure-OP commute would give us.
#
# v1 is read-only. SSH in, run:
#   python3 -m openpilot.frogpilot.common.long_autotune
# It prints a per-speed-bin table of (n, dz_can, s_pos, s_neg, R²) and writes
# nothing live. If the table looks structurally clean we promote to v2
# (carcontroller integration with preview/apply).

import argparse
import sys
import time
from pathlib import Path

import numpy as np

from cereal import log as capnp_log
from openpilot.frogpilot.common.torque_autotune import _ffill, _read_rlog_bytes, find_rlogs

# --- defaults --------------------------------------------------------------

# Speed bins (m/s). The lowest one overlaps brake_overboost (vEgo<6) on the
# brake side; we still report it but tag it so it's not used for the inverse.
DEFAULT_BINS_MS = (0.0, 4.0, 8.0, 14.0, 20.0, 28.0, 40.0)

# Mazda CAN ACCEL_CMD wire format (mazda_2019.dbc: BO_ 544 ACC,
# SG_ ACCEL_CMD : 16|12@0+). Layout is non-byte-aligned - see
# _decode_accel_cmd for the actual bit math (resolved against opendbc's
# be_bits lookup). Stock-coast / openpilot-zero both center on 2000.
#
# Msg 544 appears on multiple buses in the rlog: bus 0 (camera side,
# IDLE pattern - bytes 2,3 latched at 01 F4 with no real ACCEL_CMD
# changes) and "bus 2 echo" with src=130 (panda's own TX echo to the
# powertrain bus, mazdacan.py:create_acc_cmd). Only src=130 carries the
# ACCEL_CMD that actually drives the car.
ACC_MSG_ADDR = 544
ACC_MSG_SRC = 130                   # bus 2 TX echo
ACC_CMD_CENTER = 2000.0
# Tolerance on CAN integer for "constant" inside a window: ±10 counts
# corresponds to ±0.05 m/s² under the current 200 counts/(m/s²) affine,
# which is the same swing budget we used before.
CMD_COUNTS_TOLERANCE = 10
# Mazda's bus broadcasts 4000 (and similar high values) as an "ACC not
# active" sentinel; in practice only [1500,2500] is real commanded accel.
# Samples outside this window are dropped as invalid before any further
# processing.
CMD_VALID_LO = 1500
CMD_VALID_HI = 2500
# Max time (s) a ffilled can_cmd sample is considered "fresh". msg 544
# broadcasts at ~50 Hz when ACC is active, so anything stale by >100 ms
# is almost certainly an inactive gap we should not fit through.
CMD_MAX_STALE_S = 0.1

# Steady-state window thresholds.
WINDOW_SECONDS = 1.0           # min steady duration
AEGO_STD_MAX = 0.15            # m/s^2 - rolling std of aEgo inside window
PITCH_THRESH = 0.04            # rad

# Sanity gates for the per-bin fit (in CAN counts, not m/s^2).
DZ_MAX_COUNTS = 100            # |dz_can - 2000| <= 100  (~±0.5 m/s²)
SLOPE_LO, SLOPE_HI = 1.0 / 600.0, 1.0 / 60.0   # m/s² per count, ≈[0.00167, 0.0167]
MIN_PER_SIDE = 8               # >=8 above-center and >=8 below-center per bin
MIN_FOR_LINE = 8               # min total samples for the fallback no-deadband line

# Sample grid.
GRID_HZ = 50.0
GRID_DT = 1.0 / GRID_HZ


# --- rlog -> uniform 50 Hz arrays ------------------------------------------

def _decode_accel_cmd(dat: bytes) -> int | None:
  """Decode Mazda ACC msg 544 ACCEL_CMD (DBC: 16|12@0+, big-endian unsigned).

  opendbc resolves this to lsb=37, msb=16 (see opendbc/can/dbc.cc be_bits
  lookup). The 12-bit signal is scattered across 3 bytes:
    - byte 2, bit 0:        signal bit 11 (MSB)
    - byte 3, bits 7..0:    signal bits 10..3
    - byte 4, bits 7..5:    signal bits 2..0 (LSB)
  Sanity: 0 m/s² command should land at value 2000 (carcontroller affine).
  """
  if len(dat) < 5:
    return None
  return ((dat[2] & 1) << 11) | (dat[3] << 3) | ((dat[4] >> 5) & 7)


def _extract_one(path: Path):
  """Returns dict of uniform 50 Hz arrays for one rlog, or None on read failure.

  Keys: t, can_cmd (bus int), v_ego, a_ego, gas, brake, standstill, pitch,
        rpm (engine RPM, gear-as-(v_ego, rpm) proxy), accel_desired (MPC's
        target_accel from carControl.actuators.accel - used for long delay
        xcorr against a_ego), long_active.
  """
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
        break  # truncated tail: salvage what we have
  except Exception:
    return None

  if not events:
    return None
  events.sort(key=lambda e: e.logMonoTime)

  n = len(events)
  t_evt = np.empty(n, dtype=np.float64)
  cols = {k: np.full(n, np.nan, dtype=np.float64) for k in
          ("can_cmd", "v_ego", "a_ego", "gas", "brake", "standstill", "pitch",
           "rpm", "accel_desired", "long_active")}

  for i, ev in enumerate(events):
    t_evt[i] = ev.logMonoTime / 1e9
    try:
      which = ev.which()
      if which == "carState":
        cs = ev.carState
        cols["v_ego"][i] = cs.vEgo
        cols["a_ego"][i] = cs.aEgo
        cols["gas"][i] = 1.0 if cs.gasPressed else 0.0
        cols["brake"][i] = 1.0 if cs.brakePressed else 0.0
        cols["standstill"][i] = 1.0 if cs.standstill else 0.0
        cols["rpm"][i] = cs.engineRpm
      elif which == "carControl":
        cc = ev.carControl
        cols["accel_desired"][i] = cc.actuators.accel
        cols["long_active"][i] = 1.0 if cc.longActive else 0.0
      elif which == "liveLocationKalman":
        cols["pitch"][i] = ev.liveLocationKalman.orientationNED.value[1]
      elif which == "can":
        for frame in ev.can:
          if frame.address == ACC_MSG_ADDR and frame.src == ACC_MSG_SRC:
            cmd = _decode_accel_cmd(bytes(frame.dat))
            # Drop the "ACC not active" sentinel range so it doesn't
            # blow out the ffill or get treated as a valid sample.
            if cmd is not None and CMD_VALID_LO <= cmd <= CMD_VALID_HI:
              cols["can_cmd"][i] = float(cmd)
            break
    except Exception:
      continue

  if not np.isfinite(t_evt).any():
    return None
  # Early-boot events (e.g. initData) have logMonoTimes from before the
  # actual drive started - one such outlier blows the grid up to ~26min
  # when the real drive is 60s. Bound the grid to the carState timespan,
  # which is what we actually fit against.
  cs_idx = np.where(np.isfinite(cols["v_ego"]))[0]
  if cs_idx.size == 0:
    return None
  t0 = float(t_evt[cs_idx[0]])
  t1 = float(t_evt[cs_idx[-1]])
  if t1 - t0 < 2.0:
    return None  # rlog too short to contain a steady window

  # Track freshness of can_cmd separately so we can reject samples that
  # were ffilled across an inactive-ACC gap (sentinel-filtered to NaN).
  cmd_valid = np.isfinite(cols["can_cmd"])
  cmd_last_t = np.where(cmd_valid, t_evt, np.nan)
  cmd_last_t = _ffill(cmd_last_t)

  # ffill on event-indexed arrays, then sample onto a uniform 50 Hz grid using
  # right-side searchsorted (latest event at or before each grid tick).
  for k in cols:
    cols[k] = _ffill(cols[k])

  ng = int((t1 - t0) / GRID_DT) + 1
  t_grid = t0 + np.arange(ng) * GRID_DT
  idx = np.searchsorted(t_evt, t_grid, side="right") - 1
  idx = np.clip(idx, 0, n - 1)

  out = {"t": t_grid}
  for k, v in cols.items():
    out[k] = v[idx]
  # Stale-can_cmd mask: drop ffilled samples >CMD_MAX_STALE_S past the
  # last actually-active frame. Reuses the same idx so it stays aligned.
  cmd_age = t_grid - cmd_last_t[idx]
  stale = ~np.isfinite(cmd_age) | (cmd_age > CMD_MAX_STALE_S)
  out["can_cmd"] = np.where(stale, np.nan, out["can_cmd"])
  return out


# --- steady-state window detection ----------------------------------------

def _rolling_min_max(x: np.ndarray, w: int):
  """Naive O(n*w) rolling min/max - w is small (50) so this is fine."""
  n = x.shape[0]
  if n < w:
    return np.full(n, np.nan), np.full(n, np.nan)
  # Stride trick would be cheaper; for w=50 and n~1e5 the loop is still cheap.
  cmin = np.full(n, np.nan)
  cmax = np.full(n, np.nan)
  for off in range(w):
    seg = x[off:n - w + 1 + off]
    if off == 0:
      cmin[w - 1:] = seg
      cmax[w - 1:] = seg
    else:
      cmin[w - 1:] = np.minimum(cmin[w - 1:], seg)
      cmax[w - 1:] = np.maximum(cmax[w - 1:], seg)
  return cmin, cmax


def _rolling_std(x: np.ndarray, w: int):
  n = x.shape[0]
  out = np.full(n, np.nan)
  if n < w:
    return out
  # np.cumsum propagates NaN, so any leading NaN poisons the whole result.
  # ffill, then patch any remaining prefix NaN with the first finite value;
  # callers exclude those rows via their own validity mask anyway.
  x = np.asarray(x, dtype=float)
  if np.isnan(x).any():
    x = _ffill(x)
    if x.size and np.isnan(x[0]):
      finite = x[~np.isnan(x)]
      x = np.where(np.isnan(x), finite[0] if finite.size else 0.0, x)
  c1 = np.concatenate(([0.0], np.cumsum(x)))
  c2 = np.concatenate(([0.0], np.cumsum(x * x)))
  s = c1[w:] - c1[:-w]
  sq = c2[w:] - c2[:-w]
  var = (sq - s * s / w) / max(1, w - 1)
  out[w - 1:] = np.sqrt(np.clip(var, 0.0, None))
  return out


def _collect_steady_samples(rec):
  """Returns (n, 5) array of (v_ego, can_cmd, a_ego, pitch, rpm) medians per
  steady-state window. Pitch is stored (not filtered) so the fit can apply
  a gravity correction at fit time instead of throwing hill samples away.
  RPM lets later fits bin by (v_ego, rpm) which is sufficient gear context
  on a manual transmission (same speed at different RPM = different gear).
  """
  w = int(WINDOW_SECONDS * GRID_HZ)

  cc = rec["can_cmd"]
  ae = rec["a_ego"]
  ve = rec["v_ego"]
  ph = rec["pitch"]
  rp = rec["rpm"]
  if cc.shape[0] < w:
    return np.empty((0, 5), dtype=np.float32)

  base_valid = (np.isfinite(cc) & np.isfinite(ae) & np.isfinite(ve)
                & np.isfinite(ph) & np.isfinite(rp) & np.isfinite(rec["gas"])
                & np.isfinite(rec["brake"]) & np.isfinite(rec["standstill"]))

  # Driver-clean only. Pitch is no longer filtered - we store it.
  driver_clean = (rec["gas"] < 0.5) & (rec["brake"] < 0.5) & (rec["standstill"] < 0.5)

  # Steady-state filters: can_cmd swing within tolerance, aEgo settled.
  cc_min, cc_max = _rolling_min_max(cc, w)
  cc_swing = cc_max - cc_min
  ae_std = _rolling_std(ae, w)

  mask_all = base_valid & driver_clean
  mask_f = mask_all.astype(np.float64)
  mask_min, _ = _rolling_min_max(mask_f, w)

  ok = (np.isfinite(cc_swing) & (cc_swing <= CMD_COUNTS_TOLERANCE)
        & np.isfinite(ae_std) & (ae_std <= AEGO_STD_MAX)
        & (mask_min > 0.5))

  if not ok.any():
    return np.empty((0, 5), dtype=np.float32)

  rows = []
  i = w - 1
  end = cc.shape[0]
  while i < end:
    if ok[i]:
      lo = i - w + 1
      rows.append((float(np.median(ve[lo:i + 1])),
                   float(np.median(cc[lo:i + 1])),
                   float(np.median(ae[lo:i + 1])),
                   float(np.median(ph[lo:i + 1])),
                   float(np.median(rp[lo:i + 1]))))
      i += w
    else:
      i += 1
  if not rows:
    return np.empty((0, 5), dtype=np.float32)
  return np.asarray(rows, dtype=np.float32)


# --- long delay xcorr (per-window) -----------------------------------------
#
# Mirrors the lateral delay diagnostic (see frogpilot/common/lag_diagnostic.py)
# but on the longitudinal signal pair: desired = carControl.actuators.accel,
# actual = carState.aEgo. Both in m/s², same units, only time-shifted. The
# lag between them is the end-to-end pipeline delay (carcontroller +
# CAN + Mazda's onboard ACC + powertrain settling).

# Same band-bass thresholds as lateral - long ping-pong (if any) would also
# be around 1 Hz.
DELAY_OSC_LO_HZ = 0.5
DELAY_OSC_HI_HZ = 2.0
DELAY_OSC_WINDOW_S = 2.0
DELAY_OSC_RATIO = 0.35

DELAY_WINDOW_LEN_S = 6.0
DELAY_WINDOW_HOP_S = 2.0
DELAY_MAX_LAG_S = 0.8
DELAY_MIN_NCC = 0.70
DELAY_MIN_ACCEL_STD = 0.10   # m/s² - need meaningful command variation


def _moving_mean(x: np.ndarray, w: int) -> np.ndarray:
  n = x.shape[0]
  if w <= 1 or n < w:
    return x.copy()
  x = np.where(np.isnan(x), 0.0, x)
  c = np.concatenate(([0.0], np.cumsum(x)))
  out = (c[w:] - c[:-w]) / w
  pad_lo = w // 2
  pad_hi = n - out.shape[0] - pad_lo
  return np.concatenate([np.full(pad_lo, out[0]), out, np.full(pad_hi, out[-1])])


def _oscillation_mask_long(x: np.ndarray) -> np.ndarray:
  if x.size < int(DELAY_OSC_WINDOW_S * GRID_HZ):
    return np.zeros_like(x, dtype=bool)
  hp_w = max(2, int(1.0 / (DELAY_OSC_LO_HZ * GRID_DT)))
  lp_w = max(2, int(1.0 / (DELAY_OSC_HI_HZ * GRID_DT)))
  hp = x - _moving_mean(x, hp_w)
  bp = _moving_mean(hp, lp_w)
  energy_w = int(DELAY_OSC_WINDOW_S * GRID_HZ)
  bp_pow = _moving_mean(bp * bp, energy_w)
  tot_pow = _moving_mean(x * x, energy_w) + 1e-9
  return (bp_pow / tot_pow) > DELAY_OSC_RATIO


def _xcorr_peak(desired: np.ndarray, actual: np.ndarray):
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
  max_lag = min(int(DELAY_MAX_LAG_S * GRID_HZ), n - 1)
  ncc = np.empty(max_lag + 1, dtype=np.float64)
  for k in range(max_lag + 1):
    ncc[k] = float((x[: n - k] * y[k:]).sum() / (n - k))
  peak = int(np.argmax(ncc))
  if peak == 0 or peak == max_lag:
    return None  # boundary artifact
  return float(peak / GRID_HZ), float(ncc[peak])


def _collect_delay_samples(rec):
  """Returns (n, 4) float32 array of (v_ego, lag_s, ncc, rpm) per filtered
  6s window. Filtering: OP long active, no driver override, command-side
  variation >= MIN_ACCEL_STD, no 0.5-2 Hz oscillation contamination.
  """
  desired = rec["accel_desired"]
  actual = rec["a_ego"]
  v_ego = rec["v_ego"]
  rpm = rec["rpm"]
  long_act = rec["long_active"]
  gas = rec["gas"]
  brake = rec["brake"]
  standstill = rec["standstill"]
  n = desired.size

  valid = (np.isfinite(desired) & np.isfinite(actual) & np.isfinite(v_ego)
           & np.isfinite(rpm) & np.isfinite(long_act) & np.isfinite(gas)
           & np.isfinite(brake) & np.isfinite(standstill)
           & (long_act > 0.5) & (gas < 0.5) & (brake < 0.5) & (standstill < 0.5))
  if not valid.any():
    return np.empty((0, 4), dtype=np.float32)

  osc = _oscillation_mask_long(desired)
  valid &= ~osc

  win = int(DELAY_WINDOW_LEN_S * GRID_HZ)
  hop = int(DELAY_WINDOW_HOP_S * GRID_HZ)

  rows = []
  i = 0
  while i + win <= n:
    sl = slice(i, i + win)
    if valid[sl].all() and desired[sl].std() >= DELAY_MIN_ACCEL_STD:
      r = _xcorr_peak(desired[sl], actual[sl])
      if r is not None and r[1] >= DELAY_MIN_NCC:
        rows.append((float(np.median(v_ego[sl])), r[0], r[1],
                     float(np.median(rpm[sl]))))
    i += hop
  if not rows:
    return np.empty((0, 4), dtype=np.float32)
  return np.asarray(rows, dtype=np.float32)


# --- piecewise-affine with deadband ---------------------------------------

def _fit_pwl_deadband(can_cmd: np.ndarray, ae: np.ndarray):
  """Fit aEgo = s_pos*(can - dz_hi) for can>dz_hi, 0 in [dz_lo, dz_hi],
  s_neg*(can - dz_lo) for can<dz_lo. Inputs are raw 12-bit ints centered
  on ~2000. Slopes are m/s² per CAN count. Grid search over (dz_lo, dz_hi)
  inside ±DZ_MAX_COUNTS of 2000. Returns dict or None if gates fail.
  """
  if (can_cmd > ACC_CMD_CENTER).sum() < MIN_PER_SIDE or (can_cmd < ACC_CMD_CENTER).sum() < MIN_PER_SIDE:
    return None

  dz_grid = np.arange(-DZ_MAX_COUNTS, DZ_MAX_COUNTS + 1, 10) + ACC_CMD_CENTER
  best = None
  for dz_lo in dz_grid:
    for dz_hi in dz_grid:
      if dz_hi < dz_lo:
        continue
      mt = can_cmd > dz_hi
      mb = can_cmd < dz_lo
      if mt.sum() < MIN_PER_SIDE or mb.sum() < MIN_PER_SIDE:
        continue
      x_t = can_cmd[mt] - dz_hi
      y_t = ae[mt]
      x_b = can_cmd[mb] - dz_lo
      y_b = ae[mb]
      st = float(x_t @ y_t) / float(x_t @ x_t)
      sb = float(x_b @ y_b) / float(x_b @ x_b)
      if not (SLOPE_LO <= st <= SLOPE_HI) or not (SLOPE_LO <= sb <= SLOPE_HI):
        continue
      pred_t = st * x_t
      pred_b = sb * x_b
      sse = float(((y_t - pred_t) ** 2).sum() + ((y_b - pred_b) ** 2).sum())
      if best is None or sse < best["sse"]:
        r2_t = 1.0 - float(((y_t - pred_t) ** 2).sum()) / max(1e-9, float(((y_t - y_t.mean()) ** 2).sum()))
        r2_b = 1.0 - float(((y_b - pred_b) ** 2).sum()) / max(1e-9, float(((y_b - y_b.mean()) ** 2).sum()))
        best = {"dz_lo": float(dz_lo), "dz_hi": float(dz_hi),
                "s_pos": st, "s_neg": sb, "r2_pos": r2_t, "r2_neg": r2_b,
                "n_pos": int(mt.sum()), "n_neg": int(mb.sum()), "sse": sse}
  return best


# --- main ------------------------------------------------------------------

def _fit_line(can_cmd: np.ndarray, ae: np.ndarray):
  """Fallback when the deadband fit gates fail: regress aEgo against
  (can_cmd - 2000) with no deadband, free intercept. Returns
  (slope_per_100ct, intercept, r2). slope is m/s² per 100 CAN counts."""
  if can_cmd.size < MIN_FOR_LINE:
    return None
  x = (can_cmd - ACC_CMD_CENTER)
  X = np.column_stack([x, np.ones_like(x)])
  beta, *_ = np.linalg.lstsq(X, ae, rcond=None)
  pred = X @ beta
  ss_res = float(((ae - pred) ** 2).sum())
  ss_tot = max(1e-9, float(((ae - ae.mean()) ** 2).sum()))
  return float(beta[0] * 100.0), float(beta[1]), 1.0 - ss_res / ss_tot


def _print_row(label, n, fit, line):
  if fit is not None:
    s_pos_100 = fit['s_pos'] * 100.0
    s_neg_100 = fit['s_neg'] * 100.0
    print(f"  {label:>10s}  n={n:5d}  "
          f"dz_can=[{fit['dz_lo']:.0f},{fit['dz_hi']:.0f}]  "
          f"+slope={s_pos_100:.3f}/100ct (n={fit['n_pos']:4d}, R²={fit['r2_pos']:+.3f})  "
          f"-slope={s_neg_100:.3f}/100ct (n={fit['n_neg']:4d}, R²={fit['r2_neg']:+.3f})")
    return
  if line is not None:
    slope100, intercept, r2 = line
    print(f"  {label:>10s}  n={n:5d}  [deadband fit gated; fallback line] "
          f"slope={slope100:+.3f}/100ct  intercept={intercept:+.3f}m/s²  R²={r2:+.3f}")
    return
  print(f"  {label:>10s}  n={n:5d}  -- too few samples even for the line fallback --")


def main():
  ap = argparse.ArgumentParser(description="Mazda Gen2 long static-map diagnostic")
  ap.add_argument("--limit-logs", type=int, default=0, help="0 = all available")
  ap.add_argument("--skip-tail", type=int, default=0,
                  help="skip the last N rlogs (the konik dir tail is parked junk)")
  ap.add_argument("--max-samples", type=int, default=200000)
  ap.add_argument("--bins", type=str, default=",".join(f"{b:g}" for b in DEFAULT_BINS_MS))
  ap.add_argument("--use-store", action="store_true",
                  help="read from /data/media/0/long_autotune/samples.f32 "
                       "(populated by the GUI's Collect Long Samples button) "
                       "instead of re-scanning rlogs.")
  args = ap.parse_args()

  bins = np.asarray([float(x) for x in args.bins.split(",")], dtype=float)
  if bins.size < 2:
    sys.exit("--bins needs at least two edges")

  if args.use_store:
    from openpilot.frogpilot.common.long_collect import load_store, SAMPLES_PATH
    samples = load_store()
    if samples.shape[0] == 0:
      sys.exit(f"store at {SAMPLES_PATH} is empty; press 'Collect Long Samples' first")
    print(f"Loaded {samples.shape[0]} samples from {SAMPLES_PATH}")
  else:
    rlogs = find_rlogs()
    if args.skip_tail > 0:
      rlogs = rlogs[:-args.skip_tail]
    if args.limit_logs > 0:
      rlogs = rlogs[-args.limit_logs:]
    if not rlogs:
      sys.exit("no rlogs under /data/media/0/realdata*")

    print(f"Scanning {len(rlogs)} rlogs for steady-state long samples...")
    print(f"(Reading bus ACCEL_CMD from msg {ACC_MSG_ADDR}; BlendedACC state irrelevant.)")
    t_start = time.time()

    all_rows = []
    total_samples = 0
    for i, path in enumerate(rlogs, 1):
      rec = _extract_one(path)
      if rec is None:
        print(f"  [{i:3d}/{len(rlogs)}] {path.parent.name}: unreadable")
        continue
      rows = _collect_steady_samples(rec)
      total_samples += rows.shape[0]
      if rows.shape[0]:
        all_rows.append(rows)
      print(f"  [{i:3d}/{len(rlogs)}] {path.parent.name}: +{rows.shape[0]} steady windows  (running total {total_samples})")
      if total_samples >= args.max_samples:
        print(f"  -- hit --max-samples cap ({args.max_samples}); stopping ingest")
        break

    if not all_rows:
      sys.exit("no steady-state windows found")

    samples = np.concatenate(all_rows, axis=0)  # (N, 3): v_ego, can_cmd, a_ego
    print(f"\nCollected {samples.shape[0]} steady windows in {time.time() - t_start:.1f}s")

  v = samples[:, 0]
  cmd = samples[:, 1]
  ae = samples[:, 2]
  med = float(np.median(cmd))
  print(f"can_cmd range observed: [{cmd.min():.0f}, {cmd.max():.0f}]  median {med:.0f}")
  if not (1700.0 <= med <= 2300.0):
    print("WARNING: median can_cmd is not near 2000 - decode of ACC msg 544")
    print("ACCEL_CMD may be wrong, or the dataset is heavily skewed. Bailing.")
    sys.exit(3)

  print("\nPer-bin piecewise-affine-with-deadband fit (input units: 12-bit CAN ints, output: m/s²):")
  print("  bin (m/s)   n              deadband              +slope (above dz_hi)            -slope (below dz_lo)")
  for i in range(bins.size - 1):
    lo, hi = bins[i], bins[i + 1]
    mask = (v >= lo) & (v < hi)
    n_bin = int(mask.sum())
    label = f"{lo:g}-{hi:g}"
    if n_bin == 0:
      print(f"  {label:>10s}  n=    0  --")
      continue
    fit = _fit_pwl_deadband(cmd[mask], ae[mask])
    line = _fit_line(cmd[mask], ae[mask]) if fit is None else None
    _print_row(label, n_bin, fit, line)

  print(f"\n(Gates: per-side n >= {MIN_PER_SIDE}, |dz_can - {ACC_CMD_CENTER:.0f}| <= {DZ_MAX_COUNTS},")
  print(f" slope in [{SLOPE_LO*100:.3f}, {SLOPE_HI*100:.3f}] m/s² per 100 counts.")
  print(" Lowest bin overlaps Mazda's brake_overboost regime on the brake side -")
  print(" its -slope reflects overboost-distorted samples, ignore for inverse design.)")


if __name__ == "__main__":
  main()
