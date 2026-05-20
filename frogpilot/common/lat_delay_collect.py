#!/usr/bin/env python3
# Mazda lateral delay rolling sample collector + per-speed fit.
#
# The diagnostic in lag_diagnostic.py established that the user's true
# lateral delay is speed-dependent (0.60 → 0.34 → 0.44 → 0.49 across
# 8-14/14-20/20-28/28-40 m/s bins for this car) and that lagd's single-
# value learner systematically misses by enough to drive the loop unstable
# at 45-50 mph. This module:
#
#   1. Collects per-window (v_ego, lag_s, ncc) samples each time the user
#      taps "Collect Delay Samples", reusing the diagnostic's filtered
#      cross-correlation pipeline (ping-pong windows excluded).
#   2. On "Fit Delay Table", aggregates per speed bin -> median delay,
#      builds a breakpoint table, gates it through sanity checks, and
#      stashes the candidate in LatDelayPending (NOT live).
#   3. On "Apply Delay Table", commits the pending candidate to
#      LatDelayTable. lagd.py interpolates from this at runtime (see
#      lagd consumer change) instead of publishing a single learned value.
#
# Safety nets, since lagd's output drives every steering command:
#   - Per-bin gates: n_samples >= MIN_BIN_N, IQR_width <= MAX_IQR_S,
#     ncc_median >= MIN_NCC.
#   - Table-level: every breakpoint clipped to [MIN_DELAY_S, MAX_DELAY_S].
#   - Bad table -> fallback path in lagd does the existing thing.
#   - Default off: lagd only consumes the table when use_steer_delay_table
#     is True, which only flips when the user taps Apply.

import json
import os
from pathlib import Path

import numpy as np

from openpilot.common.params import Params
from openpilot.frogpilot.common.lag_diagnostic import (
  DEFAULT_BINS_MS, MIN_NCC, _extract_lateral, _find_bin,
  _oscillation_mask, _xcorr_lag,
  GRID_HZ, WINDOW_LEN_S, WINDOW_HOP_S, MIN_VEGO, MIN_SIGNAL_STD,
)
from openpilot.frogpilot.common.torque_autotune import find_rlogs

# --- storage ---------------------------------------------------------------

STORE_DIR = Path("/data/media/0/lat_delay_autotune")
SAMPLES_PATH = STORE_DIR / "samples.f32"      # v_ego, lag_s, ncc per filtered window
PROCESSED_PATH = STORE_DIR / "processed.json"

SAMPLE_COLS = 3
SAMPLE_BYTES = SAMPLE_COLS * 4                # float32
MAX_STORE_BYTES = 100 * 1024 * 1024           # 100 MB - way more than needed (1 sample/window is sparse)

STATUS_PARAM = "LatDelayStatus"
PENDING_PARAM = "LatDelayPending"              # JSON breakpoint table preview
APPLIED_PARAM = "LatDelayTable"                # JSON breakpoint table live (consumed by lagd)

# Per-bin sanity gates. Medians are robust; n drives stability not IQR.
MIN_BIN_N = 10                                 # >=10 filtered windows per bin
MAX_IQR_S = 0.50                               # only reject genuinely broken bins
MIN_BIN_NCC = 0.80                             # median NCC per bin

# Whole-table sanity (every interpolated value falls in this range).
MIN_DELAY_S = 0.10
MAX_DELAY_S = 0.80

# Bin center (used as the breakpoint v_ego in the table).
BIN_CENTERS_MS = tuple(0.5 * (lo + hi) for lo, hi in zip(DEFAULT_BINS_MS[:-1], DEFAULT_BINS_MS[1:]))


# --- status helpers --------------------------------------------------------

params = Params()


def _set_status(msg: str) -> None:
  Params("/dev/shm/params").put(STATUS_PARAM, msg)


# --- per-rlog xcorr extraction (reuse the diagnostic pipeline) -------------

def extract_segment(path: Path):
  """Returns (N, 3) float32 (v_ego, lag, ncc) per filtered window, or None.

  Empty (0, 3) means "log read OK, no usable windows" (skipped, not retried).
  None means "couldn't read" (caller leaves it OUT of processed for later retry).
  """
  rec = _extract_lateral(path)
  if rec is None:
    return None

  v_ego = rec["v_ego"]
  steering = rec["steering_pressed"]
  lat_act = rec["lat_active"]
  desired = rec["desired_curvature"] * v_ego * v_ego
  actual = rec["actual_lat_accel"]
  n = desired.size

  valid = (np.isfinite(desired) & np.isfinite(actual) & np.isfinite(v_ego)
           & (lat_act > 0.5) & (steering < 0.5) & (v_ego >= MIN_VEGO))
  if not valid.any():
    return np.empty((0, 3), dtype=np.float32)

  osc = _oscillation_mask(desired)
  valid_filt = valid & ~osc

  win = int(WINDOW_LEN_S * GRID_HZ)
  hop = int(WINDOW_HOP_S * GRID_HZ)

  rows = []
  i = 0
  while i + win <= n:
    sl = slice(i, i + win)
    if valid_filt[sl].all() and desired[sl].std() >= MIN_SIGNAL_STD:
      v_med = float(np.median(v_ego[sl]))
      if _find_bin(v_med) is not None:
        r = _xcorr_lag(desired[sl], actual[sl])
        if r is not None and r[1] >= MIN_NCC:
          rows.append((v_med, r[0], r[1]))
    i += hop

  if not rows:
    return np.empty((0, 3), dtype=np.float32)
  return np.asarray(rows, dtype=np.float32)


# --- store management (mirrors torque/long autotune) -----------------------

def _load_processed() -> set[str]:
  try:
    return set(json.loads(PROCESSED_PATH.read_text()))
  except Exception:
    return set()


def _save_processed(processed: set[str]) -> None:
  PROCESSED_PATH.write_text(json.dumps(sorted(processed)))


def _append_samples(rows: np.ndarray) -> None:
  with open(SAMPLES_PATH, "ab") as f:
    f.write(rows.tobytes())


def _trim_store() -> None:
  try:
    size = SAMPLES_PATH.stat().st_size
  except FileNotFoundError:
    return
  if size <= MAX_STORE_BYTES:
    return
  keep_rows = MAX_STORE_BYTES // SAMPLE_BYTES
  drop_bytes = size - keep_rows * SAMPLE_BYTES
  tmp = SAMPLES_PATH.with_suffix(".tmp")
  with open(SAMPLES_PATH, "rb") as src, open(tmp, "wb") as dst:
    src.seek(drop_bytes)
    while True:
      chunk = src.read(8 * 1024 * 1024)
      if not chunk:
        break
      dst.write(chunk)
  os.replace(tmp, SAMPLES_PATH)


def load_store() -> np.ndarray:
  if not SAMPLES_PATH.is_file():
    return np.empty((0, SAMPLE_COLS), dtype=np.float32)
  try:
    arr = np.fromfile(SAMPLES_PATH, dtype=np.float32)
  except MemoryError:
    return np.empty((0, SAMPLE_COLS), dtype=np.float32)
  arr = arr[:arr.shape[0] - (arr.shape[0] % SAMPLE_COLS)]
  return arr.reshape(-1, SAMPLE_COLS)


# --- fit -------------------------------------------------------------------

def fit_store() -> dict | None:
  """Returns {'breakpoints': [(v, delay), ...], 'n': total, 'per_bin': [...]}
  or None if too few samples / sanity gates fail.
  """
  data = load_store()
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
    # Sanity gates per bin.
    entry["ok"] = (entry["n"] >= MIN_BIN_N
                   and entry["iqr_w"] <= MAX_IQR_S
                   and entry["ncc_med"] >= MIN_BIN_NCC)
    per_bin.append(entry)

  # Build breakpoint table from bins that passed; refuse if fewer than 2
  # bins are usable (can't interpolate with one point).
  bps = []
  for entry, v_center in zip(per_bin, BIN_CENTERS_MS):
    if entry is None or not entry["ok"]:
      continue
    bps.append((float(v_center), float(np.clip(entry["median"],
                                               MIN_DELAY_S, MAX_DELAY_S))))
  if len(bps) < 2:
    return None

  return {"breakpoints": bps, "n": int(data.shape[0]), "per_bin": per_bin}


# --- main entries ----------------------------------------------------------

def _idle_status() -> str:
  try:
    n_samples = SAMPLES_PATH.stat().st_size // SAMPLE_BYTES
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
  return (f"Idle|{n_samples} samples|"
          f"Delay store: {n_samples} samples.  Tap to ingest new rlogs.{table_str}")


def restore_idle_status() -> None:
  """Refresh status on boot so the panel shows the current store size."""
  if Params("/dev/shm/params").get(STATUS_PARAM):
    return
  _set_status(_idle_status())


def run_collect() -> None:
  STORE_DIR.mkdir(parents=True, exist_ok=True)
  processed = _load_processed()
  rlogs = find_rlogs()
  todo = [p for p in rlogs if str(p) not in processed]
  if not todo:
    _set_status(_idle_status())
    return

  new_samples = 0
  for i, path in enumerate(todo):
    _set_status(f"Collect|Processing {i + 1}/{len(todo)}|"
                f"Ingesting {path.parent.name} (+{new_samples} new windows so far)")
    rows = extract_segment(path)
    if rows is None:
      continue   # unreadable - retry later
    if rows.shape[0]:
      _append_samples(rows)
      new_samples += rows.shape[0]
    processed.add(str(path))
    if i % 20 == 0:
      _save_processed(processed)

  _save_processed(processed)
  _trim_store()
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


def run_fit() -> None:
  fit = fit_store()
  if fit is None:
    _set_status("Refused|Need more data|"
                "Fit refused: too few samples per bin, or per-bin IQR/NCC "
                "gates failed. Collect more drives and retry.")
    return
  params.put(PENDING_PARAM, json.dumps(fit))
  _set_status(_preview_status(fit))


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


def apply_pending() -> None:
  raw = params.get(PENDING_PARAM)
  if not raw:
    _set_status("No previewed delay table to apply. Run Fit first.")
    return
  try:
    fit = json.loads(raw)
    bps = fit["breakpoints"]
    # Re-clip on apply, as a defense in case PENDING was edited externally.
    bps = [(float(v), float(np.clip(d, MIN_DELAY_S, MAX_DELAY_S))) for v, d in bps]
    if len(bps) < 2:
      raise ValueError("need ≥2 breakpoints")
  except Exception:
    params.remove(PENDING_PARAM)
    _set_status("Previewed delay table was invalid. Re-run Fit.")
    return

  params.put(APPLIED_PARAM, json.dumps({"breakpoints": bps}))
  params.remove(PENDING_PARAM)
  short = "→".join(f"{d:.2f}" for _, d in bps)
  _set_status(f"Done|Applied ({short}) — reboot to use|"
              f"Delay table applied. Lagd will interpolate liveDelay.lateralDelay "
              f"by current v_ego using: "
              + ", ".join(f"{v:.1f}m/s→{d:.3f}s" for v, d in bps)
              + ".  Reboot for clean startup.")


def reset_table() -> None:
  """Clear the applied delay table (revert to default lagd behavior)."""
  params.remove(APPLIED_PARAM)
  params.remove(PENDING_PARAM)
  _set_status(_idle_status())
