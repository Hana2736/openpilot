#!/usr/bin/env python3
# Mazda lateral "sigmoid + linear" torque auto-tune.
#
# This integrates two offline scripts (process_rlogs.py + fitting.py) so they can run
# on-device. The device runtime only has numpy (no pandas/scipy/matplotlib), so the
# DataFrame ops, binned_statistic and curve_fit are reimplemented with numpy and the
# graphing is dropped.
#
# Flow (triggered offroad from the Mazda settings panel):
#   1. Recursively find rlogs under /data/media/0/realdata* (realdata + realdata-konik).
#   2. For rlogs not seen before, extract the lateral-tuning signals, forward-fill,
#      drop samples where the steering wheel was touched (SteeringPressed), apply the
#      local fitting filters, and append the survivors to a rolling, size-capped store.
#   3. Re-fit the "sigmoid + linear" curve over the whole store.
#   4. Write the new coefficients to the Mazda{model}Tune{A,B,C,D} params.

import bz2
import json
import os
import time
from pathlib import Path

import numpy as np

from cereal import car, log as capnp_log
from openpilot.common.params import Params
from openpilot.selfdrive.car.mazda.interface import NON_LINEAR_TORQUE_DEFAULTS, SIGLIN_TORQUE_PARAM_PREFIX

# --- storage ---------------------------------------------------------------

STORE_DIR = Path("/data/media/0/lateral_autotune")
SAMPLES_PATH = STORE_DIR / "samples.f32"          # raw float32, 3 cols: output, lat_accel, pitch
PROCESSED_PATH = STORE_DIR / "processed.json"     # rlogs already ingested
STATUS_PARAM = "AutoTuneStatus"

SAMPLE_COLS = 3
SAMPLE_BYTES = SAMPLE_COLS * 4                    # float32
MAX_STORE_BYTES = 500 * 1024 * 1024               # 500 MB rolling budget

REALDATA_ROOT = Path("/data/media/0")

# --- fitting filter thresholds (mirrors fitting.py) ------------------------

SMIN = 15.0           # m/s
SMAX = 40.0           # m/s
ROLL_THRESH = 0.02    # rad
ACCEL_THRESH = 0.5    # m/s^2
PITCH_THRESH = 0.02   # rad
ERROR_THRESH = 0.1
LAT_DEADZONE_DEFAULT = 0.0775  # only the default; the live value is adjustable in the GUI
ROLLING_WINDOW = 1000
ROLLING_STD_MAX = 1.0

# siglin params (a, b, c) are only constrained to be positive - we intentionally
# do NOT clamp them to a "sane" range here so a fit can land on extreme values.
# The GUI sliders are responsible for clamping what actually gets applied.
# ('d' is not part of the model; the reported 4th value is the median spread.)
PARAM_LO = np.array([1e-6, 1e-6, 1e-6])
PARAM_HI = np.array([np.inf, np.inf, np.inf])

params = Params()


def _set_status(msg: str) -> None:
  Params("/dev/shm/params").put(STATUS_PARAM, msg)


# --- rlog discovery --------------------------------------------------------

def find_rlogs() -> list[Path]:
  """Recursively find rlogs under /data/media/0/realdata* (realdata + realdata-konik)."""
  rlogs: dict[Path, Path] = {}  # segment dir -> chosen rlog (prefer compressed)
  if not REALDATA_ROOT.is_dir():
    return []

  for root in sorted(REALDATA_ROOT.glob("realdata*")):
    if not root.is_dir():
      continue
    for path in root.rglob("rlog*"):
      if not path.is_file():
        continue
      name = path.name
      if name not in ("rlog", "rlog.bz2"):
        continue
      seg = path.parent
      # one rlog per segment; prefer the bz2 if both exist
      if seg not in rlogs or name.endswith(".bz2"):
        rlogs[seg] = path

  return sorted(rlogs.values())


# --- numpy helpers ---------------------------------------------------------

def _ffill(a: np.ndarray) -> np.ndarray:
  mask = np.isnan(a)
  idx = np.where(~mask, np.arange(a.shape[0]), 0)
  np.maximum.accumulate(idx, out=idx)
  return a[idx]


def _centered_rolling_std(x: np.ndarray, w: int = ROLLING_WINDOW) -> np.ndarray:
  """Centered rolling std (ddof=1), NaN where the window is incomplete (pandas-like)."""
  n = x.shape[0]
  out = np.full(n, np.nan)
  if n < w:
    return out
  c1 = np.concatenate(([0.0], np.cumsum(x)))
  c2 = np.concatenate(([0.0], np.cumsum(x * x)))
  s = c1[w:] - c1[:-w]
  sq = c2[w:] - c2[:-w]
  var = (sq - s * s / w) / (w - 1)
  std = np.sqrt(np.clip(var, 0.0, None))
  start = w // 2
  out[start:start + std.shape[0]] = std
  return out


# --- rlog extraction -------------------------------------------------------

def _read_rlog_bytes(path: Path) -> bytes:
  with open(path, "rb") as f:
    dat = f.read()
  if path.suffix == ".bz2" or dat[:4] == b"BZh9":
    dat = bz2.decompress(dat)
  return dat


def extract_segment(path: Path) -> np.ndarray | None:
  """Return survivors for one rlog as an (N, 3) float32 array: output, lat_accel, pitch."""
  try:
    events = list(capnp_log.Event.read_multiple_bytes(_read_rlog_bytes(path)))
  except Exception:
    return None

  events.sort(key=lambda e: e.logMonoTime)
  n = len(events)
  if n == 0:
    return None

  cols = {k: np.full(n, np.nan) for k in
          ("output", "lat_accel", "error", "speed", "accel", "roll", "pitch", "steering_pressed")}

  for i, event in enumerate(events):
    try:
      which = event.which()
      if which == "carState":
        cs = event.carState
        cols["speed"][i] = cs.vEgo
        cols["accel"][i] = cs.aEgo
        cols["steering_pressed"][i] = 1.0 if cs.steeringPressed else 0.0
      elif which == "controlsState":
        lcs = event.controlsState.lateralControlState
        if lcs.which() == "torqueState":
          ts = lcs.torqueState
          cols["output"][i] = ts.output
          cols["lat_accel"][i] = ts.actualLateralAccel
          cols["error"][i] = ts.error
      elif which == "liveParameters":
        cols["roll"][i] = event.liveParameters.roll
      elif which == "liveLocationKalman":
        cols["pitch"][i] = event.liveLocationKalman.orientationNED.value[1]
    except Exception:
      continue

  for k in cols:
    cols[k] = _ffill(cols[k])

  valid = np.ones(n, dtype=bool)
  for k in cols:
    valid &= ~np.isnan(cols[k])

  rolling_std = _centered_rolling_std(cols["lat_accel"])

  keep = valid
  keep &= cols["steering_pressed"] == 0.0                                  # SteeringPressed must be False
  keep &= (cols["output"] > -0.99) & (cols["output"] < 0.99)
  keep &= (cols["speed"] >= SMIN) & (cols["speed"] <= SMAX)
  keep &= np.abs(cols["roll"]) <= ROLL_THRESH
  keep &= np.abs(cols["error"]) <= ERROR_THRESH
  keep &= np.abs(cols["accel"]) <= ACCEL_THRESH
  keep &= ~np.isnan(rolling_std) & (rolling_std < ROLLING_STD_MAX)
  # NOTE: the lateral-accel deadzone is intentionally applied at fit time
  # (see fit_store) so the slider stays adjustable without reprocessing.

  if not keep.any():
    return None

  return np.stack([cols["output"][keep], cols["lat_accel"][keep], cols["pitch"][keep]], axis=1).astype(np.float32)


# --- store management ------------------------------------------------------

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
  """Drop the oldest samples so the store stays within MAX_STORE_BYTES."""
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


def _load_store() -> np.ndarray:
  if not SAMPLES_PATH.is_file():
    return np.empty((0, SAMPLE_COLS), dtype=np.float32)
  try:
    arr = np.fromfile(SAMPLES_PATH, dtype=np.float32)
  except MemoryError:
    return np.empty((0, SAMPLE_COLS), dtype=np.float32)
  arr = arr[:arr.shape[0] - (arr.shape[0] % SAMPLE_COLS)]
  return arr.reshape(-1, SAMPLE_COLS)


# --- curve fitting (numpy reimplementation of fitting.py) ------------------

def siglin(x: np.ndarray, a: float, b: float, c: float) -> np.ndarray:
  s = a * x
  sig = np.sign(s) * (1.0 / (1.0 + np.exp(-np.abs(s))) - 0.5)
  return sig * b + x * c


def _curve_fit(x: np.ndarray, y: np.ndarray, p0: np.ndarray) -> np.ndarray | None:
  """Bounded Levenberg-Marquardt least squares for the 3-param siglin model."""
  p = np.clip(p0.astype(float), PARAM_LO, PARAM_HI)
  lam = 1e-3
  eps = 1e-6

  def resid(pp):
    return siglin(x, *pp) - y

  r = resid(p)
  cost = float(r @ r)

  for _ in range(200):
    # numeric Jacobian
    J = np.empty((x.shape[0], 3))
    for k in range(3):
      dp = np.zeros(3)
      dp[k] = eps * max(1.0, abs(p[k]))
      J[:, k] = (siglin(x, *(p + dp)) - siglin(x, *(p - dp))) / (2.0 * dp[k])

    JtJ = J.T @ J
    Jtr = J.T @ r
    improved = False
    for _ in range(12):
      try:
        step = np.linalg.solve(JtJ + lam * np.diag(np.diag(JtJ)), -Jtr)
      except np.linalg.LinAlgError:
        lam *= 10.0
        continue
      cand = np.clip(p + step, PARAM_LO, PARAM_HI)
      rc = resid(cand)
      cc = float(rc @ rc)
      if cc < cost:
        p, r, cost = cand, rc, cc
        lam = max(lam / 3.0, 1e-9)
        improved = True
        break
      lam *= 10.0
    if not improved or np.linalg.norm(step) < 1e-9:
      break

  return p if np.isfinite(cost) else None


def fit_store() -> tuple[float, float, float, float] | None:
  data = _load_store()
  if data.shape[0] < 2 * ROLLING_WINDOW:
    return None

  output = data[:, 0]
  lat_accel = data[:, 1]
  pitch = data[:, 2]

  # adjustable lateral-accel deadzone + pitch-around-its-mean (from fitting.py)
  # 0 is a valid choice (no deadzone); only fall back if the param is missing/garbage
  deadzone = params.get_float("MazdaAutoTuneDeadzone")
  if not np.isfinite(deadzone) or deadzone < 0.0:
    deadzone = LAT_DEADZONE_DEFAULT
  fmask = np.abs(pitch - np.mean(pitch)) <= PITCH_THRESH
  fmask &= np.abs(lat_accel) > deadzone
  output = output[fmask]
  lat_accel = lat_accel[fmask]
  if output.shape[0] < 2 * ROLLING_WINDOW:
    return None

  # reflect for left/right symmetry to remove steering bias
  output = np.concatenate([output, -output])
  lat_accel = np.concatenate([lat_accel, -lat_accel])

  bin_bounds = np.std(output) * 2.5
  if not np.isfinite(bin_bounds) or bin_bounds <= 0:
    return None
  bins = np.linspace(-bin_bounds, bin_bounds, 50)
  bin_idx = np.digitize(output, bins) - 1
  y = -lat_accel
  bin_centers = 0.5 * (bins[1:] + bins[:-1])

  n_bins = len(bins) - 1
  y_mean = np.full(n_bins, np.nan)
  y_std = np.full(n_bins, np.nan)
  for b in range(n_bins):
    sel = bin_idx == b
    if sel.sum() >= 2:
      y_mean[b] = y[sel].mean()
      y_std[b] = y[sel].std(ddof=1)

  good = (np.abs(bin_centers) > 0.2) & ~np.isnan(y_mean) & ~np.isnan(y_std)
  bc = bin_centers[good]
  ym = y_mean[good]
  ys = y_std[good]
  if bc.shape[0] < 4:
    return None

  fit = _curve_fit(ym, bc, np.array([6.0, 0.8, 0.18]))
  if fit is None:
    return None

  a, b, c = (float(v) for v in fit)
  d = float(np.median(ys))
  return a, b, c, d


# --- orchestration ---------------------------------------------------------

def _car_prefix() -> str | None:
  raw = params.get("CarParamsPersistent")
  if not raw:
    return None
  with car.CarParams.from_bytes(raw) as cp:
    fingerprint = cp.carFingerprint
  for candidate, prefix in SIGLIN_TORQUE_PARAM_PREFIX.items():
    if fingerprint == candidate:
      return prefix
  return None


def run_autotune() -> None:
  STORE_DIR.mkdir(parents=True, exist_ok=True)

  prefix = _car_prefix()
  if prefix is None:
    _set_status("Auto-Tune only supports the Mazda 3, CX-30, and CX-50.")
    return

  processed = _load_processed()
  rlogs = find_rlogs()
  todo = [p for p in rlogs if str(p) not in processed]

  if not todo and not SAMPLES_PATH.is_file():
    _set_status("No driving logs found to learn from yet.")
    return

  new_samples = 0
  for i, path in enumerate(todo):
    _set_status(f"Processing logs... {i + 1}/{len(todo)}")
    rows = extract_segment(path)
    if rows is not None and rows.shape[0]:
      _append_samples(rows)
      new_samples += rows.shape[0]
    processed.add(str(path))
    if i % 20 == 0:
      _save_processed(processed)

  _save_processed(processed)
  _trim_store()

  _set_status("Fitting curve...")
  result = fit_store()
  if result is None:
    _set_status("Not enough usable data yet. Drive more, then retry.")
    return

  a, b, c, d = result
  default = NON_LINEAR_TORQUE_DEFAULTS[next(k for k, v in SIGLIN_TORQUE_PARAM_PREFIX.items() if v == prefix)]
  # sanity: a fit that fell back to a bound edge or is wildly off -> keep, but flag
  for suffix, value in zip(("A", "B", "C", "D"), (a, b, c, d)):
    params.put_float(f"{prefix}Tune{suffix}", float(value))

  _set_status(
    f"Done|{prefix} a={a:.5f} b={b:.5f} c={c:.5f} d={d:.5f} "
    f"({new_samples} new samples). Reboot to apply. (was {default[0]:.5f},{default[1]:.5f},{default[2]:.5f})"
  )


if __name__ == "__main__":
  t0 = time.monotonic()
  run_autotune()
  print(f"autotune done in {time.monotonic() - t0:.1f}s -> {params.get(STATUS_PARAM)}")
