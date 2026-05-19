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
PENDING_PARAM = "AutoTunePending"   # previewed but not-yet-applied fit (JSON)

SAMPLE_COLS = 3
SAMPLE_BYTES = SAMPLE_COLS * 4                    # float32
MAX_STORE_BYTES = 500 * 1024 * 1024               # 500 MB rolling budget

REALDATA_ROOT = Path("/data/media/0")

# --- fitting filter thresholds (mirrors fitting.py) ------------------------

SMIN = 15.0           # m/s
SMAX = 40.0           # m/s
ROLL_THRESH = 0.02    # rad
ACCEL_THRESH = 0.2    # m/s^2
PITCH_THRESH = 0.04   # rad
ERROR_THRESH = 0.1
LAT_DEADZONE_DEFAULT = 0.05  # only the default; the live value is adjustable in the GUI
ROLLING_WINDOW = 500
ROLLING_STD_MAX = 1.0

# siglin model is now (sig*b + la*c + d): a, b, c are gains and are only
# constrained to be positive; d is an additive torque offset and may be
# negative, so it is left fully unbounded. We intentionally do NOT clamp the
# gains to a "sane" range - the GUI sliders clamp what actually gets applied.
PARAM_LO = np.array([1e-6, 1e-6, 1e-6, -np.inf])
PARAM_HI = np.array([np.inf, np.inf, np.inf, np.inf])

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
  x = np.asarray(x, dtype=float)
  n = x.shape[0]
  out = np.full(n, np.nan)
  if n < w:
    return out
  # np.cumsum propagates NaN, so a single leading NaN (events before the first
  # controlsState, which ffill can't fill) would poison the whole result and
  # wipe out every sample. Forward- then back-fill before the cumsum; rows in
  # that NaN region are excluded by the caller's validity mask anyway.
  if np.isnan(x).any():
    x = _ffill(x)
    if x.size and np.isnan(x[0]):
      finite = x[~np.isnan(x)]
      x = np.where(np.isnan(x), finite[0] if finite.size else 0.0, x)
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


EMPTY_SAMPLES = np.empty((0, 3), dtype=np.float32)


def extract_segment(path: Path) -> np.ndarray | None:
  """Survivors for one rlog as an (N, 3) float32 array: output, lat_accel, pitch.

  Returns an empty (0, 3) array if the log read fine but nothing passed the
  filters (e.g. a parked segment) - the caller treats that as "done".
  Returns None only if the log could not be read at all (so it can be retried).
  rlogs are very commonly truncated at the tail (power-down mid-write), so we
  iterate message-by-message and keep everything parsed before the break
  instead of throwing the whole 30+ MB segment away on one bad message.
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
        break  # truncated/corrupt from here on - salvage what we have
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
    return EMPTY_SAMPLES  # read OK, just nothing usable here (e.g. parked) -> don't retry

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

def siglin(x: np.ndarray, a: float, b: float, c: float, d: float) -> np.ndarray:
  s = a * x
  sig = np.sign(s) * (1.0 / (1.0 + np.exp(-np.abs(s))) - 0.5)
  return sig * b + x * c + d


def _curve_fit(x: np.ndarray, y: np.ndarray, p0: np.ndarray) -> np.ndarray | None:
  """Bounded Levenberg-Marquardt least squares for the 4-param siglin model."""
  p = np.clip(p0.astype(float), PARAM_LO, PARAM_HI)
  lam = 1e-3
  eps = 1e-6
  n_params = p.shape[0]

  def resid(pp):
    return siglin(x, *pp) - y

  r = resid(p)
  cost = float(r @ r)

  for _ in range(200):
    # numeric Jacobian
    J = np.empty((x.shape[0], n_params))
    for k in range(n_params):
      dp = np.zeros(n_params)
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

  # NOTE: no left/right reflection - the model's 'd' offset term now captures
  # steering/torque bias instead of it being averaged out.

  bin_bounds = np.std(output) * 2.5
  if not np.isfinite(bin_bounds) or bin_bounds <= 0:
    return None
  bins = np.linspace(-bin_bounds, bin_bounds, 50)
  bin_idx = np.digitize(output, bins) - 1
  y = -lat_accel
  bin_centers = 0.5 * (bins[1:] + bins[:-1])

  n_bins = len(bins) - 1
  y_mean = np.full(n_bins, np.nan)
  for b in range(n_bins):
    sel = bin_idx == b
    if sel.sum() >= 2:
      y_mean[b] = y[sel].mean()

  good = (np.abs(bin_centers) > 0.2) & ~np.isnan(y_mean)
  bc = bin_centers[good]
  ym = y_mean[good]
  if bc.shape[0] < 4:
    return None

  fit = _curve_fit(ym, bc, np.array([6.0, 0.8, 0.18, 0.0]))
  if fit is None:
    return None

  a, b, c, d = (float(v) for v in fit)
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
    if rows is None:
      # couldn't read the log at all - leave it OUT of processed so a future
      # run can retry it instead of permanently burning the drive.
      continue
    if rows.shape[0]:
      _append_samples(rows)
      new_samples += rows.shape[0]
    processed.add(str(path))            # read OK (even if 0 survivors) -> done
    if i % 20 == 0:
      _save_processed(processed)

  _save_processed(processed)
  _trim_store()

  _set_status("Fitting curve...")
  result = fit_store()
  if result is None:
    _set_status("Not enough usable data yet. Drive more, then retry.")
    return

  a, b, c, d = (float(v) for v in result)
  # Preview only: stash the candidate, do NOT touch the live params. The user
  # reviews the numbers and taps "Apply Auto-Tune" to commit (apply_pending).
  params.put(PENDING_PARAM, json.dumps({"prefix": prefix, "a": a, "b": b, "c": c, "d": d,
                                        "n": int(new_samples)}))
  # status protocol: STATE|<short button label>|<full wrapping detail>
  _set_status(f"Preview|Preview ready — tap Apply|{prefix}  "
              f"a={a:.5f}  b={b:.5f}  c={c:.5f}  d={d:.5f}  ·  {new_samples} samples")


def apply_pending() -> None:
  raw = params.get(PENDING_PARAM)
  if not raw:
    _set_status("No previewed tune to apply. Run Auto-Tune first.")
    return
  try:
    p = json.loads(raw)
    prefix = p["prefix"]
    vals = (p["a"], p["b"], p["c"], p["d"])
  except Exception:
    params.remove(PENDING_PARAM)
    _set_status("Previewed tune was invalid. Re-run Auto-Tune.")
    return

  for suffix, value in zip(("A", "B", "C", "D"), vals):
    params.put_float(f"{prefix}Tune{suffix}", float(value))
  params.remove(PENDING_PARAM)
  _set_status(f"Done|Applied — reboot to use|{prefix} applied  "
              f"a={vals[0]:.5f}  b={vals[1]:.5f}  c={vals[2]:.5f}  d={vals[3]:.5f}")


if __name__ == "__main__":
  t0 = time.monotonic()
  run_autotune()
  print(f"autotune done in {time.monotonic() - t0:.1f}s -> {params.get(STATUS_PARAM)}")
