#!/usr/bin/env python3
# Mazda open-loop lateral rolling collector + siglin fitter.
#
# Lives alongside torque_autotune.py (which collects closed-loop torqueState
# samples for the math-lock-anchored fit).  This module exists because that
# closed-loop store has a fundamental identifiability problem: the lat
# controller is in the loop, so the recorded (torque, lat_accel) pairs are
# never truly open-loop and `a` (sigmoid curvature) is unidentifiable from
# the data.
#
# Open-loop sampling: we filter rlog windows where lat control is DISENGAGED
# (carControl.latActive=False) and the driver is actively steering
# (|cs.steeringTorque| > MIN_DRIVER_TORQUE).  In that regime, the EPS motor
# torque (cs.steeringTorqueEps) is the only force driving the wheel; the
# resulting lateral acceleration is the plant's pure response.
#
# Schema v1, 5 cols (float32 each):
#   0: driver_torque_raw      (cs.steeringTorque, raw EPS-driver units)
#   1: eps_motor_torque_raw   (cs.steeringTorqueEps, raw EPS-motor units)
#   2: lat_accel              (m/s², from torqueState.actualLateralAccel
#                              OR derived yawRate*vEgo as fallback)
#   3: v_ego                  (m/s)
#   4: pitch                  (rad, liveLocationKalman.orientationNED.value[1])
#
# We store BOTH torque signals because the plant we want to identify is
# (eps_motor_torque -> lat_accel), but the driver_torque is useful both for
# filtering (driver actually steering) and for cross-referencing later when
# we figure out the unit conversion to OP-normalized torque.
#
# Apply path: REFUSES for now and surfaces the fit values in the status.
# Reason: a siglin fit in raw EPS-motor-torque units is not directly
# applicable to OP's normalized torque output [-1,1] without a calibration
# constant.  Future work: a "calibration collector" that records both
# torqueState.output AND steeringTorqueEps during closed-loop driving so
# we can solve for the scaling factor, then this apply path can write
# Mazda{Model}TuneA/B/C/D directly.

import bz2
import json
import os
from pathlib import Path

import numpy as np

from cereal import log as capnp_log
from openpilot.common.params import Params
from openpilot.frogpilot.common.torque_autotune import _ffill, find_rlogs

# --- storage ---------------------------------------------------------------

STORE_DIR = Path("/data/media/0/lat_openloop")
SAMPLES_PATH = STORE_DIR / "samples.f32"
PROCESSED_PATH = STORE_DIR / "processed.json"
SCHEMA_PATH = STORE_DIR / "schema_version"
SCHEMA_VERSION = 1

SAMPLE_COLS = 5
SAMPLE_BYTES = SAMPLE_COLS * 4
MAX_STORE_BYTES = 100 * 1024 * 1024            # 100 MB rolling budget

# --- filters ---------------------------------------------------------------

MIN_DRIVER_TORQUE = 0.3        # raw EPS units; below this the driver is just resting hand
MIN_VEGO = 1.0                 # m/s - exclude standstill (no lat_accel observable)
MAX_VEGO = 18.0                # m/s - open-loop sweep is meant for low/mid speeds
MIN_LAT_ACCEL_MAG = 0.05       # m/s² - below this is near-noise floor

STATUS_PARAM = "LatOpenLoopStatus"
PENDING_PARAM = "LatOpenLoopPending"


params = Params()


def _set_status(msg: str) -> None:
  Params("/dev/shm/params").put(STATUS_PARAM, msg)


# --- store management ------------------------------------------------------

def _migrate_schema() -> None:
  try:
    current = int(SCHEMA_PATH.read_text())
  except Exception:
    current = 0
  if current < SCHEMA_VERSION:
    for p in (SAMPLES_PATH, PROCESSED_PATH):
      try:
        p.unlink()
      except FileNotFoundError:
        pass
  SCHEMA_PATH.write_text(str(SCHEMA_VERSION))


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


# --- rlog extraction -------------------------------------------------------

def _read_rlog_bytes(path: Path) -> bytes:
  with open(path, "rb") as f:
    dat = f.read()
  if path.suffix == ".bz2" or dat[:4] == b"BZh9":
    dat = bz2.decompress(dat)
  return dat


EMPTY_SAMPLES = np.empty((0, SAMPLE_COLS), dtype=np.float32)


def extract_segment(path: Path):
  """Extract per-event open-loop samples from one rlog.  Returns (N, 5) float32.
  Empty (0, 5) is "log read OK, no usable open-loop windows" -> mark processed.
  None is "log unreadable" -> caller leaves it unprocessed for retry.
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
        break       # truncated tail, salvage what we have
  except Exception:
    return None

  events.sort(key=lambda e: e.logMonoTime)
  n = len(events)
  if n == 0:
    return None

  cols = {k: np.full(n, np.nan) for k in
          ("driver_torque", "eps_motor_torque", "lat_accel",
           "v_ego", "pitch", "lat_active")}

  for i, event in enumerate(events):
    try:
      which = event.which()
      if which == "carState":
        cs = event.carState
        cols["v_ego"][i] = cs.vEgo
        cols["driver_torque"][i] = cs.steeringTorque
        cols["eps_motor_torque"][i] = cs.steeringTorqueEps
      elif which == "carControl":
        cols["lat_active"][i] = 1.0 if event.carControl.latActive else 0.0
      elif which == "controlsState":
        lcs = event.controlsState.lateralControlState
        if lcs.which() == "torqueState":
          cols["lat_accel"][i] = lcs.torqueState.actualLateralAccel
      elif which == "liveLocationKalman":
        cols["pitch"][i] = event.liveLocationKalman.orientationNED.value[1]
    except Exception:
      continue

  for k in cols:
    cols[k] = _ffill(cols[k])

  valid = np.ones(n, dtype=bool)
  for k in cols:
    valid &= ~np.isnan(cols[k])

  # Open-loop window definition:
  #   lat OFF, driver actively steering, real lat_accel, moderate speed.
  keep = valid
  keep &= cols["lat_active"] < 0.5                                # OP lat OFF
  keep &= np.abs(cols["driver_torque"]) >= MIN_DRIVER_TORQUE      # driver actually steering
  keep &= np.abs(cols["lat_accel"]) >= MIN_LAT_ACCEL_MAG          # above noise floor
  keep &= (cols["v_ego"] >= MIN_VEGO) & (cols["v_ego"] <= MAX_VEGO)

  if not keep.any():
    return EMPTY_SAMPLES

  return np.stack([cols["driver_torque"][keep],
                   cols["eps_motor_torque"][keep],
                   cols["lat_accel"][keep],
                   cols["v_ego"][keep],
                   cols["pitch"][keep]], axis=1).astype(np.float32)


# --- fit -------------------------------------------------------------------

def siglin(x, a, b, c, d):
  s = a * x
  sig = np.sign(s) * (1.0 / (1.0 + np.exp(-np.abs(s))) - 0.5)
  return sig * b + x * c + d


def _curve_fit_lm(x, y, p0, lo, hi, max_iter=200):
  p = np.clip(p0.astype(float), lo, hi)
  lam = 1e-3
  eps = 1e-6

  def resid(pp):
    return siglin(x, *pp) - y

  r = resid(p)
  cost = float(r @ r)

  for _ in range(max_iter):
    J = np.empty((x.shape[0], 4))
    for k in range(4):
      dp = np.zeros(4)
      dp[k] = eps * max(1.0, abs(p[k]))
      J[:, k] = (siglin(x, *(p + dp)) - siglin(x, *(p - dp))) / (2.0 * dp[k])
    JtJ = J.T @ J
    Jtr = J.T @ r
    improved = False
    step = np.zeros(4)
    for _ in range(12):
      try:
        step = np.linalg.solve(JtJ + lam * np.diag(np.diag(JtJ)), -Jtr)
      except np.linalg.LinAlgError:
        lam *= 10.0
        continue
      cand = np.clip(p + step, lo, hi)
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
  return p, cost


def fit_store() -> dict | None:
  """Fits siglin to (eps_motor_torque, -lat_accel).  Returns the fit + a
  diagnostic dict.  Params are in RAW EPS-motor-torque units, not OP
  normalized [-1,1] units - apply path refuses for now.
  """
  data = load_store()
  if data.shape[0] < 200:
    return None

  eps_t = data[:, 1].astype(np.float64)
  lat_a = data[:, 2].astype(np.float64)
  v_ego = data[:, 3].astype(np.float64)
  pitch = data[:, 4].astype(np.float64)

  # Same direction convention as torque_autotune: x = -lat_accel, y = torque
  x = -lat_a
  y = eps_t

  # Bounds intentionally wide since units are raw, not OP-normalized.
  lo = np.array([1e-6, 1e-6, 1e-6, -10.0])
  hi = np.array([100.0, 10.0, 5.0, 10.0])
  p, cost = _curve_fit_lm(x, y, np.array([6.0, 0.5, 0.1, 0.0]), lo, hi)
  a, b, c, d = (float(v) for v in p)

  return {
    "a": a, "b": b, "c": c, "d": d,
    "slope_at_zero": 0.25 * a * b + c,
    "rmse": float(np.sqrt(cost / x.size)),
    "n": int(x.size),
    "v_ego_range": (float(v_ego.min()), float(v_ego.max())),
    "lat_accel_range": (float(lat_a.min()), float(lat_a.max())),
    "eps_torque_range": (float(eps_t.min()), float(eps_t.max())),
  }


# --- status helpers --------------------------------------------------------

def _idle_status() -> str:
  try:
    n_samples = SAMPLES_PATH.stat().st_size // SAMPLE_BYTES
  except FileNotFoundError:
    n_samples = 0
  return (f"Idle|{n_samples} open-loop samples|"
          f"Open-loop store: {n_samples} samples (lat-off driver-steered, schema v1). "
          f"Drive a parking-lot serpentine with OP lat disengaged to fill the "
          f"transition zone, then tap Collect to ingest the resulting rlogs.")


def restore_idle_status() -> None:
  if Params("/dev/shm/params").get(STATUS_PARAM):
    return
  _set_status(_idle_status())


def _preview_status(fit: dict) -> str:
  return (f"Preview|Fit ready (RAW EPS units — manual scaling needed)|"
          f"a={fit['a']:.3f}  b={fit['b']:.3f}  c={fit['c']:.3f}  d={fit['d']:.3f}  "
          f"slope-at-zero={fit['slope_at_zero']:.3f}  RMSE={fit['rmse']:.4f}  "
          f"n={fit['n']:,}  v_ego={fit['v_ego_range'][0]:.1f}-{fit['v_ego_range'][1]:.1f}m/s  "
          f"|la|={fit['lat_accel_range'][1]:.2f}m/s²  "
          f"eps_torque range=[{fit['eps_torque_range'][0]:.2f},{fit['eps_torque_range'][1]:.2f}].  "
          f"NOTE: a/b/c/d are in raw EPS motor torque units, not OP-normalized [-1,1]. "
          f"Apply is disabled pending a calibration step.")


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

def run_collect() -> None:
  STORE_DIR.mkdir(parents=True, exist_ok=True)
  _migrate_schema()
  processed = _load_processed()
  rlogs = find_rlogs()
  todo = [p for p in rlogs if str(p) not in processed]
  if not todo:
    _set_status(_idle_status())
    return

  new_samples = 0
  for i, path in enumerate(todo):
    _set_status(f"Collect|Processing {i + 1}/{len(todo)}|"
                f"Ingesting {path.parent.name} (+{new_samples} open-loop samples so far)")
    rows = extract_segment(path)
    if rows is None:
      continue   # unreadable -> retry later
    if rows.shape[0]:
      _append_samples(rows)
      new_samples += rows.shape[0]
    processed.add(str(path))
    if i % 20 == 0:
      _save_processed(processed)

  _save_processed(processed)
  _trim_store()
  _set_status(_idle_status())


def run_fit() -> None:
  fit = fit_store()
  if fit is None:
    _set_status("Refused|Need more data|"
                "Open-loop fit refused: <200 samples in store. Drive a parking-lot "
                "serpentine (OP lat disengaged, 5-15 mph, slow steering input) and "
                "re-tap Collect, then Fit.")
    return
  params.put(PENDING_PARAM, json.dumps(fit))
  _set_status(_preview_status(fit))


def apply_pending() -> None:
  raw = params.get(PENDING_PARAM)
  if not raw:
    _set_status("No previewed open-loop fit to apply. Run Fit first.")
    return
  _set_status("Refused|Apply not yet wired|"
              "Apply path intentionally disabled: open-loop fit is in raw EPS "
              "motor torque units, while Mazda{Model}TuneA/B/C/D expect OP-"
              "normalized [-1,1] units. The conversion factor isn't yet "
              "characterized. Use the Preview values for diagnostic purposes; "
              "if you want to try them live, manually edit Mazda{Model}TuneA-D "
              "via SSH after sanity-checking against the math-lock baseline.")


def reset_table() -> None:
  params.remove(PENDING_PARAM)
  _set_status(_idle_status())
