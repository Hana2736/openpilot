#!/usr/bin/env python3
# Mazda open-loop lateral rolling collector + siglin fitter + auto-K apply.
#
# Closed-loop torque_autotune cannot identify siglin's `a` because the
# (torque, lat_accel) pairs are sampled under OP's lat controller (whatever
# tune we ship biases the data toward itself).  This module fixes that:
#
#   1. Open-loop store: rlog windows where lat is DISENGAGED and the
#      driver is actively steering.  In that regime the only force on
#      the wheel is the EPS motor (assist + driver input through it),
#      so (eps_motor_torque -> lat_accel) is the plant's unbiased
#      response.  Fit siglin on this -> a/b/c/d in raw EPS-motor units.
#
#   2. Calibration store: from the SAME rlogs' lat-ON windows, paired
#      (torqueState.output, cs.steeringTorqueEps).  Robust linear
#      regression gives K = EPS-units per OP-normalized unit.  This is
#      the unknown that lets us scale the open-loop fit into the
#      [-1,+1] OP normalization that Mazda{Model}TuneA/B/C/D expect.
#      siglin under scaling: a stays the same, b/c/d divide by K.
#
#   3. Apply: convert open-loop a/b/c/d via K, run the same coverage
#      gate as torque_autotune (siglin must reach ±1 over lat_accel ∈
#      [-8, 8]), clip to PARAM_LO/HI, write Mazda{Model}TuneA-D.
#
# Both stores accumulate via the same Collect tap (one rlog pass, two
# files written).  Either passively from normal driving (any time you
# disengage OP and steer) or actively from a parking-lot serpentine
# session - whichever produces the data faster.
#
# Schemas:
#   samples.f32      v1, 5 cols: driver_torque, eps_motor_torque,
#                                 lat_accel, v_ego, pitch
#   cal_samples.f32  v1, 2 cols: torque_output_normalized, eps_motor_torque

import bz2
import json
import os
from pathlib import Path

import numpy as np

from cereal import car, log as capnp_log
from openpilot.common.params import Params
from openpilot.frogpilot.common.torque_autotune import _ffill, find_rlogs
from openpilot.selfdrive.car.mazda.interface import (
  NON_LINEAR_TORQUE_DEFAULTS, SIGLIN_TORQUE_PARAM_PREFIX,
)

# --- storage ---------------------------------------------------------------

STORE_DIR = Path("/data/media/0/lat_openloop")
SAMPLES_PATH = STORE_DIR / "samples.f32"
CAL_SAMPLES_PATH = STORE_DIR / "cal_samples.f32"
PROCESSED_PATH = STORE_DIR / "processed.json"
SCHEMA_PATH = STORE_DIR / "schema_version"
SCHEMA_VERSION = 1

SAMPLE_COLS = 5
SAMPLE_BYTES = SAMPLE_COLS * 4
CAL_SAMPLE_COLS = 2
CAL_SAMPLE_BYTES = CAL_SAMPLE_COLS * 4
MAX_STORE_BYTES = 100 * 1024 * 1024

# --- open-loop filters -----------------------------------------------------

MIN_DRIVER_TORQUE = 0.3        # raw EPS units; below = hand resting
MIN_VEGO = 1.0
MAX_VEGO = 18.0
MIN_LAT_ACCEL_MAG = 0.05

# --- calibration filters ---------------------------------------------------
# Want clean lat-ON samples where OP is actually pushing the wheel and the
# EPS torque reading isn't dominated by sensor noise around zero.
CAL_MIN_OP_OUTPUT_MAG = 0.05   # OP commanding meaningfully
CAL_MIN_EPS_TORQUE_MAG = 5.0   # EPS motor responding meaningfully
CAL_MIN_VEGO = 3.0
CAL_MIN_PAIRS = 200            # min samples to compute K
CAL_K_MIN = 20.0               # sanity bounds on K
CAL_K_MAX = 2000.0

# --- siglin parameter bounds (mirror torque_autotune for safety) -----------
PARAM_LO = np.array([1e-6, 1e-6, 1e-6, -1.0])
PARAM_HI = np.array([30.0, 3.0, 3.0, 1.0])

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
    for p in (SAMPLES_PATH, CAL_SAMPLES_PATH, PROCESSED_PATH):
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


def _append_rows(path: Path, rows: np.ndarray) -> None:
  with open(path, "ab") as f:
    f.write(rows.tobytes())


def _trim_one(path: Path, sample_bytes: int) -> None:
  try:
    size = path.stat().st_size
  except FileNotFoundError:
    return
  if size <= MAX_STORE_BYTES:
    return
  keep_rows = MAX_STORE_BYTES // sample_bytes
  drop_bytes = size - keep_rows * sample_bytes
  tmp = path.with_suffix(".tmp")
  with open(path, "rb") as src, open(tmp, "wb") as dst:
    src.seek(drop_bytes)
    while True:
      chunk = src.read(8 * 1024 * 1024)
      if not chunk:
        break
      dst.write(chunk)
  os.replace(tmp, path)


def _trim_store() -> None:
  _trim_one(SAMPLES_PATH, SAMPLE_BYTES)
  _trim_one(CAL_SAMPLES_PATH, CAL_SAMPLE_BYTES)


def _load_rows(path: Path, cols: int) -> np.ndarray:
  if not path.is_file():
    return np.empty((0, cols), dtype=np.float32)
  try:
    arr = np.fromfile(path, dtype=np.float32)
  except MemoryError:
    return np.empty((0, cols), dtype=np.float32)
  arr = arr[:arr.shape[0] - (arr.shape[0] % cols)]
  return arr.reshape(-1, cols)


def load_store() -> np.ndarray:
  return _load_rows(SAMPLES_PATH, SAMPLE_COLS)


def load_cal_store() -> np.ndarray:
  return _load_rows(CAL_SAMPLES_PATH, CAL_SAMPLE_COLS)


# --- rlog extraction -------------------------------------------------------

def _read_rlog_bytes(path: Path) -> bytes:
  with open(path, "rb") as f:
    dat = f.read()
  if path.suffix == ".bz2" or dat[:4] == b"BZh9":
    dat = bz2.decompress(dat)
  return dat


def extract_segment(path: Path):
  """Returns (openloop_rows, cal_rows) tuple of float32 arrays, or None.
  openloop_rows: (N, 5) for lat-OFF driver-steered windows.
  cal_rows: (M, 2) for lat-ON OP-commanding windows.
  Empty array on either side is fine ("nothing usable in that regime"
  for this rlog); both empty still marks the rlog processed.
  None means the log couldn't be read - caller leaves it OUT of processed
  for a retry.
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
        break
  except Exception:
    return None

  events.sort(key=lambda e: e.logMonoTime)
  n = len(events)
  if n == 0:
    return None

  cols = {k: np.full(n, np.nan) for k in
          ("driver_torque", "eps_motor_torque", "lat_accel",
           "v_ego", "pitch", "lat_active", "op_output")}

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
          ts = lcs.torqueState
          cols["lat_accel"][i] = ts.actualLateralAccel
          cols["op_output"][i] = ts.output
      elif which == "liveLocationKalman":
        cols["pitch"][i] = event.liveLocationKalman.orientationNED.value[1]
    except Exception:
      continue

  for k in cols:
    cols[k] = _ffill(cols[k])

  base_valid = np.ones(n, dtype=bool)
  for k in ("driver_torque", "eps_motor_torque", "lat_accel", "v_ego",
            "pitch", "lat_active", "op_output"):
    base_valid &= ~np.isnan(cols[k])

  # Open-loop: lat OFF, driver actively steering, real lat_accel, low-mid speed.
  ol_keep = base_valid.copy()
  ol_keep &= cols["lat_active"] < 0.5
  ol_keep &= np.abs(cols["driver_torque"]) >= MIN_DRIVER_TORQUE
  ol_keep &= np.abs(cols["lat_accel"]) >= MIN_LAT_ACCEL_MAG
  ol_keep &= (cols["v_ego"] >= MIN_VEGO) & (cols["v_ego"] <= MAX_VEGO)

  # Calibration: lat ON, OP commanding meaningfully, EPS responding meaningfully.
  cal_keep = base_valid.copy()
  cal_keep &= cols["lat_active"] >= 0.5
  cal_keep &= np.abs(cols["op_output"]) >= CAL_MIN_OP_OUTPUT_MAG
  cal_keep &= np.abs(cols["eps_motor_torque"]) >= CAL_MIN_EPS_TORQUE_MAG
  cal_keep &= cols["v_ego"] >= CAL_MIN_VEGO

  if ol_keep.any():
    ol_rows = np.stack([cols["driver_torque"][ol_keep],
                        cols["eps_motor_torque"][ol_keep],
                        cols["lat_accel"][ol_keep],
                        cols["v_ego"][ol_keep],
                        cols["pitch"][ol_keep]], axis=1).astype(np.float32)
  else:
    ol_rows = np.empty((0, SAMPLE_COLS), dtype=np.float32)

  if cal_keep.any():
    cal_rows = np.stack([cols["op_output"][cal_keep],
                         cols["eps_motor_torque"][cal_keep]], axis=1).astype(np.float32)
  else:
    cal_rows = np.empty((0, CAL_SAMPLE_COLS), dtype=np.float32)

  return ol_rows, cal_rows


# --- K calibration ---------------------------------------------------------

def compute_K() -> dict | None:
  """Robust slope of eps_motor_torque ≈ K * torqueState.output.

  Theil-Sen style: take per-sample slope eps/output (signed), filter to
  same-sign pairs (we don't want the slope to flip sign from sensor
  noise), use the median as the K estimate.  Returns None if insufficient
  or out-of-sanity-range data.
  """
  cal = load_cal_store()
  if cal.shape[0] < CAL_MIN_PAIRS:
    return None
  op_out = cal[:, 0].astype(np.float64)
  eps_t = cal[:, 1].astype(np.float64)
  # Same-sign only.
  same_sign = (op_out * eps_t) > 0
  if same_sign.sum() < CAL_MIN_PAIRS:
    return None
  slopes = eps_t[same_sign] / op_out[same_sign]
  K = float(np.median(slopes))
  K_mad = float(np.median(np.abs(slopes - K)))
  if not (CAL_K_MIN <= K <= CAL_K_MAX):
    return {"K": K, "K_mad": K_mad, "n": int(same_sign.sum()),
            "ok": False, "reason": f"K={K:.1f} outside sanity bounds [{CAL_K_MIN},{CAL_K_MAX}]"}
  return {"K": K, "K_mad": K_mad, "n": int(same_sign.sum()), "ok": True}


# --- siglin curve fitting --------------------------------------------------

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
  """Fits siglin on (eps_motor_torque, -lat_accel) in raw EPS units,
  then computes K from calibration store and converts to OP-normalized
  units.  Returns combined dict, or None if either side fails.
  """
  data = load_store()
  if data.shape[0] < 200:
    return None

  eps_t = data[:, 1].astype(np.float64)
  lat_a = data[:, 2].astype(np.float64)
  v_ego = data[:, 3].astype(np.float64)

  x = -lat_a
  y = eps_t

  # Bounds wide because we're fitting in raw EPS torque units (typically
  # tens to low hundreds in EPS units).  Converted to OP-norm below.
  lo_eps = np.array([1e-6, 1e-6, 1e-6, -200.0])
  hi_eps = np.array([100.0, 500.0, 100.0, 200.0])
  p, cost = _curve_fit_lm(x, y, np.array([6.0, 50.0, 5.0, 0.0]), lo_eps, hi_eps)
  a_eps, b_eps, c_eps, d_eps = (float(v) for v in p)

  cal = compute_K()
  result = {
    "a_eps": a_eps, "b_eps": b_eps, "c_eps": c_eps, "d_eps": d_eps,
    "slope_at_zero_eps": 0.25 * a_eps * b_eps + c_eps,
    "rmse_eps": float(np.sqrt(cost / x.size)),
    "n": int(x.size),
    "v_ego_range": (float(v_ego.min()), float(v_ego.max())),
    "lat_accel_range": (float(lat_a.min()), float(lat_a.max())),
    "eps_torque_range": (float(eps_t.min()), float(eps_t.max())),
    "calibration": cal,
  }
  if cal is not None and cal.get("ok"):
    K = cal["K"]
    # siglin under output scaling y' = y/K:
    #   a stays the same (operates on x), b/c/d divide by K.
    result.update({
      "a": a_eps,
      "b": b_eps / K,
      "c": c_eps / K,
      "d": d_eps / K,
      "slope_at_zero": (0.25 * a_eps * b_eps + c_eps) / K,
    })
  return result


# --- car-prefix detection (mirror torque_autotune) -------------------------

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


# --- status helpers --------------------------------------------------------

def _idle_status() -> str:
  try:
    n = SAMPLES_PATH.stat().st_size // SAMPLE_BYTES
  except FileNotFoundError:
    n = 0
  try:
    nc = CAL_SAMPLES_PATH.stat().st_size // CAL_SAMPLE_BYTES
  except FileNotFoundError:
    nc = 0
  return (f"Idle|{n} open-loop + {nc} cal|"
          f"Open-loop store: {n} samples (lat-off driver-steered). "
          f"Calibration store: {nc} paired (op_output, eps_torque) samples "
          f"(lat-on, for converting raw EPS units to OP-normalized).  Drive "
          f"with OP lat disengaged at low-mid speed for open-loop data; the "
          f"calibration data auto-accumulates from any lat-on driving.")


def restore_idle_status() -> None:
  if Params("/dev/shm/params").get(STATUS_PARAM):
    return
  _set_status(_idle_status())


def _preview_status(fit: dict) -> str:
  cal = fit.get("calibration")
  if cal is None:
    cal_str = "no calibration data yet (need ≥200 lat-ON same-sign pairs)"
    op_str = "apply DISABLED until calibration accrues"
  elif not cal.get("ok"):
    cal_str = f"K={cal['K']:.1f} (REJECTED: {cal.get('reason','out of sanity bounds')})"
    op_str = "apply DISABLED"
  else:
    cal_str = f"K={cal['K']:.1f}±{cal['K_mad']:.1f} (median, n={cal['n']:,})"
    op_str = (f"OP-normalized: a={fit['a']:.5f}  b={fit['b']:.5f}  "
              f"c={fit['c']:.5f}  d={fit['d']:.5f}  slope-at-0={fit['slope_at_zero']:.3f}")

  return (f"Preview|Fit ready ({op_str.split(': ',1)[-1] if 'OP' in op_str else 'cal pending'})|"
          f"RAW EPS:  a={fit['a_eps']:.3f}  b={fit['b_eps']:.3f}  c={fit['c_eps']:.3f}  "
          f"d={fit['d_eps']:.3f}  slope-at-0={fit['slope_at_zero_eps']:.3f}  "
          f"RMSE={fit['rmse_eps']:.4f}  n={fit['n']:,}  "
          f"v_ego={fit['v_ego_range'][0]:.1f}-{fit['v_ego_range'][1]:.1f}m/s  "
          f"|la|≤{abs(fit['lat_accel_range'][1]):.2f}m/s²  ·  "
          f"calibration: {cal_str}  ·  {op_str}")


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

  new_ol, new_cal = 0, 0
  for i, path in enumerate(todo):
    _set_status(f"Collect|Processing {i + 1}/{len(todo)}|"
                f"Ingesting {path.parent.name} (+{new_ol} open-loop, "
                f"+{new_cal} calibration so far)")
    res = extract_segment(path)
    if res is None:
      continue
    ol_rows, cal_rows = res
    if ol_rows.shape[0]:
      _append_rows(SAMPLES_PATH, ol_rows)
      new_ol += ol_rows.shape[0]
    if cal_rows.shape[0]:
      _append_rows(CAL_SAMPLES_PATH, cal_rows)
      new_cal += cal_rows.shape[0]
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
                "Open-loop fit refused: <200 lat-OFF samples in store. Drive "
                "with OP lat disengaged at 1-18 m/s and re-tap Collect, then Fit.")
    return
  params.put(PENDING_PARAM, json.dumps(fit))
  _set_status(_preview_status(fit))


def _covers(a, b, c, d) -> bool:
  return (0.5 * b + 8.0 * c + d > 1.02) and (-0.5 * b - 8.0 * c + d < -1.02)


def apply_pending() -> None:
  raw = params.get(PENDING_PARAM)
  if not raw:
    _set_status("No previewed open-loop fit to apply. Run Fit first.")
    return
  try:
    fit = json.loads(raw)
  except Exception:
    params.remove(PENDING_PARAM)
    _set_status("Previewed open-loop fit was invalid. Re-run Fit.")
    return

  cal = fit.get("calibration")
  if cal is None or not cal.get("ok"):
    _set_status("Refused|Calibration not ready|"
                "Apply refused: K not computed or out of sanity range. "
                "Drive more (with OP lat ON) to accrue calibration pairs, "
                "then re-run Fit to refresh the pending preview.")
    return

  prefix = _car_prefix()
  if prefix is None:
    _set_status("Refused|Unsupported fingerprint|"
                "Open-loop apply only writes Mazda3/CX-30/CX-50 tune params.")
    return

  a = float(fit["a"]); b = float(fit["b"]); c = float(fit["c"]); d = float(fit["d"])

  # Same ±1 coverage gate torque_autotune uses.  Sparse hard-cornering
  # data can collapse c; if so, graft the baseline c (same fallback as
  # torque_autotune.apply_pending) and re-test.
  if not _covers(a, b, c, d):
    c_baseline = float(NON_LINEAR_TORQUE_DEFAULTS[
        next(k for k, v in SIGLIN_TORQUE_PARAM_PREFIX.items() if v == prefix)][2])
    if _covers(a, b, c_baseline, d):
      c = c_baseline
    else:
      _set_status(
        f"Refused|Fit can't reach ±1 torque|"
        f"max={0.5 * b + 8.0 * c + d:.3f} min={-0.5 * b - 8.0 * c + d:.3f} from "
        f"a={a:.3f} b={b:.3f} c={c:.4f} d={d:.3f}; baseline-c fallback "
        f"({c_baseline:.3f}) also insufficient.  Open-loop b is too small for "
        f"current K={cal['K']:.1f}; more open-loop data at higher |lat_accel| "
        f"may help."
      )
      return

  vals = tuple(float(np.clip(v, lo, hi)) for v, lo, hi in zip((a, b, c, d), PARAM_LO, PARAM_HI))
  for suffix, value in zip(("A", "B", "C", "D"), vals):
    params.put_float(f"{prefix}Tune{suffix}", value)
  params.remove(PENDING_PARAM)
  _set_status(f"Done|Applied (K={cal['K']:.1f}) — reboot to use|"
              f"{prefix} written from open-loop fit  "
              f"a={vals[0]:.5f}  b={vals[1]:.5f}  c={vals[2]:.5f}  d={vals[3]:.5f}  "
              f"(converted from raw EPS via K={cal['K']:.1f}±{cal['K_mad']:.1f}, "
              f"n_cal={cal['n']:,}).")


def reset_table() -> None:
  params.remove(PENDING_PARAM)
  _set_status(_idle_status())
