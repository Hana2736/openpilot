#!/usr/bin/env python3
# Mazda open-loop lateral collector + per-band a-extraction + closed-loop
# cross-fit for b/c/d.
#
# Background: closed-loop torque_autotune can't identify siglin's `a` because
# OP's lat controller biases the (torque, lat_accel) sampling.  We tried
# extracting motor torque from the EPS for an unbiased fit but Mazda Gen2
# doesn't populate cs.steeringTorqueEps - it's always 0.  This module uses
# a different route:
#
#   * `a` is unit-independent on the torque axis.  Whether torque is in raw
#     EPS units, driver-torque units, or OP-normalized [-1,+1] units, the
#     sigmoid curvature parameter `a` (which operates on lat_accel) is the
#     same.  Only b/c/d rescale with units.
#
#   * So: fit `a` from open-loop (driver_torque, lat_accel) - it doesn't
#     matter that driver-torque has unknown EPS-assist scaling, the
#     CURVATURE is preserved.  Per-speed-band fit lets us see if `a`
#     varies with speed (EPS assist is speed-dependent); a stable `a`
#     across bands means the plant identification is reliable.
#
#   * Then with `a` fixed, cross-fit b/c/d on the existing closed-loop
#     torque_autotune store.  With `a` known, the siglin model is LINEAR
#     in (b, c, d) - closed-form lstsq, no convergence issues, no bias
#     from `a` being unidentifiable.  Result is in OP-normalized [-1,+1]
#     units, ready to write to Mazda{Model}TuneA-D.
#
# Schemas:
#   samples.f32   v1, 5 cols: driver_torque, eps_motor_torque (unused on
#                              Gen2 — always 0), lat_accel, v_ego, pitch
#
# Apply path runs the same ±1 coverage gate as torque_autotune and grafts
# the baseline c if needed.

import bz2
import json
import os
from pathlib import Path

import numpy as np

from cereal import car, log as capnp_log
from openpilot.common.params import Params
from openpilot.frogpilot.common.torque_autotune import (
  LAT_DEADZONE_DEFAULT, PARAM_HI, PARAM_LO, PITCH_THRESH, _ffill, find_rlogs,
)
from openpilot.selfdrive.car.mazda.interface import (
  NON_LINEAR_TORQUE_DEFAULTS, SIGLIN_TORQUE_PARAM_PREFIX,
)

# --- storage ---------------------------------------------------------------

STORE_DIR = Path("/data/media/0/lat_openloop")
SAMPLES_PATH = STORE_DIR / "samples.f32"
PROCESSED_PATH = STORE_DIR / "processed.json"
SCHEMA_PATH = STORE_DIR / "schema_version"
SCHEMA_VERSION = 3  # v2 used wrong cereal field name (.acceleration); v3 fixes to .accelerationCalibrated

SAMPLE_COLS = 5
SAMPLE_BYTES = SAMPLE_COLS * 4
MAX_STORE_BYTES = 100 * 1024 * 1024

# Closed-loop torque_autotune store - read directly for cross-fit.
CLOSED_LOOP_SAMPLES_PATH = Path("/data/media/0/lateral_autotune/samples.f32")
CLOSED_LOOP_COLS = 3                  # output, lat_accel, pitch

# --- open-loop filters -----------------------------------------------------

MIN_DRIVER_TORQUE = 0.3
MIN_VEGO = 1.0
MAX_VEGO = 18.0
MIN_LAT_ACCEL_MAG = 0.05

# --- per-speed-band fit ----------------------------------------------------
# EPS assist is speed-dependent; fitting `a` separately per band shows if
# the plant's sigmoid curvature is consistent across speeds (good) or
# varies wildly (the single-value model breaks down).
OL_BINS_MS = (1.0, 4.0, 8.0, 14.0, 18.0)
OL_BIN_CENTERS = tuple(0.5 * (lo + hi) for lo, hi in zip(OL_BINS_MS[:-1], OL_BINS_MS[1:]))
MIN_BAND_SAMPLES = 100               # per-band minimum for siglin fit
A_VARIATION_REL = 0.5                # warn if max(a)/min(a) > 1+this across bands

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


def _append_rows(path: Path, rows: np.ndarray) -> None:
  with open(path, "ab") as f:
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


def _load_closed_loop() -> np.ndarray:
  if not CLOSED_LOOP_SAMPLES_PATH.is_file():
    return np.empty((0, CLOSED_LOOP_COLS), dtype=np.float32)
  try:
    arr = np.fromfile(CLOSED_LOOP_SAMPLES_PATH, dtype=np.float32)
  except MemoryError:
    return np.empty((0, CLOSED_LOOP_COLS), dtype=np.float32)
  arr = arr[:arr.shape[0] - (arr.shape[0] % CLOSED_LOOP_COLS)]
  return arr.reshape(-1, CLOSED_LOOP_COLS)


# --- rlog extraction (open-loop windows only) ------------------------------

def _read_rlog_bytes(path: Path) -> bytes:
  with open(path, "rb") as f:
    dat = f.read()
  if path.suffix == ".bz2" or dat[:4] == b"BZh9":
    dat = bz2.decompress(dat)
  return dat


def extract_segment(path: Path):
  """Open-loop windows from one rlog as (N, 5) float32, or None if unreadable."""
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
      elif which == "liveLocationKalman":
        llk = event.liveLocationKalman
        cols["pitch"][i] = llk.orientationNED.value[1]
        # Lateral accel from kalman-calibrated body-frame acceleration
        # (y-axis = lateral).  torqueState.actualLateralAccel doesn't
        # publish when lat control is OFF, so this is the only source
        # that works for open-loop windows.  The kalman filter publishes
        # this every cycle regardless of OP engagement.
        cols["lat_accel"][i] = llk.accelerationCalibrated.value[1]
    except Exception:
      continue

  for k in cols:
    cols[k] = _ffill(cols[k])

  valid = np.ones(n, dtype=bool)
  for k in ("driver_torque", "lat_accel", "v_ego", "pitch", "lat_active"):
    valid &= ~np.isnan(cols[k])
  # eps_motor_torque is always-zero on Gen2 (Mazda doesn't populate it);
  # don't require it to be non-NaN, just record what's there.

  keep = valid
  keep &= cols["lat_active"] < 0.5
  keep &= np.abs(cols["driver_torque"]) >= MIN_DRIVER_TORQUE
  keep &= np.abs(cols["lat_accel"]) >= MIN_LAT_ACCEL_MAG
  keep &= (cols["v_ego"] >= MIN_VEGO) & (cols["v_ego"] <= MAX_VEGO)

  if not keep.any():
    return np.empty((0, SAMPLE_COLS), dtype=np.float32)

  return np.stack([cols["driver_torque"][keep],
                   cols["eps_motor_torque"][keep],
                   cols["lat_accel"][keep],
                   cols["v_ego"][keep],
                   cols["pitch"][keep]], axis=1).astype(np.float32)


# --- siglin + nonlinear fit (for per-band `a` extraction) ------------------

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


def _fit_a_per_band(driver_torque, lat_accel, v_ego):
  """Per-speed-band siglin fit on (driver_torque, -lat_accel).  Returns list
  of per-band dicts with {lo, hi, n, a, rmse, ok}.  `a` is unit-independent
  on the torque axis, so the value here is directly reusable for the OP-
  normalized cross-fit even though b/c/d here are in driver-torque units.
  """
  # Wide bounds since driver-torque has unknown EPS-assist scaling - b/c/d
  # may end up anywhere; only `a` matters.
  lo = np.array([1e-6, 1e-6, 1e-6, -1e4])
  hi = np.array([30.0, 1e4, 1e4, 1e4])

  bands = []
  for i in range(len(OL_BINS_MS) - 1):
    v_lo, v_hi = OL_BINS_MS[i], OL_BINS_MS[i + 1]
    m = (v_ego >= v_lo) & (v_ego < v_hi)
    entry = {"lo": float(v_lo), "hi": float(v_hi), "n": int(m.sum())}
    if entry["n"] < MIN_BAND_SAMPLES:
      entry["ok"] = False
      entry["reason"] = f"n={entry['n']}<{MIN_BAND_SAMPLES}"
      bands.append(entry)
      continue
    x = -lat_accel[m].astype(np.float64)
    y = driver_torque[m].astype(np.float64)
    # Seed b/c from data scale so LM doesn't waste iterations climbing.
    b0 = float(np.percentile(np.abs(y), 95))
    p0 = np.array([10.0, b0, b0 * 0.1, 0.0])
    p, cost = _curve_fit_lm(x, y, p0, lo, hi)
    rmse = float(np.sqrt(cost / x.size))
    a_fit = float(p[0])
    entry.update({"a": a_fit, "b_drv": float(p[1]), "c_drv": float(p[2]),
                  "d_drv": float(p[3]), "rmse_drv": rmse,
                  "ok": True})
    bands.append(entry)
  return bands


# --- closed-loop cross-fit for b/c/d (a fixed) -----------------------------

def _cross_fit_bcd(a_fixed):
  """With `a` fixed, the siglin model is linear in (b, c, d).  lstsq on the
  closed-loop torque_autotune store gives b/c/d in OP-normalized units.
  Returns dict {b, c, d, n, rmse} or None if insufficient closed-loop data.
  """
  data = _load_closed_loop()
  if data.shape[0] < 1000:
    return None
  output = data[:, 0].astype(np.float64)
  lat_accel = data[:, 1].astype(np.float64)
  pitch = data[:, 2].astype(np.float64)

  # Same filters torque_autotune uses at fit time.
  fmask = np.abs(pitch - np.mean(pitch)) <= PITCH_THRESH
  fmask &= np.abs(lat_accel) > LAT_DEADZONE_DEFAULT
  output = output[fmask]
  lat_accel = lat_accel[fmask]
  if output.shape[0] < 1000:
    return None

  x = -lat_accel
  y = output

  # S_a(x) is the sigmoid basis with fixed a; the model is y = b*S + c*x + d.
  s = a_fixed * x
  S = np.sign(s) * (1.0 / (1.0 + np.exp(-np.abs(s))) - 0.5)
  A = np.column_stack([S, x, np.ones_like(x)])
  coef, residuals, rank, _ = np.linalg.lstsq(A, y, rcond=None)
  pred = A @ coef
  rmse = float(np.sqrt(((y - pred) ** 2).mean()))
  return {"b": float(coef[0]), "c": float(coef[1]), "d": float(coef[2]),
          "n": int(x.size), "rmse": rmse}


# --- combined fit ----------------------------------------------------------

def fit_store() -> dict | None:
  """Per-band `a` from open-loop driver-torque + closed-loop lstsq for
  b/c/d in OP-normalized units.  Returns combined dict or None.
  """
  ol = load_store()
  if ol.shape[0] < MIN_BAND_SAMPLES:
    return None
  driver_torque = ol[:, 0]
  lat_accel = ol[:, 2]
  v_ego = ol[:, 3]

  bands = _fit_a_per_band(driver_torque, lat_accel, v_ego)
  passing = [b for b in bands if b.get("ok")]
  if not passing:
    return None

  # Representative `a`: sample-count-weighted median across passing bands.
  # Median is more robust than mean against one weird-low-data band; weighting
  # by n biases toward the band we have the most evidence for.
  weighted_a = []
  for b in passing:
    weighted_a.extend([b["a"]] * b["n"])
  a_chosen = float(np.median(weighted_a))
  a_per_band = [b["a"] for b in passing]
  a_min, a_max = min(a_per_band), max(a_per_band)
  a_variation = (a_max - a_min) / max(a_min, 1e-6)

  cross = _cross_fit_bcd(a_chosen)
  if cross is None:
    return {"a_chosen": a_chosen, "per_band": bands, "cross": None,
            "a_variation": a_variation,
            "warn": "closed-loop store too small for cross-fit"}

  return {
    "a_chosen": a_chosen,
    "a_per_band": a_per_band,
    "a_variation": a_variation,
    "per_band": bands,
    "cross": cross,
    "a": a_chosen,
    "b": cross["b"], "c": cross["c"], "d": cross["d"],
    "slope_at_zero": 0.25 * a_chosen * cross["b"] + cross["c"],
    "n_openloop": int(ol.shape[0]),
    "n_closedloop": cross["n"],
  }


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
    nc = CLOSED_LOOP_SAMPLES_PATH.stat().st_size // (CLOSED_LOOP_COLS * 4)
  except FileNotFoundError:
    nc = 0
  return (f"Idle|{n} open-loop + {nc:,} closed-loop|"
          f"Open-loop store: {n} lat-OFF driver-steered samples (need ≥{MIN_BAND_SAMPLES} "
          f"in at least one speed band).  Closed-loop torque_autotune store: "
          f"{nc:,} samples (reused for b/c/d cross-fit with `a` fixed).  Drive "
          f"with OP lat disengaged at 1-18 m/s to populate the open-loop side.")


def restore_idle_status() -> None:
  if Params("/dev/shm/params").get(STATUS_PARAM):
    return
  _set_status(_idle_status())


def _preview_status(fit: dict) -> str:
  cross = fit.get("cross")
  bands = fit.get("per_band", [])
  per_band_str = "  ".join(
    (f"{b['lo']:g}-{b['hi']:g}m/s: a={b.get('a',0):.2f} n={b['n']} rmse_drv={b.get('rmse_drv',0):.2f} OK"
     if b.get('ok') else f"{b['lo']:g}-{b['hi']:g}m/s: SKIP ({b.get('reason','?')})")
    for b in bands
  )
  a_var = fit.get("a_variation", 0.0)
  a_var_warn = "  [a varies >50% across bands - single-tune approximation may be poor]" if a_var > A_VARIATION_REL else ""
  if cross is None:
    return (f"Refused|Closed-loop store too small|"
            f"a chosen (median across bands): {fit['a_chosen']:.3f}  ·  "
            f"per-band: {per_band_str}.  Cross-fit needs ≥1000 closed-loop samples; "
            f"populate torque_autotune first.")
  return (f"Preview|Cross-fit ready (a={fit['a']:.2f} b={fit['b']:.3f} c={fit['c']:.3f} d={fit['d']:.3f})|"
          f"OP-normalized:  a={fit['a']:.5f}  b={fit['b']:.5f}  c={fit['c']:.5f}  "
          f"d={fit['d']:.5f}  slope-at-0={fit['slope_at_zero']:.3f}  "
          f"cross-fit RMSE={cross['rmse']:.4f} (n_closed={cross['n']:,}, n_open={fit['n_openloop']:,})"
          f"{a_var_warn}.  Per-band a: {per_band_str}")


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

  new = 0
  for i, path in enumerate(todo):
    _set_status(f"Collect|Processing {i + 1}/{len(todo)}|"
                f"Ingesting {path.parent.name} (+{new} open-loop samples so far)")
    rows = extract_segment(path)
    if rows is None:
      continue
    if rows.shape[0]:
      _append_rows(SAMPLES_PATH, rows)
      new += rows.shape[0]
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
                f"No speed band has ≥{MIN_BAND_SAMPLES} open-loop samples.  Drive "
                f"with OP lat disengaged at 1-18 m/s (parking-lot serpentine is "
                f"ideal) and re-tap Collect, then Fit.")
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

  if fit.get("cross") is None or "b" not in fit:
    _set_status("Refused|No cross-fit in pending|"
                "Pending fit lacks the closed-loop cross-fit step. Re-run Fit.")
    return

  prefix = _car_prefix()
  if prefix is None:
    _set_status("Refused|Unsupported fingerprint|"
                "Open-loop apply only writes Mazda3/CX-30/CX-50 tune params.")
    return

  a = float(fit["a"]); b = float(fit["b"]); c = float(fit["c"]); d = float(fit["d"])

  # Same ±1 coverage gate torque_autotune uses, with baseline-c fallback.
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
        f"({c_baseline:.3f}) also insufficient.  Cross-fit b is too small; "
        f"closed-loop store may need more high-|lat_accel| coverage."
      )
      return

  vals = tuple(float(np.clip(v, lo, hi)) for v, lo, hi in zip((a, b, c, d), PARAM_LO, PARAM_HI))
  for suffix, value in zip(("A", "B", "C", "D"), vals):
    params.put_float(f"{prefix}Tune{suffix}", value)
  params.remove(PENDING_PARAM)
  a_var = fit.get("a_variation", 0.0)
  warn = "  WARNING: a varied >50% across speed bands - check feel carefully" if a_var > A_VARIATION_REL else ""
  _set_status(f"Done|Applied (a={vals[0]:.2f} b={vals[1]:.2f}) — reboot to use|"
              f"{prefix} written from open-loop+cross-fit  "
              f"a={vals[0]:.5f}  b={vals[1]:.5f}  c={vals[2]:.5f}  d={vals[3]:.5f}.  "
              f"a from per-band open-loop median; b/c/d from closed-loop lstsq with "
              f"a fixed.{warn}")


def reset_table() -> None:
  params.remove(PENDING_PARAM)
  _set_status(_idle_status())
