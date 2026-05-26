#!/usr/bin/env python3
# Mazda Gen2 longitudinal sample collector.
#
# Walks all rlogs under /data/media/0/realdata*, runs each through the
# steady-state extractor from long_autotune.py, and appends the survivors
# to a rolling 500 MB binary store at /data/media/0/long_autotune/samples.f32.
# Stores three float32 cols per sample: v_ego, can_cmd (12-bit ACCEL_CMD on
# bus 2), a_ego. processed.json tracks already-ingested rlogs so each run is
# incremental.
#
# Trigger pattern mirrors lateral autotune: the GUI's "Collect Long Samples"
# button sets params_memory.LongAutoTuneCollect=True, frogpilot_process picks
# it up offroad-only, runs this via run_thread_with_lock("mazda_long_collect").
# No fitting here - the SSH diagnostic (long_autotune.py main) reads the store
# and prints the per-bin table. Promotion to v2 (carcontroller integration)
# happens once we have enough data to trust the fits.

import json
import os
from pathlib import Path

import numpy as np

from openpilot.common.params import Params
from openpilot.frogpilot.common.long_autotune import (
  _collect_delay_samples, _collect_steady_samples, _extract_one,
)
from openpilot.frogpilot.common.torque_autotune import find_rlogs

# --- storage ---------------------------------------------------------------

STORE_DIR = Path("/data/media/0/long_autotune")
# v3 schema: static 5 cols (v_ego, can_cmd, a_ego, pitch, rpm) - unchanged
# from v2.  Delay store gained a 5th column: mean signed accel_desired per
# window, used to split throttle (>0) vs brake (<0) populations at fit time
# (Mazda has two actuators with different delays - see _collect_delay_samples).
# v2 was 4-col delay; v1 was 3-col static (no pitch/rpm/delay at all).
SAMPLES_PATH = STORE_DIR / "samples.f32"
DELAY_SAMPLES_PATH = STORE_DIR / "delay_samples.f32"
PROCESSED_PATH = STORE_DIR / "processed.json"
SCHEMA_VERSION = 3
SCHEMA_PATH = STORE_DIR / "schema_version"

SAMPLE_COLS = 5
SAMPLE_BYTES = SAMPLE_COLS * 4                    # float32
DELAY_SAMPLE_COLS = 5
DELAY_SAMPLE_BYTES = DELAY_SAMPLE_COLS * 4
MAX_STORE_BYTES = 500 * 1024 * 1024               # 500 MB rolling budget per file

STATUS_PARAM = "LongAutoTuneStatus"


def _set_status(msg: str) -> None:
  Params("/dev/shm/params").put(STATUS_PARAM, msg)


# --- per-rlog extraction wrapper -------------------------------------------

def extract_segment(path: Path):
  """Returns (static_rows, delay_rows) tuple of float32 arrays, or None.

  static_rows: (N, 5) = (v_ego, can_cmd, a_ego, pitch, rpm) per steady window.
  delay_rows:  (M, 4) = (v_ego, lag_s, ncc, rpm)             per xcorr window.

  Empty arrays are valid - "log read OK, nothing usable in that mode"
  (e.g. parked → no static samples; stock-ACC drive → no delay samples since
  OP long wasn't commanding). Caller marks the rlog processed in either
  case. None means "couldn't read at all" - the rlog stays OUT of processed
  for a later retry.
  """
  rec = _extract_one(path)
  if rec is None:
    return None
  return _collect_steady_samples(rec), _collect_delay_samples(rec)


# --- store management ------------------------------------------------------

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
  """Drop oldest samples in `path` so it stays within MAX_STORE_BYTES."""
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
  _trim_one(DELAY_SAMPLES_PATH, DELAY_SAMPLE_BYTES)


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
  """Static samples: (N, 5) = (v_ego, can_cmd, a_ego, pitch, rpm)."""
  return _load_rows(SAMPLES_PATH, SAMPLE_COLS)


def load_delay_store() -> np.ndarray:
  """Delay samples: (N, 4) = (v_ego, lag_s, ncc, rpm)."""
  return _load_rows(DELAY_SAMPLES_PATH, DELAY_SAMPLE_COLS)


def _migrate_schema() -> None:
  """Wipe any v1 store on first run after the v2 schema bump.

  v1 stored 3 cols (v_ego, can_cmd, a_ego); v2 adds pitch + rpm and stores
  5 cols. Concatenating them would corrupt the reshape. We also reset
  processed.json so the rlogs that contributed v1 samples get reprocessed
  under the v2 schema (gives us pitch + rpm + delay samples from them too).
  """
  try:
    current = int(SCHEMA_PATH.read_text())
  except Exception:
    current = 1 if SAMPLES_PATH.is_file() else SCHEMA_VERSION
  if current < SCHEMA_VERSION:
    for p in (SAMPLES_PATH, DELAY_SAMPLES_PATH, PROCESSED_PATH):
      try:
        p.unlink()
      except FileNotFoundError:
        pass
  SCHEMA_PATH.write_text(str(SCHEMA_VERSION))


# --- main entry ------------------------------------------------------------

def _idle_status() -> str:
  """STATE|short button label|full wrapping detail."""
  try:
    static_rows = SAMPLES_PATH.stat().st_size // SAMPLE_BYTES
  except FileNotFoundError:
    static_rows = 0
  try:
    delay_rows = DELAY_SAMPLES_PATH.stat().st_size // DELAY_SAMPLE_BYTES
  except FileNotFoundError:
    delay_rows = 0
  return (f"Idle|{static_rows} static + {delay_rows} delay|"
          f"Static store: {static_rows} samples (5 cols incl. pitch+rpm). "
          f"Delay store: {delay_rows} xcorr windows (4 cols incl. rpm). "
          f"Tap to ingest any new rlogs.")


def run_collect() -> None:
  STORE_DIR.mkdir(parents=True, exist_ok=True)
  _migrate_schema()

  processed = _load_processed()
  rlogs = find_rlogs()
  todo = [p for p in rlogs if str(p) not in processed]

  if not todo:
    _set_status(_idle_status())
    return

  new_static = 0
  new_delay = 0
  for i, path in enumerate(todo):
    _set_status(f"Collect|Processing {i + 1}/{len(todo)}|"
                f"Ingesting {path.parent.name} (+{new_static} static, "
                f"+{new_delay} delay so far)")
    res = extract_segment(path)
    if res is None:
      continue                              # unreadable - retry later
    static_rows, delay_rows = res
    if static_rows.shape[0]:
      _append_rows(SAMPLES_PATH, static_rows)
      new_static += static_rows.shape[0]
    if delay_rows.shape[0]:
      _append_rows(DELAY_SAMPLES_PATH, delay_rows)
      new_delay += delay_rows.shape[0]
    processed.add(str(path))
    if i % 20 == 0:
      _save_processed(processed)

  _save_processed(processed)
  _trim_store()
  _set_status(_idle_status())


def restore_idle_status() -> None:
  """Refresh status on boot/offroad so the panel shows current store size."""
  if Params("/dev/shm/params").get(STATUS_PARAM):
    return
  _set_status(_idle_status())
