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
from openpilot.frogpilot.common.long_autotune import _collect_steady_samples, _extract_one
from openpilot.frogpilot.common.torque_autotune import find_rlogs

# --- storage ---------------------------------------------------------------

STORE_DIR = Path("/data/media/0/long_autotune")
SAMPLES_PATH = STORE_DIR / "samples.f32"          # v_ego, can_cmd, a_ego
PROCESSED_PATH = STORE_DIR / "processed.json"

SAMPLE_COLS = 3
SAMPLE_BYTES = SAMPLE_COLS * 4                    # float32
MAX_STORE_BYTES = 500 * 1024 * 1024               # 500 MB rolling budget

STATUS_PARAM = "LongAutoTuneStatus"


def _set_status(msg: str) -> None:
  Params("/dev/shm/params").put(STATUS_PARAM, msg)


# --- per-rlog extraction wrapper -------------------------------------------

def extract_segment(path: Path):
  """Returns (N, 3) float32 array or None on read failure.

  Mirrors torque_autotune.extract_segment: empty (0, 3) means "log read OK
  but nothing usable" (parked, no ACC engagement, etc.) - the caller marks
  it processed so we don't retry. None means "couldn't read at all" - the
  caller leaves it OUT of processed so a later run gets another try.
  """
  rec = _extract_one(path)
  if rec is None:
    return None
  rows = _collect_steady_samples(rec)
  # _collect_steady_samples returns float32 already; (0, 3) is the standard
  # "nothing usable" sentinel that signals "done, don't retry".
  return rows


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


def load_store() -> np.ndarray:
  """Return the full rolling store as an (N, 3) float32 array (or empty)."""
  if not SAMPLES_PATH.is_file():
    return np.empty((0, SAMPLE_COLS), dtype=np.float32)
  try:
    arr = np.fromfile(SAMPLES_PATH, dtype=np.float32)
  except MemoryError:
    return np.empty((0, SAMPLE_COLS), dtype=np.float32)
  arr = arr[:arr.shape[0] - (arr.shape[0] % SAMPLE_COLS)]
  return arr.reshape(-1, SAMPLE_COLS)


# --- main entry ------------------------------------------------------------

def _idle_status() -> str:
  """STATE|short button label|full wrapping detail."""
  try:
    bytes_ = SAMPLES_PATH.stat().st_size
    rows = bytes_ // SAMPLE_BYTES
  except FileNotFoundError:
    bytes_, rows = 0, 0
  pct = 100.0 * bytes_ / MAX_STORE_BYTES if MAX_STORE_BYTES else 0.0
  return (f"Idle|{rows} samples ({pct:.1f}%)|"
          f"Store: {rows} samples ({bytes_/1024/1024:.1f} MB / "
          f"{MAX_STORE_BYTES/1024/1024:.0f} MB cap).  "
          f"Tap to ingest any new rlogs.")


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
                f"Ingesting {path.parent.name} (+{new_samples} new samples so far)")
    rows = extract_segment(path)
    if rows is None:
      # Unreadable; leave OUT of processed so a later run retries.
      continue
    if rows.shape[0]:
      _append_samples(rows)
      new_samples += rows.shape[0]
    processed.add(str(path))             # read OK -> mark done
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
