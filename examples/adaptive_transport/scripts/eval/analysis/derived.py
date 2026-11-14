"""Results that need the raw current traces beyond their cycles files.

Almost every hardware result is computed from per-send cycles (power_common.load_cycles), which
the published data set holds for every capture. A few results read samples within a cycle: the
example traces of the packet-trace figure, the idle
floor of analyze_deep_sleep.py. For those, this module stores the computed values in measurement_data/derived/<name>.json:

    get(name, key, compute, traces)

returns compute() where every trace in ``traces`` is present, and otherwise the value stored under
``key``. With WRITE_DERIVED=1 in the environment, a computed value is also stored, which is how the
release export fills derived/ from the raw traces. JSON keeps floats exact (repr round-trips).
"""
from __future__ import annotations

import json
import os
from pathlib import Path

EVAL = Path(__file__).resolve().parent.parent
# DERIVED_DIR in the environment: read and write elsewhere (the release export writes into the data
# repository it builds, never into the raw measurement_data/)
DERIVED_DIR = Path(os.environ.get("DERIVED_DIR") or EVAL / "measurement_data" / "derived")
WRITE = os.environ.get("WRITE_DERIVED") == "1"


def key_for(path) -> str:
    """A trace's path relative to measurement_data/, the natural key of a value derived from it."""
    p = Path(path).absolute()
    data = (EVAL / "measurement_data").absolute()
    try:
        return p.relative_to(data).as_posix()
    except ValueError:
        return p.name


def _load(name: str) -> dict:
    path = DERIVED_DIR / f"{name}.json"
    return json.loads(path.read_text()) if path.exists() else {}


def get(name: str, key: str, compute, traces, optional: bool = False):
    """compute() if every path in ``traces`` exists (and stored with WRITE_DERIVED=1), else the
    stored value. A missing stored value raises, or returns None if ``optional``."""
    if all(Path(t).exists() for t in traces):
        value = compute()
        if WRITE:
            stored = _load(name)
            stored[key] = value
            DERIVED_DIR.mkdir(parents=True, exist_ok=True)
            (DERIVED_DIR / f"{name}.json").write_text(json.dumps(stored, indent=1, sort_keys=True))
        return value
    stored = _load(name)
    if key in stored:
        return stored[key]
    if optional:
        return None
    raise FileNotFoundError(f"no raw trace ({', '.join(Path(t).name for t in traces)}) and no stored "
                            f"value '{key}' in {DERIVED_DIR / (name + '.json')}")
