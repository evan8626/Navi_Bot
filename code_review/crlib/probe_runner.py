"""Run every probe in one probe file and print one JSON verdict per probe.

Executed as a subprocess by ``crlib.probes`` so that a hang, a crash, or
global state leaked by the code under test (the mock ROS bus, logging
handlers) cannot affect the report build or other probes::

    python code_review/crlib/probe_runner.py code_review/probes/probe_x.py

Each verdict is written to the real stdout as ``@@PROBE@@ {json}``. Anything
the probed code prints or logs is captured and discarded.
"""

from __future__ import annotations

import contextlib
import importlib.util
import io
import json
import logging
import sys
import time
import traceback
from pathlib import Path

SENTINEL = "@@PROBE@@"
CODE_REVIEW_DIR = Path(__file__).resolve().parent.parent
REPO_ROOT = CODE_REVIEW_DIR.parent


def _load_probe_module(probe_file: Path) -> None:
    """Import ``probe_file`` so its ``@probe`` decorators populate the registry."""
    spec = importlib.util.spec_from_file_location(probe_file.stem, probe_file)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load probe file {probe_file}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)


def _run_one(spec) -> dict:
    """Run a single probe with its output silenced; never raises."""
    started = time.perf_counter()
    sink = io.StringIO()
    result: dict = {"id": spec.finding_id, "deps": list(spec.deps)}
    try:
        with contextlib.redirect_stdout(sink), contextlib.redirect_stderr(sink):
            verdict = spec.func()
        result["status"] = "reproduced" if verdict.reproduced else "fixed"
        result["detail"] = verdict.detail
        result["data"] = verdict.data
    except Exception as exc:  # noqa: BLE001 - a broken probe must surface, not pass
        last_frames = traceback.format_exc(limit=-3).strip().splitlines()
        result["status"] = "error"
        result["detail"] = f"{type(exc).__name__}: {exc}"
        result["data"] = {"traceback": last_frames[-6:]}
    result["seconds"] = round(time.perf_counter() - started, 3)
    return result


def main(argv: list[str]) -> int:
    if len(argv) != 2:
        print("usage: probe_runner.py <probe_file.py>", file=sys.stderr)
        return 2
    sys.path.insert(0, str(REPO_ROOT))
    sys.path.insert(0, str(CODE_REVIEW_DIR))
    logging.disable(logging.CRITICAL)

    from crlib import probe_api  # imported after sys.path is prepared

    real_stdout = sys.stdout
    sink = io.StringIO()
    with contextlib.redirect_stdout(sink), contextlib.redirect_stderr(sink):
        _load_probe_module(Path(argv[1]).resolve())
    for spec in probe_api.REGISTRY.values():
        verdict = _run_one(spec)
        print(f"{SENTINEL} {json.dumps(verdict, default=str)}", file=real_stdout, flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
