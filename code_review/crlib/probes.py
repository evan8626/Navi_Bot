"""Run the reproduction probes, re-using cached verdicts where nothing changed.

Each ``probes/probe_*.py`` file runs in its own interpreter (see
``probe_runner``). A file's verdicts are cached against a hash of the probe
file plus every source file its probes declared as dependencies, so a rebuild
after editing one module re-runs only the probes that could be affected.
"""

from __future__ import annotations

import hashlib
import json
import os
import re
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from . import config
from .findings import load_json, save_json

RUNNER = Path(__file__).resolve().parent / "probe_runner.py"
SENTINEL = "@@PROBE@@"
PROBE_ID = re.compile(r"""@probe\(\s*["']([A-Za-z0-9_-]+)["']""")


def probe_files() -> list[Path]:
    """Probe files in a stable order."""
    return sorted(config.PROBES_DIR.glob("probe_*.py"))


def _digest(probe_file: Path, dependencies: list[str]) -> str:
    digest = hashlib.sha1()
    digest.update(sys.version.encode())
    for path in [probe_file, RUNNER.with_name("probe_api.py")]:
        digest.update(path.read_bytes())
    for dependency in sorted(set(dependencies)):
        target = config.REPO_ROOT / dependency
        digest.update(dependency.encode())
        digest.update(target.read_bytes() if target.is_file() else b"<missing>")
    return digest.hexdigest()


def _is_fresh(entry: dict | None, probe_file: Path) -> bool:
    if not entry or not entry.get("results"):
        return False
    dependencies = [dep for result in entry["results"] for dep in result.get("deps", [])]
    return entry.get("digest") == _digest(probe_file, dependencies)


def _failure(probe_file: Path, reason: str) -> list[dict]:
    """One error verdict per probe declared in a file that could not be run."""
    declared = PROBE_ID.findall(probe_file.read_text(encoding="utf-8"))
    return [{
        "id": probe_id, "status": "error", "detail": reason, "data": {}, "deps": [], "seconds": 0.0
    }
            for probe_id in declared]


def _run_file(probe_file: Path) -> tuple[str, dict]:
    """Execute one probe file in a subprocess and collect its verdicts."""
    environment = dict(os.environ, PYTHONIOENCODING="utf-8", PYTHONDONTWRITEBYTECODE="1")
    started = time.time()
    try:
        completed = subprocess.run(
            [sys.executable, str(RUNNER), str(probe_file)], cwd=config.REPO_ROOT, env=environment,
            capture_output=True, text=True, encoding="utf-8", errors="replace",
            timeout=config.PROBE_TIMEOUT_SECONDS,
        )
        results = [json.loads(line[len(SENTINEL):]) for line in completed.stdout.splitlines()
                   if line.startswith(SENTINEL)]
        if not results:
            tail = (completed.stderr or completed.stdout).strip().splitlines()[
                -1:
            ] or ["no output"]
            results = _failure(probe_file, f"probe file failed to run: {tail[0][:200]}")
    except subprocess.TimeoutExpired:
        results = _failure(probe_file, f"probe file timed out after "
                                       f"{config.PROBE_TIMEOUT_SECONDS} s")
    except OSError as error:
        results = _failure(probe_file, f"could not start the probe runner: {error}")
    for result in results:
        result["ran_at"] = started
        result["probe_file"] = config.relative(probe_file)
    dependencies = [dep for result in results for dep in result.get("deps", [])]
    return probe_file.name, {"digest": _digest(probe_file, dependencies), "results": results}


def run_all(force: bool = False) -> tuple[dict[str, dict], dict]:
    """Verdict per finding id, plus statistics about what was (re-)run."""
    cache = load_json(config.PROBE_CACHE)
    files = probe_files()
    stale = [path for path in files if force or not _is_fresh(cache.get(path.name), path)]
    started = time.perf_counter()
    if stale:
        with ThreadPoolExecutor(max_workers=config.PROBE_WORKERS) as pool:
            for name, entry in pool.map(_run_file, stale):
                cache[name] = entry
    for name in set(cache) - {path.name for path in files}:
        del cache[name]
    if stale:
        save_json(config.PROBE_CACHE, cache)
    verdicts = {result["id"]: result for entry in cache.values() for result in entry["results"]}
    stats = {"files": len(files), "rerun": len(stale), "probes": len(verdicts),
             "seconds": round(time.perf_counter() - started, 1)}
    return verdicts, stats
