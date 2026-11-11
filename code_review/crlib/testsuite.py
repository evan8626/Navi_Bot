"""Run the project's standalone test suites and keep the last results."""

from __future__ import annotations

import hashlib
import os
import re
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
from pathlib import Path

from . import config
from .findings import load_json, save_json
from .sources import SourceFile

RESULT_LINE = re.compile(r"Results:\s*(\d+)\s*/\s*(\d+)\s*passed")
SUITE_TIMEOUT_SECONDS = 600
WORKFLOW = config.REPO_ROOT / ".github" / "workflows" / "test.yml"


def suites() -> list[Path]:
    """Every ``tests/*_test.py`` script."""
    return sorted((config.REPO_ROOT / "tests").glob("*_test.py"))


def sources_digest(files: dict[str, SourceFile]) -> str:
    """Hash of all Python sources: test results are stale once it changes."""
    digest = hashlib.sha1()
    for path in sorted(files):
        if files[path].is_python:
            digest.update(path.encode())
            digest.update(files[path].text.encode("utf-8", errors="replace"))
    return digest.hexdigest()


def _run_suite(suite: Path) -> dict:
    environment = dict(os.environ, PYTHONIOENCODING="utf-8", PYTHONDONTWRITEBYTECODE="1")
    started = time.perf_counter()
    try:
        completed = subprocess.run(
            [sys.executable, "-u", str(suite)], cwd=config.REPO_ROOT, env=environment,
            capture_output=True, text=True, encoding="utf-8", errors="replace",
            timeout=SUITE_TIMEOUT_SECONDS,
        )
        output, exit_code = completed.stdout + completed.stderr, completed.returncode
    except subprocess.TimeoutExpired:
        output, exit_code = f"timed out after {SUITE_TIMEOUT_SECONDS} s", -1
    counts = RESULT_LINE.findall(output)
    passed, total = (int(counts[-1][0]), int(counts[-1][1])) if counts else (0, 0)
    failing = [line.strip()[:200] for line in output.splitlines()
               if re.search(r"\bFAIL\b|Traceback|Error:", line)][:12]
    return {"suite": suite.name, "exit": exit_code, "passed": passed, "total": total,
            "seconds": round(time.perf_counter() - started, 1), "failing_lines": failing}


def run_all(files: dict[str, SourceFile]) -> dict:
    """Run every suite (four at a time) and store the outcome."""
    started = time.perf_counter()
    with ThreadPoolExecutor(max_workers=4) as pool:
        results = list(pool.map(_run_suite, suites()))
    record = {
        "ran_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "digest": sources_digest(files),
        "seconds": round(time.perf_counter() - started, 1),
        "suites": results,
    }
    save_json(config.TESTS_FILE, record)
    return record


def load(files: dict[str, SourceFile]) -> dict | None:
    """Last stored results, annotated with staleness and CI coverage."""
    record = load_json(config.TESTS_FILE)
    workflow = WORKFLOW.read_text(encoding="utf-8") if WORKFLOW.exists() else ""
    known = {result["suite"]: result for result in record.get("suites", [])}
    rows = []
    for suite in suites():
        row = dict(known.get(suite.name, {"suite": suite.name, "exit": None, "passed": 0,
                                          "total": 0,
                                           "seconds": 0, "failing_lines": []}))
        row["in_ci"] = f"tests/{suite.name}" in workflow
        rows.append(row)
    return {
        "ran_at": record.get("ran_at"),
        "seconds": record.get("seconds"),
        "stale": bool(record) and record.get("digest") != sources_digest(files),
        "suites": rows,
    }
