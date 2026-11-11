"""Describe the repository state the report was built from."""

from __future__ import annotations

import subprocess

from . import config


def _git(*arguments: str) -> str:
    """Output of a read-only git command, or an empty string if it fails."""
    try:
        result = subprocess.run(["git", *arguments], cwd=config.REPO_ROOT, capture_output=True,
                                text=True, encoding="utf-8", errors="replace", timeout=20)
    except (OSError, subprocess.SubprocessError):
        return ""
    return result.stdout.strip() if result.returncode == 0 else ""


def describe() -> dict:
    """Branch, commit and uncommitted files of the working tree."""
    commit = _git("rev-parse", "--short", "HEAD")
    status = _git("status", "--porcelain")
    dirty = [line[3:].strip().strip('"') for line in status.splitlines() if line.strip()]
    return {
        "available": bool(commit),
        "branch": _git("rev-parse", "--abbrev-ref", "HEAD"),
        "commit": commit,
        "subject": _git("log", "-1", "--format=%s")[:120],
        "committed": _git("log", "-1", "--format=%cI"),
        "dirty": [path for path in dirty if not path.startswith("code_review/")],
    }
