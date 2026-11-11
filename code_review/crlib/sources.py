"""Discover and read the files the report analyses."""

from __future__ import annotations

import subprocess
from dataclasses import dataclass
from pathlib import Path

from . import config


@dataclass
class SourceFile:
    """One analysed file: repo-relative path plus its decoded text and lines."""

    path: str
    text: str

    @property
    def lines(self) -> list[str]:
        return self.text.splitlines()

    @property
    def suffix(self) -> str:
        return Path(self.path).suffix.lower()

    @property
    def is_python(self) -> bool:
        return self.suffix in config.PYTHON_SUFFIXES

    @property
    def is_web(self) -> bool:
        return self.suffix in config.WEB_SUFFIXES

    @property
    def is_test(self) -> bool:
        return self.path.startswith("tests/") or self.path.endswith("_test.py")


def _git_listed_files() -> list[str] | None:
    """Tracked plus untracked-but-not-ignored files, or None outside a git repo."""
    try:
        result = subprocess.run(
            ["git", "ls-files", "--cached", "--others", "--exclude-standard"],
            cwd=config.REPO_ROOT, capture_output=True, text=True, check=True, timeout=30,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    return [line.strip().strip('"') for line in result.stdout.splitlines() if line.strip()]


def _walked_files() -> list[str]:
    """Fallback discovery when git is unavailable."""
    skip = {".git", "node_modules", "__pycache__"}
    found = []
    for path in config.REPO_ROOT.rglob("*"):
        if path.is_file() and not skip.intersection(path.parts):
            found.append(path.relative_to(config.REPO_ROOT).as_posix())
    return found


def _wanted(repo_path: str) -> bool:
    if repo_path.startswith(config.EXCLUDED_PREFIXES) or repo_path in config.EXCLUDED_FILES:
        return False
    if "__pycache__" in repo_path:
        return False
    return Path(repo_path).suffix.lower() in config.TEXT_SUFFIXES or repo_path == ".gitignore"


def list_paths() -> list[str]:
    """Sorted repo-relative paths of every file the report covers."""
    listed = _git_listed_files()
    candidates = listed if listed is not None else _walked_files()
    if (config.REPO_ROOT / ".gitignore").exists():
        candidates.append(".gitignore")        # ignored by its own rule in this repo
    existing = {
        path for path in candidates if _wanted(path) and (config.REPO_ROOT / path).is_file()
    }
    return sorted(existing)


def read(repo_path: str) -> SourceFile:
    """Read one file as text (undecodable bytes are replaced, never fatal)."""
    raw = (config.REPO_ROOT / repo_path).read_bytes()
    return SourceFile(repo_path, raw.decode("utf-8", errors="replace"))


def load_all() -> dict[str, SourceFile]:
    """Every analysed file, keyed by repo-relative path."""
    return {path: read(path) for path in list_paths()}


def watched_paths() -> list[Path]:
    """Files whose modification should trigger a live rebuild."""
    watched = [config.REPO_ROOT / path for path in list_paths()]
    for folder in (config.FINDINGS_DIR, config.PROBES_DIR, config.STATE_DIR):
        if folder.exists():
            watched.extend(path for path in folder.rglob("*") if path.is_file())
    return watched
