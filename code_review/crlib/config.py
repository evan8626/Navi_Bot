"""Paths and thresholds for the code-review report.

Thresholds follow what the project already declares as its standard:
package.xml lists ``ament_flake8`` and ``ament_pep257`` as test dependencies,
so the style baseline is PEP 8 as ament configures it (99-column lines) plus
PEP 257 docstrings.
"""

from __future__ import annotations

from pathlib import Path

CODE_REVIEW_DIR = Path(__file__).resolve().parent.parent
REPO_ROOT = CODE_REVIEW_DIR.parent

FINDINGS_DIR = CODE_REVIEW_DIR / "findings"
PROBES_DIR = CODE_REVIEW_DIR / "probes"
WEB_DIR = CODE_REVIEW_DIR / "web"
STATE_DIR = CODE_REVIEW_DIR / "state"
OUT_DIR = CODE_REVIEW_DIR / "out"

BASELINE_FILE = STATE_DIR / "baseline.json"
TRIAGE_FILE = STATE_DIR / "triage.json"
REPORT_JSON = OUT_DIR / "report.json"
REPORT_HTML = OUT_DIR / "report.html"
HISTORY_FILE = OUT_DIR / "history.json"
PROBE_CACHE = OUT_DIR / "probe_cache.json"
TESTS_FILE = OUT_DIR / "tests.json"

PROJECT_NAME = "Navi Bot"

# Files the report analyses. Paths are repo-relative with forward slashes.
PYTHON_SUFFIXES = {".py"}
WEB_SUFFIXES = {".html", ".js"}
TEXT_SUFFIXES = {".py", ".html", ".js", ".yaml", ".yml", ".xml", ".md", ".toml", ".cfg",
                 ".json", ".txt", ".ps1", ".msg", ".rviz"}
EXCLUDED_PREFIXES = ("code_review/", "telemetry_dashboard/node_modules/", ".git/")
EXCLUDED_FILES = {"telemetry_dashboard/anime.min.js", "telemetry_dashboard/package-lock.json"}

# Code the owner writes himself as a learning exercise: fix hints for findings
# in these paths are collapsed in the report until he asks to see them.
LEARNING_PREFIXES = ("navi_bot/",)

# --- style thresholds -----------------------------------------------------
MAX_LINE_LENGTH = 99            # ament_flake8's limit
MAX_WEB_LINE_LENGTH = 140
MAX_FUNCTION_LINES = 60
VERY_LONG_FUNCTION_LINES = 120
MAX_COMPLEXITY = 10
HIGH_COMPLEXITY = 20
MAX_NESTING = 4
MAX_PARAMETERS = 6
MAX_FILE_LINES = 600
MAX_WEB_FUNCTION_LINES = 80
DENSE_LINE_STATEMENTS = 3       # statements on one line of JavaScript

# --- duplication ----------------------------------------------------------
DUPLICATE_MIN_TOKENS = 80
DUPLICATE_SIMILARITY = 0.80
DUPLICATE_SHINGLE = 6

# --- probes ---------------------------------------------------------------
PROBE_TIMEOUT_SECONDS = 180
PROBE_WORKERS = 4

# --- severity -------------------------------------------------------------
SEVERITIES = ("critical", "high", "medium", "low", "info")
SEVERITY_WEIGHT = {"critical": 10, "high": 5, "medium": 2, "low": 1, "info": 0}

# --- live server ----------------------------------------------------------
DEFAULT_PORT = 8377
WATCH_INTERVAL_SECONDS = 1.0
CONTEXT_LINES = 3               # source lines shown either side of a finding
FINGERPRINT_RADIUS = 4          # lines either side hashed to detect edits


def relative(path: Path) -> str:
    """Repo-relative path with forward slashes."""
    return path.resolve().relative_to(REPO_ROOT).as_posix()


def is_learning_code(repo_path: str) -> bool:
    """True for files whose fix hints stay collapsed by default."""
    return repo_path.startswith(LEARNING_PREFIXES)
