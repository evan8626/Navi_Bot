"""Curated findings: load them, find them in today's code, decide their status.

A finding lives in ``code_review/findings/*.toml`` and points at code with an
``anchor`` - a literal piece of the offending line - instead of a line number.
On every build the anchor is located again, so the report follows the code as
it moves, and one of four mechanisms decides whether the finding is still open:

1. a reproduction probe with the same id (authoritative: the behaviour itself);
2. a declared pattern (``open_if_present`` / ``open_if_absent`` /
   ``open_if_missing_file``) for defects that are a matter of text;
3. otherwise the code around the anchor is fingerprinted when the finding is
   first seen (``state/baseline.json``); if that code later changes or the
   anchor disappears the finding is flagged "changed - re-check";
4. a manual decision recorded in ``state/triage.json`` overrides 3.
"""

from __future__ import annotations

import hashlib
import json
import re
import tomllib
from datetime import datetime, timezone
from pathlib import Path

from . import config
from .sources import SourceFile

REQUIRED_FIELDS = ("id", "title", "severity", "category", "file")
PROSE_FIELDS = ("what", "why", "evidence", "hint")
OPEN_STATUSES = {"open", "changed", "acknowledged"}
TRIAGE_STATUSES = {"acknowledged", "wontfix", "resolved"}


def load() -> list[dict]:
    """Read every finding from the TOML files, validated and in file order."""
    findings: list[dict] = []
    seen: dict[str, str] = {}
    for path in sorted(config.FINDINGS_DIR.glob("*.toml")):
        document = tomllib.loads(path.read_text(encoding="utf-8"))
        for entry in document.get("finding", []):
            missing = [name for name in REQUIRED_FIELDS if not entry.get(name)]
            if missing:
                raise ValueError(f"{path.name}: finding {entry.get('id', '?')} lacks "
                                 f"{', '.join(missing)}")
            if entry["severity"] not in config.SEVERITIES:
                raise ValueError(f"{path.name}: {entry['id']} has unknown severity "
                                 f"{entry['severity']!r}")
            if entry["id"] in seen:
                raise ValueError(f"{entry['id']} is defined in both {seen[entry['id']]} and "
                                 f"{path.name}")
            seen[entry["id"]] = path.name
            for name in PROSE_FIELDS:
                entry[name] = _unwrap(entry.get(name, ""))
            entry["defined_in"] = path.name
            findings.append(entry)
    return findings


def _unwrap(text: str) -> str:
    """Join hard-wrapped lines; blank lines stay as paragraph breaks."""
    paragraphs = re.split(r"\n\s*\n", text.strip())
    return "\n\n".join(
        " ".join(paragraph.split()) for paragraph in paragraphs if paragraph.strip()
    )


# ----------------------------------------------------------------- locating

def _matching_lines(lines: list[str], anchor: str) -> list[int]:
    """1-based numbers of the lines containing ``anchor`` (whitespace-tolerant)."""
    exact = [number for number, line in enumerate(lines, start=1) if anchor in line]
    if exact:
        return exact
    squeezed = " ".join(anchor.split())
    return [
        number for number, line in enumerate(lines, start=1) if squeezed in " ".join(line.split())
    ]


def _locate(finding: dict, lines: list[str], remembered: dict | None) -> tuple[int | None, int]:
    """(line of the finding today, index of that match among all matches)."""
    anchor = finding.get("anchor", "")
    if not anchor:
        return None, 0
    matches = _matching_lines(lines, anchor)
    if not matches:
        return None, 0
    if remembered and remembered.get("matches") == len(matches) and remembered.get(
        "occurrence", 0
    ) < len(matches):
        index = remembered["occurrence"]
    else:
        hint = (remembered or {}).get("line") or finding.get("reviewed_line") or matches[0]
        index = min(range(len(matches)), key=lambda i: abs(matches[i] - hint))
    return matches[index], index


def _fingerprint(lines: list[str], line: int) -> str:
    """Hash of the code around ``line``, insensitive to indentation and position."""
    first = max(0, line - 1 - config.FINGERPRINT_RADIUS)
    window = [text.strip() for text in lines[first:line + config.FINGERPRINT_RADIUS]]
    return hashlib.sha1("\n".join(window).encode("utf-8")).hexdigest()[:16]


def _excerpt(lines: list[str], line: int) -> dict:
    first = max(1, line - config.CONTEXT_LINES)
    last = min(len(lines), line + config.CONTEXT_LINES)
    return {"start": first, "focus": line, "lines": lines[first - 1:last]}


# ------------------------------------------------------------------- status

def _pattern_found(pattern: str, text: str) -> bool:
    if pattern.startswith("re:"):
        return re.search(pattern[3:], text, re.MULTILINE) is not None
    return pattern in text


def _status_from_rules(finding: dict, text: str) -> tuple[str, str] | None:
    """Status decided by a declared text pattern, if the finding has one."""
    if finding.get("open_if_present"):
        present = _pattern_found(finding["open_if_present"], text)
        return ("open", "the flagged text is still present") if present else (
            "likely_fixed", "the flagged text is gone - confirm the fix")
    if finding.get("open_if_absent"):
        present = _pattern_found(finding["open_if_absent"], text)
        return (
            "likely_fixed", "the missing piece now appears - confirm the fix"
        ) if present else (
            "open", "the expected text is still absent")
    if finding.get("open_if_missing_file"):
        exists = (config.REPO_ROOT / finding["open_if_missing_file"]).exists()
        return ("likely_fixed", f"{finding['open_if_missing_file']} now exists") if exists else (
            "open", f"{finding['open_if_missing_file']} does not exist")
    return None


def _status_from_probe(result: dict) -> tuple[str, str]:
    if result["status"] == "reproduced":
        return "open", "reproduced by its probe"
    if result["status"] == "fixed":
        return "fixed", "its probe no longer reproduces the defect"
    return "changed", f"its probe could not run ({result['detail']}) - re-check"


def _status_from_code(
    line: int | None, touched: bool, has_anchor: bool, file_exists: bool
) -> tuple[str, str]:
    if not file_exists:
        return "changed", "the file no longer exists - re-check"
    if has_anchor and line is None:
        return "changed", "the reviewed line is gone - re-check whether this is fixed"
    if touched:
        return "changed", "the code here was edited after the review - re-check"
    return "open", "the reviewed code is unchanged"


def _apply_triage(status: str, detail: str, decision: dict | None) -> tuple[str, str]:
    if not decision or status in ("fixed", "likely_fixed"):
        return status, detail
    label = {
        "acknowledged": "acknowledged", "wontfix": "marked won't fix",
        "resolved": "marked resolved"
    }
    return decision["status"], f"{label[decision['status']]} by you" + (
        f" (and {detail})" if status == "changed" else "")


# ----------------------------------------------------------------- evaluate

def load_json(path: Path) -> dict:
    """Read a JSON object from ``path``; a missing or corrupt file is empty."""
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}


def save_json(path: Path, data: dict) -> None:
    """Write ``data`` atomically so a crash never leaves half a file."""
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(path)


def evaluate(findings: list[dict], files: dict[str, SourceFile], probe_results: dict[str, dict],
             rebaseline: bool = False) -> list[dict]:
    """Attach today's location, status and evidence to every finding."""
    stored = load_json(config.BASELINE_FILE)
    baseline = {} if rebaseline else stored.get("findings", {})
    triage = load_json(config.TRIAGE_FILE)
    baseline_changed = rebaseline
    evaluated = []
    for finding in findings:
        source = files.get(finding["file"])
        lines = source.lines if source else []
        remembered = baseline.get(finding["id"])
        line, occurrence = _locate(finding, lines, remembered)
        fingerprint = _fingerprint(lines, line) if line else ""
        if remembered is None and line:
            remembered = {"fingerprint": fingerprint, "occurrence": occurrence, "line": line,
                          "matches": len(_matching_lines(lines, finding["anchor"]))}
            baseline[finding["id"]] = remembered
            baseline_changed = True
        touched = bool(line and remembered and remembered["fingerprint"] != fingerprint)

        probe = probe_results.get(finding["id"])
        if probe:
            status, detail = _status_from_probe(probe)
        else:
            status, detail = _status_from_rules(finding, source.text if source else "") or \
                _status_from_code(line, touched, bool(finding.get("anchor")), source is not None)
        status, detail = _apply_triage(status, detail, triage.get(finding["id"]))

        evaluated.append(dict(
            finding,
            line=line,
            status=status,
            status_detail=detail,
            touched=touched,
            excerpt=_excerpt(lines, line) if line else None,
            probe=probe,
            triage=triage.get(finding["id"]),
            learning=config.is_learning_code(finding["file"]),
            verification="probe" if probe else "pattern" if _status_from_rules(
                finding, ""
            ) else "manual",
        ))
    if baseline_changed:
        created = stored.get("created") if not rebaseline else None
        save_json(config.BASELINE_FILE, {
            "created": created or datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "note": "Fingerprints of the code around each finding when it was reviewed. "
                    "Regenerate with: python code_review/review.py baseline",
            "findings": baseline,
        })
    return evaluated


def record_triage(finding_id: str, status: str, note: str) -> None:
    """Store (or clear, with status 'clear') a manual decision about a finding."""
    triage = load_json(config.TRIAGE_FILE)
    if status == "clear":
        triage.pop(finding_id, None)
    elif status in TRIAGE_STATUSES:
        triage[finding_id] = {"status": status, "note": note.strip()[:2000],
                              "updated": datetime.now(timezone.utc).isoformat(timespec="seconds")}
    else:
        raise ValueError(f"unknown triage status {status!r}")
    save_json(config.TRIAGE_FILE, triage)
