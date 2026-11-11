"""Assemble the report from its parts and write it to disk."""

from __future__ import annotations

import ast
import hashlib
import json
import platform
import time
from collections import Counter
from datetime import datetime, timezone

from . import (
    __version__, config, duplication, gitinfo, probes, pychecks, stylechecks, testchecks,
    testsuite, webchecks,
)
from . import findings as curated
from . import sources
from .rules import RULES, Hit
from .sources import SourceFile

DATA_PLACEHOLDER = "__REPORT_JSON__"
MAX_HITS_PER_GROUP = 400
HISTORY_LIMIT = 300

# path -> (text digest, hits, metrics, syntax tree); lets a live rebuild re-analyse only edited
# files
AnalysisCache = dict[str, tuple[str, list[Hit], dict, "ast.AST | None"]]


def _analyze_file(source: SourceFile) -> tuple[list[Hit], dict, ast.AST | None]:
    """Hits, metrics and syntax tree for one file; `# noqa` on a line silences its hits."""
    hits: list[Hit] = []
    metrics: dict = {}
    tree = None
    if source.is_python:
        hits, metrics, tree = pychecks.analyze(source)
        hits = hits + stylechecks.analyze(source, tree) + testchecks.analyze(source, tree)
    elif source.is_web:
        hits = webchecks.analyze(source)
    lines = source.lines
    kept = [hit for hit in hits
            if not (0 < hit.line <= len(lines) and "# noqa" in lines[hit.line - 1])]
    return kept, metrics, tree


def analyze(files: dict[str, SourceFile], cache: AnalysisCache) -> tuple[
    dict[str, list[Hit]], dict[str, dict]
]:
    """Automated hits and metrics for every file, re-using unchanged results."""
    for path in set(cache) - set(files):
        del cache[path]
    for path, source in files.items():
        digest = hashlib.sha1(source.text.encode("utf-8", errors="replace")).hexdigest()
        if path not in cache or cache[path][0] != digest:
            cache[path] = (digest, *_analyze_file(source))
    hits = {path: list(entry[1]) for path, entry in cache.items()}
    parsed = {
        path: (files[path], entry[3]) for path, entry in cache.items() if entry[3] is not None
    }
    for path, duplicate_hits in duplication.find_duplicates(parsed).items():
        hits[path].extend(duplicate_hits)
    return hits, {path: entry[2] for path, entry in cache.items()}


def _automated_groups(hits_by_file: dict[str, list[Hit]]) -> list[dict]:
    """One entry per (rule, file), listing every line where the rule fired."""
    groups = []
    for path in sorted(hits_by_file):
        by_rule: dict[str, list[Hit]] = {}
        for hit in hits_by_file[path]:
            by_rule.setdefault(hit.rule, []).append(hit)
        for rule_id, rule_hits in sorted(by_rule.items()):
            rule = RULES[rule_id]
            ordered = sorted(rule_hits, key=lambda hit: hit.line)
            groups.append({
                "id": f"auto/{rule_id}/{path}", "rule": rule_id, "file": path,
                "title": rule.title, "severity": rule.severity, "category": rule.category,
                "count": len(ordered),
                "hits": [
                    {"line": hit.line, "text": hit.text} for hit in ordered[:MAX_HITS_PER_GROUP]
                ],
            })
    return groups


def _file_rows(files: dict[str, SourceFile], metrics: dict[str, dict], evaluated: list[dict],
               groups: list[dict]) -> list[dict]:
    open_by_file: dict[str, Counter] = {}
    for finding in evaluated:
        if finding["status"] in curated.OPEN_STATUSES:
            open_by_file.setdefault(finding["file"], Counter())[finding["severity"]] += 1
    automated = Counter()
    for group in groups:
        automated[group["file"]] += group["count"]
    rows = []
    for path, source in sorted(files.items()):
        open_counts = open_by_file.get(path, Counter())
        rows.append({
            "path": path,
            "kind": "python" if source.is_python else "web" if source.is_web else "other",
            "is_test": source.is_test,
            "lines": len(source.lines),
            "metrics": metrics.get(path, {}),
            "open": {severity: open_counts.get(severity, 0) for severity in config.SEVERITIES},
            "automated": automated.get(path, 0),
            "weight": sum(config.SEVERITY_WEIGHT[s] * n for s, n in open_counts.items()),
        })
    return rows


def _summary(
    evaluated: list[dict], groups: list[dict], rows: list[dict], verdicts: dict[str, dict]
) -> dict:
    open_findings = [f for f in evaluated if f["status"] in curated.OPEN_STATUSES]
    open_by_severity = Counter(f["severity"] for f in open_findings)
    automated_by_severity = Counter()
    for group in groups:
        automated_by_severity[group["severity"]] += group["count"]
    python_rows = [row for row in rows if row["kind"] == "python"]
    public = sum(row["metrics"].get("public", 0) for row in python_rows)
    functions = sum(row["metrics"].get("functions", 0) for row in python_rows)
    return {
        "curated_total": len(evaluated),
        "by_status": dict(Counter(f["status"] for f in evaluated)),
        "open": len(open_findings),
        "open_by_severity": {s: open_by_severity.get(s, 0) for s in config.SEVERITIES},
        "weighted_open": sum(config.SEVERITY_WEIGHT[f["severity"]] for f in open_findings),
        "automated_hits": sum(group["count"] for group in groups),
        "automated_by_severity": {s: automated_by_severity.get(s, 0) for s in config.SEVERITIES},
        "files": len(rows),
        "lines": sum(row["lines"] for row in rows),
        "python_files": len(python_rows),
        "docstring_coverage": round(
            100 * sum(r["metrics"].get("documented", 0) for r in python_rows) / public
        )
        if public else None,
        "type_hint_coverage": round(
            100 * sum(r["metrics"].get("annotated", 0) for r in python_rows) / functions
        )
        if functions else None,
        "probes": dict(Counter(result["status"] for result in verdicts.values())),
    }


def _update_history(summary: dict, git: dict) -> list[dict]:
    history = curated.load_json(config.HISTORY_FILE).get("entries", [])
    snapshot = {"commit": git.get("commit", ""), "open": summary["open_by_severity"],
                "weighted": summary["weighted_open"], "automated": summary["automated_hits"],
                "fixed": summary["by_status"].get("fixed", 0) + summary[
                    "by_status"
                ].get("likely_fixed", 0)
                + summary["by_status"].get("resolved", 0)}
    comparable = {
        key: value for key, value in (history[-1] if history else {}).items() if key != "at"
    }
    if comparable != snapshot:
        history.append(dict(snapshot, at=datetime.now(timezone.utc).isoformat(timespec="seconds")))
        history = history[-HISTORY_LIMIT:]
        curated.save_json(config.HISTORY_FILE, {"entries": history})
    return history


def build(
    cache: AnalysisCache | None = None, *, force_probes: bool = False,
    rebaseline: bool = False, live: bool = False,
) -> dict:
    """Analyse the repository and return the complete report as plain data."""
    started = time.perf_counter()
    cache = {} if cache is None else cache
    files = sources.load_all()
    hits_by_file, metrics = analyze(files, cache)
    verdicts, probe_stats = probes.run_all(force=force_probes)
    evaluated = curated.evaluate(curated.load(), files, verdicts, rebaseline=rebaseline)
    groups = _automated_groups(hits_by_file)
    rows = _file_rows(files, metrics, evaluated, groups)
    summary = _summary(evaluated, groups, rows, verdicts)
    git = gitinfo.describe()
    return {
        "meta": {
            "project": config.PROJECT_NAME,
            "generated_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "repo_root": str(config.REPO_ROOT),
            "git": git,
            "live": live,
            "python": platform.python_version(),
            "tool_version": __version__,
            "build_seconds": round(time.perf_counter() - started, 2),
            "probe_stats": probe_stats,
            "severity_weight": config.SEVERITY_WEIGHT,
        },
        "summary": summary,
        "findings": evaluated,
        "automated": groups,
        "rules": {rule.id: {"title": rule.title, "severity": rule.severity,
                            "category": rule.category,
                            "why": rule.why} for rule in RULES.values()},
        "files": rows,
        "tests": testsuite.load(files),
        "history": _update_history(summary, git),
    }


def to_json(report: dict) -> str:
    """Serialise the report; numpy scalars and other strays become strings."""
    return json.dumps(report, ensure_ascii=False, default=str)


def embed(report_json: str) -> str:
    """The report page with its data embedded - a single self-contained file."""
    template = (config.WEB_DIR / "report.html").read_text(encoding="utf-8")
    safe = report_json.replace("<", "\\u003c").replace("\u2028", "\\u2028").replace(
        "\u2029", "\\u2029"
    )
    return template.replace(DATA_PLACEHOLDER, safe)


def write(report: dict) -> None:
    """Write ``out/report.json`` and the standalone ``out/report.html``."""
    config.OUT_DIR.mkdir(parents=True, exist_ok=True)
    ignore = config.OUT_DIR / ".gitignore"
    if not ignore.exists():
        ignore.write_text("# generated by code_review/review.py\n*\n", encoding="utf-8")
    static = to_json(dict(report, meta=dict(report["meta"], live=False)))
    config.REPORT_JSON.write_text(static, encoding="utf-8")
    config.REPORT_HTML.write_text(embed(static), encoding="utf-8")


def console_summary(report: dict) -> str:
    """A few lines for the terminal after a build."""
    summary, meta = report["summary"], report["meta"]
    severities = ", ".join(
        f"{count} {name}" for name, count in summary["open_by_severity"].items() if count
    )
    statuses = ", ".join(
        f"{count} {name.replace('_', ' ')}" for name, count in sorted(summary["by_status"].items())
    )
    return "\n".join([
        f"{meta['project']} code review - "
        f"{meta['git'].get('branch', '?')}@{meta['git'].get('commit', '?')}",
        f"  reviewed findings : {summary['curated_total']} ({statuses})",
        f"  open by severity  : {severities or 'none'}",
        f"  automated checks  : {summary['automated_hits']} hits across {summary['files']} files",
        f"  probes            : {meta['probe_stats']['probes']} "
        f"({meta['probe_stats']['rerun']} of {meta['probe_stats']['files']} files re-run, "
        f"{meta['probe_stats']['seconds']} s)",
        f"  built in {meta['build_seconds']} s",
    ])
