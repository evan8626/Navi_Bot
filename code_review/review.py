#!/usr/bin/env python3
"""Navi Bot live code-review report.

    python code_review/review.py serve      live report in the browser; updates as you save files
    python code_review/review.py            build code_review/out/report.html once
    python code_review/review.py tests      run every test suite, then rebuild
    python code_review/review.py check      exit 1 if open findings at/above --fail-on remain
    python code_review/review.py baseline   re-record the reviewed-code fingerprints
    python code_review/review.py hook       print (or --install) a post-commit hook

Standard library only; needs Python 3.11 or newer. See code_review/README.md.
"""

from __future__ import annotations

import argparse
import sys
import webbrowser
from pathlib import Path

if sys.version_info < (3, 11):
    sys.exit("code_review needs Python 3.11+ (it reads TOML with the standard library's tomllib).")

sys.path.insert(0, str(Path(__file__).resolve().parent))

from crlib import config, findings, report, server, sources, testsuite  # noqa: E402

HOOK_MARKER = "# navi-bot code review hook"
HOOK_SCRIPT = f"""#!/bin/sh
{HOOK_MARKER}: refresh the static report after every commit
python code_review/review.py build --quiet >/dev/null 2>&1 &
"""


def _build(arguments: argparse.Namespace, *, rebaseline: bool = False) -> dict:
    data = report.build(force_probes=arguments.force_probes, rebaseline=rebaseline)
    report.write(data)
    if not arguments.quiet:
        print(report.console_summary(data))
        print(f"  report: {config.REPORT_HTML}")
    return data


def command_build(arguments: argparse.Namespace) -> int:
    _build(arguments)
    if arguments.open:
        webbrowser.open(config.REPORT_HTML.as_uri())
    return 0


def command_serve(arguments: argparse.Namespace) -> int:
    server.serve(port=arguments.port, open_browser=not arguments.no_browser)
    return 0


def command_tests(arguments: argparse.Namespace) -> int:
    print("running the test suites (four at a time; the DWA integration suites take about a "
          "minute)...")
    record = testsuite.run_all(sources.load_all())
    for suite in record["suites"]:
        verdict = "ok  " if suite["exit"] == 0 else "FAIL"
        print(f"  {verdict} {suite['suite']:<34} {suite['passed']}/{suite['total']}  "
              f"{suite['seconds']} s")
    _build(arguments)
    return 0 if all(suite["exit"] == 0 for suite in record["suites"]) else 1


def command_check(arguments: argparse.Namespace) -> int:
    data = _build(arguments)
    threshold = config.SEVERITIES.index(arguments.fail_on)
    blocking = [finding for finding in data["findings"]
                if finding["status"] in findings.OPEN_STATUSES
                and config.SEVERITIES.index(finding["severity"]) <= threshold]
    for finding in blocking:
        print(f"  {finding['severity']:<8} {finding['id']:<9} "
              f"{finding['file']}:{finding['line'] or '?'}  {finding['title']}")
    print(f"{len(blocking)} open finding(s) at or above '{arguments.fail_on}'")
    return 1 if blocking else 0


def command_baseline(arguments: argparse.Namespace) -> int:
    _build(arguments, rebaseline=True)
    print(f"fingerprints re-recorded in {config.BASELINE_FILE}")
    return 0


def command_hook(arguments: argparse.Namespace) -> int:
    hook = config.REPO_ROOT / ".git" / "hooks" / "post-commit"
    if not arguments.install:
        print(f"This post-commit hook keeps code_review/out/report.html current.\n"
              f"Install it with:  python code_review/review.py hook --install\n\n--- {hook} "
              f"---\n{HOOK_SCRIPT}")
        return 0
    if hook.exists() and HOOK_MARKER not in hook.read_text(encoding="utf-8", errors="replace"):
        print(f"{hook} already exists and was not written by this tool - add the line by "
              f"hand:\n{HOOK_SCRIPT}")
        return 1
    hook.write_text(HOOK_SCRIPT, encoding="utf-8", newline="\n")
    print(f"installed {hook}")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Navi Bot live code-review report.")
    parser.add_argument("--quiet", action="store_true", help="no console summary")
    parser.add_argument("--force-probes", action="store_true", help="re-run every probe, "
                                                                    "ignoring the cache")
    commands = parser.add_subparsers(dest="command")
    build = commands.add_parser("build", help="build out/report.html once (default)")
    build.add_argument("--open", action="store_true", help="open the report in the browser")
    serve = commands.add_parser("serve", help="serve the live report and rebuild on every change")
    serve.add_argument("--port", type=int, default=config.DEFAULT_PORT)
    serve.add_argument("--no-browser", action="store_true", help="do not open a browser window")
    commands.add_parser("tests", help="run every test suite, then rebuild")
    check = commands.add_parser("check", help="non-zero exit while serious findings remain")
    check.add_argument("--fail-on", choices=config.SEVERITIES[:4], default="critical")
    commands.add_parser("baseline", help="re-record the fingerprints of the reviewed code")
    hook = commands.add_parser("hook", help="post-commit hook that rebuilds the static report")
    hook.add_argument("--install", action="store_true")
    for sub in commands.choices.values():
        sub.add_argument("--quiet", action="store_true", default=argparse.SUPPRESS)
        sub.add_argument("--force-probes", action="store_true", default=argparse.SUPPRESS)

    arguments = parser.parse_args(argv)
    arguments.open = getattr(arguments, "open", False)
    handlers = {None: command_build, "build": command_build, "serve": command_serve,
                "tests": command_tests, "check": command_check, "baseline": command_baseline,
                "hook": command_hook}
    return handlers[arguments.command](arguments)


if __name__ == "__main__":
    sys.exit(main())
