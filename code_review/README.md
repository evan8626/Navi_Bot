# Navi Bot code review

A code review that stays attached to the code. The report is generated from the
repository every time it is built, so it changes when the code changes:

- **Reviewed findings** (273 of them) are stored as data and located in the
  current source by a snippet of the offending line, so line numbers follow your
  edits.
- **70 of those findings have a probe** - a small script that re-creates the bug
  against today's code. Fix the bug and the finding turns *Fixed* on the next
  build, however you restructured the code. Break it again and it reopens.
- **Style, naming, nesting, dead code and duplication** are checked from the
  syntax tree on every build (about 1,200 hits at the time of the review).

Standard library only. Needs Python 3.11+ (it reads TOML with `tomllib`); the
project's own interpreter (3.14) and the ROS 2 pixi environment (3.12) both work.

## Using it

Double-click **`Open Code Review.bat`** in this folder, or run:

```bash
python code_review/review.py serve
```

Either opens the report at <http://127.0.0.1:8377/> and keeps it current: save
any file and the page updates within a second or two, with a note saying what
changed. Leave the window open while you work; close it to stop. Launching it
again while it is already running just opens the page.

Without the server, `code_review/out/report.html` is a snapshot from the last
build that opens straight from disk (it does not update by itself).

| Command | What it does |
|---|---|
| `python code_review/review.py serve` | Live report; rebuilds on every save |
| `python code_review/review.py` | Build `code_review/out/report.html` once (open it with a double-click) |
| `python code_review/review.py tests` | Run all 21 test suites, four at a time, then rebuild |
| `python code_review/review.py check --fail-on high` | Exit 1 while open findings at that severity remain (for CI or a pre-push hook) |
| `python code_review/review.py baseline` | Re-record the reviewed-code fingerprints (after you have re-reviewed) |
| `python code_review/review.py hook` | Show a post-commit hook that refreshes the static report; `--install` writes it |

The first build runs every probe (about 10 s). After that only the probes whose
source files changed are re-run.

## How a finding gets closed

| Status | Meaning |
|---|---|
| **Open** | Its probe still reproduces the bug, or the reviewed code is unchanged. |
| **Fixed** | Its probe ran against your code and the bug no longer happens. |
| **Looks fixed** | A text pattern that marked the defect is gone (or the missing piece appeared). Glance at it. |
| **Re-check** | No probe covers it and the code under the finding was edited or removed. Decide whether your change fixed it, then mark it resolved in the page. |
| **Resolved / Acknowledged / Won't fix** | Your call, stored in `state/triage.json`. |

Hints towards a fix are folded away for anything under `navi_bot/` - that is the
code you are writing yourself. Tick *Reveal hints* in the page if you want them.

## What is in this folder

```
review.py            entry point (the commands above)
findings/*.toml      the reviewed findings, one file per area - edit freely
probes/probe_*.py    reproduction probes; a probe's id is the finding it backs
crlib/               the engine
    pychecks.py        naming, structure, dead code, docstrings (syntax tree)
    stylechecks.py     spacing, blank lines, comments, line length (tokens)
    testchecks.py      test registration in the custom runners
    duplication.py     near-identical functions across files
    webchecks.py       heuristics for the dashboard's HTML/JS
    findings.py        locate findings in today's code and decide their status
    probes.py          run probes in subprocesses, cache by dependency hash
    report.py          assemble everything; write report.json / report.html
    server.py          live server (127.0.0.1 only) with change watching
    rules.py           what each automated check means and why it matters
web/report.html      the page (one file, no dependencies)
state/baseline.json  fingerprints of the code under each finding at review time
state/triage.json    your decisions (created when you make the first one)
evidence/            the reviewers' throwaway repro scripts, archived as-is
out/                 generated output (ignored by git)
```

## Adding or changing a finding

Append a table to the relevant file in `findings/`:

```toml
[[finding]]
id = "RC-007"                       # any unique id
title = "One-line statement of the defect"
severity = "medium"                 # critical | high | medium | low | info
category = "bug"
file = "navi_bot/robot_controller.py"
anchor = "self.control_period = 0.020"   # literal text from the offending line
what = '''What is wrong.'''
why = '''Why it matters.'''
hint = '''Direction for a fix.'''
```

Optional keys: `open_if_present` / `open_if_absent` (literal text, or `re:` plus a
regular expression, searched in `file`) and `open_if_missing_file` make a finding
close itself when the text changes. To back a finding with behaviour instead,
add a probe with the same id:

```python
# code_review/probes/probe_robot_controller.py
@probe("RC-007", deps=["navi_bot/robot_controller.py"])
def control_period_is_configurable():
    ...
    return reproduced("what was observed") if still_broken else not_reproduced("what was observed")
```

A probe that raises is reported as "could not run", never as fixed.

## The automated checks

The baseline is the one the project already declares: `package.xml` lists
`ament_flake8` and `ament_pep257` as test dependencies, so the limits are PEP 8
as ament configures it (99-column lines) and PEP 257 docstrings. Thresholds live
in `crlib/config.py`. A line containing `# noqa` is skipped, as with flake8.

This folder is excluded from its own report (`EXCLUDED_PREFIXES` in
`crlib/config.py`). Held to the same rules it passes the layout, naming and
import checks; about 25 of its functions sit above the complexity threshold of
10 (mostly flat chains of independent conditions in the syntax-tree checks).

## Where the findings came from

- **Lead review (76 findings):** every source file read line by line; each
  behavioural claim reproduced before it was written down. These carry the probes.
- **Seven Opus 5.5 review passes (197 findings after de-duplication):** four over
  the 11,000 lines of tests, three independent second opinions on the planners,
  the nodes and the dashboard. 106 of their findings duplicated the lead's and
  were merged (shown as "also reported independently by N reviewers"). Every
  critical and high finding was re-checked by the lead reviewer; the page shows
  which evidence each finding rests on.
