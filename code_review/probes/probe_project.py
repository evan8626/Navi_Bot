"""Reproduction probes for project infrastructure: CI, packaging, repository."""

import ast
import re
import subprocess
import sys
from pathlib import Path

from crlib.probe_api import probe, reproduced, not_reproduced

REPO_ROOT = Path(__file__).resolve().parents[2]
WORKFLOW = ".github/workflows/test.yml"

# import name -> distribution name, for imports whose PyPI name differs
DISTRIBUTION = {"sklearn": "scikit-learn", "yaml": "pyyaml", "cv2": "opencv-python"}
ROS_PROVIDED = {"rclpy", "launch", "launch_ros", "ament_index_python", "geometry_msgs",
                "nav_msgs", "sensor_msgs", "std_msgs", "navi_bot_interfaces", "tf2_ros"}


def _read(relative):
    return (REPO_ROOT / relative).read_text(encoding="utf-8")


def _run_blocks(workflow_text):
    """(indent, [command lines]) for every multi-line ``run: |`` block."""
    lines = workflow_text.splitlines()
    blocks = []
    for index, line in enumerate(lines):
        match = re.match(r"^(\s*)(?:-\s*)?run:\s*\|", line)
        if not match:
            continue
        commands = []
        for body in lines[index + 1:]:
            if body.strip() and len(body) - len(body.lstrip()) <= len(match.group(1)):
                break
            if body.strip():
                commands.append(body.strip())
        blocks.append(commands)
    return blocks


@probe("CI-001", deps=[WORKFLOW])
def ci_step_reports_only_the_last_test_suite():
    text = _read(WORKFLOW)
    on_windows = re.search(r"runs-on:\s*windows", text) is not None
    bash_shell = re.search(r"shell:\s*bash", text) is not None
    risky = []
    for commands in _run_blocks(text):
        pythons = [command for command in commands if command.startswith("python ")]
        guarded = any(
            "LASTEXITCODE" in command or "exit" in command.split() for command in commands
        )
        if len(pythons) > 1 and not guarded:
            risky.append(pythons)
    if not (on_windows and risky) or bash_shell:
        return not_reproduced("no multi-command PowerShell step that can hide a failing suite")
    suites = risky[0]
    return reproduced(
        f"the 'Run tests' step runs {len(suites)} suites as separate lines of one PowerShell "
        "script on windows-latest. PowerShell does not stop on a native command's non-zero exit "
        "code, and GitHub ends the script with `exit $LASTEXITCODE`, so the step's result is the "
        f"exit code of the LAST line only ({suites[-1]}). Any of the other {len(suites) - 1} "
        "suites can fail and the job is still green",
        suites=len(suites),
        last=suites[-1],
    )


@probe("CI-002", deps=[WORKFLOW])
def test_suites_missing_from_ci():
    text = _read(WORKFLOW)
    on_disk = sorted(path.name for path in (REPO_ROOT / "tests").glob("*_test.py"))
    missing = [name for name in on_disk if f"tests/{name}" not in text]
    if not missing:
        return not_reproduced(f"all {len(on_disk)} suites in tests/ are run by CI")
    return reproduced(
        f"{len(missing)} of {len(on_disk)} suites in tests/ are not run by CI: "
        f"{', '.join(missing)}. "
        "The list is maintained by hand in the workflow (and again in the dashboard and the "
        "README), so a new suite is silently skipped until someone remembers to add it",
        missing=missing,
    )


def _third_party_imports():
    stdlib = set(sys.stdlib_module_names)
    found = {}
    for path in (REPO_ROOT / "navi_bot").rglob("*.py"):
        tree = ast.parse(path.read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            names = []
            if isinstance(node, ast.Import):
                names = [alias.name for alias in node.names]
            elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
                names = [node.module]
            for name in names:
                top = name.split(".")[0]
                if top not in stdlib and top not in ROS_PROVIDED and top != "navi_bot":
                    found.setdefault(top, path.relative_to(REPO_ROOT).as_posix())
    return found


def _install_requires():
    tree = ast.parse(_read("setup.py"))
    for node in ast.walk(tree):
        if isinstance(node, ast.keyword) and node.arg == "install_requires":
            return {re.split(r"[<>=!~ \[]", element.value)[0].lower()
                    for element in node.value.elts if isinstance(element, ast.Constant)}
    return set()


@probe(
    "PK-001", deps=[
        "setup.py", "navi_bot/utils/websocket.py", "navi_bot/sensors/lidar_processing.py"
    ]
)
def imported_packages_missing_from_install_requires():
    declared = _install_requires()
    missing = {}
    for module, where in sorted(_third_party_imports().items()):
        if DISTRIBUTION.get(module, module).lower() not in declared:
            missing[module] = where
    if not missing:
        return not_reproduced(f"every third-party import is declared ({sorted(declared)})")
    listing = ", ".join(f"{module} (imported in {where})" for module, where in missing.items())
    return reproduced(
        f"imported by the package but not in setup.py install_requires: {listing}. "
        "`pip install -e .` on a clean machine - which is exactly what CI does - produces an "
        "install where these modules raise ImportError",
        missing=missing,
    )


@probe("GI-001", deps=[".gitignore"])
def compiled_files_are_tracked_by_git():
    result = subprocess.run(
        ["git", "ls-files"], cwd=REPO_ROOT, capture_output=True, text=True, check=True
    )
    tracked = [line for line in result.stdout.splitlines() if line.endswith((".pyc", ".pyo"))]
    if not tracked:
        return not_reproduced("no compiled Python files are tracked")
    return reproduced(
        f"{len(tracked)} compiled .pyc files are tracked by git ({', '.join(tracked[:3])}, ...). "
        "An ignore rule does not untrack files that are already committed, so stale "
        "cpython-311 bytecode ships with the repository and shows up in diffs",
        tracked=tracked,
    )


@probe("GI-002", deps=[".gitignore"])
def gitignore_ignores_itself():
    ignored = any(line.strip() == ".gitignore" for line in _read(".gitignore").splitlines())
    result = subprocess.run(
        ["git", "ls-files", ".gitignore"], cwd=REPO_ROOT, capture_output=True, text=True,
        check=True
    )
    tracked = bool(result.stdout.strip())
    if tracked or not ignored:
        return not_reproduced("the .gitignore file is tracked")
    return reproduced(
        "the .gitignore lists `.gitignore` itself and is not tracked, so the ignore rules exist "
        "only on this machine: a fresh clone (or CI) has none of them, and nobody else can see "
        "or review changes to them"
    )


@probe("DOC-001", deps=["README.md"])
def readme_project_structure_is_out_of_date():
    readme = _read("README.md")
    actual = [path.relative_to(REPO_ROOT).as_posix()
              for folder in ("navi_bot", "tests", "launch", "maps")
              for path in sorted((REPO_ROOT / folder).rglob("*.py"))
              if path.name != "__init__.py"]
    unlisted = [path for path in actual if Path(path).name not in readme]
    if not unlisted:
        return not_reproduced("every source file is mentioned in the README")
    return reproduced(
        f"{len(unlisted)} of {len(actual)} Python files are not mentioned anywhere in README.md "
        f"(its Project Structure tree predates them): {', '.join(unlisted)}",
        unlisted=unlisted,
    )


@probe("PK-002", deps=["setup.py", "package.xml", "navi_bot/__init__.py"])
def version_is_declared_in_three_places():
    versions = {
        "setup.py": re.search(r"version\s*=\s*['\"]([^'\"]+)", _read("setup.py")),
        "package.xml": re.search(r"<version>([^<]+)</version>", _read("package.xml")),
        "navi_bot/__init__.py": re.search(
            r"__version__\s*=\s*['\"]([^'\"]+)", _read("navi_bot/__init__.py")
        ),
    }
    found = {name: match.group(1) for name, match in versions.items() if match}
    if len(found) < 2:
        return not_reproduced(f"the version has a single source of truth ({found})")
    agree = "they agree today" if len(set(found.values())) == 1 else "they already DISAGREE"
    return reproduced(
        f"the package version is written out by hand in {len(found)} files "
        f"({', '.join(f'{name}: {value}' for name, value in found.items())}); {agree}, but "
        "nothing keeps them in step",
        versions=found,
    )
