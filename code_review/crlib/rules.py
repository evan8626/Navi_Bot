"""Catalogue of the automated checks: what each one means and why it matters."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Rule:
    """Static description of one automated check."""

    id: str
    title: str
    severity: str
    category: str
    why: str


@dataclass(frozen=True)
class Hit:
    """One place where a rule fired."""

    rule: str
    line: int
    text: str


_RULES = [
    # ------------------------------------------------------------------ naming
    Rule("naming.function", "Function or method name is not snake_case", "low", "naming",
         "PEP 8 reserves CapWords for classes. `queue.Remove(x)` reads like constructing an "
         "object; mixed conventions make every call site a small puzzle."),
    Rule("naming.class", "Class name is not CapWords", "low", "naming",
         "PEP 8: class names use the CapWords convention, so a reader can tell types from "
         "values at a glance."),
    Rule("naming.short", "Name is too short to say what it is", "low", "naming",
         "One- and two-letter names for classes and functions cannot be searched for and carry "
         "no meaning outside the textbook they came from."),
    Rule("naming.attribute", "Instance attribute name is not snake_case", "low", "naming",
         "PEP 8: instance variables are lowercase_with_underscores. Capitalised attributes "
         "look like classes or constants."),
    Rule("naming.variable", "Local variable or argument name is not lowercase", "info", "naming",
         "PEP 8: function and variable names are lowercase. Capitalised locals are sometimes "
         "kept deliberately to mirror a formula - worth a conscious decision either way."),
    Rule("naming.module", "File name is not lowercase_with_underscores", "low", "naming",
         "PEP 8: modules have short, all-lowercase names. Mixed-case file names behave "
         "differently on case-insensitive (Windows) and case-sensitive (Linux) file systems."),
    Rule("naming.builtin-shadow", "Name shadows a Python builtin", "low", "naming",
         "Rebinding `dir`, `id`, `list`, ... hides the builtin for the rest of the scope; the "
         "next use of the real one fails in a confusing way."),

    # --------------------------------------------------------------- structure
    Rule("structure.long-function", "Function is very long", "low", "design",
         "Long functions do several jobs at once; they are hard to test in pieces and hard to "
         "hold in your head."),
    Rule("structure.complexity", "Function has many independent paths", "medium", "design",
         "Cyclomatic complexity counts the paths through a function. Above about 10, the number "
         "of cases to test and reason about grows faster than anyone covers."),
    Rule("structure.nesting", "Code is nested too deeply", "low", "design",
         "Each level of indentation is another condition the reader must remember. Early "
         "returns and small helper functions flatten it."),
    Rule("structure.parameters", "Function takes many parameters", "low", "design",
         "Long parameter lists are easy to call in the wrong order and usually mean a missing "
         "data structure."),
    Rule("structure.long-file", "File is very long", "low", "design",
         "A file that needs a table of contents is several modules sharing a name."),
    Rule("structure.attr-outside-init", "Instance attribute first created outside __init__",
         "low", "design",
         "An object whose attributes appear only after a particular method has run raises "
         "AttributeError when methods are called in another order."),
    Rule("structure.no-self-use", "Method never uses self", "info", "design",
         "A method that ignores its instance is a function in disguise: it can be a module-level "
         "function or a staticmethod, which also makes it easier to test."),
    Rule("structure.inconsistent-return", "Some paths return a value, others fall off the end",
         "medium", "logic",
         "A function that returns a value on some paths and implicitly None on others forces "
         "every caller to guess; the implicit path is usually an unhandled case."),
    Rule("structure.else-after-return", "`else` after a branch that already returned",
         "info", "style",
         "When the `if` branch returns, the `else` only adds a level of indentation."),
    Rule("structure.single-line-compound", "Statement on the same line as its `if`/`else`/`for`",
         "low", "style",
         "PEP 8: compound statements go on separate lines. One-line bodies hide control flow "
         "and cannot hold a breakpoint."),
    Rule("structure.unreachable", "Code after return/raise/break/continue can never run",
         "medium", "dead-code",
         "Unreachable statements are either dead code or a sign that the control flow is not "
         "what the author intended."),

    # -------------------------------------------------------------------- docs
    Rule("docs.module-docstring", "Module has no docstring", "info", "docs",
         "PEP 257: every module should say what it is for. It is the first thing `help()` and a "
         "new reader see."),
    Rule("docs.docstring", "Public function, method or class has no docstring", "info", "docs",
         "PEP 257 (enforced by ament_pep257, which package.xml declares): public interfaces "
         "document what they take, return and assume - especially units and coordinate frames "
         "in robotics code."),
    Rule("docs.todo", "TODO / FIXME marker", "info", "docs",
         "Inventory of work the code itself says is unfinished."),
    Rule("docs.commented-code", "Commented-out code", "low", "dead-code",
         "Commented-out code is never tested or updated and is always worse than the version "
         "control history that already keeps it."),

    # ----------------------------------------------------------------- imports
    Rule("imports.unused", "Unused import", "low", "dead-code",
         "Unused imports slow start-up, create false dependencies between modules (planners "
         "importing the ROS mock) and hide the imports that matter."),
    Rule("imports.multiple", "Several modules imported on one line", "low", "style",
         "PEP 8: one import per line, so additions and removals show up as single-line diffs."),
    Rule("imports.in-function", "Import inside a function", "info", "style",
         "Function-level imports hide a module's dependencies and repeat the import lookup on "
         "every call. Sometimes deliberate (optional dependency) - then worth a comment."),

    # ------------------------------------------------------------------- style
    Rule("style.line-length", "Line longer than 99 characters", "low", "style",
         "ament_flake8 (declared in package.xml) fails lines over 99 columns. Long lines hide "
         "their right-hand side in diffs and side-by-side reviews."),
    Rule("style.trailing-whitespace", "Trailing whitespace", "low", "style",
         "Invisible characters that show up as noise in every diff touching the line."),
    Rule("style.blank-lines", "Blank-line spacing between definitions is off", "low", "style",
         "PEP 8: two blank lines around top-level definitions, one between methods, never more "
         "than two in a row. Consistent spacing is what lets the eye find definitions."),
    Rule("style.indentation", "Indentation is not a multiple of four spaces", "low", "style",
         "PEP 8: four spaces per level, no tabs. Mixed indentation is a syntax error waiting "
         "for the next edit."),
    Rule("style.operator-spacing", "Missing or extra whitespace around an operator",
         "low", "style",
         "PEP 8 spacing rules (E225, E231, E251, E275): spaces around assignment and "
         "comparison, after commas, none around keyword `=`, one after keywords."),
    Rule("style.comment-format", "Comment formatting", "info", "style",
         "PEP 8: block comments start with `# `; inline comments are separated from code by at "
         "least two spaces."),
    Rule("style.final-newline", "File does not end with exactly one newline", "low", "style",
         "POSIX tools and git expect a single trailing newline; its absence shows as a "
         "spurious change on the last line."),
    Rule("style.fstring-no-placeholder", "f-string without any placeholders", "low", "style",
         "The `f` prefix does nothing here; it usually marks a placeholder that was forgotten."),
    Rule("style.boolean-comparison", "Comparison with True/False", "low", "style",
         "`if x is False` treats None and False differently and reads as if that distinction "
         "mattered. PEP 8: use `if x` / `if not x`."),
    Rule("style.len-comparison", "len(x) compared with zero", "info", "style",
         "PEP 8: use the fact that empty sequences are false (`if items:`)."),
    Rule("style.chained-comparison", "Range test written as two comparisons", "info", "style",
         "`a <= x and x <= b` is `a <= x <= b` in Python - shorter and x is evaluated once."),

    # ------------------------------------------------------------------- logic
    Rule("logic.syntax-error", "File does not compile", "critical", "bug",
         "The module cannot be imported at all."),
    Rule("logic.unused-variable", "Variable is assigned but never used", "low", "dead-code",
         "A value computed and thrown away is either dead code or - worse - a result that was "
         "meant to be checked. In the test suites it marks assertions that were never written."),
    Rule("logic.unused-argument", "Parameter is never used", "info", "dead-code",
         "An ignored parameter promises behaviour the function does not have. Fine when a "
         "callback signature is imposed from outside."),
    Rule("logic.bare-except", "Bare `except:`", "medium", "robustness",
         "Catches everything, including KeyboardInterrupt and SystemExit, so Ctrl-C stops "
         "working inside it."),
    Rule("logic.swallowed-exception", "Exception caught and silently discarded",
         "medium", "robustness",
         "`except ...: pass` turns a failure into wrong behaviour later, far from its cause."),
    Rule("logic.broad-except", "`except Exception` without re-raising", "low", "robustness",
         "Catching every exception hides programming errors (TypeError, AttributeError) along "
         "with the failure that was expected."),
    Rule("logic.mutable-default", "Mutable default argument", "medium", "bug",
         "Default values are created once, when the function is defined, and shared by every "
         "call that does not pass the argument."),
    Rule("logic.assert", "assert used outside tests", "low", "robustness",
         "Assertions are removed under `python -O`; a check that matters must not disappear "
         "with an interpreter flag."),
    Rule("logic.logging-fstring", "f-string built for a logging call", "info", "performance",
         "The message is formatted even when that log level is switched off. It matters in "
         "loops that run thousands of times per plan."),

    # ------------------------------------------------------------------- tests
    Rule("tests.unregistered", "Test function is defined but never run", "high", "tests",
         "The suites run only the functions listed in `tests = [...]`. A test_ function missing "
         "from that list never executes, so it can never fail."),
    Rule("tests.label-mismatch", "TEST number in the log does not match its position",
         "medium", "tests",
         "The dashboard and `<suite> N` select tests by position; a log line announcing a "
         "different number sends you to the wrong test."),

    # ------------------------------------------------------------- duplication
    Rule("duplication.function", "Near-identical functions", "medium", "duplication",
         "Copies drift: a fix applied to one copy leaves the bug alive in the others. The "
         "locations listed share most of their code token for token."),

    # --------------------------------------------------------------------- web
    Rule("web.line-length", "Line longer than 140 characters", "low", "style",
         "Lines this long cannot be reviewed in a diff and usually hold several statements."),
    Rule("web.dense-line", "Several statements packed onto one line", "low", "style",
         "One statement per line is what makes stack traces, breakpoints and diffs point at "
         "the thing that changed."),
    Rule("web.long-function", "Function is very long", "low", "design",
         "Long functions mix state updates, drawing and control flow; none of it can be "
         "tested separately."),
    Rule("web.inline-handler", "Inline on* event handlers in markup", "low", "standards",
         "Inline handlers need globals, cannot be covered by a Content-Security-Policy and "
         "scatter behaviour through the markup."),
    Rule("web.innerhtml", "innerHTML assigned from interpolated values without escaping",
         "medium", "security",
         "Any interpolated value that can contain markup becomes script running in the page. "
         "Check each one: where does the value come from?"),
    Rule("web.no-csp", "Page has no Content-Security-Policy", "low", "security",
         "A CSP is the standard second line of defence for Electron apps: it stops injected "
         "markup from executing."),
    Rule("web.long-file", "File is very long", "low", "design",
         "Styles, markup and application logic in one file cannot be reused, tested or "
         "reviewed independently."),
]

RULES: dict[str, Rule] = {rule.id: rule for rule in _RULES}
