"""AST-based checks for Python files: naming, structure, dead code, docs.

Every check is a small generator ``check(tree, source) -> Iterable[Hit]``.
They are deliberately conservative: a report full of false alarms stops being
read, so anything ambiguous is either skipped or reported at ``info`` level.
"""

from __future__ import annotations

import ast
import builtins
import re
from pathlib import Path
from typing import Iterable, Iterator

from . import config
from .rules import Hit
from .sources import SourceFile

FUNCTIONS = (ast.FunctionDef, ast.AsyncFunctionDef)
SCOPES = FUNCTIONS + (ast.ClassDef, ast.Module, ast.Lambda)
NESTING = (ast.If, ast.For, ast.AsyncFor, ast.While, ast.Try, ast.With, ast.AsyncWith, ast.Match)
EXITS = (ast.Return, ast.Raise, ast.Continue, ast.Break)

SNAKE_CASE = re.compile(r"^_*[a-z][a-z0-9_]*$|^_+$")
CAP_WORDS = re.compile(r"^_*[A-Z][A-Za-z0-9]*$")
MODULE_NAME = re.compile(r"^[a-z_][a-z0-9_]*(\.launch)?$")
IMPOSED_NAME = re.compile(r"^(do_[A-Z]+|setUp|tearDown|setUpClass|tearDownClass)$")
SHADOWABLE = {name for name in dir(builtins) if name.islower() and not name.startswith("_")} - {
    "copyright", "credits", "license", "exit", "quit", "help"}
QUIET_EXCEPTIONS = {"KeyboardInterrupt", "SystemExit", "GeneratorExit", "StopIteration"}
LOG_METHODS = {"debug", "info", "warning", "warn", "error", "critical", "exception"}
LOGGER_NAMES = {"logger", "logging", "log"}


# ------------------------------------------------------------------ helpers

def _link_parents(tree: ast.AST) -> None:
    for parent in ast.walk(tree):
        for child in ast.iter_child_nodes(parent):
            child.parent = parent  # type: ignore[attr-defined]


def _scope_of(node: ast.AST) -> ast.AST:
    current = getattr(node, "parent", None)
    while current is not None and not isinstance(current, SCOPES):
        current = getattr(current, "parent", None)
    return current


def _functions(tree: ast.AST) -> Iterator[ast.FunctionDef]:
    return (node for node in ast.walk(tree) if isinstance(node, FUNCTIONS))


def _qualified_name(node: ast.AST) -> str:
    parts = [node.name]  # type: ignore[attr-defined]
    scope = _scope_of(node)
    while isinstance(scope, FUNCTIONS + (ast.ClassDef,)):
        parts.append(scope.name)
        scope = _scope_of(scope)
    return ".".join(reversed(parts))


def _is_dunder(name: str) -> bool:
    return name.startswith("__") and name.endswith("__")


def _is_method(func: ast.AST) -> bool:
    return isinstance(_scope_of(func), ast.ClassDef)


def _decorator_names(func: ast.FunctionDef) -> set[str]:
    names = set()
    for decorator in func.decorator_list:
        target = decorator.func if isinstance(decorator, ast.Call) else decorator
        names.add(target.attr if isinstance(target, ast.Attribute) else getattr(target, "id", ""))
    return names


def _is_stub(func: ast.FunctionDef) -> bool:
    """True when the body does nothing (docstring, pass, ..., raise NotImplementedError)."""
    for stmt in func.body:
        if isinstance(stmt, ast.Pass):
            continue
        if isinstance(stmt, ast.Expr) and isinstance(stmt.value, ast.Constant):
            continue
        if isinstance(stmt, ast.Raise):
            continue
        return False
    return True


def _loaded_names(node: ast.AST) -> set[str]:
    return {
        n.id for n in ast.walk(node) if isinstance(n, ast.Name) and isinstance(n.ctx, ast.Load)
    }


def _blocks(stmt: ast.stmt) -> list[list[ast.stmt]]:
    """Every statement list directly owned by a compound statement."""
    blocks = [
        getattr(stmt, name) for name in ("body", "orelse", "finalbody") if getattr(
            stmt, name, None
        )
    ]
    blocks.extend(handler.body for handler in getattr(stmt, "handlers", []))
    blocks.extend(case.body for case in getattr(stmt, "cases", []))
    return [block for block in blocks if isinstance(block, list)]


def _all_blocks(tree: ast.AST) -> Iterator[list[ast.stmt]]:
    for node in ast.walk(tree):
        if isinstance(node, ast.Module) or isinstance(node, FUNCTIONS + (ast.ClassDef,)):
            yield node.body
        elif isinstance(node, ast.stmt):
            yield from _blocks(node)


# ------------------------------------------------------------------- naming

def check_definition_names(tree: ast.AST, source: SourceFile) -> Iterable[Hit]:
    stem = Path(source.path).stem
    if not MODULE_NAME.match(stem):
        yield Hit("naming.module", 1, f"`{Path(source.path).name}` - use "
                                      f"lowercase_with_underscores")
    for node in ast.walk(tree):
        if isinstance(node, FUNCTIONS) and not _is_dunder(node.name):
            if IMPOSED_NAME.match(node.name):
                continue                               # http.server / unittest hooks
            if not SNAKE_CASE.match(node.name):
                yield Hit("naming.function", node.lineno, f"`{_qualified_name(node)}` should be "
                                                          f"snake_case")
            elif len(node.name.strip("_")) < 3:
                yield Hit("naming.short", node.lineno, f"function `{_qualified_name(node)}`")
        elif isinstance(node, ast.ClassDef):
            if not CAP_WORDS.match(node.name):
                yield Hit("naming.class", node.lineno, f"class `{node.name}` should be CapWords")
            elif len(node.name.strip("_")) < 3:
                yield Hit("naming.short", node.lineno, f"class `{node.name}`")


def check_attribute_names(tree: ast.AST, source: SourceFile) -> Iterable[Hit]:
    seen = set()
    for node in ast.walk(tree):
        is_self_store = (isinstance(node, ast.Attribute) and isinstance(node.ctx, ast.Store)
                         and isinstance(node.value, ast.Name) and node.value.id == "self")
        if is_self_store and node.attr not in seen and any(char.isupper() for char in node.attr):
            seen.add(node.attr)
            yield Hit("naming.attribute", node.lineno, f"`self.{node.attr}`")


def _bound_names(tree: ast.AST) -> Iterator[tuple[str, int, ast.AST]]:
    """(name, line, enclosing scope) for every name a file binds."""
    for node in ast.walk(tree):
        if isinstance(node, ast.Name) and isinstance(node.ctx, ast.Store):
            yield node.id, node.lineno, _scope_of(node)
        elif isinstance(node, ast.arg):
            yield node.arg, node.lineno, _scope_of(node)
        elif isinstance(node, FUNCTIONS + (ast.ClassDef,)):
            yield node.name, node.lineno, _scope_of(node)


def check_bound_names(tree: ast.AST, source: SourceFile) -> Iterable[Hit]:
    seen = set()
    for name, line, scope in _bound_names(tree):
        key = (name, id(scope))
        if key in seen:
            continue
        seen.add(key)
        in_class_body = isinstance(scope, ast.ClassDef)
        if name in SHADOWABLE and not in_class_body:
            yield Hit("naming.builtin-shadow", line, f"`{name}` hides the builtin `{name}`")
        elif isinstance(scope, FUNCTIONS) and any(
            char.isupper() for char in name
        ) and name not in ("self", "cls"):
            if not CAP_WORDS.match(name) or len(name) <= 3:
                yield Hit("naming.variable", line, f"`{name}` in `{scope.name}`")


# ---------------------------------------------------------------- structure

def _own_nodes(func: ast.AST) -> Iterator[ast.AST]:
    """Nodes inside a function, without descending into nested functions or classes."""
    pending = list(ast.iter_child_nodes(func))
    while pending:
        node = pending.pop()
        yield node
        if not isinstance(node, FUNCTIONS + (ast.ClassDef, ast.Lambda)):
            pending.extend(ast.iter_child_nodes(node))


def complexity(func: ast.AST) -> int:
    """McCabe cyclomatic complexity of one function (nested definitions count separately)."""
    score = 1
    for node in _own_nodes(func):
        if isinstance(node, (ast.If, ast.For, ast.AsyncFor, ast.While, ast.ExceptHandler,
                             ast.IfExp, ast.Assert, ast.match_case)):
            score += 1
        elif isinstance(node, ast.comprehension):
            score += 1 + len(node.ifs)
        elif isinstance(node, ast.BoolOp):
            score += len(node.values) - 1
    return score


def nesting_depth(statements: list[ast.stmt], level: int = 0) -> tuple[int, int]:
    """(deepest level, line where it is reached) for a list of statements."""
    deepest = (level, statements[0].lineno if statements else 0)
    for stmt in statements:
        if not isinstance(stmt, NESTING):
            continue
        for block in _blocks(stmt):
            is_elif = (isinstance(stmt, ast.If) and block is stmt.orelse
                       and len(block) == 1 and isinstance(block[0], ast.If))
            candidate = nesting_depth(block, level if is_elif else level + 1)
            if candidate[0] > deepest[0]:
                deepest = candidate
    return deepest


def _parameter_count(func: ast.FunctionDef) -> int:
    args = func.args
    count = len(args.posonlyargs) + len(args.args) + len(args.kwonlyargs)
    has_receiver = _is_method(func) and "staticmethod" not in _decorator_names(func)
    return count - 1 if has_receiver and count else count


def check_function_shape(tree: ast.AST, source: SourceFile) -> Iterable[Hit]:
    for func in _functions(tree):
        name = _qualified_name(func)
        length = (func.end_lineno or func.lineno) - func.lineno + 1
        if length > config.MAX_FUNCTION_LINES:
            yield Hit("structure.long-function", func.lineno, f"`{name}` is {length} lines")
        score = complexity(func)
        if score > config.MAX_COMPLEXITY:
            yield Hit("structure.complexity", func.lineno, f"`{name}` has complexity {score}")
        depth, line = nesting_depth(func.body)
        if depth > config.MAX_NESTING:
            yield Hit("structure.nesting", line, f"`{name}` nests {depth} levels deep")
        parameters = _parameter_count(func)
        if parameters > config.MAX_PARAMETERS:
            yield Hit("structure.parameters", func.lineno, f"`{name}` takes {parameters} "
                                                           f"parameters")
    if len(source.lines) > config.MAX_FILE_LINES:
        yield Hit("structure.long-file", 1, f"{len(source.lines)} lines")


def _self_attributes_stored(node: ast.AST) -> Iterator[ast.Attribute]:
    for child in ast.walk(node):
        if (isinstance(child, ast.Attribute) and isinstance(child.ctx, ast.Store)
                and isinstance(child.value, ast.Name) and child.value.id == "self"):
            yield child


def check_attributes_outside_init(tree: ast.AST, source: SourceFile) -> Iterable[Hit]:
    for cls in (node for node in ast.walk(tree) if isinstance(node, ast.ClassDef)):
        methods = [node for node in cls.body if isinstance(node, FUNCTIONS)]
        init = next((method for method in methods if method.name == "__init__"), None)
        if init is None:
            continue
        known = {attribute.attr for attribute in _self_attributes_stored(init)}
        known.update(target.id for stmt in cls.body if isinstance(stmt, ast.Assign)
                     for target in stmt.targets if isinstance(target, ast.Name))
        for method in methods:
            for attribute in _self_attributes_stored(method):
                if attribute.attr not in known:
                    known.add(attribute.attr)
                    yield Hit("structure.attr-outside-init", attribute.lineno,
                              f"`self.{attribute.attr}` first set in `{cls.name}.{method.name}`")


def check_methods_without_self(tree: ast.AST, source: SourceFile) -> Iterable[Hit]:
    if source.is_test:
        return
    for func in _functions(tree):
        if not _is_method(func) or _is_dunder(func.name) or _is_stub(func) or func.decorator_list:
            continue
        if not func.args.args or func.args.args[0].arg != "self":
            continue
        if _scope_of(func).name.startswith("Mock"):
            # stand-ins implement someone else's interface
            continue
        if "self" not in _loaded_names(func):
            yield Hit("structure.no-self-use", func.lineno, f"`{_qualified_name(func)}`")


def _always_exits(statements: list[ast.stmt]) -> bool:
    """True when control can never run off the end of ``statements``."""
    if not statements:
        return False
    last = statements[-1]
    if isinstance(last, (ast.Return, ast.Raise)):
        return True
    if isinstance(last, ast.If):
        return _always_exits(last.body) and _always_exits(last.orelse)
    if isinstance(last, (ast.With, ast.AsyncWith)):
        return _always_exits(last.body)
    if isinstance(last, ast.Try):
        if _always_exits(last.finalbody):
            return True
        tail = last.orelse or last.body
        return _always_exits(tail) and all(
            _always_exits(handler.body) for handler in last.handlers
        )
    if isinstance(last, ast.While):
        endless = isinstance(last.test, ast.Constant) and bool(last.test.value)
        return endless and not any(isinstance(node, ast.Break) for node in ast.walk(last))
    return False


def check_returns(tree: ast.AST, source: SourceFile) -> Iterable[Hit]:
    for func in _functions(tree):
        returns = [
            node for node in ast.walk(func) if isinstance(node, ast.Return) and _scope_of(
                node
            ) is func
        ]
        with_value = [node for node in returns
                      if node.value is not None and not (
                          isinstance(node.value, ast.Constant) and node.value.value is None
                      )]
        if with_value and not _always_exits(func.body):
            yield Hit("structure.inconsistent-return", func.lineno,
                      f"`{_qualified_name(func)}` returns a value on some paths and falls off "
                      f"the end on others")


def check_block_layout(tree: ast.AST, source: SourceFile) -> Iterable[Hit]:
    for node in ast.walk(tree):
        compound = isinstance(node, NESTING + FUNCTIONS + (ast.ClassDef, ast.ExceptHandler))
        body = getattr(node, "body", None)
        if compound and isinstance(body, list) and body and body[0].lineno == node.lineno:
            yield Hit(
                "structure.single-line-compound", node.lineno, source.lines[
                    node.lineno - 1
                ].strip()
            )
        if isinstance(node, ast.If) and node.orelse and isinstance(node.body[-1], EXITS):
            is_elif = len(node.orelse) == 1 and isinstance(node.orelse[0], ast.If)
            if not is_elif:
                yield Hit("structure.else-after-return", node.orelse[0].lineno,
                          f"the `if` on line {node.lineno} already leaves the block")
    for number, line in enumerate(source.lines, start=1):
        if re.match(r"^\s*(else|try|finally)\s*:\s*[^#\s]", line):
            yield Hit("structure.single-line-compound", number, line.strip())


def check_unreachable(tree: ast.AST, source: SourceFile) -> Iterable[Hit]:
    for block in _all_blocks(tree):
        for previous, following in zip(block, block[1:]):
            if isinstance(previous, EXITS):
                yield Hit("structure.unreachable", following.lineno,
                          f"follows `{type(previous).__name__.lower()}` on line {previous.lineno}")
                break


# --------------------------------------------------------------------- docs

def check_docstrings(tree: ast.AST, source: SourceFile) -> Iterable[Hit]:
    if not ast.get_docstring(tree) and source.text.strip():
        yield Hit("docs.module-docstring", 1, Path(source.path).name)
    for node in ast.walk(tree):
        if not isinstance(node, FUNCTIONS + (ast.ClassDef,)) or node.name.startswith("_"):
            continue
        if isinstance(_scope_of(node), FUNCTIONS):
            continue                                   # nested helper
        if not ast.get_docstring(node):
            kind = "class" if isinstance(node, ast.ClassDef) else "def"
            yield Hit("docs.docstring", node.lineno, f"{kind} `{_qualified_name(node)}`")


# ------------------------------------------------------------------ imports

def _imported_names(tree: ast.AST) -> Iterator[tuple[str, ast.stmt]]:
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                yield alias.asname or alias.name.split(".")[0], node
        elif isinstance(node, ast.ImportFrom) and node.module != "__future__":
            for alias in node.names:
                if alias.name != "*":
                    yield alias.asname or alias.name, node


def _exported_names(tree: ast.AST) -> set[str]:
    exported = set()
    for node in ast.walk(tree):
        targets = getattr(node, "targets", [])
        if isinstance(node, ast.Assign) and any(
            getattr(t, "id", "") == "__all__" for t in targets
        ):
            exported.update(
                e.value for e in getattr(node.value, "elts", []) if isinstance(e, ast.Constant)
            )
    return exported


def check_imports(tree: ast.AST, source: SourceFile) -> Iterable[Hit]:
    used = _loaded_names(tree) | _exported_names(tree)
    is_package_init = Path(source.path).name == "__init__.py"
    for name, node in _imported_names(tree):
        if name not in used and not is_package_init and "noqa" not in source.lines[
            node.lineno - 1
        ]:
            yield Hit("imports.unused", node.lineno, f"`{name}`")
    for node in ast.walk(tree):
        if isinstance(node, ast.Import) and len(node.names) > 1:
            yield Hit("imports.multiple", node.lineno, source.lines[node.lineno - 1].strip())
        if isinstance(node, (ast.Import, ast.ImportFrom)) and isinstance(
            _scope_of(node), FUNCTIONS
        ):
            module = getattr(node, "module", None) or node.names[0].name
            yield Hit("imports.in-function", node.lineno, f"`{module}` imported inside "
                                                          f"`{_scope_of(node).name}`")


# -------------------------------------------------------------------- logic

def check_unused_variables(tree: ast.AST, source: SourceFile) -> Iterable[Hit]:
    for func in _functions(tree):
        declared = {name for node in ast.walk(func) if isinstance(node, (ast.Global, ast.Nonlocal))
                    for name in node.names}
        used = _loaded_names(func)
        used.update(node.target.id for node in ast.walk(func)
                    if isinstance(node, ast.AugAssign) and isinstance(node.target, ast.Name))
        reported = set()
        for node in ast.walk(func):
            targets = node.targets if isinstance(node, ast.Assign) else (
                [node.target] if isinstance(node, ast.AnnAssign) and node.value is not None
                else [])
            for target in targets:
                name = getattr(target, "id", None)
                if (isinstance(target, ast.Name) and name not in used and name not in declared
                        and name not in reported and not name.startswith("_")
                        and _scope_of(node) is func):
                    reported.add(name)
                    yield Hit(
                        "logic.unused-variable", node.lineno,
                        f"`{name}` in `{_qualified_name(func)}`"
                    )


def check_unused_arguments(tree: ast.AST, source: SourceFile) -> Iterable[Hit]:
    for func in _functions(tree):
        if _is_dunder(func.name) or _is_stub(func) or func.decorator_list:
            continue
        arguments = func.args.posonlyargs + func.args.args + func.args.kwonlyargs
        if _is_method(func) and arguments:
            arguments = arguments[1:]
        used = _loaded_names(func)
        unused = [
            arg.arg for arg in arguments if arg.arg not in used and not arg.arg.startswith("_")
        ]
        if unused:
            listing = ", ".join(f"`{name}`" for name in unused)
            yield Hit("logic.unused-argument", func.lineno, f"{listing} in "
                                                            f"`{_qualified_name(func)}`")


def _handler_types(handler: ast.ExceptHandler) -> set[str]:
    node = handler.type
    elements = node.elts if isinstance(node, ast.Tuple) else [node]
    return {
        getattr(element, "id", getattr(element, "attr", "")) for element in elements
        if element is not None
    }


def check_exception_handling(tree: ast.AST, source: SourceFile) -> Iterable[Hit]:
    for handler in (node for node in ast.walk(tree) if isinstance(node, ast.ExceptHandler)):
        types = _handler_types(handler)
        silent = all(isinstance(stmt, ast.Pass) or (
            isinstance(stmt, ast.Expr) and isinstance(stmt.value, ast.Constant)
        )
                     for stmt in handler.body)
        reraises = any(isinstance(node, ast.Raise) for node in ast.walk(handler))
        if handler.type is None:
            yield Hit("logic.bare-except", handler.lineno, "catches everything, including Ctrl-C")
        elif silent and not types <= QUIET_EXCEPTIONS:
            yield Hit(
                "logic.swallowed-exception", handler.lineno,
                f"`except {', '.join(sorted(types))}: pass`"
            )
        elif types & {"Exception", "BaseException"} and not reraises and not source.is_test:
            yield Hit("logic.broad-except", handler.lineno, f"`except {', '.join(sorted(types))}`")


def check_defaults_and_asserts(tree: ast.AST, source: SourceFile) -> Iterable[Hit]:
    mutable = (ast.List, ast.Dict, ast.Set, ast.ListComp, ast.DictComp, ast.SetComp)
    for func in _functions(tree):
        for default in func.args.defaults + [d for d in func.args.kw_defaults if d is not None]:
            is_factory_call = (isinstance(default, ast.Call)
                               and getattr(default.func, "id", "") in {"list", "dict", "set"})
            if isinstance(default, mutable) or is_factory_call:
                yield Hit("logic.mutable-default", default.lineno, f"in `{_qualified_name(func)}`")
    if not source.is_test:
        for node in ast.walk(tree):
            if isinstance(node, ast.Assert):
                yield Hit("logic.assert", node.lineno, source.lines[node.lineno - 1].strip()[:90])


def check_expressions(tree: ast.AST, source: SourceFile) -> Iterable[Hit]:
    for node in ast.walk(tree):
        if isinstance(node, ast.JoinedStr) and not any(
            isinstance(v, ast.FormattedValue) for v in node.values
        ):
            if not isinstance(getattr(node, "parent", None), ast.FormattedValue):
                yield Hit(
                    "style.fstring-no-placeholder", node.lineno, source.lines[
                        node.lineno - 1
                    ].strip()[:90]
                )
        elif isinstance(node, ast.Compare):
            yield from _comparison_hits(node, source)
        elif isinstance(node, ast.BoolOp) and isinstance(node.op, ast.And):
            yield from _chained_hits(node, source)
        elif isinstance(node, ast.Call) and not source.is_test:
            yield from _logging_hits(node)


def _comparison_hits(node: ast.Compare, source: SourceFile) -> Iterable[Hit]:
    operands = [node.left] + node.comparators
    for operator, right in zip(node.ops, node.comparators):
        is_bool = isinstance(right, ast.Constant) and isinstance(right.value, bool)
        if is_bool and isinstance(operator, (ast.Is, ast.IsNot, ast.Eq, ast.NotEq)):
            yield Hit(
                "style.boolean-comparison", node.lineno, source.lines[node.lineno - 1].strip()[:90]
            )
    is_len = isinstance(node.left, ast.Call) and getattr(node.left.func, "id", "") == "len"
    zero = len(operands) == 2 and isinstance(operands[1], ast.Constant) and operands[1].value == 0
    if is_len and zero and isinstance(node.ops[0], (ast.Gt, ast.Eq, ast.NotEq)):
        yield Hit("style.len-comparison", node.lineno, source.lines[node.lineno - 1].strip()[:90])


def _chained_hits(node: ast.BoolOp, source: SourceFile) -> Iterable[Hit]:
    ordering = (ast.Lt, ast.LtE, ast.Gt, ast.GtE)
    for first, second in zip(node.values, node.values[1:]):
        simple = all(isinstance(side, ast.Compare) and len(side.ops) == 1 and isinstance(
            side.ops[0], ordering
        )
                     for side in (first, second))
        if simple and ast.dump(first.comparators[0]) == ast.dump(second.left):
            yield Hit(
                "style.chained-comparison", node.lineno, source.lines[node.lineno - 1].strip()[:90]
            )


def _logging_hits(node: ast.Call) -> Iterable[Hit]:
    func = node.func
    on_logger = (isinstance(func, ast.Attribute) and func.attr in LOG_METHODS
                 and isinstance(func.value, ast.Name) and func.value.id in LOGGER_NAMES)
    if on_logger and node.args and isinstance(node.args[0], ast.JoinedStr):
        yield Hit("logic.logging-fstring", node.lineno, f"`{func.value.id}.{func.attr}(f\"...\")`")


CHECKS = (
    check_definition_names, check_attribute_names, check_bound_names, check_function_shape,
    check_attributes_outside_init, check_methods_without_self, check_returns, check_block_layout,
    check_unreachable, check_docstrings, check_imports, check_unused_variables,
    check_unused_arguments, check_exception_handling, check_defaults_and_asserts,
    check_expressions,
)


# ------------------------------------------------------------------ metrics

def _metrics(tree: ast.AST, source: SourceFile) -> dict:
    functions = list(_functions(tree))
    public = [node for node in ast.walk(tree)
              if isinstance(node, FUNCTIONS + (ast.ClassDef,)) and not node.name.startswith("_")]
    documented = sum(1 for node in public if ast.get_docstring(node))
    annotated = sum(1 for func in functions
                    if func.returns is not None or any(arg.annotation for arg in func.args.args))
    worst = max(functions, key=complexity, default=None)
    return {
        "functions": len(functions),
        "classes": sum(1 for node in ast.walk(tree) if isinstance(node, ast.ClassDef)),
        "public": len(public),
        "documented": documented,
        "annotated": annotated,
        "max_complexity": complexity(worst) if worst else 0,
        "most_complex": _qualified_name(worst) if worst else "",
        "max_nesting": max((nesting_depth(func.body)[0] for func in functions), default=0),
        "longest_function": max(
            ((f.end_lineno or f.lineno) - f.lineno + 1 for f in functions), default=0
        ),
    }


def analyze(source: SourceFile) -> tuple[list[Hit], dict, ast.AST | None]:
    """Run every AST check on one Python file. Returns (hits, metrics, tree)."""
    try:
        tree = ast.parse(source.text)
    except SyntaxError as error:
        return [Hit("logic.syntax-error", error.lineno or 1, error.msg)], {}, None
    _link_parents(tree)
    hits = [hit for check in CHECKS for hit in check(tree, source)]
    return hits, _metrics(tree, source), tree
