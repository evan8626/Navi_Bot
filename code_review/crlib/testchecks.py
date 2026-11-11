"""Checks specific to this project's standalone test scripts.

Each ``tests/*_test.py`` file runs the functions listed in a ``tests = [...]``
list and announces them as ``TEST n: ...`` in its log. These checks keep that
hand-maintained registration honest.
"""

from __future__ import annotations

import ast
import re
from typing import Iterable

from .rules import Hit
from .sources import SourceFile

FUNCTIONS = (ast.FunctionDef, ast.AsyncFunctionDef)
TEST_LABEL = re.compile(r"\bTEST (\d+)\b")


def _loaded_names(node: ast.AST) -> set[str]:
    return {n.id for n in ast.walk(node)
            if isinstance(n, ast.Name) and isinstance(n.ctx, ast.Load)}


def _registered_tests(tree: ast.AST) -> list[str] | None:
    """Names listed in ``tests = [...]``, or None when the file has no such list."""
    for node in ast.walk(tree):
        is_list = isinstance(node, ast.Assign) and isinstance(node.value, ast.List)
        if is_list and any(getattr(target, "id", "") == "tests" for target in node.targets):
            return [element.id for element in node.value.elts if isinstance(element, ast.Name)]
    return None


def _first_test_label(func: ast.FunctionDef) -> tuple[int, int] | None:
    """(number, line) of the first 'TEST n' string literal in a test function."""
    labels = []
    for node in ast.walk(func):
        if isinstance(node, ast.Constant) and isinstance(node.value, str):
            if func.body and node is getattr(func.body[0], "value", None):
                continue                               # the docstring
            match = TEST_LABEL.search(node.value)
            if match:
                labels.append((node.lineno, int(match.group(1))))
    if not labels:
        return None
    line, number = min(labels)
    return number, line


def check_test_registration(tree: ast.AST, source: SourceFile) -> Iterable[Hit]:
    if not source.is_test or not isinstance(tree, ast.Module):
        return
    registered = _registered_tests(tree)
    if registered is None:
        return
    definitions = {node.name: node for node in tree.body if isinstance(node, FUNCTIONS)}
    referenced = _loaded_names(tree)
    for name, func in definitions.items():
        if name.startswith("test_") and name not in referenced:
            yield Hit("tests.unregistered", func.lineno, f"`{name}` is not in the `tests` list "
                                                         f"and is never called")
    for position, name in enumerate(registered, start=1):
        label = _first_test_label(definitions[name]) if name in definitions else None
        if label and label[0] != position:
            yield Hit("tests.label-mismatch", label[1],
                      f"`{name}` is test {position} in the list but logs 'TEST {label[0]}'")


def analyze(source: SourceFile, tree: ast.AST | None) -> list[Hit]:
    """Run the test-registration checks on one Python file."""
    return [] if tree is None else list(check_test_registration(tree, source))
