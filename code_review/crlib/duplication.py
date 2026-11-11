"""Find functions that are copies of each other across the code base.

Each function is reduced to a set of token "shingles" (runs of consecutive
tokens with literals blanked out). Two functions whose shingle sets mostly
overlap are near-copies even when names or constants were edited.
"""

from __future__ import annotations

import ast
import io
import tokenize
from dataclasses import dataclass, field

from . import config
from .rules import Hit
from .sources import SourceFile

IGNORED = {tokenize.COMMENT, tokenize.NL, tokenize.NEWLINE, tokenize.INDENT, tokenize.DEDENT,
           tokenize.ENCODING, tokenize.ENDMARKER}
LITERALS = {tokenize.NUMBER: "0", tokenize.STRING: "S"}
for _name in ("FSTRING_START", "FSTRING_MIDDLE", "FSTRING_END"):
    if hasattr(tokenize, _name):
        LITERALS[getattr(tokenize, _name)] = "S"


@dataclass
class Function:
    """One function reduced to what the comparison needs."""

    path: str
    name: str
    line: int
    length: int
    token_count: int
    shingles: frozenset = field(repr=False)


def _normalised_tokens(source: SourceFile) -> list[tuple[int, str]]:
    """(line, normalised text) for every meaningful token in a file."""
    try:
        stream = tokenize.generate_tokens(io.StringIO(source.text).readline)
        return [(token.start[0], LITERALS.get(token.type, token.string))
                for token in stream if token.type not in IGNORED]
    except (tokenize.TokenError, IndentationError, SyntaxError):
        return []


def _functions_in(source: SourceFile, tree: ast.AST) -> list[Function]:
    tokens = _normalised_tokens(source)
    found = []
    for node in ast.walk(tree):
        if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            continue
        end = node.end_lineno or node.lineno
        body = [text for line, text in tokens if node.lineno <= line <= end]
        if len(body) < config.DUPLICATE_MIN_TOKENS:
            continue
        size = config.DUPLICATE_SHINGLE
        shingles = frozenset(hash(tuple(body[i:i + size])) for i in range(len(body) - size + 1))
        found.append(
            Function(
                source.path, node.name, node.lineno, end - node.lineno + 1, len(body), shingles
            )
        )
    return found


def _similarity(first: Function, second: Function) -> float:
    shared = len(first.shingles & second.shingles)
    return shared / (len(first.shingles) + len(second.shingles) - shared)


def _clusters(functions: list[Function]) -> list[list[Function]]:
    """Group functions connected by pairwise similarity above the threshold."""
    functions = sorted(functions, key=lambda function: function.token_count)
    leader = list(range(len(functions)))

    def find(index: int) -> int:
        while leader[index] != index:
            leader[index] = leader[leader[index]]
            index = leader[index]
        return index

    for i, first in enumerate(functions):
        for j in range(i + 1, len(functions)):
            second = functions[j]
            if first.token_count < second.token_count * config.DUPLICATE_SIMILARITY:
                break                                   # later ones are larger still
            if _similarity(first, second) >= config.DUPLICATE_SIMILARITY:
                leader[find(j)] = find(i)
    groups: dict[int, list[Function]] = {}
    for index, function in enumerate(functions):
        groups.setdefault(find(index), []).append(function)
    return [
        sorted(group, key=lambda f: (f.path, f.line)) for group in groups.values() if len(
            group
        ) > 1
    ]


def find_duplicates(parsed: dict[str, tuple[SourceFile, ast.AST]]) -> dict[str, list[Hit]]:
    """Hits per file for every function that has near-copies elsewhere."""
    functions = [
        function for source, tree in parsed.values() for function in _functions_in(source, tree)
    ]
    hits: dict[str, list[Hit]] = {}
    for group in _clusters(functions):
        for member in group:
            others = [other for other in group if other is not member]
            closest = max(others, key=lambda other: _similarity(member, other))
            where = ", ".join(f"{other.path}:{other.line}" for other in others[:4])
            extra = f" and {len(others) - 4} more" if len(others) > 4 else ""
            text = (f"`{member.name}` ({member.length} lines) - {len(group)} copies, "
                    f"{_similarity(member, closest):.0%} identical to `{closest.name}`; also at "
                    f"{where}{extra}")
            hits.setdefault(member.path, []).append(Hit("duplication.function", member.line, text))
    return hits
