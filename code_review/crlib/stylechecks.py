"""Layout checks for Python files: spacing, blank lines, comments, line length.

These work on raw lines and the token stream rather than the syntax tree,
because whitespace does not survive parsing.
"""

from __future__ import annotations

import ast
import io
import keyword
import re
import tokenize
from typing import Iterable

from . import config
from .rules import Hit
from .sources import SourceFile

SPACED_OPERATORS = {"=", "+=", "-=", "*=", "/=", "//=", "%=", "**=", "|=", "&=", "^=",
                    "==", "!=", "<=", ">=", "<", ">", "->", ":="}
OPENING, CLOSING = set("([{"), set(")]}")
SPACED_KEYWORDS = {"return", "if", "elif", "while", "assert", "in", "not", "and", "or",
                   "yield", "raise", "del", "is"}
MARKER = re.compile(r"\b(TODO|FIXME|XXX|HACK)\b")
RULER = re.compile(r"^#[#=\-*~_ ]*$")
DIRECTIVE = re.compile(r"^#\s*(noqa|type:|pylint:|pragma:|MARK:|!|:)", re.IGNORECASE)
CODE_LIKE = re.compile(
    r"^(import\s+\w|from\s+[\w.]+\s+import\s|[\w.\[\]]+\s*(=|\+=|-=)\s*\S|[\w.]+\(.*\)\s*$"
    r"|return\b|(if|for|while|elif|with)\s.+:$|else:$|def\s+\w+\(|class\s+\w+|continue$|break$)")
# "# gain = factor" documents a formula; real commented-out code assigns more than a bare word.
PROSE_ASSIGNMENT = re.compile(r"[A-Za-z_]\w*\s*=\s*[A-Za-z_]\w*")
SKIPPED_TOKENS = {
    tokenize.NL, tokenize.NEWLINE, tokenize.INDENT, tokenize.DEDENT, tokenize.COMMENT
}


def _tokens(source: SourceFile) -> list[tokenize.TokenInfo]:
    try:
        return list(tokenize.generate_tokens(io.StringIO(source.text).readline))
    except (tokenize.TokenError, IndentationError, SyntaxError):
        return []


# ---------------------------------------------------------------- raw lines

def check_lines(source: SourceFile) -> Iterable[Hit]:
    for number, line in enumerate(source.lines, start=1):
        if len(line) > config.MAX_LINE_LENGTH:
            yield Hit("style.line-length", number, f"{len(line)} characters")
        if line != line.rstrip():
            kind = "whitespace-only line" if not line.strip() else "trailing whitespace"
            yield Hit("style.trailing-whitespace", number, kind)
    if source.text and not source.text.endswith("\n"):
        yield Hit("style.final-newline", len(source.lines), "no newline at end of file")
    elif source.text.endswith("\n\n"):
        yield Hit("style.final-newline", len(source.lines), "blank line(s) at end of file")


def check_blank_runs(source: SourceFile) -> Iterable[Hit]:
    run = 0
    for number, line in enumerate(source.lines, start=1):
        run = run + 1 if not line.strip() else 0
        if run == 3:
            yield Hit("style.blank-lines", number, "3 or more blank lines in a row")


# -------------------------------------------------------------- blank lines

def _blank_lines_above(lines: list[str], first_line: int) -> tuple[int, int, bool]:
    """(blank lines directly above, widest blank gap above, whether code precedes).

    Mirrors pycodestyle: comments between the previous code and a definition
    belong to the definition, and the widest gap anywhere in that stretch is
    the one that counts for the "expected N blank lines" rules.
    """
    index = first_line - 2
    immediate = widest = run = 0
    counting_immediate = True
    while index >= 0:
        stripped = lines[index].strip()
        if not stripped:
            run += 1
            immediate += counting_immediate
        elif stripped.startswith("#"):
            widest, run, counting_immediate = max(widest, run), 0, False
        else:
            break
        index -= 1
    return immediate, max(widest, run), index >= 0


def _first_line(node: ast.AST) -> int:
    decorators = [decorator.lineno for decorator in getattr(node, "decorator_list", [])]
    return min(decorators + [node.lineno])


def _is_docstring(node: ast.AST) -> bool:
    return (isinstance(node, ast.Expr) and isinstance(node.value, ast.Constant)
            and isinstance(node.value.value, str))


def _check_top_level_spacing(lines: list[str], tree: ast.AST) -> Iterable[Hit]:
    definitions = (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)
    body = getattr(tree, "body", [])
    for previous, node in zip([None] + body, body):
        _immediate, widest, has_code_above = _blank_lines_above(lines, _first_line(node))
        if not has_code_above or widest == 2:
            continue
        if isinstance(node, definitions):
            yield Hit("style.blank-lines", _first_line(node),
                      f"expected 2 blank lines before `{node.name}`, found {widest}")
        elif isinstance(previous, definitions):
            yield Hit("style.blank-lines", node.lineno,
                      f"expected 2 blank lines after `{previous.name}`, found {widest}")


def _check_method_spacing(lines: list[str], tree: ast.AST) -> Iterable[Hit]:
    methods = (ast.FunctionDef, ast.AsyncFunctionDef)
    for cls in (node for node in ast.walk(tree) if isinstance(node, ast.ClassDef)):
        for previous, member in zip(cls.body, cls.body[1:]):
            if not isinstance(member, methods):
                continue
            immediate, widest, _ = _blank_lines_above(lines, _first_line(member))
            if widest == 0 and not _is_docstring(previous):
                yield Hit("style.blank-lines", _first_line(member),
                          f"expected 1 blank line before method `{member.name}`, found 0")
            elif immediate >= 2:
                yield Hit("style.blank-lines", _first_line(member),
                          f"too many blank lines ({immediate}) before method `{member.name}`")


def check_definition_spacing(source: SourceFile, tree: ast.AST) -> Iterable[Hit]:
    yield from _check_top_level_spacing(source.lines, tree)
    yield from _check_method_spacing(source.lines, tree)


# ------------------------------------------------------------------- tokens

def check_indentation(tokens: list[tokenize.TokenInfo]) -> Iterable[Hit]:
    for token in tokens:
        if token.type != tokenize.INDENT:
            continue
        if "\t" in token.string:
            yield Hit("style.indentation", token.start[0], "tab used for indentation")
        elif len(token.string) % 4:
            yield Hit("style.indentation", token.start[0], f"indented {len(token.string)} spaces")


def _is_annotated_parameter(tokens: list[tokenize.TokenInfo], index: int) -> bool:
    """True when the `=` at ``index`` is the default of an annotated parameter."""
    depth = 0
    for token in reversed(tokens[:index]):
        if token.string in CLOSING:
            depth += 1
        elif token.string in OPENING:
            if depth == 0:
                return False
            depth -= 1
        elif depth == 0 and token.string == ",":
            return False
        elif depth == 0 and token.string == ":":
            return True
    return False


def _touches(left: tokenize.TokenInfo, right: tokenize.TokenInfo) -> bool:
    return left.end == right.start


def check_operator_spacing(tokens: list[tokenize.TokenInfo]) -> Iterable[Hit]:
    code = [token for token in tokens if token.type not in SKIPPED_TOKENS]
    depth = 0
    for index, token in enumerate(code[1:-1], start=1):
        before, after = code[index - 1], code[index + 1]
        if token.type == tokenize.OP and token.string in OPENING:
            depth += 1
        elif token.type == tokenize.OP and token.string in CLOSING:
            depth = max(0, depth - 1)
        same_line = before.end[0] == token.start[0] == after.start[0]
        if not same_line:
            continue
        if token.type == tokenize.OP and token.string in SPACED_OPERATORS:
            keyword_equals = (token.string == "=" and depth > 0
                              and not _is_annotated_parameter(code, index))
            crowded = _touches(before, token) or _touches(token, after)
            if keyword_equals and not (_touches(before, token) and _touches(token, after)):
                yield Hit("style.operator-spacing", token.start[0], "spaces around "
                                                                    "keyword/default `=`")
            elif not keyword_equals and crowded:
                yield Hit("style.operator-spacing", token.start[0], f"missing space around "
                                                                    f"`{token.string}`")
        elif token.type == tokenize.OP and token.string == ",":
            if _touches(token, after) and after.string not in CLOSING:
                yield Hit("style.operator-spacing", token.start[0], "missing space after `,`")
        elif (token.type == tokenize.NAME and token.string in SPACED_KEYWORDS
              and keyword.iskeyword(token.string)):
            if after.string == "(" and _touches(token, after):
                yield Hit("style.operator-spacing", token.start[0], f"missing space after "
                                                                    f"`{token.string}`")


def _looks_like_code(comment: str) -> bool:
    if not re.match(r"^#\s?\S", comment) or DIRECTIVE.match(comment):
        return False
    body = comment.lstrip("#").strip()
    if not CODE_LIKE.match(body) or PROSE_ASSIGNMENT.fullmatch(body):
        return False
    try:
        ast.parse(body + (" pass" if body.endswith(":") else ""))
    except SyntaxError:
        return False
    return True


def check_comments(tokens: list[tokenize.TokenInfo], source: SourceFile) -> Iterable[Hit]:
    lines = source.lines
    previous_code_line = 0
    for token in tokens:
        if token.type != tokenize.COMMENT:
            continue
        number, column = token.start
        text = token.string
        inline = bool(lines[number - 1][:column].strip())
        marker = MARKER.search(text)
        if marker:
            yield Hit("docs.todo", number, text.lstrip("# ").strip()[:110])
        if not inline and _looks_like_code(text):
            if previous_code_line != number - 1:
                yield Hit("docs.commented-code", number, text.strip()[:90])
            previous_code_line = number
            continue
        if number == 1 and text.startswith("#!"):
            continue
        if inline and not lines[number - 1][:column].endswith("  "):
            yield Hit("style.comment-format", number, "inline comment needs two spaces before `#`")
        elif not text.startswith("# ") and not RULER.match(text) and not DIRECTIVE.match(text):
            yield Hit("style.comment-format", number, f"`{text[:30]}` should start with `# `")


def analyze(source: SourceFile, tree: ast.AST | None) -> list[Hit]:
    """Run every layout check on one Python file."""
    tokens = _tokens(source)
    hits = list(check_lines(source))
    hits.extend(check_blank_runs(source))
    if tree is not None:
        hits.extend(check_definition_spacing(source, tree))
    hits.extend(check_indentation(tokens))
    hits.extend(check_operator_spacing(tokens))
    hits.extend(check_comments(tokens, source))
    return hits
