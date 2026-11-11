"""Heuristic checks for the dashboard's JavaScript and HTML.

There is no JavaScript parser in the standard library, so this module blanks
out strings, comments and regex literals with a small scanner and then
applies simple, explainable rules to what is left.
"""

from __future__ import annotations

import re
from typing import Iterable

from . import config
from .rules import Hit
from .sources import SourceFile

REGEX_PRECEDERS = set("(,=:[!&|?{};+-*%<>~^")
REGEX_KEYWORDS = ("return", "typeof", "case", "of", "in", "delete", "void", "throw")
FUNCTION_START = re.compile(r"\bfunction\s+([A-Za-z_$][\w$]*)\s*\(")
INLINE_HANDLER = re.compile(r"""\son[a-z]+\s*=\s*["']""")
INNER_HTML = re.compile(r"\.innerHTML\s*\+?=")
INTERPOLATION = re.compile(r"\$\{([^{}]*)\}")
SCRIPT_BLOCK = re.compile(r"<script(?![^>]*\bsrc=)[^>]*>(.*?)</script>", re.DOTALL | re.IGNORECASE)


def _skip_quoted(text: str, start: int, quote: str) -> int:
    """Index just past a '...' or "..." literal that begins at ``start``."""
    index = start + 1
    while index < len(text) and text[index] not in (quote, "\n"):
        index += 2 if text[index] == "\\" else 1
    return min(index + 1, len(text))


def _skip_template(text: str, start: int) -> int:
    """Index just past a `...` template literal, including nested ${ } parts."""
    index = start + 1
    while index < len(text) and text[index] != "`":
        if text[index] == "\\":
            index += 2
        elif text.startswith("${", index):
            index = _skip_expression(text, index + 2)
        else:
            index += 1
    return min(index + 1, len(text))


def _skip_expression(text: str, start: int) -> int:
    """Index just past the ``}`` that closes a template ``${`` expression."""
    depth, index = 1, start
    while index < len(text) and depth:
        char = text[index]
        if char in "'\"":
            index = _skip_quoted(text, index, char)
            continue
        if char == "`":
            index = _skip_template(text, index)
            continue
        depth += (char == "{") - (char == "}")
        index += 1
    return index


def _skip_regex(text: str, start: int) -> int:
    """Index just past a /regex/flags literal that begins at ``start``."""
    index, in_class = start + 1, False
    while index < len(text) and text[index] != "\n":
        char = text[index]
        if char == "\\":
            index += 2
            continue
        if char == "/" and not in_class:
            index += 1
            break
        in_class = (char == "[") or (in_class and char != "]")
        index += 1
    while index < len(text) and text[index].isalpha():
        index += 1
    return index


def _regex_can_start(code: list[str]) -> bool:
    tail = "".join(code[-12:]).rstrip()
    if not tail or tail[-1] in REGEX_PRECEDERS:
        return True
    return tail.endswith(REGEX_KEYWORDS)


def strip_literals(text: str) -> str:
    """Return ``text`` with strings, comments and regexes replaced by spaces.

    Length and line breaks are preserved, so offsets map straight back to the
    original source.
    """
    out: list[str] = []
    index = 0
    while index < len(text):
        char = text[index]
        if text.startswith("//", index):
            end = text.find("\n", index)
            end = len(text) if end == -1 else end
        elif text.startswith("/*", index):
            close = text.find("*/", index + 2)
            end = len(text) if close == -1 else close + 2
        elif char in "'\"":
            end = _skip_quoted(text, index, char)
        elif char == "`":
            end = _skip_template(text, index)
        elif char == "/" and _regex_can_start(out):
            end = _skip_regex(text, index)
        else:
            out.append(char)
            index += 1
            continue
        out.extend("\n" if skipped == "\n" else " " for skipped in text[index:end])
        index = end
    return "".join(out)


def _script_regions(source: SourceFile) -> list[tuple[int, str]]:
    """(first line number, code) for each block of JavaScript in a file."""
    if source.suffix == ".js":
        return [(1, source.text)]
    regions = []
    for match in SCRIPT_BLOCK.finditer(source.text):
        first_line = source.text.count("\n", 0, match.start(1)) + 1
        regions.append((first_line, match.group(1)))
    return regions


def _statement_count(code_line: str) -> int:
    loops = len(re.findall(r"\bfor\s*\(", code_line))
    return max(0, code_line.count(";") - 2 * loops)


def _function_lengths(code: str, first_line: int) -> Iterable[tuple[str, int, int]]:
    """(name, start line, length in lines) for each named function declaration."""
    for match in FUNCTION_START.finditer(code):
        opening = code.find("{", match.end())
        if opening == -1:
            continue
        depth, index = 1, opening + 1
        while index < len(code) and depth:
            depth += (code[index] == "{") - (code[index] == "}")
            index += 1
        start = first_line + code.count("\n", 0, match.start())
        yield match.group(1), start, code.count("\n", match.start(), index) + 1


def _check_script(source: SourceFile, first_line: int, script: str) -> Iterable[Hit]:
    code = strip_literals(script)
    for offset, code_line in enumerate(code.splitlines()):
        statements = _statement_count(code_line)
        if statements >= config.DENSE_LINE_STATEMENTS:
            yield Hit(
                "web.dense-line", first_line + offset, f"{statements} statements on one line"
            )
    for name, start, length in _function_lengths(code, first_line):
        if length > config.MAX_WEB_FUNCTION_LINES:
            yield Hit("web.long-function", start, f"`{name}` is {length} lines")


def _check_inner_html(source: SourceFile) -> Iterable[Hit]:
    for number, line in enumerate(source.lines, start=1):
        assignment = INNER_HTML.search(line)
        if not assignment:
            continue
        raw = [expr.strip() for expr in INTERPOLATION.findall(line[assignment.end():])
               if not expr.strip().startswith("esc(")]
        if raw:
            listing = ", ".join(f"`{expr[:28]}`" for expr in raw[:4])
            yield Hit("web.innerhtml", number, f"interpolates {listing} without esc()")


def analyze(source: SourceFile) -> list[Hit]:
    """Run every web check on one .html or .js file."""
    hits = []
    for number, line in enumerate(source.lines, start=1):
        if len(line) > config.MAX_WEB_LINE_LENGTH:
            hits.append(Hit("web.line-length", number, f"{len(line)} characters"))
    for first_line, script in _script_regions(source):
        hits.extend(_check_script(source, first_line, script))
    hits.extend(_check_inner_html(source))
    if source.suffix == ".html":
        handlers = [
            number for number,
            line in enumerate(source.lines, start=1) if INLINE_HANDLER.search(line)
        ]
        hits.extend(
            Hit(
                "web.inline-handler", number, source.lines[number - 1].strip()[:90]
            ) for number in handlers
        )
        if "content-security-policy" not in source.text.lower():
            hits.append(Hit("web.no-csp", 1, "no <meta http-equiv=\"Content-Security-Policy\">"))
    if len(source.lines) > 1000:
        hits.append(Hit("web.long-file", 1, f"{len(source.lines)} lines in one file"))
    return hits
