#!/usr/bin/env python3
"""Offline validation of repository Markdown links and GitHub heading anchors.

Checks inline links/images and defined reference links, including URL-encoded
paths and fragments. Code examples and external URLs are not fetched or checked.
Only Git-visible files are valid local targets; ignored developer files are not.
"""
import argparse
from collections import Counter
import html
from pathlib import Path, PurePosixPath
import posixpath
import re
import unicodedata
from urllib.parse import unquote, urlsplit

from context_check import tracked_paths


def without_fences(text):
    lines, fence = [], None
    for line in text.splitlines():
        marker = re.match(r"^\s{0,3}(`{3,}|~{3,})", line)
        if fence:
            if marker and marker[1][0] == fence[0] and len(marker[1]) >= len(fence):
                fence = None
            lines.append("")
        elif marker:
            fence = marker[1]
            lines.append("")
        else:
            lines.append(line)
    return "\n".join(lines)


def anchors(text):
    text = without_fences(text)
    result = set(re.findall(r'<(?:a|[a-z0-9]+)\b[^>]*\b(?:id|name)=[\"\']([^\"\']+)', text, re.I))
    counts = Counter()
    lines = text.splitlines()
    for index, line in enumerate(lines):
        heading = re.match(r"^\s{0,3}#{1,6}\s+(.+?)\s*#*\s*$", line)
        title = heading[1] if heading else None
        if title is None and index + 1 < len(lines) and line.strip():
            if re.fullmatch(r"\s{0,3}(?:=+|-+)\s*", lines[index + 1]):
                title = line.strip()
        if title is None:
            continue
        title = re.sub(r"\[([^\]]+)\]\([^)]*\)", r"\1", title)
        title = html.unescape(re.sub(r"<[^>]+>", "", title)).lower()
        slug = "".join(c for c in title if c in "-_ " or
                       unicodedata.category(c)[0] in "LN").replace(" ", "-")
        candidate = slug
        while candidate in result:
            counts[slug] += 1
            candidate = f"{slug}-{counts[slug]}"
        result.add(candidate)
    return result


def destinations(text):
    text = without_fences(text)
    # Remove inline code before finding links, preserving line numbers.
    text = re.sub(r"(`+).*?\1", lambda m: " " * len(m[0]), text)
    definitions = {}
    for match in re.finditer(r"^\s{0,3}\[([^\]]+)\]:\s*(<[^>]*>|\S+)", text, re.M):
        definitions[" ".join(match[1].lower().split())] = match[2].strip("<>")
    for match in re.finditer(r"\[([^\]\n]+)\](\(|\[([^\]\n]*)\])", text):
        line = text.count("\n", 0, match.start()) + 1
        if match[2].startswith("["):
            key = " ".join((match[3] or match[1]).lower().split())
            # Undefined bracket pairs are literal text under CommonMark (e.g.
            # adjacent evidence labels [E01][E02]), not broken reference links.
            if key in definitions:
                yield line, definitions[key], key
            continue
        start = match.end()
        if text[start:start + 1] == "<":
            end = text.find(">", start + 1)
            if end >= 0:
                yield line, text[start + 1:end], None
            continue
        depth, end = 0, start
        while end < len(text):
            char = text[end]
            if char == "\\":
                end += 2
                continue
            if char == "(":
                depth += 1
            elif char == ")":
                if depth == 0:
                    break
                depth -= 1
            elif char.isspace() and depth == 0:
                break  # optional Markdown title follows
            end += 1
        yield line, re.sub(r"\\([()])", r"\1", text[start:end]), None
    # Defined shortcut references and image destinations also need validation.
    for match in re.finditer(r"(?<!\])\[([^\]\n]+)\](?![(:\[])", text):
        key = " ".join(match[1].lower().split())
        if key in definitions:
            yield text.count("\n", 0, match.start()) + 1, definitions[key], key


def check(root, paths):
    visible = set(paths)
    directories = {str(parent) for path in paths for parent in PurePosixPath(path).parents}
    errors, checked = [], 0
    cache = {}
    for source in sorted(p for p in visible if p.lower().endswith(".md")):
        text = (root / source).read_text(encoding="utf-8")
        for line, destination, reference in destinations(text):
            label = f"{source}:{line}"
            if destination is None:
                errors.append(f"{label}: undefined reference [{reference}]")
                continue
            url = urlsplit(destination)
            if url.scheme or url.netloc:
                continue
            checked += 1
            path = unquote(url.path)
            # Keep link spelling for case-sensitive Linux/Git checks even when
            # the checker runs on Windows, whose resolve() can normalize case.
            relative = posixpath.normpath(source if not path else
                                         path.lstrip("/") if path.startswith("/") else
                                         str(PurePosixPath(source).parent / path))
            if relative == ".." or relative.startswith("../"):
                errors.append(f"{label}: link escapes repository: {destination}")
                continue
            target = root / relative
            if relative not in visible and relative not in directories:
                errors.append(f"{label}: missing Git-visible target: {destination}")
                continue
            if url.fragment and relative.lower().endswith(".md"):
                if relative not in cache:
                    cache[relative] = anchors(target.read_text(encoding="utf-8"))
                if unquote(url.fragment) not in cache[relative]:
                    errors.append(f"{label}: missing heading/anchor: {destination}")
    return checked, errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[2])
    args = parser.parse_args()
    checked, errors = check(args.root, tracked_paths(args.root))
    for error in errors:
        print("ERROR: " + error)
    print(f"Checked {checked} local Markdown links; {len(errors)} errors")
    return int(bool(errors))


if __name__ == "__main__":
    raise SystemExit(main())
