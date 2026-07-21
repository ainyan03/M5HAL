#!/usr/bin/env python3
"""Validate the structure and local links of public Markdown documents."""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
import tempfile
import unicodedata
from pathlib import Path
from urllib.parse import unquote, urlsplit


FENCE_RE = re.compile(r"^ {0,3}(`{3,}|~{3,})")
ATX_RE = re.compile(r"^ {0,3}(#{1,6})(.*)$")
INLINE_LINK_RE = re.compile(r"!?\[[^\]]*\]\((<[^>]+>|[^\s)]+)(?:\s+[^)]*)?\)")
REFERENCE_LINK_RE = re.compile(r"^ {0,3}\[[^\]]+\]:\s*(<[^>]+>|\S+)")
READER_RE = re.compile(r"^> \*\*読者\*\*:")
PAIR_MARKER_RE = re.compile(r"^<!-- pair: ([a-z0-9-]+) -->$")
ROOT_README_PAIR = ("README.md", "README.ja.md")


def tracked_markdown(root: Path) -> list[Path]:
    result = subprocess.run(
        ["git", "ls-files", "--cached", "--others", "--exclude-standard"],
        cwd=root,
        check=True,
        capture_output=True,
        text=True,
    )
    return [root / name for name in result.stdout.splitlines() if name.endswith(".md") and (root / name).is_file()]


def visible_lines(text: str) -> list[tuple[int, str]]:
    visible: list[tuple[int, str]] = []
    fence_char = ""
    fence_length = 0
    for number, line in enumerate(text.splitlines(), 1):
        match = FENCE_RE.match(line)
        if match:
            marker = match.group(1)
            if not fence_char:
                fence_char, fence_length = marker[0], len(marker)
                continue
            if marker[0] == fence_char and len(marker) >= fence_length and not line[match.end() :].strip():
                fence_char, fence_length = "", 0
                continue
        if not fence_char:
            visible.append((number, line))
    return visible


def heading_text(line: str) -> tuple[int, str] | None:
    match = ATX_RE.match(line)
    if not match or (match.group(2) and not match.group(2)[0].isspace()):
        return None
    text = re.sub(r"\s+#+\s*$", "", match.group(2).strip())
    return len(match.group(1)), text


def github_slug(text: str) -> str:
    text = re.sub(r"<[^>]*>", "", text)
    text = re.sub(r"!?\[([^\]]+)\]\([^)]*\)", r"\1", text)
    text = text.replace("`", "").replace("*", "").replace("~", "")
    text = re.sub(r"\s", "-", text.casefold())
    chars = []
    for char in text:
        category = unicodedata.category(char)
        if char in "-_" or not category.startswith(("P", "S", "C")):
            chars.append(char)
    return "".join(chars)


def anchors_for(text: str) -> set[str]:
    anchors: set[str] = set()
    for _, line in visible_lines(text):
        heading = heading_text(line)
        if not heading:
            continue
        base = github_slug(heading[1])
        candidate = base
        suffix = 0
        while candidate in anchors:
            suffix += 1
            candidate = f"{base}-{suffix}"
        anchors.add(candidate)
    return anchors


def link_targets(line: str) -> list[str]:
    line = re.sub(r"(`+).*?\1", "", line)
    targets = [match.group(1) for match in INLINE_LINK_RE.finditer(line)]
    reference = REFERENCE_LINK_RE.match(line)
    if reference:
        targets.append(reference.group(1))
    return [target[1:-1] if target.startswith("<") and target.endswith(">") else target for target in targets]


def paired_h2_sections(text: str) -> tuple[list[str], list[str]]:
    """Return ordered pair markers and marker/H2 adjacency errors."""
    lines = visible_lines(text)
    markers: list[str] = []
    errors: list[str] = []
    for index, (number, line) in enumerate(lines):
        marker = PAIR_MARKER_RE.fullmatch(line)
        heading = heading_text(line)
        if marker:
            markers.append(marker.group(1))
            if index + 1 >= len(lines):
                errors.append(f"line {number}: pair marker must immediately precede an H2")
                continue
            next_number, next_line = lines[index + 1]
            next_heading = heading_text(next_line)
            if next_number != number + 1 or not next_heading or next_heading[0] != 2:
                errors.append(f"line {number}: pair marker must immediately precede an H2")
        if heading and heading[0] == 2:
            if index == 0:
                errors.append(f"line {number}: every H2 must immediately follow a pair marker")
                continue
            previous_number, previous_line = lines[index - 1]
            if previous_number != number - 1 or not PAIR_MARKER_RE.fullmatch(previous_line):
                errors.append(f"line {number}: every H2 must immediately follow a pair marker")
    return markers, errors


def validate(root: Path, files: list[Path] | None = None) -> list[str]:
    root = root.resolve()
    full_scan = files is None
    files = tracked_markdown(root) if full_scan else files
    errors: list[str] = []
    text_cache: dict[Path, str] = {}
    anchor_cache: dict[Path, set[str]] = {}

    for path in files:
        path = path.resolve()
        relative = path.relative_to(root)
        text = path.read_text(encoding="utf-8")
        text_cache[path.resolve()] = text
        visible = visible_lines(text)
        h1_count = 0
        for number, line in visible:
            match = ATX_RE.match(line)
            if match and match.group(2) and not match.group(2)[0].isspace():
                errors.append(f"{relative}:{number}: ATX heading marker must be followed by a space")
                continue
            heading = heading_text(line)
            if heading and heading[0] == 1:
                h1_count += 1
        if h1_count != 1:
            errors.append(f"{relative}: expected exactly one H1 heading, found {h1_count}")

        if relative.parts and relative.parts[0] == "spec":
            first_eight = text.splitlines()[:8]
            if not any(READER_RE.match(line) for line in first_eight):
                errors.append(f"{relative}: primary reader label is missing from the first 8 lines")

        for number, line in visible:
            for raw_target in link_targets(line):
                parsed = urlsplit(raw_target)
                if parsed.scheme or raw_target.startswith(("//", "/")):
                    continue
                target_path = path if not parsed.path else path.parent / unquote(parsed.path)
                target_path = target_path.resolve()
                try:
                    target_path.relative_to(root)
                except ValueError:
                    errors.append(f"{relative}:{number}: relative link escapes the repository: {raw_target}")
                    continue
                if not target_path.exists():
                    errors.append(f"{relative}:{number}: link target does not exist: {raw_target}")
                    continue
                if not parsed.fragment:
                    continue
                if target_path.is_dir():
                    target_path = target_path / "README.md"
                if target_path.suffix.lower() != ".md" or not target_path.is_file():
                    errors.append(f"{relative}:{number}: anchor target is not a Markdown file: {raw_target}")
                    continue
                target_text = text_cache.get(target_path)
                if target_text is None:
                    target_text = target_path.read_text(encoding="utf-8")
                    text_cache[target_path] = target_text
                anchors = anchor_cache.setdefault(target_path, anchors_for(target_text))
                anchor = unquote(parsed.fragment).casefold()
                if anchor not in anchors:
                    errors.append(f"{relative}:{number}: anchor does not exist: {raw_target}")
    if full_scan:
        pair_paths = [root / name for name in ROOT_README_PAIR]
        if not all(path.is_file() for path in pair_paths):
            errors.append("README.md and README.ja.md must both exist")
        else:
            parsed = [paired_h2_sections(path.read_text(encoding="utf-8")) for path in pair_paths]
            marker_lists = [item[0] for item in parsed]
            for name, markers, section_errors in zip(ROOT_README_PAIR, marker_lists, (item[1] for item in parsed)):
                errors.extend(f"{name}: {error}" for error in section_errors)
                if not markers:
                    errors.append(f"{name}: paired README section markers are missing")
                if len(markers) != len(set(markers)):
                    errors.append(f"{name}: paired README section markers must be unique")
            if marker_lists[0] != marker_lists[1]:
                errors.append(
                    "README.md and README.ja.md must use the same ordered <!-- pair: ... --> section markers"
                )
    return errors


def self_test() -> int:
    with tempfile.TemporaryDirectory() as temporary:
        base = Path(temporary)
        root = base / "repo"
        root.mkdir()
        subprocess.run(["git", "init", "-q"], cwd=root, check=True)
        readme = root / "README.md"
        readme_ja = root / "README.ja.md"
        readme.write_text("# Readme\n\n<!-- pair: overview -->\n## Overview\n", encoding="utf-8")
        readme_ja.write_text("# Readme\n\n<!-- pair: overview -->\n## 概要\n", encoding="utf-8")
        (root / "spec").mkdir()
        good = root / "spec" / "good.md"
        good.write_text(
            "# 文書\n\n> **読者**: 利用者向け。\n\n## Unicode 見出し\n## Unicode 見出し\n"
            "```sh\n#not-a-heading\n[ignored](missing.md)\n```oops\n#still-not-a-heading\n```\n"
            "[first](#unicode-見出し) [second](#unicode-見出し-1)\n",
            encoding="utf-8",
        )
        subprocess.run(["git", "add", "README.md", "README.ja.md", "spec/good.md"], cwd=root, check=True)
        assert not validate(root), validate(root)
        assert github_slug("v0 + v2") == "v0--v2"
        collision_text = "# Foo\n## Foo\n## Foo-1\n"
        assert anchors_for(collision_text) == {"foo", "foo-1", "foo-1-1"}

        deleted = root / "deleted.md"
        deleted.write_text("# Deleted\n", encoding="utf-8")
        referring = root / "referring.md"
        referring.write_text("# Referring\n\n[deleted](deleted.md)\n", encoding="utf-8")
        subprocess.run(["git", "add", "deleted.md", "referring.md"], cwd=root, check=True)
        deleted.unlink()
        errors = validate(root)
        assert any("link target does not exist: deleted.md" in error for error in errors), errors
        assert not any(error.startswith("deleted.md:") for error in errors), errors

        bad = root / "bad.md"
        bad.write_text("#bad\n\n[missing](none.md) [anchor](spec/good.md#missing)\n", encoding="utf-8")
        errors = validate(root, [bad])
        assert any("followed by a space" in error for error in errors), errors
        assert any("exactly one H1" in error for error in errors), errors
        assert any("does not exist" in error for error in errors), errors
        assert any("anchor does not exist" in error for error in errors), errors

        no_reader = root / "spec" / "no-reader.md"
        no_reader.write_text("# No reader\n", encoding="utf-8")
        errors = validate(root, [no_reader])
        assert any("reader label is missing" in error for error in errors), errors

        (base / "outside.md").write_bytes(b"\xff")
        outside_link = root / "outside-link.md"
        outside_link.write_text("# Outside link\n\n[outside](../outside.md#anything)\n", encoding="utf-8")
        errors = validate(root, [outside_link])
        assert any("escapes the repository" in error for error in errors), errors

        bad.unlink()
        no_reader.unlink()
        outside_link.unlink()
        deleted.write_text("# Deleted\n", encoding="utf-8")
        assert not validate(root), validate(root)
        readme_ja.write_text(
            "# Readme\n\n<!-- pair: overview -->\n## 概要\n\n```md\n<!-- pair: ignored -->\n## Ignored\n```\n",
            encoding="utf-8",
        )
        assert not validate(root), validate(root)
        readme_ja.write_text("# Readme\n\n<!-- pair: start -->\n## 概要\n", encoding="utf-8")
        errors = validate(root)
        assert any("same ordered" in error for error in errors), errors
        readme_ja.write_text("# Readme\n\n<!-- pair: overview -->\n## 概要\n\n## 追加\n", encoding="utf-8")
        errors = validate(root)
        assert any("every H2" in error for error in errors), errors
        readme.unlink()
        readme_ja.unlink()
        errors = validate(root)
        assert any("must both exist" in error for error in errors), errors
    print("OK: check-docs self-test")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        return self_test()
    # Parser regressions must fail the real check, not only the opt-in flag.
    rc = self_test()
    if rc:
        return rc
    root = Path(__file__).resolve().parents[2]
    errors = validate(root)
    if errors:
        print("\n".join(errors))
        print(f"FAILED: Markdown document checks ({len(errors)} errors)")
        return 1
    print("OK: Markdown document checks")
    return 0


if __name__ == "__main__":
    sys.exit(main())
