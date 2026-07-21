#!/usr/bin/env python3
"""Check public API spellings that are reproduced in specification snippets."""

from __future__ import annotations

import re
import sys
from pathlib import Path
from typing import Optional


ROOT = Path(__file__).resolve().parents[2]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def enum_members(text: str, enum_name: str) -> list[str]:
    match = re.search(rf"enum\s+class\s+{re.escape(enum_name)}\b[^{{]*{{([^}}]+)}}", text, re.DOTALL)
    if match is None:
        raise ValueError(f"enum class {enum_name} not found")
    members: list[str] = []
    # Strip comments before splitting: a trailing comment otherwise swallows
    # the following member (false mismatch on one side, masked drift on both).
    for item in strip_cpp_comments(match.group(1)).split(","):
        token = re.match(r"\s*([A-Za-z_][A-Za-z0-9_]*)", item)
        if token is not None:
            members.append(token.group(1))
    return members


def require(condition: bool, message: str, failures: list[str]) -> None:
    if not condition:
        failures.append(message)


def class_body(text: str, class_name: str) -> str:
    match = re.search(rf"\b(?:class|struct)\s+{re.escape(class_name)}\b[^{{]*{{", text)
    if match is None:
        raise ValueError(f"class {class_name} not found")
    start = match.end()
    depth = 1
    for index in range(start, len(text)):
        if text[index] == "{":
            depth += 1
        elif text[index] == "}":
            depth -= 1
            if depth == 0:
                return text[start:index]
    raise ValueError(f"class {class_name} has no closing brace")


def strip_cpp_comments(text: str) -> str:
    # Keep newlines so diagnostics and access-section ordering remain stable.
    text = re.sub(r"/\*.*?\*/", lambda match: "\n" * match.group(0).count("\n"), text, flags=re.DOTALL)
    return re.sub(r"//[^\n]*", "", text)


def member_access(body: str, member_name: str) -> Optional[str]:
    member = re.search(rf"\b{re.escape(member_name)}\s*\(", body)
    if member is None:
        return None
    access = "private"
    for label in re.finditer(r"(?m)^\s*(public|protected|private)\s*:", body[: member.start()]):
        access = label.group(1)
    return access


def has_member_declaration(body: str, member_name: str) -> bool:
    declaration = re.compile(
        rf"(?m)^\s*(?:(?:virtual|inline|constexpr|static)\s+)*"
        rf"[A-Za-z_]\w*(?:::\w+)*(?:<[^;\n{{}}()]+>)?(?:\s*[*&])?\s+"
        rf"{re.escape(member_name)}\s*\("
    )
    return declaration.search(body) is not None


def self_test() -> None:
    plain = "enum class E : int { A, B, C, D };"
    commented = "enum class E : int {\n  A,\n  B,  // trailing comment\n  C, /* block */ D,\n};"
    dropped = "enum class E : int {\n  A,\n  B,  // trailing comment\n  D,\n};"
    assert enum_members(plain, "E") == ["A", "B", "C", "D"]
    assert enum_members(commented, "E") == ["A", "B", "C", "D"]
    assert enum_members(dropped, "E") == ["A", "B", "D"]


def main() -> int:
    self_test()
    failures: list[str] = []

    enum_pairs = (
        (
            "memory::usage_t",
            "usage_t",
            "spec/design/memory.md",
            "src/m5_hal/hal/v2/memory/allocator.hpp",
        ),
        (
            "i2c::TxUnderrun",
            "TxUnderrun",
            "spec/design/i2c_slave.md",
            "src/m5_hal/hal/v2/i2c/slave.hpp",
        ),
    )
    for label, enum_name, spec_path, header_path in enum_pairs:
        try:
            documented = enum_members(read(spec_path), enum_name)
            declared = enum_members(read(header_path), enum_name)
        except ValueError as exc:
            failures.append(f"{label}: {exc}")
            continue
        require(
            documented == declared,
            f"{label}: spec members {documented} != declaration {declared}",
            failures,
        )

    bus_spec = read("spec/design/bus_accessor.md")
    require(
        "virtual bus::CloseOutcome closeBackend();" in bus_spec,
        "bus_accessor.md must reproduce the protected IBus::closeBackend hook",
        failures,
    )
    require(
        "virtual result_t<void> release(void);" not in bus_spec,
        "bus_accessor.md still documents IBus::release",
        failures,
    )
    require(
        "virtual error_t init(const IBusConfig&" not in bus_spec,
        "bus_accessor.md still documents the removed kind-generic IBus::init",
        failures,
    )

    server_header = read("src/m5_hal/hal/v2/remote/server.hpp")
    require("CHECK8-verified" in server_header, "remote::Server documentation must name CHECK8", failures)
    require("CHECK16" not in server_header, "remote::Server documentation still names CHECK16", failures)

    try:
        bus_body = class_body(strip_cpp_comments(read("src/m5_hal/hal/v2/bus/bus.hpp")), "IBus")
    except ValueError as exc:
        failures.append(str(exc))
    else:
        for obsolete_lock in ("lock", "unlock"):
            require(
                not has_member_declaration(bus_body, obsolete_lock),
                f"bus::IBus must not reintroduce obsolete raw {obsolete_lock} API",
                failures,
            )

    provider_hooks = (
        (
            "src/m5_hal/variants/frameworks/arduino/hal/i2c/i2c.hpp",
            "Bus_arduino",
            ("transferBackend", "waitTransferBackend"),
        ),
        (
            "src/m5_hal/variants/frameworks/espidf/hal/i2c/i2c.hpp",
            "Bus_espidf",
            ("transferBackend", "waitTransferBackend", "transferBusyBackend"),
        ),
        (
            "src/m5_hal/variants/frameworks/remote/hal/i2c/i2c.hpp",
            "Bus_remote",
            ("transferBackend", "waitTransferBackend", "transferBusyBackend"),
        ),
        (
            "src/m5_hal/variants/frameworks/software/hal/i2c/i2c.hpp",
            "Bus_software",
            ("transferBackend", "waitTransferBackend", "transferBusyBackend"),
        ),
    )
    for header_path, class_name, hooks in provider_hooks:
        try:
            body = class_body(strip_cpp_comments(read(header_path)), class_name)
        except ValueError as exc:
            failures.append(str(exc))
            continue
        for hook in hooks:
            access = member_access(body, hook)
            require(
                access == "protected",
                f"{class_name}::{hook} must remain a protected checked-facade hook (found {access})",
                failures,
            )

    if failures:
        for failure in failures:
            print(f"FAILED: {failure}", file=sys.stderr)
        return 1
    print("OK: public spec API spellings match declarations")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
