#!/usr/bin/env python3
"""Check public API spellings that are reproduced in specification snippets."""

from __future__ import annotations

import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def enum_members(text: str, enum_name: str) -> list[str]:
    match = re.search(rf"enum\s+class\s+{re.escape(enum_name)}\b[^{{]*{{([^}}]+)}}", text, re.DOTALL)
    if match is None:
        raise ValueError(f"enum class {enum_name} not found")
    members: list[str] = []
    for item in match.group(1).split(","):
        token = re.match(r"\s*([A-Za-z_][A-Za-z0-9_]*)", item)
        if token is not None:
            members.append(token.group(1))
    return members


def require(condition: bool, message: str, failures: list[str]) -> None:
    if not condition:
        failures.append(message)


def main() -> int:
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
        "virtual result_t<void> release(void);" in bus_spec,
        "bus_accessor.md must reproduce IBus::release as result_t<void>",
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

    if failures:
        for failure in failures:
            print(f"FAILED: {failure}", file=sys.stderr)
        return 1
    print("OK: public spec API spellings match declarations")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
