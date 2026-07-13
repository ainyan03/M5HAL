#!/usr/bin/env python3
"""Keep public compile-time macro inputs and read-only outputs consistent."""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
CATALOG = ROOT / "spec/design/configuration.md"

INPUT_NAME_RE = re.compile(r"M5HAL_(?:CONFIG|DEBUG|EXAMPLE)_[A-Z0-9_]+")
DIRECTIVE_RE = re.compile(r"^\s*#\s*([a-zA-Z]+)\b(.*)$", re.DOTALL)
NAME_AT_START_RE = re.compile(
    r"^\s*(M5HAL_(?:CONFIG|DEBUG|EXAMPLE)_[A-Z0-9_]+)\b(.*)$", re.DOTALL
)
DEFINED_INPUT_RE = re.compile(
    r"\bdefined\s*(?:\(\s*)?(M5HAL_(?:CONFIG|DEBUG|EXAMPLE)_[A-Z0-9_]+)\b"
)

LEGACY_NAMES = {
    "M5HAL_I2C_MASTER_MAX_CLOCK_HZ",
    "M5HAL_ESPIDF_I2C_SLAVE_IRAM_ISR",
    "M5HAL_REMOTE_TCP_MAX_CONN",
    "M5HAL_I2C_SLAVE_GPIO_MARKERS",
    "M5HAL_I2C_SLAVE_MARK_TXFILL",
    "M5HAL_I2C_SLAVE_MARK_RXDRAIN",
    "M5HAL_I2C_SLAVE_MARK_STRETCH",
    "M5HAL_I2C_SLAVE_NO_TX_WATERMARK",
    "M5HAL_I2C_SLAVE_NO_CONTROLLER_CLOCK",
    "M5HAL_CONFIG_SOFTWARE_I2C_YIELD_PROBE_SPINS",
    "M5HAL_DEBUG_SOFTWARE_I2C_NO_WAIT",
    "M5HAL_CONFIG_REMOTE",
    "M5HAL_CONFIG_IDF_I2C_LEGACY",
    "M5HAL_CONFIG_IDF_I2C_SLAVE_IRAM_ISR",
    "M5HAL_CONFIG_MEMORY_TEMP_BLOCK_SIZE",
    "M5HAL_DEBUG_IDF_I2C_SLAVE_GPIO_MARKERS",
    "M5HAL_DEBUG_IDF_I2C_SLAVE_MARK_TXFILL_PIN",
    "M5HAL_DEBUG_IDF_I2C_SLAVE_MARK_RXDRAIN_PIN",
    "M5HAL_DEBUG_IDF_I2C_SLAVE_MARK_STRETCH_PIN",
    "M5HAL_DEBUG_IDF_I2C_SLAVE_NO_TX_WATERMARK",
    "M5HAL_DEBUG_IDF_I2C_SLAVE_NO_CONTROLLER_CLOCK",
    "M5HAL_V2_TARGET_PLATFORM_VARIANT_ID",
    "M5HAL_V2_TARGET_PLATFORM_PATH",
    "M5HAL_PC_BUILD",
    "M5HAL_ESPIDF_HOST_HARNESS",
    "M5HAL_HOST_HARNESS_NO_STRETCH",
    "M5HAL_EXAMPLE_HOWTOUSEI2C_FREQ",
    "M5HAL_EXAMPLE_FORCE_SOFTWARE_I2C",
    "M5HAL_EXAMPLE_HOWTOUSESPI_FREQ",
    "M5HAL_EXAMPLE_HOWTOUSEUART_BAUD",
    "M5HAL_EXAMPLE_HOWTOUSEUARTECHO_BAUD",
    "M5HAL_EXPERIMENT_REMOTE_BAUD",
    "M5HAL_EXPERIMENT_I2C_SCL",
    "M5HAL_EXPERIMENT_I2C_SDA",
    "M5HAL_EXPERIMENT_SPI_CLK",
    "M5HAL_EXPERIMENT_SPI_MOSI",
    "M5HAL_EXPERIMENT_SPI_MISO",
    "M5HAL_EXPERIMENT_SPI_CS",
    "M5HAL_EXPERIMENT_SKIP_STATIC_I2C",
    "M5HAL_EXPERIMENT_SKIP_SPI",
    "M5HAL_HIL_ECHO_BAUD",
    "M5HAL_TEST_ESPIDF_SPI_FREQ",
    "M5HAL_TEST_ESPIDF_SPI_PIN_CLK",
    "M5HAL_TEST_SOFTWARE_SPI_FREQ",
    "M5HAL_TEST_SOFTWARE_SPI_PIN_CLK",
    "M5HAL_TEST_SOFTWARE_SPI_CAPTURE_CLK",
    "M5HAL_TEST_SOFTWARE_SPI_CAPTURE_MOSI",
    "M5HAL_TEST_SOFTWARE_SPI_CAPTURE_DC",
    "M5HAL_TEST_SOFTWARE_SPI_CAPTURE_CS",
    "M5HAL_USERAPP_PUB_UART",
    "M5HAL_USERAPP_PUB_SPI",
    "M5HAL_USERAPP_PUB_I2S",
    "M5HAL_USERAPP_I2C_PORTA",
    "M5HAL_BUS_CONSOLE_HAS_USB_CDC",
    "M5HAL_REMOTE_TCP_DEFINED",
    "M5HAL_REMOTE_UART_DEFINED",
    "M5HAL_I2C_HAS_HW_BACKEND",
    "M5HAL_SPI_HAS_HW_BACKEND",
    "M5HAL_I2C_SLAVE_ISR_IRAM",
    "M5HAL_I2C_SLAVE_ISR_INTR_FLAGS",
    "M5HAL_MARK_HI",
    "M5HAL_MARK_LO",
    "M5HAL_I2C_SLAVE_TX_WM",
    "M5HAL_ESPIDF_I2C_HAVE_GEN4",
    "M5HAL_ESPIDF_I2C_HAVE_GEN5",
    "M5HAL_HPP",
}

RESERVED_OUTPUT_GUARDS = {
    "src/M5HAL_v2.hpp": {
        "M5HAL_V2_SELECTED_VARIANT_GPIO",
        "M5HAL_V2_SELECTED_VARIANT_I2C",
        "M5HAL_V2_SELECTED_VARIANT_SPI",
        "M5HAL_V2_SELECTED_VARIANT_I2S",
        "M5HAL_V2_SELECTED_VARIANT_UART",
        "M5HAL_V2_SELECTED_VARIANT_RUNTIME",
        "M5HAL_V2_SELECTED_VARIANT_RUNTIME_MUTEX",
        "M5HAL_V2_SELECTED_VARIANT_RUNTIME_TASK",
        "M5HAL_V2_SELECTED_VARIANT_RUNTIME_EVENT",
        "M5HAL_V2_TARGET_IS_PC",
    },
    "src/m5_hal/variants/platforms/_checker.hpp": {
        "M5HAL_V2_DETECTED_PLATFORM_VARIANT_ID",
        "M5HAL_V2_DETECTED_PLATFORM_VARIANT_PATH",
    },
}


def section(text: str, heading: str) -> str:
    marker = f"## {heading}"
    lines = text.splitlines()
    try:
        start = next(i for i, line in enumerate(lines) if line.strip() == marker)
    except StopIteration as exc:
        raise ValueError(f"catalog heading not found: {marker}") from exc
    end = next(
        (i for i in range(start + 1, len(lines)) if lines[i].startswith("## ")),
        len(lines),
    )
    return "\n".join(lines[start + 1 : end])


def catalog_names(text: str, heading: str, prefix: str) -> set[str]:
    pattern = re.compile(rf"^`({re.escape(prefix)}[A-Z0-9_]+)`$")
    names: set[str] = set()
    for line in section(text, heading).splitlines():
        if not line.startswith("|"):
            continue
        first_cell = line.split("|", 2)[1].strip()
        if match := pattern.fullmatch(first_cell):
            names.add(match.group(1))
    return names


def logical_directives(text: str) -> list[tuple[int, str, str]]:
    """Return (physical start line, directive, body), joining backslash lines."""
    result: list[tuple[int, str, str]] = []
    lines = text.splitlines()
    index = 0
    while index < len(lines):
        start = index + 1
        logical = lines[index]
        while logical.rstrip().endswith("\\") and index + 1 < len(lines):
            logical = logical.rstrip()[:-1] + " " + lines[index + 1].lstrip()
            index += 1
        if match := DIRECTIVE_RE.match(logical):
            result.append((start, match.group(1).lower(), match.group(2)))
        index += 1
    return result


def source_defaults() -> tuple[set[str], list[str]]:
    """Collect guarded defaults and reject definedness-based public inputs."""
    names: set[str] = set()
    errors: list[str] = []
    for source_root in (ROOT / "src", ROOT / "examples"):
        for path in source_root.rglob("*"):
            if not path.is_file():
                continue
            relative = path.relative_to(ROOT)
            text = path.read_text(encoding="utf-8", errors="ignore")
            stack: list[dict[str, object]] = []
            for line, directive, body in logical_directives(text):
                name_match = NAME_AT_START_RE.match(body)
                name = name_match.group(1) if name_match else None

                if directive == "ifdef" and name and INPUT_NAME_RE.fullmatch(name):
                    errors.append(f"{relative}:{line}: {name} must be read by value")
                elif directive in {"if", "elif"}:
                    for match in DEFINED_INPUT_RE.finditer(body):
                        errors.append(
                            f"{relative}:{line}: {match.group(1)} must be read by value"
                        )

                if directive in {"if", "ifdef", "ifndef"}:
                    is_input_guard = bool(
                        directive == "ifndef" and name and INPUT_NAME_RE.fullmatch(name)
                    )
                    stack.append(
                        {
                            "guard_name": name if is_input_guard else None,
                            "guard_line": line,
                            "has_define": False,
                            "primary_branch": True,
                        }
                    )
                    continue

                if directive in {"else", "elif"}:
                    if stack:
                        stack[-1]["primary_branch"] = False
                    continue

                if directive == "define" and name and INPUT_NAME_RE.fullmatch(name):
                    value = name_match.group(2).strip() if name_match else ""
                    guard = next(
                        (
                            frame
                            for frame in reversed(stack)
                            if frame["guard_name"] == name and frame["primary_branch"]
                        ),
                        None,
                    )
                    if guard is None:
                        errors.append(
                            f"{relative}:{line}: {name} default must be inside its #ifndef guard"
                        )
                    elif not value:
                        errors.append(f"{relative}:{line}: {name} default must have a value")
                    else:
                        guard["has_define"] = True
                        names.add(name)
                    continue

                if directive == "endif" and stack:
                    frame = stack.pop()
                    guard_name = frame["guard_name"]
                    if guard_name and not frame["has_define"]:
                        errors.append(
                            f"{relative}:{frame['guard_line']}: #ifndef {guard_name} has no guarded default"
                        )

            for frame in stack:
                guard_name = frame["guard_name"]
                if guard_name:
                    errors.append(
                        f"{relative}:{frame['guard_line']}: unterminated #ifndef {guard_name}"
                    )
    return names, errors


def tracked_and_untracked_files(root: Path) -> list[Path]:
    result = subprocess.run(
        ["git", "ls-files", "--cached", "--others", "--exclude-standard", "-z"],
        cwd=root,
        check=True,
        stdout=subprocess.PIPE,
    )
    return [root / entry.decode() for entry in result.stdout.split(b"\0") if entry]


def reserved_output_errors() -> list[str]:
    errors: list[str] = []
    for relative, names in RESERVED_OUTPUT_GUARDS.items():
        directives = logical_directives((ROOT / relative).read_text(encoding="utf-8"))
        for name in sorted(names):
            guarded = False
            for index, (_, directive, body) in enumerate(directives):
                if directive not in {"if", "ifdef", "ifndef"}:
                    continue
                has_name = bool(
                    re.search(rf"\bdefined\s*(?:\(\s*)?{re.escape(name)}\b", body)
                    or (
                        directive in {"ifdef", "ifndef"}
                        and re.match(rf"\s*{re.escape(name)}\b", body)
                    )
                )
                if not has_name:
                    continue
                depth = 1
                for _, nested_directive, _ in directives[index + 1 :]:
                    if nested_directive in {"if", "ifdef", "ifndef"}:
                        depth += 1
                    elif nested_directive == "endif":
                        depth -= 1
                        if depth == 0:
                            break
                    elif nested_directive == "error" and depth > 0:
                        guarded = True
                        break
                if guarded:
                    break
            if not guarded:
                errors.append(
                    f"{relative}: read-only output {name} lacks a #error reservation guard"
                )
    return errors


def report_set_difference(label: str, expected: set[str], actual: set[str]) -> bool:
    ok = True
    missing = sorted(expected - actual)
    extra = sorted(actual - expected)
    if missing:
        print(f"FAILED: {label}: catalog-only names: {', '.join(missing)}")
        ok = False
    if extra:
        print(f"FAILED: {label}: source-only names: {', '.join(extra)}")
        ok = False
    return ok


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--extra-git-root",
        action="append",
        default=[],
        type=Path,
        help="also reject legacy names in tracked/untracked files under this git worktree",
    )
    args = parser.parse_args()
    catalog_text = CATALOG.read_text(encoding="utf-8")
    catalog_config = catalog_names(catalog_text, "ノブ一覧", "M5HAL_CONFIG_")
    catalog_debug = catalog_names(catalog_text, "デバッグ診断 (`M5HAL_DEBUG_*`)", "M5HAL_DEBUG_")
    source_names, source_errors = source_defaults()
    source_config = {name for name in source_names if name.startswith("M5HAL_CONFIG_")}
    source_debug = {name for name in source_names if name.startswith("M5HAL_DEBUG_")}

    ok = report_set_difference("M5HAL_CONFIG_*", catalog_config, source_config)
    ok = report_set_difference("M5HAL_DEBUG_*", catalog_debug, source_debug) and ok

    for error in source_errors:
        print(f"FAILED: {error}")
        ok = False

    for error in reserved_output_errors():
        print(f"FAILED: {error}")
        ok = False

    self_path = Path(__file__).resolve()
    self_lines = self_path.read_text(encoding="utf-8").splitlines()
    legacy_declaration_start = next(
        index for index, line in enumerate(self_lines, 1) if line == "LEGACY_NAMES = {"
    )
    legacy_declaration_end = next(
        index
        for index, line in enumerate(self_lines[legacy_declaration_start:], legacy_declaration_start + 1)
        if line == "}"
    )
    scan_roots = [ROOT, *(root.resolve() for root in args.extra_git_root)]
    for scan_root in scan_roots:
        for path in tracked_and_untracked_files(scan_root):
            if (
                not path.is_file()
                or path.suffix == ".pyc"
                or "__pycache__" in path.parts
            ):
                continue
            text = path.read_text(encoding="utf-8", errors="ignore")
            for name in sorted(LEGACY_NAMES):
                token = re.compile(
                    rf"(?:(?<=[-/]D)|(?<![A-Z0-9_])){re.escape(name)}(?![A-Z0-9_])"
                )
                if not token.search(text):
                    continue
                for line_no, line in enumerate(text.splitlines(), 1):
                    if (
                        path.resolve() == self_path
                        and legacy_declaration_start <= line_no <= legacy_declaration_end
                    ):
                        continue
                    if token.search(line):
                        print(
                            f"FAILED: {path.relative_to(scan_root)}:{line_no}: legacy macro {name}"
                        )
                        ok = False

    if not ok:
        return 1
    print(
        f"OK: config macro catalog ({len(source_config)} supported, "
        f"{len(source_debug)} debug; no legacy names)"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
