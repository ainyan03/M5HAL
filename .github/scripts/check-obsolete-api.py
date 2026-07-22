#!/usr/bin/env python3
"""Reject removed v2 API spellings outside explicit migration/native allowlists."""

from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path
from typing import Optional


ROOT = Path(__file__).resolve().parents[2]
SCAN_PREFIXES = ("src/", "spec/", "examples/", "test/")
ROOT_DOCS = {"README.md", "README.ja.md"}
SOURCE_SUFFIXES = {".c", ".cc", ".cpp", ".h", ".hpp", ".inl", ".ino", ".md"}

RULES = {
    "transaction_lifecycle": re.compile(r"\b(?:beginTransaction|endTransaction)\b"),
    "scoped_transaction": re.compile(r"\bScopedTransaction\b"),
    "scoped_lock": re.compile(r"\bScopedLock\b"),
    "legacy_transaction_window": re.compile(r"\blegacy_transaction_window\b"),
    "connection_query": re.compile(r"\bisConnected\b"),
    "identity_key": re.compile(r"\bIdentityKey\b"),
    "backend_for": re.compile(r"\bBackendFor\b"),
    "remote_typed_config": re.compile(r"\b(?:RemoteTypedConfigTraits|variantFieldsAreDefault)\b"),
    "typed_registry": re.compile(r"\b(?:acquireBusTyped|releaseBus)\b"),
    "variant_bus_config": re.compile(r"\bBusConfig_[A-Za-z][A-Za-z0-9_]*\b"),
    "transaction_completion": re.compile(r"\btransactionComplete\b"),
    "native_acquire_alias": re.compile(
        r"\b(?:acquireNative|acquireNativeBorrowed|acquireNativeManaged|attachNative|openNative)\b"
    ),
    "ambiguous_connection_query": re.compile(r"\bhasConnection\b"),
    "raw_bus_lock": re.compile(r"\b(?:Bus|IBus)::(?:lock|unlock)\b|\block\(owner|`lock`\s*/\s*`unlock`"),
    "mutable_allocator_binding": re.compile(r"\bsetAllocator\b"),
    "mutable_fallback_family": re.compile(r"\bsetFallback\b"),
    "uart_specific_timeout_margin": re.compile(r"\bkRemoteUartTimeoutMarginMs\b"),
}

# These exact lines explain migration or call a framework-native API. Every
# other occurrence is a regression, including a suffix added to an allowed
# line or another occurrence elsewhere in the same file tree.
ALLOWLIST = {
    ("spec/style/legacy_v2_migration.md", "transaction_lifecycle"): frozenset(
        {
            "| Accessor `beginTransaction()` / `endTransaction()` | `beginAccess(timeout)` / `endAccess(timeout)` | lock、設定、backend開始終了、最後のI/O完了を一つのlifecycleへ統合 |",
            "| `endTransaction() -> TransferTotals` | 各`transfer`の戻り値 + `getLastTransferStatus()` | `endAccess`はlifecycle結果だけを返す |",
            "| Bus `beginTransaction(owner, cfg)` / public virtual `beginOperation(owner, context)` | non-virtual `beginOperation(context)` / `endOperation(context)` + protected `*Backend` hook | ContextはAccessor-ownedで外部構築・copy不能。raw ownerを渡さず、Config copyを各I/Oで作らない |",
            "| I2C slave `beginTransaction/endTransaction` | legacy `openWireFrame/closeWireFrame` | 外部masterが作ったframeのclaim/release。Access lifecycleではない |",
            "Arduino `SPIClass::beginTransaction/endTransaction`はframework native APIなので置換しない。",
            "dev.beginTransaction();",
            "auto totals = dev.endTransaction();",
            "1. Accessor旧`begin/endTransaction`を全削除し、明示scopeを`begin/endAccess`へ変更する",
        }
    ),
    ("spec/style/legacy_v2_migration.md", "scoped_transaction"): frozenset(
        {"| `spi::ScopedTransaction` | `bus::ScopedAccess` | SPI Accessが一つのCS frame |"}
    ),
    ("spec/style/legacy_v2_migration.md", "scoped_lock"): frozenset(
        {"| public `Bus::lock/unlock`, `ScopedLock` | Accessor内部lock seam | 利用者はAccessor lifecycleだけを使う |"}
    ),
    ("spec/style/legacy_v2_migration.md", "legacy_transaction_window"): frozenset(
        {"| `legacy_transaction_window` | `legacy_wire_frame_window` | 旧blocking stream受付を明示opt-in |"}
    ),
    ("spec/style/legacy_v2_migration.md", "transaction_completion"): frozenset(
        {"| I2C slave `transactionComplete()` | `wireFrameComplete()` | master STOP観測query |"}
    ),
    ("spec/style/legacy_v2_migration.md", "raw_bus_lock"): frozenset(
        {"| public `Bus::lock/unlock`, `ScopedLock` | Accessor内部lock seam | 利用者はAccessor lifecycleだけを使う |"}
    ),
    ("spec/style/migration.md", "transaction_lifecycle"): frozenset(
        {"旧v2 Bus取得API・`begin/endTransaction`から現行v2への移行は"}
    ),
    ("spec/style/legacy_v2_migration.md", "backend_for"): frozenset(
        {"| `BackendFor<Config>` | 通常は`Hal.<kind>.acquire(cfg)` | direct provider型が必要な場合だけ`Bus_<variant>::init(...)`をadvanced escape hatchとして使う |"}
    ),
    ("spec/design/bus_accessor.md", "scoped_lock"): frozenset(
        {"raw lock APIと`ScopedLock`は公開しない。Busの`transfer` / `waitTransfer`はbackend実装と"}
    ),
    ("spec/design/uart.md", "transaction_lifecycle"): frozenset(
        {"borrowし、inactiveなら一時 Access を開閉する。旧 `beginTransaction` / `endTransaction` と"}
    ),
    ("src/m5_hal/variants/frameworks/arduino/hal/spi/spi.inl", "transaction_lifecycle"): frozenset(
        {"    _spi->beginTransaction(", "        _spi->endTransaction();"}
    ),
}

# Pin the number of intentional lines as well as their exact spelling.
ALLOWLIST_COUNTS = {
    ("spec/style/legacy_v2_migration.md", "transaction_lifecycle"): 8,
    ("spec/style/legacy_v2_migration.md", "scoped_transaction"): 1,
    ("spec/style/legacy_v2_migration.md", "scoped_lock"): 1,
    ("spec/style/legacy_v2_migration.md", "legacy_transaction_window"): 1,
    ("spec/style/legacy_v2_migration.md", "transaction_completion"): 1,
    ("spec/style/legacy_v2_migration.md", "raw_bus_lock"): 1,
    ("spec/style/migration.md", "transaction_lifecycle"): 1,
    ("spec/style/legacy_v2_migration.md", "backend_for"): 1,
    ("spec/design/bus_accessor.md", "scoped_lock"): 1,
    ("spec/design/uart.md", "transaction_lifecycle"): 1,
    ("src/m5_hal/variants/frameworks/arduino/hal/spi/spi.inl", "transaction_lifecycle"): 2,
}

# A future allowlist edit must not append another obsolete token while keeping
# the number of exact lines unchanged.
ALLOWLIST_TOKEN_COUNTS = {
    ("spec/style/legacy_v2_migration.md", "transaction_lifecycle"): 11,
    ("spec/style/legacy_v2_migration.md", "scoped_transaction"): 1,
    ("spec/style/legacy_v2_migration.md", "scoped_lock"): 1,
    ("spec/style/legacy_v2_migration.md", "legacy_transaction_window"): 1,
    ("spec/style/legacy_v2_migration.md", "transaction_completion"): 1,
    ("spec/style/legacy_v2_migration.md", "raw_bus_lock"): 1,
    ("spec/style/migration.md", "transaction_lifecycle"): 1,
    ("spec/style/legacy_v2_migration.md", "backend_for"): 1,
    ("spec/design/bus_accessor.md", "scoped_lock"): 1,
    ("spec/design/uart.md", "transaction_lifecycle"): 2,
    ("src/m5_hal/variants/frameworks/arduino/hal/spi/spi.inl", "transaction_lifecycle"): 2,
}


def tracked_and_untracked_files() -> list[str]:
    output = subprocess.check_output(
        ["git", "ls-files", "--cached", "--others", "--exclude-standard", "-z"], cwd=ROOT
    )
    result: list[str] = []
    for encoded in output.split(b"\0"):
        if not encoded:
            continue
        relative = encoded.decode()
        path = Path(relative)
        if (not relative.startswith(SCAN_PREFIXES) and relative not in ROOT_DOCS) or path.suffix not in SOURCE_SUFFIXES:
            continue
        if relative.startswith("src/m5_hal/hal/v0/") or relative.startswith("test/v0/"):
            continue
        result.append(relative)
    return result


def matching_rule(line: str) -> Optional[str]:
    return next((name for name, pattern in RULES.items() if pattern.search(line) is not None), None)


def self_test() -> list[str]:
    failures: list[str] = []
    cases = (
        ("beginTransaction()", "transaction_lifecycle"),
        ("endTransaction()", "transaction_lifecycle"),
        ("ScopedTransaction", "scoped_transaction"),
        ("ScopedLock", "scoped_lock"),
        ("legacy_transaction_window", "legacy_transaction_window"),
        ("bus.isConnected()", "connection_query"),
        ("IdentityKey", "identity_key"),
        ("BackendFor<Config>", "backend_for"),
        ("RemoteTypedConfigTraits", "remote_typed_config"),
        ("variantFieldsAreDefault", "remote_typed_config"),
        ("acquireBusTyped", "typed_registry"),
        ("releaseBus", "typed_registry"),
        ("BusConfig_arduino", "variant_bus_config"),
        ("transactionComplete()", "transaction_completion"),
        ("acquireNative", "native_acquire_alias"),
        ("acquireNativeBorrowed", "native_acquire_alias"),
        ("acquireNativeManaged", "native_acquire_alias"),
        ("attachNative", "native_acquire_alias"),
        ("openNative", "native_acquire_alias"),
        ("hasConnection()", "ambiguous_connection_query"),
        ("IBus::lock", "raw_bus_lock"),
        ("lock(owner", "raw_bus_lock"),
        ("`lock` / `unlock`", "raw_bus_lock"),
        ("setAllocator", "mutable_allocator_binding"),
        ("setFallback", "mutable_fallback_family"),
        ("kRemoteUartTimeoutMarginMs", "uart_specific_timeout_margin"),
        ("makeHardwareBackendForI2C", None),
        ("BusBeginTransaction", None),
    )
    for text, expected in cases:
        actual = matching_rule(text)
        if actual != expected:
            failures.append(f"self-test token {text!r}: expected {expected}, got {actual}")
    covered_rules = {expected for _, expected in cases if expected is not None}
    if covered_rules != set(RULES):
        failures.append(f"self-test rule coverage mismatch: {sorted(covered_rules ^ set(RULES))}")
    if set(ALLOWLIST_COUNTS) != set(ALLOWLIST):
        failures.append("self-test: allowlist count keys do not match allowlist keys")
    if set(ALLOWLIST_TOKEN_COUNTS) != set(ALLOWLIST):
        failures.append("self-test: allowlist token-count keys do not match allowlist keys")
    allowed_key = ("src/m5_hal/variants/frameworks/arduino/hal/spi/spi.inl", "transaction_lifecycle")
    allowed_line = "    _spi->beginTransaction("
    if allowed_line not in ALLOWLIST[allowed_key]:
        failures.append("self-test: Arduino native transaction allowlist did not match")
    if f"{allowed_line} endTransaction();" in ALLOWLIST[allowed_key]:
        failures.append("self-test: allowlist accepted an obsolete suffix on an exact line")
    for key, allowed_lines in ALLOWLIST.items():
        if len(allowed_lines) != ALLOWLIST_COUNTS[key]:
            failures.append(f"self-test: exact allowlist line count mismatch: {key}")
        actual_tokens = 0
        for line in allowed_lines:
            occurrences = list(RULES[key[1]].finditer(line))
            if not occurrences:
                failures.append(f"self-test: exact allowlist line has no {key[1]} token: {key}: {line!r}")
            actual_tokens += len(occurrences)
        if actual_tokens != ALLOWLIST_TOKEN_COUNTS[key]:
            failures.append(
                f"self-test: exact allowlist token count mismatch: {key}: "
                f"expected {ALLOWLIST_TOKEN_COUNTS[key]}, got {actual_tokens}"
            )
    wrong_path = ("src/m5_hal/hal/v2/i2c/i2c.inl", "transaction_lifecycle")
    if wrong_path in ALLOWLIST:
        failures.append("self-test: obsolete transaction call is allowlisted outside Arduino SPI")
    return failures


def main() -> int:
    failures = self_test()
    allowed_hits: dict[tuple[str, str], int] = {}
    for relative in tracked_and_untracked_files():
        text = (ROOT / relative).read_text(encoding="utf-8", errors="replace")
        for line_number, line in enumerate(text.splitlines(), start=1):
            for rule_name, pattern in RULES.items():
                if pattern.search(line) is None:
                    continue
                key = (relative, rule_name)
                allowed_lines = ALLOWLIST.get(key)
                if allowed_lines is not None and line in allowed_lines:
                    allowed_hits[key] = allowed_hits.get(key, 0) + 1
                    continue
                failures.append(f"{relative}:{line_number}: {rule_name}: {line.strip()}")

    for key, expected_count in ALLOWLIST_COUNTS.items():
        actual_count = allowed_hits.get(key, 0)
        if actual_count != expected_count:
            failures.append(
                f"allowlist count mismatch: {key[0]} ({key[1]}): expected {expected_count}, got {actual_count}"
            )

    if failures:
        for failure in failures:
            print(f"FAILED: {failure}", file=sys.stderr)
        return 1
    print(
        f"OK: obsolete v2 API fence ({len(RULES)} rules, "
        f"{sum(allowed_hits.values())} intentional lines in {len(allowed_hits)} sites)"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
