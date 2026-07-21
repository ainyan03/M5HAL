#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$SCRIPT_DIR/../../.." && pwd)"
M5UTILITY_ROOT="${M5UTILITY_ROOT:-$(cd "$REPO/.." && pwd)/M5Utility}"
CXX="${CXX:-c++}"

if [ ! -f "$M5UTILITY_ROOT/src/M5Utility.hpp" ]; then
    echo "variant selection check: M5Utility not found at $M5UTILITY_ROOT" >&2
    exit 1
fi

common_flags=(
    -std=c++17
    -I"$REPO/src"
    -I"$M5UTILITY_ROOT/src"
    -fsyntax-only
)

"$CXX" "${common_flags[@]}" "$SCRIPT_DIR/build_check_variant_default_fence.cpp"
"$CXX" "${common_flags[@]}" "$SCRIPT_DIR/build_check_variant_override.cpp"

tmp_dir="$(mktemp -d "${TMPDIR:-/tmp}/m5hal-variant-selection.XXXXXX")"
trap 'rm -rf "$tmp_dir"' EXIT
rejected_source="$SCRIPT_DIR/build_check_variant_rejected.cpp"

expect_rejected() {
    local kind="$1"
    local variant_id="$2"
    local case_name="$3"
    local log="$tmp_dir/${kind}_${case_name}.log"

    if "$CXX" "${common_flags[@]}" \
        "-DM5HAL_TEST_VARIANT_KIND_${kind}=1" \
        "-DM5HAL_TEST_VARIANT_ID=${variant_id}" \
        "$rejected_source" >"$log" 2>&1; then
        echo "variant selection check: $kind $case_name unexpectedly compiled" >&2
        exit 1
    fi

    if ! grep -Fq \
        "M5HAL_CONFIG_VARIANT_${kind} selects an unavailable variant or one that does not offer" \
        "$log"; then
        echo "variant selection check: $kind $case_name reported the wrong diagnostic" >&2
        tail -20 "$log" >&2
        exit 1
    fi
}

expect_typo_rejected() {
    local kind="$1"
    local typo_id=M5HAL_V2_VARIANT_ID_FRAMEWROK_SOFTWARE
    local log="$tmp_dir/${kind}_typo.log"

    if "$CXX" "${common_flags[@]}" \
        "-DM5HAL_TEST_VARIANT_KIND_${kind}=1" \
        "-DM5HAL_TEST_VARIANT_ID=${typo_id}" \
        "$rejected_source" >"$log" 2>&1; then
        echo "variant selection check: $kind typo unexpectedly compiled" >&2
        exit 1
    fi

    if ! grep -Fq "$typo_id" "$log"; then
        echo "variant selection check: $kind typo did not identify the undefined named ID" >&2
        tail -20 "$log" >&2
        exit 1
    fi
}

for kind in RUNTIME RUNTIME_MUTEX RUNTIME_TASK RUNTIME_EVENT GPIO I2C SPI I2S PDM UART; do
    expect_typo_rejected "$kind"
    expect_rejected "$kind" 65535 unknown
    case "$kind" in
        I2C|SPI)
            nonoffer_id=M5HAL_V2_VARIANT_ID_FRAMEWORK_POSIX
            ;;
        *)
            nonoffer_id=M5HAL_V2_VARIANT_ID_FRAMEWORK_SOFTWARE
            ;;
    esac
    expect_rejected "$kind" "$nonoffer_id" nonoffer

    case "$kind" in
        RUNTIME)
            unavailable_id=M5HAL_V2_VARIANT_ID_FRAMEWORK_ARDUINO
            ;;
        RUNTIME_MUTEX|RUNTIME_TASK|RUNTIME_EVENT)
            unavailable_id=M5HAL_V2_VARIANT_ID_FRAMEWORK_FREERTOS
            ;;
        GPIO)
            unavailable_id=M5HAL_V2_VARIANT_ID_FRAMEWORK_ARDUINO
            ;;
        *)
            unavailable_id=M5HAL_V2_VARIANT_ID_FRAMEWORK_REMOTE
            ;;
    esac
    expect_rejected "$kind" "$unavailable_id" unavailable
done

echo "variant selection check: PASS"
