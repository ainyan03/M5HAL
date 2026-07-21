#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$SCRIPT_DIR/../../.." && pwd)"

failed=0
while IFS= read -r file; do
    # Strip comments so public documentation examples do not mask the
    # production invariant. Scan the entire v2 implementation, including the
    # files that own the default facade; allow only the exact public singleton
    # declarations/definitions and the two centralized compatibility seams.
    relative=${file#"$REPO/"}
    allowed='a^'
    expected=0
    case "$relative" in
        src/M5HAL_v2.cpp)
            allowed='^[[:space:]]*(static const LocalResourceContext resources = getM5_Hal\(\)\.resourceDomain\(\)\.localResources\(\);|Hal& M5_Hal = getM5_Hal\(\);)[[:space:]]*$'
            expected=2
            ;;
        src/m5_hal/hal/v2/m5_hal.hpp)
            allowed='^[[:space:]]*(friend Hal& getM5_Hal\(\);|inline Hal& getM5_Hal\(\)|extern Hal& M5_Hal;)[[:space:]]*$'
            expected=3
            ;;
        src/m5_hal/hal/v2/memory/allocator.inl)
            allowed='^[[:space:]]*return getM5_Hal\(\)\.Memory;[[:space:]]*$'
            expected=1
            ;;
    esac

    hits=$(perl -0777 -pe 's{//.*?$}{}gm; s{/\*.*?\*/}{}gs' "$file" | grep -E '\b(M5_Hal|getM5_Hal)\b' || true)
    actual=$(printf '%s\n' "$hits" | awk 'NF { ++count } END { print count + 0 }')
    unexpected=$(printf '%s\n' "$hits" | grep -Ev "^[[:space:]]*$|$allowed" || true)
    if [ "$actual" -ne "$expected" ] || [ -n "$unexpected" ]; then
        echo "resource-domain global fence: unexpected default Hal reference: $file" >&2
        if [ -n "$unexpected" ]; then
            printf '%s\n' "$unexpected" >&2
        fi
        echo "resource-domain global fence: expected $expected allowed occurrence(s), found $actual" >&2
        failed=1
    fi
done < <(
    printf '%s\n' "$REPO/src/M5HAL_v2.cpp"
    find "$REPO/src/m5_hal/variants" "$REPO/src/m5_hal/hal/v2" \
        -type f \( -name '*.cpp' -o -name '*.hpp' -o -name '*.inl' \)
)

if [ "$failed" -ne 0 ]; then
    exit 1
fi
echo "resource-domain global fence: PASS"
