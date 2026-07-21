#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$SCRIPT_DIR/../../.." && pwd)"

file_list="$(mktemp "${TMPDIR:-/tmp}/m5hal-resource-domain-files.XXXXXX")"
root_list="$(mktemp "${TMPDIR:-/tmp}/m5hal-resource-domain-root.XXXXXX")"
trap 'rm -f "$file_list" "$root_list"' EXIT

fixed_file="$REPO/src/M5HAL_v2.cpp"
if [ ! -f "$fixed_file" ]; then
    echo "resource-domain global fence: required file is missing: $fixed_file" >&2
    exit 1
fi
printf '%s\0' "$fixed_file" > "$file_list"

for root in "$REPO/src/m5_hal/variants" "$REPO/src/m5_hal/hal/v2"; do
    if [ ! -d "$root" ]; then
        echo "resource-domain global fence: required scan root is missing: $root" >&2
        exit 1
    fi
    if ! find "$root" -type f \( -name '*.cpp' -o -name '*.hpp' -o -name '*.inl' \) \
        -print0 > "$root_list"; then
        echo "resource-domain global fence: failed to enumerate scan root: $root" >&2
        exit 1
    fi
    if [ ! -s "$root_list" ]; then
        echo "resource-domain global fence: scan root has no matching source files: $root" >&2
        exit 1
    fi
    cat "$root_list" >> "$file_list"
done

failed=0
while IFS= read -r -d '' file; do
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

    if ! stripped=$(perl -0777 -pe 's{//.*?$}{}gm; s{/\*.*?\*/}{}gs' "$file"); then
        echo "resource-domain global fence: failed to read source file: $file" >&2
        failed=1
        continue
    fi
    hits=$(printf '%s\n' "$stripped" | grep -E '\b(M5_Hal|getM5_Hal)\b' || true)
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
done < "$file_list"

if [ "$failed" -ne 0 ]; then
    exit 1
fi
echo "resource-domain global fence: PASS"
