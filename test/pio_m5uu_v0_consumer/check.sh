#!/usr/bin/env bash
# Build the exact pinned M5UnitUnified consumer and prove source provenance.
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
M5HAL_ROOT="$(cd "$PROJECT_DIR/../.." && pwd)"
M5STACK_DIR="$(cd "$M5HAL_ROOT/.." && pwd)"
source "$PROJECT_DIR/pins.env"

verify_checkout() {
    local name="$1" expected="$2" root="$M5STACK_DIR/$1"
    local actual
    actual="$(git -C "$root" rev-parse HEAD)"
    if [ "$actual" != "$expected" ]; then
        echo "FAILED: $name pin mismatch: expected $expected, got $actual" >&2
        return 1
    fi
    if [ -n "$(git -C "$root" status --porcelain --untracked-files=all)" ]; then
        echo "FAILED: $name checkout is dirty" >&2
        return 1
    fi
}

verify_checkout M5Utility "$M5UTILITY_COMMIT"
verify_checkout M5UnitUnified "$M5UNITUNIFIED_COMMIT"

export PLATFORMIO_WORKSPACE_DIR="${PLATFORMIO_WORKSPACE_DIR:-$PROJECT_DIR/.pio}"
# Provenance is accepted only from this invocation. A persistent local-ci
# workspace may contain dependency records from an older source graph, so
# remove the selected environment's build artifacts before compiling.
pio run -d "$PROJECT_DIR" -e m5uu_v0_consumer -t clean
pio run -d "$PROJECT_DIR" -e m5uu_v0_consumer

build_dir="$PLATFORMIO_WORKSPACE_DIR/build/m5uu_v0_consumer"
for name in M5HAL M5Utility M5UnitUnified; do
    source_root="$M5STACK_DIR/$name/src/"
    if ! grep -rlF --include='*.d' "$source_root" "$build_dir" >/dev/null; then
        echo "FAILED: dependency records do not prove local $name source consumption" >&2
        exit 1
    fi
done

if grep -rnE --include='*.d' '/libdeps/[^/]*/(M5HAL|M5Utility|M5UnitUnified)/src/' "$build_dir"; then
    echo "FAILED: dependency records contain a registry/library-cache source copy" >&2
    exit 1
fi

verify_checkout M5Utility "$M5UTILITY_COMMIT"
verify_checkout M5UnitUnified "$M5UNITUNIFIED_COMMIT"

echo "OK: pinned M5UnitUnified consumes local M5HAL v0 and M5Utility sources"
