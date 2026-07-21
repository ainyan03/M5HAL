#!/usr/bin/env bash
# Deterministic native libFuzzer lane for pure frame decoding and receive-only
# bytecode validation. Requires clang with ASan/UBSan support.
set -euo pipefail

repo_dir="$(cd "$(dirname "$0")/.." && pwd)"
utility_dir="$(cd "$repo_dir/../M5Utility" && pwd)"
build_dir="$repo_dir/.generated/fuzz"
runs="${FUZZ_RUNS:-2000}"
max_len="${FUZZ_MAX_LEN:-4096}"
compiler="${CXX:-clang++}"

mkdir -p "$build_dir/artifacts"

common_sources=(
    "$repo_dir/src/M5HAL_v2.cpp"
    "$utility_dir/src/m5_utility/button_status.cpp"
    "$utility_dir/src/m5_utility/base64.cpp"
    "$utility_dir/src/m5_utility/string.cpp"
    "$utility_dir/src/m5_utility/murmurhash3.cpp"
    "$utility_dir/src/m5_utility/compatibility_feature.cpp"
)
common_flags=(
    -std=c++17 -O1 -g -fno-omit-frame-pointer
    -I"$repo_dir/src" -I"$utility_dir/src"
)

fuzzer_mode=1
probe="$build_dir/libfuzzer-probe"
if ! "$compiler" -std=c++17 -fsanitize=fuzzer -x c++ /dev/null -o "$probe" >/dev/null 2>&1; then
    fuzzer_mode=0
    echo "fuzz-test: libFuzzer runtime unavailable; replaying corpus with ASan+UBSan" >&2
fi

run_target() {
    local name="$1"
    local source="$2"
    local corpus="$repo_dir/test/v2/fuzz/corpus/$name"
    local artifacts="$build_dir/artifacts/$name"
    local binary="$build_dir/$name"
    mkdir -p "$artifacts"

    if [[ "$fuzzer_mode" == 0 ]]; then
        "$compiler" "${common_flags[@]}" -fsanitize=address,undefined "$source" \
            "$repo_dir/test/v2/fuzz/standalone_main.cpp" "${common_sources[@]}" -o "$binary"
        local seeds=()
        while IFS= read -r seed; do
            seeds+=("$seed")
        done < <(find "$corpus" -type f -print | sort)
        "$binary" "${seeds[@]}"
        return
    fi

    "$compiler" "${common_flags[@]}" -fsanitize=fuzzer,address,undefined "$source" \
        "${common_sources[@]}" -o "$binary"
    if ! "$binary" "$corpus" -runs="$runs" -max_len="$max_len" -timeout=5 -rss_limit_mb=2048 \
        -artifact_prefix="$artifacts/"; then
        local crash
        crash="$(find "$artifacts" -type f ! -name '*.minimized' -print | sort | tail -1)"
        if [[ -n "$crash" ]]; then
            "$binary" -minimize_crash=1 -runs=100000 -exact_artifact_path="$crash.minimized" "$crash" || true
            echo "fuzz-test: retained minimized reproducer at $crash.minimized" >&2
        fi
        return 1
    fi
}

run_target frame_codec "$repo_dir/test/v2/fuzz/fuzz_frame_codec.cpp"
run_target bytecode_receive "$repo_dir/test/v2/fuzz/fuzz_bytecode_receive.cpp"
