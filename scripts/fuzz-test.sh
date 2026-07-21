#!/usr/bin/env bash
# Deterministic native libFuzzer lane for pure frame decoding and receive-only
# bytecode validation. Requires clang with ASan/UBSan support.
set -euo pipefail

repo_dir="$(cd "$(dirname "$0")/.." && pwd)"
utility_dir="$(cd "$repo_dir/../M5Utility" && pwd)"
build_dir="$repo_dir/.generated/fuzz"
runs="${FUZZ_RUNS:-2000}"
max_len="${FUZZ_MAX_LEN:-4096}"
require_libfuzzer="${FUZZ_REQUIRE_LIBFUZZER:-0}"

# Honor an explicit CXX as a hard toolchain selection. Otherwise pick the
# first compiler whose libFuzzer probe links. Apple clang ships no libFuzzer
# runtime, so Homebrew/system LLVM is tried after clang++; with no
# fuzzer-capable candidate the first one still drives REPLAY mode.
# The probe must define LLVMFuzzerTestOneInput: the runtime's own main
# references it, so linking an empty translation unit always fails.
probe_fuzzer() {
    printf 'extern "C" int LLVMFuzzerTestOneInput(const unsigned char*, __SIZE_TYPE__) { return 0; }\n' |
        "$1" -std=c++17 -fsanitize=fuzzer -x c++ - -o "$2" >/dev/null 2>&1
}
compiler=""
fuzzer_mode=0
probe="$(mktemp "${TMPDIR:-/tmp}/m5hal-libfuzzer-probe.XXXXXX")"
trap 'rm -f "$probe"' EXIT
if [[ -n "${CXX:-}" ]]; then
    compiler_candidates=("$CXX")
else
    compiler_candidates=(clang++ /opt/homebrew/opt/llvm/bin/clang++ /usr/local/opt/llvm/bin/clang++)
fi
for candidate in "${compiler_candidates[@]}"; do
    [[ -n "$candidate" ]] || continue
    command -v "$candidate" >/dev/null 2>&1 || continue
    [[ -n "$compiler" ]] || compiler="$candidate"
    if probe_fuzzer "$candidate" "$probe"; then
        compiler="$candidate"
        fuzzer_mode=1
        break
    fi
done
if [[ -z "$compiler" ]]; then
    echo "fuzz-test: no usable C++ compiler found" >&2
    exit 1
fi

case "$require_libfuzzer" in
    0|1) ;;
    *)
        echo "fuzz-test: FUZZ_REQUIRE_LIBFUZZER must be 0 or 1" >&2
        exit 2
        ;;
esac

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

if [[ "$fuzzer_mode" == 0 ]]; then
    if [[ "$require_libfuzzer" == 1 ]]; then
        echo "fuzz-test: no libFuzzer-capable compiler found and FUZZ_REQUIRE_LIBFUZZER=1" >&2
        exit 1
    fi
    echo "fuzz-test: REPLAY mode; libFuzzer runtime unavailable, replaying corpus with ASan+UBSan" >&2
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
        local seed_list="$build_dir/$name.seeds"
        local seeds=()
        if ! find "$corpus" -type f -print0 > "$seed_list"; then
            echo "fuzz-test: REPLAY failed to enumerate corpus for $name: $corpus" >&2
            return 1
        fi
        while IFS= read -r -d '' seed; do
            if [[ ! -r "$seed" ]]; then
                echo "fuzz-test: REPLAY corpus seed is not readable for $name: $seed" >&2
                return 1
            fi
            seeds+=("$seed")
        done < "$seed_list"
        if [[ "${#seeds[@]}" -eq 0 ]]; then
            echo "fuzz-test: REPLAY requires at least one corpus seed for $name: $corpus" >&2
            return 1
        fi
        echo "fuzz-test: REPLAY target=$name seeds=${#seeds[@]}" >&2
        "$binary" "${seeds[@]}"
        return
    fi

    "$compiler" "${common_flags[@]}" -fsanitize=fuzzer,address,undefined "$source" \
        "${common_sources[@]}" -o "$binary"
    # libFuzzer writes new inputs into the FIRST corpus dir; keep the tracked
    # corpus read-only (second position) so a gate run never dirties the tree.
    local work_corpus="$build_dir/corpus/$name"
    rm -rf "$work_corpus"
    mkdir -p "$work_corpus"
    if ! "$binary" "$work_corpus" "$corpus" -runs="$runs" -max_len="$max_len" -timeout=5 -rss_limit_mb=2048 \
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
