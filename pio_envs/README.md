# PlatformIO env layout

This directory is the registry for where M5HAL's PlatformIO environments live
and how they are selected. Exact environment definitions, boards, frameworks,
flags, and pinned toolchains remain authoritative in the referenced config
files.

## Loading policy

[`../platformio.ini`](../platformio.ini) always defines the common bases and
native test environments. It also loads `pio_envs/v0/*.ini` and
`pio_envs/v2/*.ini`, which keeps public examples visible in the PlatformIO GUI.

Verification and HIL groups use the `.ini.cli` suffix, so the default glob does
not load them. Select one for a command with `M5HAL_PIO_EXTRA_CONFIG`; no config
file needs to be copied or generated.

```sh
M5HAL_PIO_EXTRA_CONFIG=pio_envs/v2/check.ini.cli \
  pio run -e v2_check_native
M5HAL_PIO_EXTRA_CONFIG='pio_envs/*/check.ini.cli' \
  pio run -e v0_check_native -e v2_check_native
```

## Env groups

| Config | Visibility | Purpose |
|---|---|---|
| [`../platformio.ini`](../platformio.ini) | always loaded | common bases; native unit, sanitizer, and ESP-IDF fake tests |
| [`v2/examples.ini`](v2/examples.ini) | GUI/default | public examples and remote example programs |
| [`v0/check.ini.cli`](v0/check.ini.cli) | on demand | v0 header and target compatibility fences |
| [`v0v2/check.ini.cli`](v0v2/check.ini.cli) | on demand | v0/v2 same-TU coexistence fences |
| [`v2/check.ini.cli`](v2/check.ini.cli) | on demand | v2 public API and target compatibility fences |
| [`v2/test.ini.cli`](v2/test.ini.cli) | on demand | embedded self-tests |
| [`v2/hil.ini.cli`](v2/hil.ini.cli) | on demand | HIL device, master, and host programs |

The config files are the sole inventory of individual env names. CI selects
from these same files through `M5HAL_PIO_EXTRA_CONFIG`; the current matrices are
authoritative in [`.github/workflows/`](../.github/workflows/).

## Naming

Verification envs use a version and purpose prefix such as `v0_check_*`,
`v0v2_check_*`, `v2_check_*`, `v2_test_*`, and `v2_hil_*`. Public examples use
`HowToUse_*`, `RemoteServer_*`, and `RemoteTest_*` so they remain easy to find
in the GUI. Board and framework suffixes are defined by the owning config; do
not infer support from a similar env name.

## Choosing a group

- Use `platformio.ini` directly for host unit and fake-backend tests.
- Load `v2/check.ini.cli` (or the corresponding v0/coexistence group) for a
  compile fence.
- Load `v2/test.ini.cli` for a device self-test that determines its own result.
- Load `v2/hil.ini.cli` only with the fixture instructions under
  [`../test/v2/hil/`](../test/v2/hil/).
- Use `v2/examples.ini` for public examples; examples are not the authoritative
  compile-fence inventory.

Toolchain generations and platform pins are implementation details of
`platformio.ini` and the owning group. Keep related pins together there rather
than duplicating their values in documentation.
