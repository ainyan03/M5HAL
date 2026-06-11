# PlatformIO env layout

PlatformIO GUI should show as few choices as possible while still exposing
the public examples. The default `platformio.ini` therefore loads the short
GUI-facing operation menu envs and the user example envs:

```bash
pio run -e v1_exp_menu_arduino
pio run -e v1_exp_menu_idf5
pio run -e v1_exp_bench_arduino
pio run -e v1_exp_remote_idf5
pio run -e v1_exp_remote_host
pio run -e HowToUse_I2C_esp32
pio run -e HowToUse_SPI_esp32
pio run -e HowToUse_UART_esp32
pio run -e HowToUse_UARTEcho_esp32
pio run -e HowToUse_Bytecode_esp32
```

The remaining check/test/advanced experiment envs are CLI/CI-only
verification builds. They live in
`*.ini.cli` files. The `.cli` suffix means PlatformIO's `extra_configs`
glob (`pio_envs/**/*.ini`) does not pick them up, so they never appear in
the GUI. Load one on demand (locally or in CI) by pointing the
`M5HAL_PIO_EXTRA_CONFIG` env var at the file or a glob — `platformio.ini`
adds `${sysenv.M5HAL_PIO_EXTRA_CONFIG}` to `extra_configs`, so there is no
copying and nothing to clean up. Unset, it expands to empty and is ignored.

## GUI-visible env groups (`*.ini`)

- `v1/experiments.ini`: short M5Stack BASIC operation / benchmark menus and the remote-bus harness (device + PC menu)
- `v1/examples.ini`: public v1 examples for users

## CLI-only env groups (`*.ini.cli`)

- `v0/check.ini.cli`: v0 header/build compatibility checks
- `v0v1/check.ini.cli`: v0+v1 same-TU coexistence checks on device targets
- `v1/check.ini.cli`: v1 header/build compatibility and shared API-surface checks
- `v1/test.ini.cli`: native and embedded test envs
- `v1/experiments_advanced.ini.cli`: focused benchmarks, logic-analyzer sketches, backend smoke tests, and the WiFi/TCP variants of the remote-bus harness (`v1_exp_remote_tcp_*`, kept out of the GUI because they expect WiFi credentials)

## Enabling on demand

Point `M5HAL_PIO_EXTRA_CONFIG` at the `.ini.cli` file (or a glob) for the run.
No copying, nothing to remove.

```bash
M5HAL_PIO_EXTRA_CONFIG=pio_envs/v1/check.ini.cli pio run -e v1_check_native
# a glob loads several at once:
M5HAL_PIO_EXTRA_CONFIG='pio_envs/*/check.ini.cli' pio run -e v0_check_native -e v1_check_native
```

CI sets the same env var (as a step `env:`) before running CLI-only envs. See
the workflow files under `.github/workflows/` for which envs each job builds.

The v1 check envs use `test/v1/build_check/build_check.hpp` as the common
compile fence. Public examples stay focused on readable Arduino IDE sketches;
the build-check header is allowed to be denser and to touch many I2C / SPI /
UART API overloads in one place. The same header is also called by a native
gtest so CI stubs and unit tests do not drift apart.

## ESP-IDF version probes

The default ESP32 PlatformIO base is pinned to `espressif32@6.12.0`
(ESP-IDF 5.x generation) so ordinary check/example envs are reproducible.
The check env files also expose explicit ESP-IDF major-version probes:

- `*_espidf4`: `espressif32@5.4.0`, legacy ESP-IDF driver compatibility
- `*_espidf`: default IDF 5.x stable check
- `*_espidf6`: `espressif32@7.0.1`, forward-compatibility probe

### Toolchain coexistence in a shared core_dir

IDF5 (GCC14) and IDF6 (GCC15) both ship the `toolchain-xtensa-esp-elf`
package, so in a shared `core_dir` the last-installed generation takes the
unversioned "active" slot and the espidf cmake `tool_version_check` of the
other generation then fails. To keep both switchable from one `core_dir`,
`common_esp32_espidf5` / `common_esp32_espidf6` pin their exact toolchain
via `platform_packages` (`@14.2.0+20241119` / `@15.2.0+20251204`). Keep
those pins in sync with the platform pins. IDF4 uses a different package
(`toolchain-xtensa-esp32`) and the arduino framework does not run the
strict check, so neither needs a pin. (CI runners are per-job isolated, so
this only matters for local multi-generation builds.)

There are no generated temporary `.ini` files anymore (the env var loads the
`.ini.cli` directly). To make an env group permanently GUI-visible, rename its
`.ini.cli` to `.ini` so the `extra_configs` glob picks it up. Public examples
are intentionally committed as `v1/examples.ini`.
