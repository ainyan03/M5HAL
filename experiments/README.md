# M5HAL experiments

`experiments/` contains hardware-in-the-loop (HIL) test code that
requires real wiring between two devices and cannot run as a standard
unit test.

User-facing examples live in `examples/`. Internal development
experiments have been retired (recoverable from git history).

## HIL tests

Source: `experiments/v2/test/`
PIO envs: `pio_envs/v2/hil.ini.cli` (CLI-only, not GUI-visible).
Runner: `experiments/v2/test/hil-run.sh`

See `pio_envs/v2/hil.ini.cli` for usage instructions.
