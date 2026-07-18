# CLAUDE.md — guidance for AI coding agents

This repository is a **Klipper fork** for the **Prusa Core One+** (STM32F427 xBuddy +
STM32H503 xBuddy-extension). Start with **[FORK.md](FORK.md)** (what's in the fork, build &
flash) and **[CHANGES.md](CHANGES.md)** (GPLv3 §5 modification disclosure). User-facing
overview is in **[README.md](README.md)**.

## Hard rules

- **Never commit secrets.** Printer *configuration* (`printer.cfg`, `boards/xbuddy.cfg`,
  `h5/extension.cfg`) carries real MCU serials and lives host-side in
  **[coreone-host](https://github.com/packerlschupfer/coreone-host)**, **not** here. This fork
  stays secret-free.
- **Do not commit build artifacts.** Prebuilt H503 images embed a version string and would
  drift from source — build them with the `coreone/` tooling instead. `.venv/` and build
  output are ignored.
- **Do not re-introduce an overlay / apply-script.** Port work is committed directly on the
  release branch; the single-fork model is deliberate (see FORK.md → "Maintaining the fork").
- **Match upstream Klipper's code style** (C for MCU, Python for host `extras/`).

## Build / flash

Reproducible build & flash tooling lives in **`coreone/`** — F427 via `coreone/.config` + `make`
and `coreone/pack-bbf.sh`; H503 via `coreone/h5/`. Flashing (SWD / DFU / BBF / RS-485) via
`coreone/flash.sh`. See `coreone/docs/OWNERS_GUIDE.md`.

## Upstream

Base is pinned vanilla Klipper (see FORK.md). Upgrading = `git rebase` onto the new upstream
tag, then rebuild + reflash both MCUs from the new base (host↔MCU versions must agree).
