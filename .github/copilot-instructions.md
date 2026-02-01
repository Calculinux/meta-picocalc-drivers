# Copilot Instructions for picocalc-drivers

## Repository Overview

This repository contains out-of-tree Linux kernel driver sources for PicoCalc hardware and the authoritative device tree sources used by both the kernel and U-Boot builds in Calculinux.

## Device Tree Source-of-Truth

Device tree files live in this repository and use **prefixed names** to make their purpose explicit:

- **Linux**: `linux-rk3506-luckfox-lyra.dtsi`, `linux-rk3506g-luckfox-lyra.dts`
- **U-Boot**: `uboot-rk3506-luckfox.dtsi`, `uboot-rk3506-luckfox.dts`

The Yocto recipe `picocalc-devicetree` installs these into the sysroot using the **historical filenames** (no prefixes), so downstream kernel/U-Boot recipes continue to reference:

- `rk3506-luckfox-lyra.dtsi`
- `rk3506g-luckfox-lyra.dts`
- `rk3506-luckfox.dtsi`
- `rk3506-luckfox.dts`

Do **not** add duplicate copies of these files in the meta layers. Always update the versions here and let the recipe propagate them.

## Contribution Notes

- Keep kernel driver subdirectories isolated (one driver per directory).
- Prefer kernel build system conventions (Kbuild/Makefile with `modules`/`modules_install`).
- Maintain correct SPDX license headers in new files.
