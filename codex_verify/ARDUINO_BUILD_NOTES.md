# Arduino Build Notes

The repo no longer vendors Eigen under `MazeMap/MazeMap/Eigen`.
The shared Eigen checkout now lives at:

- `MazeMap/eigen-5.0.0`

For Arduino / Teensy builds, use the repo helper:

- `codex_verify/build_and_verify_latest.ps1`

The `.cmd` wrapper at `codex_verify/build_and_verify_latest.cmd` just launches that PowerShell helper.

The helper keeps the fix repo-local. It does not modify the installed Teensy core under `AppData`.

## Required Setup

- Install Arduino CLI, or install Arduino IDE and use its bundled `arduino-cli.exe`.
- Install the PJRC Teensy board package for `teensy:avr:teensy41`.
- If the Teensy package is not already configured, add the PJRC Boards Manager URL:
  `https://www.pjrc.com/teensy/package_teensy_index.json`
- Keep the shared Eigen tree at `MazeMap/eigen-5.0.0`.

## How The Helper Supplies Eigen To Arduino

The shared Eigen checkout is not kept under `MazeMap/MazeMap/libraries`, so the helper stages a repo-local Arduino library at runtime under:

- `codex_verify/arduino_libraries/Eigen`

That staged library exposes:

- `src/Eigen` -> junction to `MazeMap/eigen-5.0.0/Eigen`
- `src/Eigen.h` -> tiny entry header so Arduino CLI can resolve the library during include scanning

Arduino CLI then sees Eigen through its normal `--libraries` discovery path, so no Teensy platform files or global Arduino libraries need to be edited.

For Arduino builds, `MazeMap/MazeMap/EigenCompat.h` also applies two repo-side compatibility rules before including Eigen:

- it strips the Teensy `F`, `B1`, `B2`, and `B3` macros while Eigen headers are parsed,
- it enables a smaller Arduino-only Eigen configuration (`EIGEN_DONT_VECTORIZE`, `EIGEN_DONT_PARALLELIZE`, `EIGEN_NO_DEBUG`, `EIGEN_UNROLLING_LIMIT=0`) so the firmware fits Teensy 4.1 RAM1.

## Current Arduino Status

The required build path is:

- `codex_verify/build_and_verify_latest.cmd --no-pause`

That wrapper runs the firmware build first, then the Release host rebuild, then the Release unit tests.
The firmware build forces the Teensy 4.1 `Optimize` board option to `Faster with LTO` (`opt=o2lto`), which maps to `-O2` plus link-time optimization.
The firmware build now compiles directly into the canonical repo-local output directory under `codex_verify/arduino_build/firmware` instead of staging the compile in a separate work directory first.

The verified firmware artifacts are emitted to the canonical upload location:

- `codex_verify/arduino_build/firmware`

Before compiling, the helper deletes only `codex_verify/arduino_build/firmware/MazeMap.ino.hex` so the upload path cannot reuse a stale firmware image if the new build fails.
