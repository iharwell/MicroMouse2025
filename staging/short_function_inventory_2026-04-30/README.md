# Sub-3 Logical Line Function Inventory

This directory contains a repo-wide inventory of functions whose bodies are under 3 logical lines.

Definition used by these reports:
- C/C++/Arduino/PowerShell: fewer than 3 non-empty, non-comment body lines after removing brace-only lines.
- Python: fewer than 3 executable body statements after ignoring an initial docstring.

Scope notes:
- Included only tracked source-like files: `.c`, `.cc`, `.cpp`, `.h`, `.hh`, `.hpp`, `.ino`, `.inl`, `.ipp`, `.py`, and `.ps1`.
- Excluded generated, vendor, binary, and data-heavy trees such as `MazeMap/eigen-5.0.0/`, `codex_verify/arduino_libraries/Eigen/`, `codex_verify/arduino_build*/`, `codex_verify/isolated_release*/`, `MazeMap/MazeSimulation/x64/`, `tooling/__pycache__/`, `Maze Files/`, `DriveProgramFiles/`, `TestResults/`, and `staging/`.

Summary:
- Files scanned: 293
- Matching functions: 1431

Reports:
- [MazeMap core](C:/Users/thene/source/repos/MicroMouse2025/staging/short_function_inventory_2026-04-30/core.md)
  - Files scanned: 209
  - Matching functions: 1271
- [Tests, simulation, and tools](C:/Users/thene/source/repos/MicroMouse2025/staging/short_function_inventory_2026-04-30/tests-simulation-tools.md)
  - Files scanned: 46
  - Matching functions: 56
- [Tooling, scripts, and root Python](C:/Users/thene/source/repos/MicroMouse2025/staging/short_function_inventory_2026-04-30/tooling-scripts-root.md)
  - Files scanned: 23
  - Matching functions: 80
- [codex_verify](C:/Users/thene/source/repos/MicroMouse2025/staging/short_function_inventory_2026-04-30/codex-verify.md)
  - Files scanned: 15
  - Matching functions: 24
