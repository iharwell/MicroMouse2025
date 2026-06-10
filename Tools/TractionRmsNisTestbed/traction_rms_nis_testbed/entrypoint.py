"""Unified command dispatch for the standalone traction RMS/NIS testbed."""

from __future__ import annotations

import sys
from typing import Iterable

from .cli import main as export_main
from .estimator_core import main as replay_main
from .estimator_core import ukf_validation_main
from .scoring import main as scoring_main
from .tuning import main as tuning_main


def main(argv: Iterable[str] | None = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    if args and args[0] in ("plan", "score"):
        return scoring_main(args)
    if args and args[0] == "tune":
        return tuning_main(args[1:])
    if args and args[0] == "replay":
        return replay_main(args[1:])
    if args and args[0] == "validate-ukf":
        return ukf_validation_main(args[1:])
    if "--candidate-config" in args or "--covariance-config" in args:
        return replay_main(args)
    return export_main(args)
