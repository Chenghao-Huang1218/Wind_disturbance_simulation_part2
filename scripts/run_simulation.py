#!/usr/bin/env python3
"""Run the independent dual UAV project directly from its source tree."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from dual_uav_interaction.cli import main

if __name__ == "__main__":
    raise SystemExit(main())
