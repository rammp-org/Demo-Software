#!/usr/bin/env python3
"""
Run the PID Tuner application.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from pid_tuner.main import main

if __name__ == "__main__":
    main()
