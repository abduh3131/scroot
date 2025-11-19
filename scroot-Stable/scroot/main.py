#!/usr/bin/env python3
"""Convenience launcher for the autonomy stack."""

from __future__ import annotations

import sys

from autonomy_launcher import main as launch_main


if __name__ == "__main__":
    launch_main(sys.argv[1:])
