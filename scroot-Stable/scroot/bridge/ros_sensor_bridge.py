#!/usr/bin/env python3
"""Compatibility shim that now forwards to ai_input_bridge."""

from __future__ import annotations

from .ai_input_bridge import main


if __name__ == "__main__":  # pragma: no cover
    main()
