#!/usr/bin/env python3
"""
ASRI Simulator - Main Application Entry Point

A modern rocket simulation and analysis tool developed by ASRI
at the University of KwaZulu-Natal.
"""
import sys
import os

# Add the current directory to Python path to ensure imports work
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, current_dir)

from gui.main_window import main

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"Application error: {e}")
        input("Press Enter to exit...")
        sys.exit(1)
