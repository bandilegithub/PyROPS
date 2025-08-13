"""
ASRI Simulator Package
A comprehensive rocket simulation application.
"""

__version__ = "2.3.12"
__author__ = "Aerospace Systems Research Institute"

from .config.settings_manager import AppSettingsManager
from .gui.main_window import create_application

settings = AppSettingsManager()

__all__ = [
    'settings',
    'create_application'
]
