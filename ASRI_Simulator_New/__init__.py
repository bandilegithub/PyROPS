"""
ASRI Simulator Package
A comprehensive rocket simulation application.
"""

__version__ = "2.3.12"
__author__ = "Aerospace Systems Research Institute"

from .config.settings import APP_VERSION, APP_TITLE
from .gui.main_window import create_application

__all__ = [
    'APP_VERSION',
    'APP_TITLE',
    'create_application'
]
