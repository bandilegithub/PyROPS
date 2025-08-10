"""
Utility functions and helpers for the ASRI Simulator.
"""

import os
import subprocess
import webbrowser
from pathlib import Path
from typing import Dict, List, Optional, Union
import pandas as pd
import numpy as np


class FileManager:
    """File management utilities."""

    @staticmethod
    def ensure_directory_exists(path: Union[str, Path]) -> Path:
        """Ensure a directory exists, create if it doesn't."""
        path_obj = Path(path)
        path_obj.mkdir(parents=True, exist_ok=True)
        return path_obj

    @staticmethod
    def read_path_file(filename: str = "Path.txt") -> str:
        """Read the working directory from path file."""
        try:
            with open(filename, 'r') as f:
                return f.read().strip()
        except FileNotFoundError:
            return os.getcwd()

    @staticmethod
    def write_path_file(directory: str, filename: str = "Path.txt") -> bool:
        """Write the working directory to path file."""
        try:
            with open(filename, 'w') as f:
                f.write(directory)
            return True
        except Exception as e:
            print(f"Error writing path file: {e}")
            return False

    @staticmethod
    def validate_file_exists(filepath: Union[str, Path]) -> bool:
        """Check if a file exists."""
        return Path(filepath).exists()


class ExternalTools:
    """Interface to external tools and applications."""

    @staticmethod
    def open_website(url: str) -> bool:
        """Open a website in the default browser."""
        try:
            webbrowser.open(url)
            return True
        except Exception as e:
            print(f"Error opening website: {e}")
            return False

    @staticmethod
    def run_missile_datcom(input_file: str, output_dir: str) -> bool:
        """Run MissileDATCOM analysis."""
        try:
            # Placeholder for MissileDATCOM execution
            print(f"Running MissileDATCOM with input: {input_file}")
            # subprocess.run(['datcom.exe', input_file], cwd=output_dir)
            return True
        except Exception as e:
            print(f"Error running MissileDATCOM: {e}")
            return False

    @staticmethod
    def open_file_explorer(directory: str) -> bool:
        """Open file explorer at specified directory."""
        try:
            if os.name == 'nt':  # Windows
                os.startfile(directory)
            elif os.name == 'posix':  # macOS and Linux
                subprocess.run(['open', directory])
            return True
        except Exception as e:
            print(f"Error opening file explorer: {e}")
            return False


class DataProcessor:
    """Data processing and analysis utilities."""

    @staticmethod
    def interpolate_data(x_data: List[float], y_data: List[float],
                        x_new: List[float]) -> List[float]:
        """Interpolate data using linear interpolation."""
        try:
            return np.interp(x_new, x_data, y_data).tolist()
        except Exception as e:
            print(f"Error interpolating data: {e}")
            return []

    @staticmethod
    def smooth_data(data: List[float], window_size: int = 5) -> List[float]:
        """Smooth data using a moving average."""
        try:
            data_array = np.array(data)
            smoothed = np.convolve(data_array, np.ones(window_size)/window_size, mode='same')
            return smoothed.tolist()
        except Exception as e:
            print(f"Error smoothing data: {e}")
            return data

    @staticmethod
    def calculate_statistics(data: List[float]) -> Dict[str, float]:
        """Calculate basic statistics for a dataset."""
        try:
            data_array = np.array(data)
            return {
                'mean': float(np.mean(data_array)),
                'std': float(np.std(data_array)),
                'min': float(np.min(data_array)),
                'max': float(np.max(data_array)),
                'median': float(np.median(data_array)),
                'count': len(data)
            }
        except Exception as e:
            print(f"Error calculating statistics: {e}")
            return {}


class ParameterValidator:
    """Validate simulation parameters."""

    @staticmethod
    def validate_time_parameters(max_time: float, time_step: float) -> List[str]:
        """Validate time-related parameters."""
        errors = []

        if max_time <= 0:
            errors.append("Maximum simulation time must be positive")

        if time_step <= 0:
            errors.append("Time step size must be positive")

        if time_step > max_time:
            errors.append("Time step size cannot be larger than maximum simulation time")

        if max_time / time_step > 1000000:
            errors.append("Time step too small - will result in excessive computation time")

        return errors

    @staticmethod
    def validate_geometry_parameters(radius: float, length: float) -> List[str]:
        """Validate rocket geometry parameters."""
        errors = []

        if radius <= 0:
            errors.append("Rocket body radius must be positive")

        if length <= 0:
            errors.append("Rocket body length must be positive")

        if length / radius < 5:
            errors.append("Rocket length-to-diameter ratio is very low")

        return errors

    @staticmethod
    def validate_mass_properties(mass: float, com_x: float, moi_values: List[float]) -> List[str]:
        """Validate mass properties."""
        errors = []

        if mass <= 0:
            errors.append("Mass must be positive")

        if any(moi <= 0 for moi in moi_values):
            errors.append("Moments of inertia must be positive")

        return errors


class Logger:
    """Simple logging utility."""

    def __init__(self, log_file: str = "simulation.log"):
        self.log_file = log_file

    def log(self, message: str, level: str = "INFO"):
        """Log a message with timestamp."""
        import datetime
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        log_entry = f"[{timestamp}] {level}: {message}"

        print(log_entry)

        try:
            with open(self.log_file, 'a') as f:
                f.write(log_entry + "\n")
        except Exception as e:
            print(f"Error writing to log file: {e}")

    def info(self, message: str):
        """Log an info message."""
        self.log(message, "INFO")

    def warning(self, message: str):
        """Log a warning message."""
        self.log(message, "WARNING")

    def error(self, message: str):
        """Log an error message."""
        self.log(message, "ERROR")
