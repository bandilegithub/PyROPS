"""
Utility functions and helpers for the ASRI Simulator.
"""

import os
import math
import subprocess
import webbrowser
from pathlib import Path
from typing import Dict, List, Optional, Union, Any, Tuple
import pandas as pd
import numpy as np


def get_project_root() -> Path:
    """Get the project root directory."""
    current_path = Path(__file__).parent
    while current_path != current_path.parent:
        if (current_path / "pyproject.toml").exists() or (current_path / "setup.py").exists():
            return current_path
        current_path = current_path.parent
    return Path.cwd()


def validate_file_path(filepath: Union[str, Path]) -> bool:
    """Validate if a file path exists and is accessible."""
    try:
        path_obj = Path(filepath)
        return path_obj.exists() and path_obj.is_file()
    except Exception:
        return False


def safe_float_conversion(value: Any, default: float = 0.0) -> float:
    """Safely convert a value to float with fallback."""
    try:
        if pd.isna(value):
            return default
        return float(value)
    except (ValueError, TypeError):
        return default


def safe_int_conversion(value: Any, default: int = 0) -> int:
    """Safely convert a value to integer with fallback."""
    try:
        if pd.isna(value):
            return default
        return int(float(value))  # Convert to float first to handle string numbers
    except (ValueError, TypeError):
        return default


def normalize_path(path: Union[str, Path]) -> Path:
    """Normalize a file path and resolve it."""
    return Path(path).resolve()


def create_directory_if_not_exists(directory: Union[str, Path]) -> Path:
    """Create a directory if it doesn't exist."""
    path_obj = Path(directory)
    path_obj.mkdir(parents=True, exist_ok=True)
    return path_obj


def load_excel_data(filepath: Union[str, Path], sheet_name: Optional[str] = None, **kwargs) -> pd.DataFrame:
    """Load data from Excel file with error handling."""
    try:
        if sheet_name:
            return pd.read_excel(filepath, sheet_name=sheet_name, **kwargs)
        else:
            return pd.read_excel(filepath, **kwargs)
    except Exception as e:
        print(f"Error loading Excel file {filepath}: {e}")
        return pd.DataFrame()


def calculate_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Calculate distance between two geographical points using Haversine formula."""
    # Convert latitude and longitude from degrees to radians
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

    # Haversine formula
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.asin(math.sqrt(a))

    # Radius of earth in kilometers
    r = 6371
    return c * r


def degrees_to_radians(degrees: float) -> float:
    """Convert degrees to radians."""
    return math.radians(degrees)


def radians_to_degrees(radians: float) -> float:
    """Convert radians to degrees."""
    return math.degrees(radians)


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

    @staticmethod
    def get_file_size(filepath: Union[str, Path]) -> int:
        """Get file size in bytes."""
        try:
            return Path(filepath).stat().st_size
        except (OSError, FileNotFoundError):
            return 0

    @staticmethod
    def copy_file(source: Union[str, Path], destination: Union[str, Path]) -> bool:
        """Copy a file from source to destination."""
        try:
            import shutil
            shutil.copy2(source, destination)
            return True
        except Exception as e:
            print(f"Error copying file: {e}")
            return False


class ValidationUtils:
    """Validation utilities for data and parameters."""

    @staticmethod
    def validate_positive_number(value: Union[int, float], name: str = "Value") -> bool:
        """Validate that a number is positive."""
        try:
            num_value = float(value)
            if num_value <= 0:
                print(f"{name} must be positive, got: {value}")
                return False
            return True
        except (ValueError, TypeError):
            print(f"{name} must be a valid number, got: {value}")
            return False

    @staticmethod
    def validate_range(value: Union[int, float], min_val: float, max_val: float, name: str = "Value") -> bool:
        """Validate that a value is within a specified range."""
        try:
            num_value = float(value)
            if not (min_val <= num_value <= max_val):
                print(f"{name} must be between {min_val} and {max_val}, got: {value}")
                return False
            return True
        except (ValueError, TypeError):
            print(f"{name} must be a valid number, got: {value}")
            return False

    @staticmethod
    def validate_file_extension(filepath: Union[str, Path], allowed_extensions: List[str]) -> bool:
        """Validate that a file has an allowed extension."""
        path_obj = Path(filepath)
        extension = path_obj.suffix.lower()

        if extension not in [ext.lower() for ext in allowed_extensions]:
            print(f"File extension '{extension}' not allowed. Allowed: {allowed_extensions}")
            return False
        return True

    @staticmethod
    def validate_coordinates(latitude: float, longitude: float) -> bool:
        """Validate geographical coordinates."""
        if not ValidationUtils.validate_range(latitude, -90, 90, "Latitude"):
            return False
        if not ValidationUtils.validate_range(longitude, -180, 180, "Longitude"):
            return False
        return True

    @staticmethod
    def validate_excel_columns(df: pd.DataFrame, required_columns: List[str]) -> bool:
        """Validate that required columns exist in a DataFrame."""
        missing_columns = [col for col in required_columns if col not in df.columns]
        if missing_columns:
            print(f"Missing required columns: {missing_columns}")
            return False
        return True

    @staticmethod
    def validate_simulation_parameters(params: Dict[str, Any]) -> Tuple[bool, List[str]]:
        """Validate simulation parameters and return validation status and error messages."""
        errors = []

        # Validate time parameters
        if 'max_simulation_time' in params:
            if not ValidationUtils.validate_positive_number(params['max_simulation_time'], "Maximum simulation time"):
                errors.append("Invalid maximum simulation time")

        if 'time_step_size' in params:
            if not ValidationUtils.validate_positive_number(params['time_step_size'], "Time step size"):
                errors.append("Invalid time step size")

        # Validate coordinates
        if 'launch_latitude' in params and 'launch_longitude' in params:
            if not ValidationUtils.validate_coordinates(params['launch_latitude'], params['launch_longitude']):
                errors.append("Invalid launch coordinates")

        # Validate rocket dimensions
        if 'rocket_body_radius' in params:
            if not ValidationUtils.validate_positive_number(params['rocket_body_radius'], "Rocket body radius"):
                errors.append("Invalid rocket body radius")

        if 'rocket_body_length' in params:
            if not ValidationUtils.validate_positive_number(params['rocket_body_length'], "Rocket body length"):
                errors.append("Invalid rocket body length")

        return len(errors) == 0, errors


class ExternalTools:
    """Interface to external tools and applications."""

    @staticmethod
    def open_file_explorer(path: Union[str, Path]) -> bool:
        """Open file explorer at the specified path."""
        try:
            path_obj = Path(path)
            if path_obj.exists():
                if os.name == 'nt':  # Windows
                    subprocess.run(['explorer', str(path_obj)], check=True)
                elif os.name == 'posix':  # macOS and Linux
                    subprocess.run(['open', str(path_obj)], check=True)
                return True
            else:
                print(f"Path does not exist: {path}")
                return False
        except Exception as e:
            print(f"Error opening file explorer: {e}")
            return False

    @staticmethod
    def open_url(url: str) -> bool:
        """Open a URL in the default web browser."""
        try:
            webbrowser.open(url)
            return True
        except Exception as e:
            print(f"Error opening URL: {e}")
            return False

    @staticmethod
    def run_external_command(command: List[str], working_dir: Optional[Union[str, Path]] = None) -> Tuple[bool, str]:
        """Run an external command and return success status and output."""
        try:
            result = subprocess.run(
                command,
                cwd=working_dir,
                capture_output=True,
                text=True,
                check=True
            )
            return True, result.stdout
        except subprocess.CalledProcessError as e:
            return False, e.stderr
        except Exception as e:
            return False, str(e)


class DataUtils:
    """Data processing and analysis utilities."""

    @staticmethod
    def interpolate_data(x_data: List[float], y_data: List[float], x_new: float) -> float:
        """Perform linear interpolation for a given x value."""
        try:
            return np.interp(x_new, x_data, y_data)
        except Exception as e:
            print(f"Error interpolating data: {e}")
            return 0.0

    @staticmethod
    def smooth_data(data: List[float], window_size: int = 5) -> List[float]:
        """Apply moving average smoothing to data."""
        try:
            if len(data) < window_size:
                return data

            smoothed = []
            for i in range(len(data)):
                start = max(0, i - window_size // 2)
                end = min(len(data), i + window_size // 2 + 1)
                smoothed.append(sum(data[start:end]) / (end - start))

            return smoothed
        except Exception as e:
            print(f"Error smoothing data: {e}")
            return data

    @staticmethod
    def calculate_statistics(data: List[float]) -> Dict[str, float]:
        """Calculate basic statistics for a list of numbers."""
        try:
            if not data:
                return {}

            return {
                'mean': np.mean(data),
                'median': np.median(data),
                'std': np.std(data),
                'min': np.min(data),
                'max': np.max(data),
                'count': len(data)
            }
        except Exception as e:
            print(f"Error calculating statistics: {e}")
            return {}

