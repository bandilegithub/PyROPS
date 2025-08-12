import json
from pathlib import Path
from typing import Any, Optional, Dict

class AppSettingsManager:
    """
    Handles loading and accessing configuration from appsettings.json.
    Provides easy access to nested configuration values and manages settings persistence.
    """

    def __init__(self, config_path: Optional[str] = None):
        if config_path is None:
            config_path = Path(__file__).parent / "appsettings.json"
        self.config_path = Path(config_path)
        self._settings = self._load_settings()

    def _load_settings(self) -> dict:
        """Load settings from the JSON configuration file."""
        if not self.config_path.exists():
            raise FileNotFoundError(f"Config file not found: {self.config_path}")

        try:
            with open(self.config_path, "r", encoding="utf-8") as f:
                return json.load(f)
        except json.JSONDecodeError as e:
            raise ValueError(f"Invalid JSON in config file: {e}")

    def get(self, *keys: str, default: Any = None) -> Any:
        """
        Retrieve a value from the config using nested keys.

        Args:
            *keys: Nested keys to navigate the config structure
            default: Default value if key path doesn't exist

        Returns:
            The value at the specified key path or default

        Example:
            get('simulation', 'time_max') -> 1200.0
            get('rocket', 'body_radius') -> 0.087
        """
        value = self._settings
        for key in keys:
            if isinstance(value, dict) and key in value:
                value = value[key]
            else:
                return default
        return value

    def get_section(self, section: str) -> Dict[str, Any]:
        """
        Get an entire configuration section.

        Args:
            section: The section name (e.g., 'simulation', 'rocket')

        Returns:
            Dictionary containing all settings in that section
        """
        return self.get(section, default={})

    def set(self, *keys: str, value: Any):
        """
        Set a value in the config (in-memory only, call save() to persist).

        Args:
            *keys: Nested keys path where to set the value
            value: The value to set

        Example:
            set('simulation', 'time_max', value=1500.0)
        """
        if not keys:
            raise ValueError("At least one key must be provided")

        d = self._settings
        for key in keys[:-1]:
            d = d.setdefault(key, {})
        d[keys[-1]] = value

    def update_section(self, section: str, values: Dict[str, Any]):
        """
        Update multiple values in a configuration section.

        Args:
            section: The section name to update
            values: Dictionary of key-value pairs to update
        """
        if section not in self._settings:
            self._settings[section] = {}
        self._settings[section].update(values)

    def save(self):
        """Save the current settings to the config file."""
        try:
            with open(self.config_path, "w", encoding="utf-8") as f:
                json.dump(self._settings, f, indent=2, ensure_ascii=False)
        except IOError as e:
            raise IOError(f"Failed to save config file: {e}")

    def reload(self):
        """Reload the settings from the file, discarding any unsaved changes."""
        self._settings = self._load_settings()

    def get_all_settings(self) -> Dict[str, Any]:
        """
        Get all settings as a flat dictionary for easy access.

        Returns:
            Flattened dictionary with dot-notation keys
        """
        return self._flatten_dict(self._settings)

    def update_setting(self, key: str, value: Any, save_immediately: bool = True):
        """
        Update a specific setting using dot notation and optionally save.

        Args:
            key: Setting key (can use dot notation like 'simulation.time_max')
            value: New value
            save_immediately: Whether to save to file immediately
        """
        keys = key.split('.')
        self.set(*keys, value=value)

        if save_immediately:
            self.save()

    def _flatten_dict(self, d: dict, parent_key: str = '', sep: str = '.') -> Dict[str, Any]:
        """
        Flatten a nested dictionary using dot notation.

        Args:
            d: Dictionary to flatten
            parent_key: Parent key prefix
            sep: Separator character

        Returns:
            Flattened dictionary
        """
        items = []
        for k, v in d.items():
            new_key = f"{parent_key}{sep}{k}" if parent_key else k
            if isinstance(v, dict):
                items.extend(self._flatten_dict(v, new_key, sep=sep).items())
            else:
                items.append((new_key, v))
        return dict(items)

    def get_simulation_params(self) -> Dict[str, Any]:
        """Get all simulation parameters."""
        return self.get_section('simulation')

    def get_rocket_params(self) -> Dict[str, Any]:
        """Get all rocket parameters."""
        return self.get_section('rocket')

    def get_monte_carlo_params(self) -> Dict[str, Any]:
        """Get all Monte Carlo parameters."""
        return self.get_section('monte_carlo')

    def get_app_info(self) -> Dict[str, Any]:
        """Get application information."""
        return self.get_section('application')

    def backup_settings(self, backup_path: Optional[str] = None):
        """
        Create a backup of the current settings.

        Args:
            backup_path: Path for backup file. If None, creates timestamped backup.
        """
        if backup_path is None:
            from datetime import datetime
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            backup_path = self.config_path.parent / f"appsettings_backup_{timestamp}.json"

        backup_path = Path(backup_path)
        with open(backup_path, "w", encoding="utf-8") as f:
            json.dump(self._settings, f, indent=2, ensure_ascii=False)

    def restore_settings(self, backup_path: str):
        """
        Restore settings from a backup file.

        Args:
            backup_path: Path to the backup file
        """
        backup_path = Path(backup_path)
        if not backup_path.exists():
            raise FileNotFoundError(f"Backup file not found: {backup_path}")

        with open(backup_path, "r", encoding="utf-8") as f:
            self._settings = json.load(f)

        self.save()

    def validate_settings(self) -> bool:
        """
        Validate that all required settings are present and have valid types.

        Returns:
            True if settings are valid, False otherwise
        """
        required_sections = ['simulation', 'rocket', 'monte_carlo', 'application']

        for section in required_sections:
            if section not in self._settings:
                print(f"Missing required section: {section}")
                return False

        # Validate numeric values in simulation section
        sim_params = self.get_simulation_params()
        numeric_fields = ['time_max', 'time_step', 'launch_lat', 'launch_lon',
                         'launch_alt', 'launch_elev', 'launch_azim']

        for field in numeric_fields:
            if field not in sim_params or not isinstance(sim_params[field], (int, float)):
                print(f"Invalid or missing numeric field: simulation.{field}")
                return False

        return True

    def reset_to_defaults(self):
        """Reset all settings to default values."""
        default_settings = {
            "simulation": {
                "time_max": 1200.0,
                "time_step": 0.02,
                "launch_lat": -34.600,
                "launch_lon": 20.300,
                "launch_alt": 0.0,
                "launch_elev": 80.0,
                "launch_azim": -100.0,
                "solver_8th": 1,
                "rel_tolerance": 1.0,
                "abs_tolerance": 1.0,
                "first_step": 0.02,
                "max_step": 0.02
            },
            "rocket": {
                "body_radius": 0.087,
                "body_length": 4.920,
                "rail_length": 7.0,
                "nozzle_area": 0.007056,
                "cad_mass": 49.35224149,
                "cad_com_x": 1.386632,
                "cad_com_y": 0.0,
                "cad_com_z": 0.0,
                "cad_moi_x": 0.04023116,
                "cad_moi_y": 180.8297,
                "cad_moi_z": 180.8297
            },
            "monte_carlo": {
                "mc_runs": 1000,
                "mc_enable": 1,
                "mc_detailed": 1
            }
        }

        self._settings.update(default_settings)
        self.save()

    def __str__(self) -> str:
        """String representation of the settings manager."""
        return f"AppSettingsManager(config_path={self.config_path})"

    def __repr__(self) -> str:
        """Detailed string representation."""
        return f"AppSettingsManager(config_path='{self.config_path}', sections={list(self._settings.keys())})"
