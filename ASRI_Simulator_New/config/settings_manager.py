import json
from pathlib import Path
from typing import Any, Optional

class AppSettingsManager:
    """
    Handles loading and accessing configuration from appsettings.json.
    """
    def __init__(self, config_path: Optional[str] = None):
        if config_path is None:
            config_path = Path(__file__).parent / "appsettings.json"
        self.config_path = Path(config_path)
        self._settings = self._load_settings()

    def _load_settings(self) -> dict:
        if not self.config_path.exists():
            raise FileNotFoundError(f"Config file not found: {self.config_path}")
        with open(self.config_path, "r", encoding="utf-8") as f:
            return json.load(f)

    def get(self, *keys: str, default: Any = None) -> Any:
        """
        Retrieve a value from the config using nested keys.
        Example: get('simulation', 'max_simulation_time')
        """
        value = self._settings
        for key in keys:
            if isinstance(value, dict) and key in value:
                value = value[key]
            else:
                return default
        return value

    def reload(self):
        """Reload the settings from the file."""
        self._settings = self._load_settings()

    def set(self, *keys: str, value: Any):
        """
        Set a value in the config (in-memory only).
        Example: set('simulation', 'max_simulation_time', value=1500.0)
        """
        d = self._settings
        for key in keys[:-1]:
            d = d.setdefault(key, {})
        d[keys[-1]] = value

    def save(self):
        """
        Save the current settings to the config file.
        """
        with open(self.config_path, "w", encoding="utf-8") as f:
            json.dump(self._settings, f, indent=2)

