"""
Data models for the ASRI Simulator application.
"""

from dataclasses import dataclass
from typing import Dict, Any, Optional
import pandas as pd
from pathlib import Path


@dataclass
class SimulationParameters:
    """Data class for simulation parameters."""

    # Time and location parameters
    max_simulation_time: float = 1200.0
    time_step_size: float = 0.02
    launch_latitude: float = -34.600
    launch_longitude: float = 20.300
    launch_altitude: float = 0.000
    launch_elevation: float = 80.000
    launch_azimuth: float = -100.000

    # Rocket geometry
    rocket_body_radius: float = 0.087
    rocket_body_length: float = 4.920
    launch_rail_length: float = 7.0
    nozzle_exit_area: float = 0.007056

    # Analysis parameters
    thrust_polynomial_degree: int = 6
    missile_datcom_cards: int = 20

    # Mass properties (dry)
    cad_mass_dry: float = 49.35224149
    cad_com_x: float = 1.386632
    cad_com_y: float = 0.0
    cad_com_z: float = 0.0
    cad_moi_x: float = 0.04023116
    cad_moi_y: float = 180.8297
    cad_moi_z: float = 180.8297

    # Propulsion
    time_burn: float = 17.1
    fuel_density: float = 1065.0
    fuel_radius: float = 0.0735

    # Recovery
    parachute_cd: float = 1.3
    parachute_diameter: float = 1.0
    parachute_delay: float = 10.0

    # Directory
    directory: str = ""


@dataclass
class MonteCarloParameters:
    """Data class for Monte Carlo simulation parameters."""

    number_runs: int = 100
    launch_elevation_lower: float = 75.0
    launch_elevation_upper: float = 85.0
    launch_azimuth_lower: float = -105.0
    launch_azimuth_upper: float = -95.0
    thrust_misalignment_yaw_lower: float = -2.0
    thrust_misalignment_yaw_upper: float = 2.0
    thrust_misalignment_pitch_lower: float = -2.0
    thrust_misalignment_pitch_upper: float = 2.0
    thrust_magnitude_lower: float = 0.9
    thrust_magnitude_upper: float = 1.1
    wind_magnitude_lower: float = 0.0
    wind_magnitude_upper: float = 10.0
    wind_direction_lower: float = 0.0
    wind_direction_upper: float = 360.0


class ParameterManager:
    """Manages loading and saving of simulation parameters."""

    def __init__(self, settings_file_path: str = ""):
        self.settings_file_path = settings_file_path

    def load_from_excel(self, file_path: str) -> SimulationParameters:
        """Load parameters from Excel file."""
        try:
            df = pd.read_excel(file_path, header=0)
            params = SimulationParameters()

            # Map Excel columns to parameter attributes
            column_mapping = {
                "Maximum Simulation Time": "max_simulation_time",
                "Time Step Size": "time_step_size",
                "Launch Latitude": "launch_latitude",
                "Launch Longitude": "launch_longitude",
                "Launch Altitude": "launch_altitude",
                "Launch Elevation": "launch_elevation",
                "Launch Azimuth": "launch_azimuth",
                "Rocket Body Radius": "rocket_body_radius",
                "Rocket Body Length": "rocket_body_length",
                "Launch Rail Length": "launch_rail_length",
                "Nozzle Exit Area": "nozzle_exit_area",
                "Thrust Polynomial Degree": "thrust_polynomial_degree",
                "MissileDATCOM Cards": "missile_datcom_cards",
                "SolidWorks Mass": "cad_mass_dry",
                "SolidWorks COMx": "cad_com_x",
                "SolidWorks COMy": "cad_com_y",
                "SolidWorks COMz": "cad_com_z",
                "SolidWorks MOIx": "cad_moi_x",
                "SolidWorks MOIy": "cad_moi_y",
                "SolidWorks MOIz": "cad_moi_z",
                "Time Burn": "time_burn",
                "Density Fuel": "fuel_density",
                "Radius Fuel": "fuel_radius",
                "CD Parachute": "parachute_cd",
                "Diameter Parachute": "parachute_diameter",
                "Parachute Deployment Delay": "parachute_delay"
            }

            for excel_col, param_attr in column_mapping.items():
                if excel_col in df.columns and len(df) > 0:
                    value = df.at[0, excel_col]
                    if pd.notna(value):
                        setattr(params, param_attr, float(value))

            return params

        except Exception as e:
            print(f"Error loading parameters from Excel: {e}")
            return SimulationParameters()

    def save_to_excel(self, params: SimulationParameters, file_path: str) -> bool:
        """Save parameters to Excel file."""
        try:
            # Create DataFrame with parameter data
            data = {
                "Maximum Simulation Time": [params.max_simulation_time],
                "Time Step Size": [params.time_step_size],
                "Launch Latitude": [params.launch_latitude],
                "Launch Longitude": [params.launch_longitude],
                "Launch Altitude": [params.launch_altitude],
                "Launch Elevation": [params.launch_elevation],
                "Launch Azimuth": [params.launch_azimuth],
                "Rocket Body Radius": [params.rocket_body_radius],
                "Rocket Body Length": [params.rocket_body_length],
                "Launch Rail Length": [params.launch_rail_length],
                "Nozzle Exit Area": [params.nozzle_exit_area],
                "Thrust Polynomial Degree": [params.thrust_polynomial_degree],
                "MissileDATCOM Cards": [params.missile_datcom_cards],
                "SolidWorks Mass": [params.cad_mass_dry],
                "SolidWorks COMx": [params.cad_com_x],
                "SolidWorks COMy": [params.cad_com_y],
                "SolidWorks COMz": [params.cad_com_z],
                "SolidWorks MOIx": [params.cad_moi_x],
                "SolidWorks MOIy": [params.cad_moi_y],
                "SolidWorks MOIz": [params.cad_moi_z],
                "Time Burn": [params.time_burn],
                "Density Fuel": [params.fuel_density],
                "Radius Fuel": [params.fuel_radius],
                "CD Parachute": [params.parachute_cd],
                "Diameter Parachute": [params.parachute_diameter],
                "Parachute Deployment Delay": [params.parachute_delay]
            }

            df = pd.DataFrame(data)
            df.to_excel(file_path, index=False)
            return True

        except Exception as e:
            print(f"Error saving parameters to Excel: {e}")
            return False
