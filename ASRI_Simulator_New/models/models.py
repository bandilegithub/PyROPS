"""
Data models for the ASRI Simulator application.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Any, Optional, Tuple
import pandas as pd
import numpy as np
from pathlib import Path
from datetime import datetime


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
class RocketConfiguration:
    """Configuration data for the rocket."""

    name: str = "Default Rocket"
    description: str = ""

    # Geometry
    nose_cone_length: float = 0.5
    nose_cone_type: str = "ogive"
    body_tube_length: float = 3.0
    body_tube_diameter: float = 0.174
    fin_count: int = 4
    fin_chord: float = 0.2
    fin_span: float = 0.15
    fin_sweep: float = 30.0

    # Mass properties
    dry_mass: float = 45.0
    propellant_mass: float = 15.0
    recovery_mass: float = 2.0

    # Center of mass and moments of inertia
    center_of_mass: Tuple[float, float, float] = (1.5, 0.0, 0.0)
    moments_of_inertia: Tuple[float, float, float] = (0.05, 150.0, 150.0)


@dataclass
class Engine:
    """Engine/motor configuration and performance data."""

    name: str = "Default Engine"
    manufacturer: str = ""
    designation: str = ""

    # Physical properties
    diameter: float = 0.075
    length: float = 0.5
    mass: float = 2.0

    # Performance
    total_impulse: float = 5000.0
    burn_time: float = 15.0
    average_thrust: float = 333.0
    peak_thrust: float = 500.0

    # Thrust curve data
    thrust_curve_time: List[float] = field(default_factory=list)
    thrust_curve_force: List[float] = field(default_factory=list)

    # Propellant properties
    propellant_type: str = "solid"
    specific_impulse: float = 200.0


@dataclass
class Propellant:
    """Propellant properties and consumption data."""

    name: str = "Default Propellant"
    type: str = "solid"  # solid, liquid, hybrid

    # Physical properties
    density: float = 1800.0  # kg/m³
    specific_impulse: float = 200.0  # seconds

    # Combustion properties
    combustion_temperature: float = 3000.0  # K
    molecular_weight: float = 25.0  # g/mol
    gamma: float = 1.2  # specific heat ratio

    # Mass flow and consumption
    initial_mass: float = 15.0  # kg
    burn_rate: float = 0.01  # m/s
    pressure_exponent: float = 0.5


@dataclass
class Aerodynamics:
    """Aerodynamic coefficients and properties."""

    # Drag coefficients
    cd_nose: float = 0.15
    cd_body: float = 0.02
    cd_fins: float = 0.01
    cd_base: float = 0.12
    cd_total: float = 0.3

    # Lift coefficients
    cl_alpha: float = 2.0  # per radian
    cl_fins: float = 1.5

    # Moment coefficients
    cm_alpha: float = -0.1  # per radian
    cm_q: float = -10.0  # pitch damping

    # Center of pressure
    center_of_pressure: float = 2.5  # m from nose

    # Stability
    static_margin: float = 0.2

    # Reference areas
    reference_area: float = 0.0238  # m²
    reference_length: float = 4.0  # m

    # Mach number dependent data
    mach_numbers: List[float] = field(default_factory=list)
    cd_mach: List[float] = field(default_factory=list)


@dataclass
class Environment:
    """Environmental conditions for simulation."""

    # Atmospheric conditions
    temperature: float = 288.15  # K (15°C)
    pressure: float = 101325.0  # Pa
    humidity: float = 0.5  # relative humidity (0-1)

    # Wind conditions
    wind_speed: float = 5.0  # m/s
    wind_direction: float = 270.0  # degrees from north
    wind_altitude_profile: List[Tuple[float, float, float]] = field(default_factory=list)  # (alt, speed, dir)

    # Gravity model
    gravity_model: str = "WGS84"  # "constant", "spherical", "WGS84"
    local_gravity: float = 9.81  # m/s²

    # Location
    latitude: float = -34.6  # degrees
    longitude: float = 20.3  # degrees
    elevation: float = 0.0  # m above sea level


@dataclass
class FlightData:
    """Flight data point at a specific time."""

    time: float = 0.0

    # Position (Earth-fixed coordinates)
    position_x: float = 0.0  # East
    position_y: float = 0.0  # North
    position_z: float = 0.0  # Up

    # Velocity (Earth-fixed coordinates)
    velocity_x: float = 0.0
    velocity_y: float = 0.0
    velocity_z: float = 0.0

    # Acceleration (Earth-fixed coordinates)
    acceleration_x: float = 0.0
    acceleration_y: float = 0.0
    acceleration_z: float = 0.0

    # Attitude (Euler angles)
    pitch: float = 0.0  # radians
    yaw: float = 0.0    # radians
    roll: float = 0.0   # radians

    # Angular velocity
    pitch_rate: float = 0.0  # rad/s
    yaw_rate: float = 0.0    # rad/s
    roll_rate: float = 0.0   # rad/s

    # Flight parameters
    altitude: float = 0.0
    velocity_magnitude: float = 0.0
    mach_number: float = 0.0
    dynamic_pressure: float = 0.0

    # Forces and moments
    thrust: float = 0.0
    drag: float = 0.0
    lift: float = 0.0
    weight: float = 0.0

    # Mass properties
    mass: float = 0.0
    center_of_mass: float = 0.0

    # Flight phase
    phase: str = "prelaunch"  # prelaunch, boost, coast, descent, recovery


@dataclass
class Trajectory:
    """Complete trajectory data for a flight."""

    flight_data: List[FlightData] = field(default_factory=list)

    # Summary statistics
    max_altitude: float = 0.0
    max_velocity: float = 0.0
    max_acceleration: float = 0.0
    max_mach: float = 0.0
    flight_time: float = 0.0
    range_x: float = 0.0
    range_y: float = 0.0

    # Key events
    liftoff_time: float = 0.0
    burnout_time: float = 0.0
    apogee_time: float = 0.0
    landing_time: float = 0.0

    # Recovery data
    parachute_deployment_time: float = 0.0
    landing_velocity: float = 0.0

    def add_data_point(self, data: FlightData) -> None:
        """Add a flight data point to the trajectory."""
        self.flight_data.append(data)

        # Update summary statistics
        if data.altitude > self.max_altitude:
            self.max_altitude = data.altitude
        if data.velocity_magnitude > self.max_velocity:
            self.max_velocity = data.velocity_magnitude
        if abs(data.acceleration_z) > self.max_acceleration:
            self.max_acceleration = abs(data.acceleration_z)
        if data.mach_number > self.max_mach:
            self.max_mach = data.mach_number

    def get_dataframe(self) -> pd.DataFrame:
        """Convert trajectory data to pandas DataFrame."""
        if not self.flight_data:
            return pd.DataFrame()

        data_dict = {}
        for field_name in self.flight_data[0].__dataclass_fields__.keys():
            data_dict[field_name] = [getattr(point, field_name) for point in self.flight_data]

        return pd.DataFrame(data_dict)


@dataclass
class SimulationResults:
    """Complete simulation results."""

    trajectory: Trajectory = field(default_factory=Trajectory)
    parameters: SimulationParameters = field(default_factory=SimulationParameters)
    rocket_config: RocketConfiguration = field(default_factory=RocketConfiguration)
    environment: Environment = field(default_factory=Environment)

    # Simulation metadata
    simulation_id: str = ""
    timestamp: datetime = field(default_factory=datetime.now)
    simulation_time: float = 0.0  # computation time
    convergence_status: str = "success"

    # Monte Carlo results (if applicable)
    monte_carlo_runs: List[Trajectory] = field(default_factory=list)
    statistical_summary: Dict[str, Any] = field(default_factory=dict)

    def save_to_file(self, filepath: str) -> bool:
        """Save simulation results to file."""
        try:
            # Convert trajectory to DataFrame and save as Excel/CSV
            df = self.trajectory.get_dataframe()
            if filepath.endswith('.xlsx'):
                df.to_excel(filepath, index=False)
            elif filepath.endswith('.csv'):
                df.to_csv(filepath, index=False)
            else:
                return False
            return True
        except Exception as e:
            print(f"Error saving results: {e}")
            return False


class DataManager:
    """Manages loading, saving, and processing of simulation data."""

    def __init__(self, data_directory: str = ""):
        self.data_directory = Path(data_directory) if data_directory else Path.cwd()

    def load_simulation_parameters(self, filepath: str) -> SimulationParameters:
        """Load simulation parameters from Excel file."""
        try:
            df = pd.read_excel(filepath, header=0)
            params = SimulationParameters()

            # Column mapping for Excel file
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
            print(f"Error loading parameters: {e}")
            return SimulationParameters()

    def save_simulation_parameters(self, params: SimulationParameters, filepath: str) -> bool:
        """Save simulation parameters to Excel file."""
        try:
            data = {
                "Parameter": [
                    "Maximum Simulation Time", "Time Step Size", "Launch Latitude",
                    "Launch Longitude", "Launch Altitude", "Launch Elevation",
                    "Launch Azimuth", "Rocket Body Radius", "Rocket Body Length",
                    "Launch Rail Length", "Nozzle Exit Area", "Thrust Polynomial Degree",
                    "MissileDATCOM Cards", "SolidWorks Mass", "SolidWorks COMx",
                    "SolidWorks COMy", "SolidWorks COMz", "SolidWorks MOIx",
                    "SolidWorks MOIy", "SolidWorks MOIz", "Time Burn",
                    "Density Fuel", "Radius Fuel", "CD Parachute",
                    "Diameter Parachute", "Parachute Deployment Delay"
                ],
                "Value": [
                    params.max_simulation_time, params.time_step_size, params.launch_latitude,
                    params.launch_longitude, params.launch_altitude, params.launch_elevation,
                    params.launch_azimuth, params.rocket_body_radius, params.rocket_body_length,
                    params.launch_rail_length, params.nozzle_exit_area, params.thrust_polynomial_degree,
                    params.missile_datcom_cards, params.cad_mass_dry, params.cad_com_x,
                    params.cad_com_y, params.cad_com_z, params.cad_moi_x,
                    params.cad_moi_y, params.cad_moi_z, params.time_burn,
                    params.fuel_density, params.fuel_radius, params.parachute_cd,
                    params.parachute_diameter, params.parachute_delay
                ]
            }

            df = pd.DataFrame(data)
            df.to_excel(filepath, index=False)
            return True
        except Exception as e:
            print(f"Error saving parameters: {e}")
            return False

    def load_thrust_curve(self, filepath: str) -> Tuple[List[float], List[float]]:
        """Load thrust curve data from Excel file."""
        try:
            df = pd.read_excel(filepath)
            time_col = df.columns[0]
            thrust_col = df.columns[1]

            time_data = df[time_col].dropna().tolist()
            thrust_data = df[thrust_col].dropna().tolist()

            return time_data, thrust_data
        except Exception as e:
            print(f"Error loading thrust curve: {e}")
            return [], []

    def load_aerodynamic_data(self, filepath: str) -> Aerodynamics:
        """Load aerodynamic coefficients from file."""
        try:
            df = pd.read_excel(filepath)
            aero = Aerodynamics()

            # Map columns to aerodynamic properties
            if 'Mach' in df.columns:
                aero.mach_numbers = df['Mach'].dropna().tolist()
            if 'CD' in df.columns:
                aero.cd_mach = df['CD'].dropna().tolist()
            if 'CL_alpha' in df.columns and not df['CL_alpha'].empty:
                aero.cl_alpha = df['CL_alpha'].iloc[0]
            if 'CM_alpha' in df.columns and not df['CM_alpha'].empty:
                aero.cm_alpha = df['CM_alpha'].iloc[0]

            return aero
        except Exception as e:
            print(f"Error loading aerodynamic data: {e}")
            return Aerodynamics()

    def export_trajectory(self, trajectory: Trajectory, filepath: str, format: str = "excel") -> bool:
        """Export trajectory data to file."""
        try:
            df = trajectory.get_dataframe()

            if format.lower() == "excel":
                df.to_excel(filepath, index=False)
            elif format.lower() == "csv":
                df.to_csv(filepath, index=False)
            else:
                return False

            return True
        except Exception as e:
            print(f"Error exporting trajectory: {e}")
            return False
