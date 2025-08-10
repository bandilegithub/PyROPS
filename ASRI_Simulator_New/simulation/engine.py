"""
Main simulation engine for the ASRI Simulator.
"""

import numpy as np
from typing import List, Tuple, Dict, Any
from dataclasses import dataclass
import pandas as pd


@dataclass
class SimulationState:
    """Current state of the simulation."""
    time: float = 0.0
    position: np.ndarray = None
    velocity: np.ndarray = None
    acceleration: np.ndarray = None
    mass: float = 0.0
    thrust: float = 0.0

    def __post_init__(self):
        if self.position is None:
            self.position = np.zeros(3)
        if self.velocity is None:
            self.velocity = np.zeros(3)
        if self.acceleration is None:
            self.acceleration = np.zeros(3)


class SimulationEngine:
    """Main simulation engine for rocket trajectory calculations."""

    def __init__(self, parameters: Dict[str, Any]):
        self.parameters = parameters
        self.state = SimulationState()
        self.results = {
            'time': [],
            'altitude': [],
            'velocity': [],
            'acceleration': [],
            'position_x': [],
            'position_y': [],
            'position_z': []
        }

    def initialize_simulation(self):
        """Initialize simulation parameters and state."""
        self.state.mass = self.parameters.get('cad_mass_dry', 50.0)
        self.state.position[2] = self.parameters.get('launch_altitude', 0.0)

    def calculate_forces(self, state: SimulationState) -> np.ndarray:
        """Calculate all forces acting on the rocket."""
        forces = np.zeros(3)

        # Gravity force
        gravity = -9.81 * state.mass
        forces[2] += gravity

        # Thrust force (simplified)
        if state.time <= self.parameters.get('time_burn', 17.1):
            thrust_magnitude = self.parameters.get('thrust_force', 1000.0)
            forces[2] += thrust_magnitude

        # Drag force (simplified)
        velocity_magnitude = np.linalg.norm(state.velocity)
        if velocity_magnitude > 0:
            drag_coefficient = 0.3
            air_density = 1.225
            reference_area = np.pi * (self.parameters.get('rocket_body_radius', 0.087) ** 2)
            drag_magnitude = 0.5 * drag_coefficient * air_density * velocity_magnitude**2 * reference_area
            drag_direction = -state.velocity / velocity_magnitude
            forces += drag_magnitude * drag_direction

        return forces

    def integrate_step(self, dt: float):
        """Perform one integration step using Euler method."""
        forces = self.calculate_forces(self.state)
        self.state.acceleration = forces / self.state.mass

        # Update velocity and position
        self.state.velocity += self.state.acceleration * dt
        self.state.position += self.state.velocity * dt
        self.state.time += dt

        # Store results
        self.results['time'].append(self.state.time)
        self.results['altitude'].append(self.state.position[2])
        self.results['velocity'].append(np.linalg.norm(self.state.velocity))
        self.results['acceleration'].append(np.linalg.norm(self.state.acceleration))
        self.results['position_x'].append(self.state.position[0])
        self.results['position_y'].append(self.state.position[1])
        self.results['position_z'].append(self.state.position[2])

    def run_simulation(self) -> Dict[str, List[float]]:
        """Run the complete simulation."""
        self.initialize_simulation()

        dt = self.parameters.get('time_step_size', 0.02)
        max_time = self.parameters.get('max_simulation_time', 1200.0)

        while (self.state.time < max_time and
               self.state.position[2] >= 0):  # Stop when rocket hits ground
            self.integrate_step(dt)

        return self.results

    def get_summary_statistics(self) -> Dict[str, float]:
        """Calculate summary statistics from simulation results."""
        if not self.results['altitude']:
            return {}

        max_altitude = max(self.results['altitude'])
        max_velocity = max(self.results['velocity'])
        max_acceleration = max(self.results['acceleration'])
        flight_time = self.results['time'][-1] if self.results['time'] else 0

        return {
            'max_altitude': max_altitude,
            'max_velocity': max_velocity,
            'max_acceleration': max_acceleration,
            'flight_time': flight_time
        }


class TrajectoryAnalyzer:
    """Analyze trajectory results and generate reports."""

    def __init__(self, results: Dict[str, List[float]]):
        self.results = results

    def find_apogee(self) -> Tuple[float, float]:
        """Find apogee time and altitude."""
        if not self.results['altitude']:
            return 0.0, 0.0

        max_altitude = max(self.results['altitude'])
        max_index = self.results['altitude'].index(max_altitude)
        apogee_time = self.results['time'][max_index]

        return apogee_time, max_altitude

    def calculate_range(self) -> float:
        """Calculate downrange distance."""
        if not self.results['position_x'] or not self.results['position_y']:
            return 0.0

        final_x = self.results['position_x'][-1]
        final_y = self.results['position_y'][-1]

        return np.sqrt(final_x**2 + final_y**2)

    def export_to_dataframe(self) -> pd.DataFrame:
        """Export results to pandas DataFrame."""
        return pd.DataFrame(self.results)
