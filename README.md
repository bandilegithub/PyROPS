# PyROPS - Python Rocket Performance Simulator

ASRI (Aerospace Systems Research Institute) Simulator for rocket performance analysis and simulation.

## Description

PyROPS is a comprehensive Python-based rocket performance simulator developed by the Aerospace Systems Research Institute. It provides advanced simulation capabilities for rocket trajectory analysis, performance optimization, and mission planning.

## Features

- Rocket trajectory simulation
- Performance analysis
- Monte Carlo analysis
- 3D visualization with OpenGL
- Aerodynamics modeling
- Thrust curve analysis
- Wind modeling
- Mass properties calculation

## Installation

This project uses Poetry for dependency management. To install:

```bash
poetry install
```

> **Note:** The simulator requires the `tkinter` library for its graphical interface. `tkinter` is included with most standard Python installations on Windows and macOS. On some Linux distributions, you may need to install it separately. For example, on Ubuntu/Debian:
>
> ```bash
> sudo apt-get install python3-tk
> ```

## Usage

Run the simulator using:

```bash
poetry run pyrops
```

Or directly:

```bash
python ASRI_Simulator/launcher.pyw
```

## Dependencies

- Python ^3.8
- NumPy ^1.21.0
- Pandas ^1.3.0
- SciPy ^1.7.0
- Matplotlib ^3.4.0
- Pillow ^10.0.0
- Pygame ^2.1.0
- PyOpenGL ^3.1.0
- OpenPyXL ^3.0.0

## Development

For development, install with dev dependencies:

```bash
poetry install --with dev
```

Run tests:

```bash
poetry run pytest
```

## License

Copyright Aerospace Systems Research Institute
