# The Rain Integration LabVIEW

This repository contains Python-based GUI tools and simulators for integrating rain and fog control systems with a LabVIEW-style interface. The applications communicate with a controller over TCP using newline-delimited JSON frames and are built with [PyQt6](https://pypi.org/project/PyQt6/).

## Contents

- `src/rain_gui_5.py` – GUI for the rain system.
- `src/fog_system_5.py` – GUI for the fog system with keyboard shortcuts and exclusive modes.
- `test/labview_rain_sim.py` – Simple simulator for the rain plant.
- `test/labview_fog_sim.py` – Simple simulator for the fog plant.

## Requirements

- Python 3.10+
- PyQt6 (`pip install PyQt6`)

## Running the GUIs

Start the rain GUI:

```bash
python src/rain_gui_5.py
```

Start the fog GUI:

```bash
python src/fog_system_5.py
```

Each GUI expects to connect to a controller over TCP. The provided simulators can be used for development or testing.

## Running the Simulators

Launch the rain simulator:

```bash
python test/labview_rain_sim.py
```

Launch the fog simulator:

```bash
python test/labview_fog_sim.py
```

These scripts emulate the plant side of the system and respond with status frames for the GUIs.
