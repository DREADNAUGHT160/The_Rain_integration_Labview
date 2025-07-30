# The Rain Integration LabView

This project provides a simple PySide6 based GUI for controlling a driving robot and platform robot. Configuration files live in `src/init` while trajectory files reside in `src/trajectories`.

## Features

- Real-time control of both a driving robot and a platform robot.
- Configuration driven through plain-text files under `src/init`.
- Trajectory management with editable files in `src/trajectories`.

## Setup

1. Create a Python environment (optional but recommended).
2. Install the required packages:

```bash
pip install -r requirement.txt
```

## Running

Launch the application with:

```bash
python src/main.py
```

On start, the GUI reads settings from `src/init/config.conf` and loads the last saved trajectory from `src/init/load.traj_conf`.

## Repository Structure

- `src/modules/` – application modules and handlers
- `src/init/` – configuration files
- `src/trajectories/` – trajectory definitions
- `src/main.py` – starts the GUI

## Notes

If you move the project files, ensure that the configuration and trajectory paths still point to `src/init` as expected by the code.
