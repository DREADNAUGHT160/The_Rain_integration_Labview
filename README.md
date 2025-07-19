# The Rain Integration LabView

This project provides a simple PySide6 based GUI for controlling a driving robot and platform robot. Configuration files live in `src/init` while trajectory files reside in `src/trajectories`.

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

### Rain Integration

The repository includes a second GUI for controlling the LabVIEW rain and fog
system. Open it from the **Control Center** by clicking the `rain` button or run
it directly:

```bash
python src/modules/rainHandler.py
```

The control panel communicates with LabVIEW over TCP. Adjust `TCP_IP` and
`TCP_PORT` at the top of `src/modules/rainHandler.py` if your LabVIEW setup uses
different connection parameters.

## Repository Structure

- `src/modules/` – application modules and handlers
- `src/init/` – configuration files
- `src/trajectories/` – trajectory definitions
- `src/main.py` – starts the GUI

## Notes

If you move the project files, ensure that the configuration and trajectory paths still point to `src/init` as expected by the code.
