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
=======
# Rain & Fog System GUIs

This repository provides two PyQt6 GUI applications to control and monitor **Rain** and **Fog** simulation systems.  
Both GUIs communicate with **LabVIEW** over TCP sockets using **newline-delimited JSON**.  

---

## 📂 Files
- `rain_GUI_newline.py` → GUI for Rain Simulation  
- `fog_system_gui_rain_tcp_kb.py` → GUI for Fog Simulation  

---

## 🚀 Setup & Run

1. Install dependencies:
   ```bash
   pip install pyqt6
   ```
2. Run one of the GUIs directly:
   ```bash
   python rain_GUI_newline.py
   ```
   or
   ```bash
   python fog_system_gui_rain_tcp_kb.py
   ```

---

## 🌐 TCP Communication

Both GUIs:
- Act as **TCP clients**, connect to LabVIEW server  
- **Send JSON messages** terminated by `\n`  
- **Receive JSON messages** from LabVIEW for status feedback  

⚠️ **LabVIEW requirement**:  
- Keys must **always be present** in the JSON (no missing fields).  
- Values may change, but key names are fixed.  

---

## 📡 JSON Protocol

### 🌧 Rain System

#### Sent → LabVIEW
```json
{
  "com": 1,
  "v1": 0, "v2": 1, "v3": 0,
  "v4": 0, "v5": 0, "v6": 0,
  "cv": 0,
  "wv": 1,
  "int": 25
}
```

- `com` → ON/OFF  
- `v1..v6` → Segment valves  
- `cv` → Cleaning valve  
- `wv` → Water valve  
- `int` → Rain intensity (mm/h)  

#### Received ← LabVIEW
```json
{
  "test_start": true,
  "pump1": true,
  "pump2": false,
  "pump3": false,
  "int1": 12.3,
  "int2": 11.9,
  "int3": 12.1,
  "main_tank": 85.0
}
```

- `test_start` → System test active  
- `pump1..pump3` → Pump status  
- `int1..int3` → Rain intensity sensors (mm/h)  
- `main_tank` → Tank level (%)  

---

### 🌫 Fog System

#### Sent → LabVIEW
```json
{
  "b1": true,
  "b2": false, "b3": true, "b4": false,
  "b5": false, "b6": false, "b7": false, "b8": false,
  "b9": true, "b10": false, "b11": false,
  "b12": 10, "b13": 0, "b14": 0,
  "b15": 20, "b16": 0, "b17": 0,
  "b18": false,
  "b19": 0
}
```

- `b1` → Master ON/OFF  
- `b2` → Manual mode  
- `b3` → Interval mode  
- `b4` → Visibility mode  
- `b5..b8` → Pump & zones (manual)  
- `b9..b11` → Interval mode: zone enables  
- `b12..b14` → Interval ON-times (s)  
- `b15..b17` → Interval OFF-times (s)  
- `b18` → Visibility regulation enable  
- `b19` → Target visibility (m)  

#### Received ← LabVIEW
```json
{
  "r1": 68.5,
  "r2": 21.4,
  "r3": true,
  "r4": false,
  "r5": 80.0,
  "r6": true,
  "r7": false,
  "r8": false
}
```

- `r1` → Humidity (%)  
- `r2` → Temperature (°C)  
- `r3` → Relief valve  
- `r4` → Pump status  
- `r5` → Visibility actual (m)  
- `r6..r8` → Zone 1–3 status  

---

## 🖥 Features

### Rain GUI
- Toggle ON/OFF, Cleaning, Water valves  
- Enable 6 rain segments  
- Set rain intensity (mm/h)  
- Auto-send (1 Hz) mode  
- Feedback: pumps, test start, sensor values, tank level  

### Fog GUI
- ON/OFF control  
- **3 exclusive modes**: Manual / Interval / Visibility  
- Interval timers per zone (ON/OFF durations)  
- Visibility control by target visibility (m)  
- Auto-send (1 Hz)  
- Feedback: humidity, temperature, pump/valves, actual visibility  
- Keyboard shortcuts (Ctrl+O = toggle, M/I/V = modes, etc.)  

---

## 🔧 Adapting or Extending

### ✅ Creating a New System (e.g., Snow GUI)
1. Copy one of the GUIs as a template  
2. Define a new `self.state = {...}` with all required variables for LabVIEW  
3. Update the buttons/spinboxes to modify these variables  
4. Ensure `send_json()` always sends **all keys** + `\n` at the end  
5. Adapt `update_status_panel()` to show the feedback you expect from LabVIEW  
6. Test with LabVIEW to confirm JSON matches the required format  

---

## 🔗 Integration into Another GUI

If you want to run Rain/Fog GUIs as **part of a larger PyQt6 application**:

1. **Convert GUI class into a widget**  
   - Both programs define `RainSystemGUI(QWidget)` and `FogSystemGUI(QWidget)`.  
   - You can import them directly and add them to your own layouts:
     ```python
     from rain_GUI_newline import RainSystemGUI
     from fog_system_gui_rain_tcp_kb import FogSystemGUI

     class MainApp(QWidget):
         def __init__(self):
             super().__init__()
             layout = QVBoxLayout(self)
             self.rain = RainSystemGUI()
             self.fog = FogSystemGUI()
             layout.addWidget(self.rain)
             layout.addWidget(self.fog)
     ```

2. **Remove the `if __name__ == "__main__":` section**  
   - That block is only for standalone use.  
   - When embedding in another GUI, just import the classes.  

3. **Shared TCP logic**  
   - If multiple subsystems should connect to the same LabVIEW server,  
     consider creating a **shared TCP manager thread** and passing it into each GUI.  

4. **Avoid multiple `QApplication` instances**  
   - Only the main program should create `QApplication`.  
   - Rain/Fog GUIs are just widgets, so they will reuse the same app.  

---

## ⌨️ Fog System Shortcuts

- **Ctrl+O** → Master ON/OFF  
- **M** → Manual mode  
- **I** → Interval mode  
- **V** → Visibility mode  
- **P** → Pump toggle  
- **1/2/3** → Zone 1/2/3 toggle  
- **Enter** → Send JSON immediately  
- **Ctrl+R** → Reconnect  

---
