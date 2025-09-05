# fog_labview_sim_gui_full.py
# Fog "LabVIEW" simulator with GUI (PyQt6), single-port, newline-delimited JSON.
# RX (from controller): { "b1"... "b19" }
# TX (to controller)  : { "r1"... "r8" }
#
# r-map:
#   r1: humidity %                 (leit_read_Luftfeuchtigkeit)
#   r2: temperature °C            (leit_read_Temperatur)
#   r3: relief valve 0/1          (leit_read_Entlastungsventile_Status)
#   r4: pump 0/1                   (leit_read_Pumpe_Status)
#   r5: visibility IST (m)        (leit_read_Sichtweite_IST)
#   r6..r8: zone1..zone3 status (bool)
#
# Requires: pip install PyQt6

import json, socket, threading, time, sys
from dataclasses import dataclass, asdict
from PyQt6 import QtCore, QtWidgets

BIND_IP_DEFAULT = "0.0.0.0"   # listen on all interfaces by default
PORT_DEFAULT    = 5052        # single port (RX+TX)
SEND_HZ         = 5.0         # status send rate

# -------------------- simulation core --------------------
@dataclass
class Cmd:
    b1: bool=False; b2: bool=False; b3: bool=False; b4: bool=False
    b5: bool=False; b6: bool=False; b7: bool=False; b8: bool=False
    b9: bool=False; b10: bool=False; b11: bool=False
    b12: int=0; b13: int=0; b14: int=0; b15: int=0; b16: int=0; b17: int=0
    b18: bool=False; b19: int=0

@dataclass
class Plant:
    vis_m: float=200.0
    hum: float=45.0
    tmp: float=21.0
    relief: bool=False
    pump: bool=False
    z1: bool=False; z2: bool=False; z3: bool=False
    # interval timers
    t1: float=0.0; t2: float=0.0; t3: float=0.0
    on1: bool=False; on2: bool=False; on3: bool=False

class FogSimServer:
    """TCP server + simple fog plant model. One controller client at a time."""
    def __init__(self, bind_ip=BIND_IP_DEFAULT, port=PORT_DEFAULT, hz=SEND_HZ):
        self.bind_ip, self.port = bind_ip, port
        self.dt = 1.0 / float(hz)
        self.cmd = Cmd()
        self.plant = Plant()
        self._srv_thread = None
        self._stop = threading.Event()
        self._conn = None
        self._lock = threading.Lock()   # protects cmd/plant/connected
        self.connected = False
        self.last_rx = {}               # last received b*-frame for UI

    # ---- public control ----
    def start(self):
        if self._srv_thread and self._srv_thread.is_alive():
            return
        self._stop.clear()
        self._srv_thread = threading.Thread(target=self._serve, daemon=True)
        self._srv_thread.start()

    def stop(self):
        self._stop.set()
        with self._lock:
            if self._conn:
                try: self._conn.close()
                except: pass
            self._conn = None
            self.connected = False

    def is_running(self): 
        return bool(self._srv_thread and self._srv_thread.is_alive())

    # Snapshot for GUI
    def snapshot(self):
        with self._lock:
            return {
                "connected": self.connected,
                "cmd": asdict(self.cmd),
                "plant": asdict(self.plant),
                "last_rx": dict(self.last_rx),
            }

    # ---- networking ----
    def _serve(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.bind((self.bind_ip, self.port))
            srv.listen(1)
            srv.settimeout(1.0)
            while not self._stop.is_set():
                try:
                    c, addr = srv.accept()
                except socket.timeout:
                    continue

                with self._lock:
                    if self._conn:
                        try: self._conn.close()
                        except: pass
                    self._conn = c
                    self.connected = True

                rx = threading.Thread(target=self._rx_loop, args=(c,), daemon=True)
                rx.start()
                self._tx_loop(c)  # returns when client disconnects

                with self._lock:
                    self.connected = False
                    if self._conn is c: self._conn = None
                try: c.close()
                except: pass
        finally:
            try: srv.close()
            except: pass

    def _rx_loop(self, c: socket.socket):
        buf = b""; c.settimeout(2.0)
        while not self._stop.is_set():
            try:
                chunk = c.recv(4096)
                if not chunk:
                    return
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    if not line: continue
                    try:
                        obj = json.loads(line.decode("utf-8"))
                        self._apply_cmd(obj)
                    except Exception:
                        # ignore bad frames (keeps sim robust)
                        pass
            except (socket.timeout, BlockingIOError):
                continue
            except Exception:
                return

    def _send_status(self, c: socket.socket, obj: dict):
        line = json.dumps(obj, separators=(",", ":")) + "\n"
        c.sendall(line.encode("utf-8"))

    # ---- model/update ----
    def _apply_cmd(self, obj: dict):
        # only b-keys; cast to correct types
        with self._lock:
            self.last_rx = {k: obj[k] for k in obj if k.startswith("b")}
            for k, v in list(self.last_rx.items()):
                try:
                    if k in {"b12","b13","b14","b15","b16","b17","b19"}:
                        setattr(self.cmd, k, int(v))
                    else:
                        setattr(self.cmd, k, bool(v))
                except Exception:
                    pass

    def _interval_zone(self, dt, en, on_s, off_s, which):
        P = self.plant
        if not en or on_s <= 0 or off_s <= 0:
            if which==1: P.on1=False; P.t1=0.0
            if which==2: P.on2=False; P.t2=0.0
            if which==3: P.on3=False; P.t3=0.0
            return False
        if which==1:
            P.t1 += dt; period = on_s if P.on1 else off_s
            if P.t1 >= period: P.t1=0.0; P.on1 = not P.on1
            return P.on1
        if which==2:
            P.t2 += dt; period = on_s if P.on2 else off_s
            if P.t2 >= period: P.t2=0.0; P.on2 = not P.on2
            return P.on2
        P.t3 += dt; period = on_s if P.on3 else off_s
        if P.t3 >= period: P.t3=0.0; P.on3 = not P.on3
        return P.on3

    def _step(self, dt):
        C, P = self.cmd, self.plant
        pump = False; relief = False; z1 = z2 = z3 = False

        if C.b1:
            # Manual
            if C.b2 and not C.b3 and not C.b4:
                pump = C.b5; z1, z2, z3 = C.b6, C.b7, C.b8
            # Interval
            elif C.b3 and not C.b2 and not C.b4:
                z1 = self._interval_zone(dt, C.b9,  C.b12, C.b15, 1)
                z2 = self._interval_zone(dt, C.b10, C.b13, C.b16, 2)
                z3 = self._interval_zone(dt, C.b11, C.b14, C.b17, 3)
                pump = z1 or z2 or z3
            # Visibility
            elif C.b4 and not C.b2 and not C.b3:
                if C.b18:
                    tgt = max(0, int(C.b19)); H = 5.0
                    if P.vis_m > tgt + H:  pump=True;  relief=False
                    elif P.vis_m < tgt - H: pump=False; relief=True
                    z1 = z2 = z3 = pump
                else:
                    pump = False

        P.pump, P.relief, P.z1, P.z2, P.z3 = pump, relief, z1, z2, z3

        # Plant dynamics
        fog_rate = 3.0; clear_rate = 2.0; bonus = 2.0
        if pump: P.vis_m = max(10.0, P.vis_m - fog_rate*dt)
        else:    P.vis_m = min(250.0, P.vis_m + (clear_rate + (bonus if relief else 0))*dt)
        if pump: P.hum = min(95.0, P.hum + 1.0*dt)
        else:
            leak = 0.5 + (0.5 if relief else 0.0)
            P.hum = max(40.0, P.hum - leak*dt)
        P.tmp += (-0.02 if pump else 0.01)*dt
        P.tmp = max(5.0, min(40.0, P.tmp))

    def _status_frame(self):
        P = self.plant
        return {
            "r1": round(P.hum, 1),
            "r2": round(P.tmp, 1),
            "r3": 1 if P.relief else 0,
            "r4": 1 if P.pump else 0,
            "r5": int(P.vis_m),
            "r6": bool(P.z1),
            "r7": bool(P.z2),
            "r8": bool(P.z3),
        }

    def _tx_loop(self, c: socket.socket):
        last = time.time()
        c.settimeout(2.0)
        while not self._stop.is_set():
            time.sleep(self.dt)
            dt = time.time() - last; last = time.time()
            try:
                with self._lock:
                    self._step(dt)
                    frame = self._status_frame()
                self._send_status(c, frame)
            except (BrokenPipeError, ConnectionResetError, OSError):
                return

# -------------------- GUI --------------------
class Lamp(QtWidgets.QLabel):
    def __init__(self, d=14):
        super().__init__(); self.setFixedSize(d, d); self.setStatus(False)
    def setStatus(self, v):
        on = (bool(v) if not isinstance(v,(int,float)) else v!=0)
        color = "#27ae60" if on else "#5c5c5c"
        self.setStyleSheet(f"border-radius:{self.width()//2}px; background:{color}; border:1px solid #222;")

def _box(label):
    w = QtWidgets.QWidget(); lay = QtWidgets.QVBoxLayout(w); lay.setContentsMargins(0,0,0,0)
    lab = QtWidgets.QLabel(label); lab.setStyleSheet("font-weight:600;")
    val = QtWidgets.QLineEdit(); val.setReadOnly(True); val.setPlaceholderText("--")
    lay.addWidget(lab); lay.addWidget(val); return w, val

class SimWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Fog LabVIEW Simulator (single-port)")
        self.sim = FogSimServer()
        self._build_ui()

        # Poll simulator state 10 Hz
        self.timer = QtCore.QTimer(self); self.timer.setInterval(100)
        self.timer.timeout.connect(self.refresh); self.timer.start()

    def _build_ui(self):
        central = QtWidgets.QWidget(); self.setCentralWidget(central)
        root = QtWidgets.QHBoxLayout(central); root.setContentsMargins(8,8,8,8); root.setSpacing(16)

        # Left: server control + plant readouts
        left = QtWidgets.QVBoxLayout(); root.addLayout(left, 3)

        grp = QtWidgets.QGroupBox("Connection"); left.addWidget(grp)
        g = QtWidgets.QGridLayout(grp)
        self.ed_host = QtWidgets.QLineEdit(BIND_IP_DEFAULT)
        self.ed_port = QtWidgets.QSpinBox(); self.ed_port.setRange(1,65535); self.ed_port.setValue(PORT_DEFAULT)
        self.btn_start = QtWidgets.QPushButton("Start")
        self.btn_stop  = QtWidgets.QPushButton("Stop"); self.btn_stop.setEnabled(False)
        self.lbl_state = QtWidgets.QLabel("• Disconnected"); self.lbl_state.setStyleSheet("color:#c0392b; font-weight:600;")
        self.btn_start.clicked.connect(self.start_server); self.btn_stop.clicked.connect(self.stop_server)
        g.addWidget(QtWidgets.QLabel("IP"),   0, 0); g.addWidget(self.ed_host, 0, 1)
        g.addWidget(QtWidgets.QLabel("Port"), 1, 0); g.addWidget(self.ed_port, 1, 1)
        g.addWidget(self.btn_start, 2, 0);    g.addWidget(self.btn_stop, 2, 1)
        g.addWidget(self.lbl_state, 3, 0, 1, 2)

        left.addWidget(QtWidgets.QFrame(frameShape=QtWidgets.QFrame.Shape.HLine))

        # Plant readouts
        hv, self.ed_hum = _box("Humidity (%)")
        tv, self.ed_tmp = _box("Temperature (°C)")
        vv, self.ed_vis = _box("Visibility IST (m)")
        left.addWidget(hv); left.addWidget(tv); left.addWidget(vv)

        grp2 = QtWidgets.QGroupBox("Status Lamps"); left.addWidget(grp2)
        gl = QtWidgets.QGridLayout(grp2)
        self.l_relief = Lamp(); self.l_pump = Lamp(); self.l_z1 = Lamp(); self.l_z2 = Lamp(); self.l_z3 = Lamp()
        gl.addWidget(QtWidgets.QLabel("Relief Valve"),0,0); gl.addWidget(self.l_relief,0,1)
        gl.addWidget(QtWidgets.QLabel("Pump"),1,0); gl.addWidget(self.l_pump,1,1)
        gl.addWidget(QtWidgets.QLabel("Zone 1"),2,0); gl.addWidget(self.l_z1,2,1)
        gl.addWidget(QtWidgets.QLabel("Zone 2"),3,0); gl.addWidget(self.l_z2,3,1)
        gl.addWidget(QtWidgets.QLabel("Zone 3"),4,0); gl.addWidget(self.l_z3,4,1)

        left.addStretch(1)

        # Right: last received b*-frame + last sent r*-frame
        right = QtWidgets.QVBoxLayout(); root.addLayout(right, 2)
        grp3 = QtWidgets.QGroupBox("Last RX (b1..b19)"); right.addWidget(grp3)
        v = QtWidgets.QVBoxLayout(grp3)
        self.rx_view = QtWidgets.QPlainTextEdit(); self.rx_view.setReadOnly(True)
        self.rx_view.setPlaceholderText("Shows the last command frame received from your controller...")
        v.addWidget(self.rx_view)

        grp4 = QtWidgets.QGroupBox("Last TX (r1..r8)"); right.addWidget(grp4)
        v2 = QtWidgets.QVBoxLayout(grp4)
        self.tx_view = QtWidgets.QPlainTextEdit(); self.tx_view.setReadOnly(True)
        self.tx_view.setPlaceholderText("Shows the most recent status frame sent to the controller...")
        v2.addWidget(self.tx_view)

    # ---- server control ----
    def start_server(self):
        if self.sim.is_running(): return
        self.sim.bind_ip = self.ed_host.text().strip()
        self.sim.port = int(self.ed_port.value())
        self.sim.start()
        self.btn_start.setEnabled(False); self.btn_stop.setEnabled(True)
        self.lbl_state.setText("• Listening"); self.lbl_state.setStyleSheet("color:#27ae60; font-weight:600;")

    def stop_server(self):
        self.sim.stop()
        self.btn_start.setEnabled(True); self.btn_stop.setEnabled(False)
        self.lbl_state.setText("• Disconnected"); self.lbl_state.setStyleSheet("color:#c0392b; font-weight:600;")

    # ---- periodic UI refresh ----
    def refresh(self):
        snap = self.sim.snapshot()
        # connection indicator
        if self.sim.is_running():
            if snap["connected"]:
                self.lbl_state.setText("• Connected")
                self.lbl_state.setStyleSheet("color:#27ae60; font-weight:600;")
            else:
                self.lbl_state.setText("• Listening")
                self.lbl_state.setStyleSheet("color:#27ae60; font-weight:600;")
        else:
            self.lbl_state.setText("• Disconnected")
            self.lbl_state.setStyleSheet("color:#c0392b; font-weight:600;")

        # plant values
        p = snap["plant"]
        self.ed_hum.setText(f"{p['hum']:.1f}")
        self.ed_tmp.setText(f"{p['tmp']:.1f}")
        self.ed_vis.setText(f"{p['vis_m']:.0f}")
        self.l_relief.setStatus(p["relief"]); self.l_pump.setStatus(p["pump"])
        self.l_z1.setStatus(p["z1"]); self.l_z2.setStatus(p["z2"]); self.l_z3.setStatus(p["z3"])

        # debug views
        if snap["last_rx"]:
            self.rx_view.setPlainText(json.dumps(snap["last_rx"], indent=2))
        status_frame = {
            "r1": round(p["hum"], 1), "r2": round(p["tmp"], 1),
            "r3": 1 if p["relief"] else 0, "r4": 1 if p["pump"] else 0,
            "r5": int(p["vis_m"]), "r6": bool(p["z1"]),
            "r7": bool(p["z2"]), "r8": bool(p["z3"]),
        }
        self.tx_view.setPlainText(json.dumps(status_frame, indent=2))

def main():
    app = QtWidgets.QApplication(sys.argv)
    w = SimWindow(); w.resize(1100, 650); w.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
