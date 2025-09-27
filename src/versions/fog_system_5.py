# fog_system_gui_rain_tcp_kb.py
# Fog System GUI — same TCP as your rain app + keyboard shortcuts + exclusive modes.

import sys, json, socket, threading, time
from PyQt6.QtWidgets import (
    QApplication, QWidget, QPushButton, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QGridLayout, QFrame, QCheckBox, QSizePolicy,
    QSpacerItem, QSpinBox, QStackedWidget, QGroupBox, QFormLayout
)
from PyQt6.QtGui import QColor, QPainter, QFont, QKeySequence, QShortcut
from PyQt6.QtCore import Qt, pyqtSignal, QObject, QSize, QTimer
from PyQt6.QtWidgets import QButtonGroup

# ---------- Small round status LED ----------
class CircleIndicator(QLabel):
    def __init__(self, size=16, color="grey"):
        super().__init__()
        self._color = QColor(color); self._size = size
        self.setMinimumSize(QSize(size, size)); self.setMaximumSize(QSize(size, size))
        self.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
    def setColor(self, color): self._color = QColor(color); self.update()
    def paintEvent(self, e):
        p = QPainter(self); p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.setBrush(self._color); p.setPen(Qt.PenStyle.NoPen); p.drawEllipse(0, 0, self._size, self._size)

# ---------- Thread → UI signals ----------
class Communicate(QObject):
    update_status = pyqtSignal(dict)
    update_connection = pyqtSignal(bool)

# ---------- Connection status box ----------
class ConnectionBox(QFrame):
    def __init__(self, ip: str, port: int):
        super().__init__()
        self.setObjectName("connBox")
        self.setStyleSheet("""QFrame#connBox{border:1px solid #777;border-radius:6px;background:#222;}""")
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.setMinimumHeight(68)
        lay = QVBoxLayout(self); lay.setContentsMargins(8,6,8,6); lay.setSpacing(2)
        self.status_lbl = QLabel("● Disconnected")
        self.status_lbl.setStyleSheet("color:red;font-weight:bold;font-size:13px;")
        self.ip_lbl = QLabel(f"IP: {ip}   Port: {port}")
        self.ip_lbl.setStyleSheet("color:#ccc;font-size:11px;")
        lay.addWidget(self.status_lbl); lay.addWidget(self.ip_lbl)
    def set_connected(self, ok: bool):
        if ok:
            self.status_lbl.setText("● Connected"); self.status_lbl.setStyleSheet("color:green;font-weight:bold;font-size:13px;")
        else:
            self.status_lbl.setText("● Disconnected"); self.status_lbl.setStyleSheet("color:red;font-weight:bold;font-size:13px;")

# ======================= Fog GUI (rain-style TCP) =======================
class FogSystemGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Fog System Control (Rain-style TCP)")
        self.resize(600,300)

        # === TCP config (same as rain) ===
        self.tcp_ip = "127.0.0.1"    # set your LV IP
        self.tcp_port = 5052         # set your LV port
        self.sock = None
        self.connected = False

        # === Control state to send (b1..b19) ===
        self.state = {
            "b1": False, "b2": False, "b3": False, "b4": False,
            "b5": False, "b6": False, "b7": False, "b8": False,
            "b9": False, "b10": False, "b11": False,
            "b12": 0, "b13": 0, "b14": 0, "b15": 0, "b16": 0, "b17": 0,
            "b18": False, "b19": 0
        }

        SP = 10
        root = QHBoxLayout(self); root.setContentsMargins(SP, SP, SP, SP); root.setSpacing(SP)

        # ===== Left: ON/OFF (top) → 3 modes → mode panel =====
        left = QVBoxLayout(); left.setSpacing(SP)

        def bigbtn(text):
            b = QPushButton(text); b.setCheckable(True); b.setMinimumSize(180, 110)
            b.setStyleSheet("""
                QPushButton { background:lightgray; color:black; font-size:14px; font-weight:bold; border-radius:10px; }
                QPushButton:checked { background:#19a14a; color:black; }
            """); return b

        # ON/OFF
        self.btn_onoff = bigbtn("ON/OFF"); self.btn_onoff.toggled.connect(lambda v: self._set("b1", bool(v)))
        left.addWidget(self.btn_onoff)

        # mode row (exclusive group)
        mode_row = QHBoxLayout()
        self.btn_manual   = bigbtn("Manual")
        self.btn_interval = bigbtn("Interval")
        self.btn_vis      = bigbtn("Visibility")
        mode_row.addWidget(self.btn_manual); mode_row.addWidget(self.btn_interval); mode_row.addWidget(self.btn_vis)
        left.addLayout(mode_row)

        self.mode_group = QButtonGroup(self)
        for i,b in enumerate((self.btn_manual, self.btn_interval, self.btn_vis)): self.mode_group.addButton(b, i)
        self.mode_group.setExclusive(True)
        self.btn_manual.toggled.connect(self._mode_toggled)
        self.btn_interval.toggled.connect(self._mode_toggled)
        self.btn_vis.toggled.connect(self._mode_toggled)

        # mode-specific stacked panel
        self.stack = QStackedWidget()
        # Manual
        p_manual = QWidget(); m = QHBoxLayout(p_manual)
        self.btn_pump = bigbtn("Pump"); self.btn_z1 = bigbtn("Zone 1")
        self.btn_z2 = bigbtn("Zone 2"); self.btn_z3 = bigbtn("Zone 3")
        for btn, key in [(self.btn_pump,"b5"), (self.btn_z1,"b6"), (self.btn_z2,"b7"), (self.btn_z3,"b8")]:
            btn.toggled.connect(lambda chk, k=key: self._set(k, bool(chk)))
        m.addWidget(self.btn_pump); m.addWidget(self.btn_z1); m.addWidget(self.btn_z2); m.addWidget(self.btn_z3)
        self.stack.addWidget(p_manual)

        # Interval
        p_interval = QWidget(); gl = QGridLayout(p_interval)
        self.cb_it_z1 = QCheckBox("Z1 enable"); self.cb_it_z2 = QCheckBox("Z2 enable"); self.cb_it_z3 = QCheckBox("Z3 enable")
        self.cb_it_z1.toggled.connect(lambda v: self._set("b9",  bool(v)))
        self.cb_it_z2.toggled.connect(lambda v: self._set("b10", bool(v)))
        self.cb_it_z3.toggled.connect(lambda v: self._set("b11", bool(v)))
        def sp(): s=QSpinBox(); s.setRange(0,3600); s.setSingleStep(1); s.setKeyboardTracking(True); return s
        self.sp_z1_on, self.sp_z2_on, self.sp_z3_on = sp(), sp(), sp()
        self.sp_z1_off, self.sp_z2_off, self.sp_z3_off = sp(), sp(), sp()
        self.sp_z1_on.valueChanged.connect(lambda v: self._set("b12", int(v)))
        self.sp_z2_on.valueChanged.connect(lambda v: self._set("b13", int(v)))
        self.sp_z3_on.valueChanged.connect(lambda v: self._set("b14", int(v)))
        self.sp_z1_off.valueChanged.connect(lambda v: self._set("b15", int(v)))
        self.sp_z2_off.valueChanged.connect(lambda v: self._set("b16", int(v)))
        self.sp_z3_off.valueChanged.connect(lambda v: self._set("b17", int(v)))
        gl.addWidget(QLabel("Enable:"),0,0); gl.addWidget(self.cb_it_z1,0,1); gl.addWidget(self.cb_it_z2,0,2); gl.addWidget(self.cb_it_z3,0,3)
        gl.addWidget(QLabel("On (s)"),1,0); gl.addWidget(self.sp_z1_on,1,1); gl.addWidget(self.sp_z2_on,1,2); gl.addWidget(self.sp_z3_on,1,3)
        gl.addWidget(QLabel("Off (s)"),2,0); gl.addWidget(self.sp_z1_off,2,1); gl.addWidget(self.sp_z2_off,2,2); gl.addWidget(self.sp_z3_off,2,3)
        self.stack.addWidget(p_interval)

        # Visibility
        p_vis = QWidget(); f = QFormLayout(p_vis)
        self.cb_vs_start = QCheckBox("Start regulation"); self.cb_vs_start.toggled.connect(lambda v: self._set("b18", bool(v)))
        self.sp_vs_tgt = QSpinBox(); self.sp_vs_tgt.setRange(0,100000); self.sp_vs_tgt.setKeyboardTracking(True)
        self.sp_vs_tgt.valueChanged.connect(lambda v: self._set("b19", int(v)))
        f.addRow(self.cb_vs_start); f.addRow("Target visibility (m):", self.sp_vs_tgt)
        self.stack.addWidget(p_vis)

        left.addWidget(self.stack, 1)

        # bottom controls: Send / Auto broadcast (1 Hz)
        bottom = QHBoxLayout()
        self.send_btn = QPushButton("Send Now"); self.send_btn.setMinimumHeight(36)
        self.send_btn.setStyleSheet("background:steelblue;color:white;font-weight:bold;font-size:12px;")
        self.send_btn.clicked.connect(self.send_json)
        self.auto_cb = QCheckBox("Auto broadcast (1 Hz)")
        bottom.addWidget(self.send_btn); bottom.addWidget(self.auto_cb)
        left.addLayout(bottom)
        root.addLayout(left, 3)

        # ===== Right: connection box + status (r1..r8) =====
        right = QVBoxLayout(); right.setSpacing(SP)
        self.conn_box = ConnectionBox(self.tcp_ip, self.tcp_port); right.addWidget(self.conn_box)

        status_grid = QGridLayout(); status_grid.setHorizontalSpacing(SP); status_grid.setVerticalSpacing(SP)
        self.val_boxes = {}
        def val_row(r, key, title):
            name = QLabel(title); name.setFont(QFont("Arial", 12))
            edit = QLineEdit(); edit.setReadOnly(True); edit.setPlaceholderText("--"); edit.setMinimumHeight(32)
            status_grid.addWidget(name, r, 0); status_grid.addWidget(edit, r, 1); self.val_boxes[key]=edit
        val_row(0,"r1","Humidity (%)"); val_row(1,"r2","Temperature (°C)"); val_row(2,"r5","Visibility IST (m)")

        lamp_grid = QGridLayout(); self.status_indicators={}
        def lamp_row(r, key, title):
            name = QLabel(title); name.setFont(QFont("Arial",12))
            dot = CircleIndicator(16, "red")
            lamp_grid.addWidget(name, r, 0); lamp_grid.addWidget(dot, r, 1); self.status_indicators[key]=dot
        lamp_row(0,"r3","Relief Valve"); lamp_row(1,"r4","Pump"); lamp_row(2,"r6","Zone 1"); lamp_row(3,"r7","Zone 2"); lamp_row(4,"r8","Zone 3")

        right.addLayout(status_grid); right.addLayout(lamp_grid)
        right.addItem(QSpacerItem(0,0,QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding))
        root.addLayout(right, 2)

        # ---------- Signals/threads ----------
        self.c = Communicate()
        self.c.update_status.connect(self.update_status_panel)
        self.c.update_connection.connect(self.update_connection_status)
        threading.Thread(target=self.tcp_loop, daemon=True).start()

        # 1 Hz auto-broadcast (same as rain)
        self.timer = QTimer(self); self.timer.setInterval(1000)
        self.timer.timeout.connect(self.send_json)
        self.auto_cb.toggled.connect(lambda on: (self.timer.start() if on else self.timer.stop()))

        # ---------- Keyboard shortcuts ----------
        QShortcut(QKeySequence("Ctrl+O"), self, lambda: self.btn_onoff.toggle())
        QShortcut(QKeySequence("M"), self, lambda: self._select_mode("manual"))
        QShortcut(QKeySequence("I"), self, lambda: self._select_mode("interval"))
        QShortcut(QKeySequence("V"), self, lambda: self._select_mode("visibility"))
        QShortcut(QKeySequence("Return"), self, self.send_json)

        # Manual mode toggles
        QShortcut(QKeySequence("P"), self, lambda: self.btn_pump.toggle())
        QShortcut(QKeySequence("1"), self, lambda: self.btn_z1.toggle())
        QShortcut(QKeySequence("2"), self, lambda: self.btn_z2.toggle())
        QShortcut(QKeySequence("3"), self, lambda: self.btn_z3.toggle())

        # Interval focus shortcuts (then use ↑/↓ or type)
        QShortcut(QKeySequence("Alt+1"), self, lambda: self.sp_z1_on.setFocus())
        QShortcut(QKeySequence("Alt+2"), self, lambda: self.sp_z2_on.setFocus())
        QShortcut(QKeySequence("Alt+3"), self, lambda: self.sp_z3_on.setFocus())
        QShortcut(QKeySequence("Shift+Alt+1"), self, lambda: self.sp_z1_off.setFocus())
        QShortcut(QKeySequence("Shift+Alt+2"), self, lambda: self.sp_z2_off.setFocus())
        QShortcut(QKeySequence("Shift+Alt+3"), self, lambda: self.sp_z3_off.setFocus())

        # Visibility shortcuts
        QShortcut(QKeySequence("S"), self, lambda: self.cb_vs_start.toggle())
        QShortcut(QKeySequence("T"), self, lambda: self.sp_vs_tgt.setFocus())

    # ===== Mode handling =====
    def _select_mode(self, which: str):
        if which=="manual":   self.btn_manual.setChecked(True)
        elif which=="interval": self.btn_interval.setChecked(True)
        elif which=="visibility": self.btn_vis.setChecked(True)
        # _mode_toggled will run due to setChecked(True)

    def _mode_toggled(self, _checked: bool):
        # Update state booleans from the exclusive group
        self.state["b2"] = self.btn_manual.isChecked()
        self.state["b3"] = self.btn_interval.isChecked()
        self.state["b4"] = self.btn_vis.isChecked()
        # swap panel
        if self.state["b2"]: self.stack.setCurrentIndex(0)
        elif self.state["b3"]: self.stack.setCurrentIndex(1)
        elif self.state["b4"]: self.stack.setCurrentIndex(2)
        # neutralize & sync UI so other modes "shut down"
        self._neutralize_for_mode()
        self._sync_widgets_from_state()

    def _set(self, key, val):
        self.state[key] = val
        self._neutralize_for_mode()

    def _neutralize_for_mode(self):
        if not self.state["b1"]:
            for k in list(self.state.keys()):
                if k == "b1": continue
                self.state[k] = False if isinstance(self.state[k], bool) else 0
            return
        if self.state["b2"] and not self.state["b3"] and not self.state["b4"]:
            for k in ("b9","b10","b11"): self.state[k]=False
            for k in ("b12","b13","b14","b15","b16","b17"): self.state[k]=0
            self.state["b18"]=False; self.state["b19"]=0
        elif self.state["b3"] and not self.state["b2"] and not self.state["b4"]:
            for k in ("b5","b6","b7","b8"): self.state[k]=False
            self.state["b18"]=False; self.state["b19"]=0
        elif self.state["b4"] and not self.state["b2"] and not self.state["b3"]:
            for k in ("b5","b6","b7","b8"): self.state[k]=False
            for k in ("b9","b10","b11"): self.state[k]=False
            for k in ("b12","b13","b14","b15","b16","b17"): self.state[k]=0
        elif not self.state["b2"] and not self.state["b3"] and not self.state["b4"]:
            for k in list(self.state.keys()):
                if k == "b1": continue
                self.state[k] = False if isinstance(self.state[k], bool) else 0

    def _sync_widgets_from_state(self):
        # make UI reflect state after mode switch so "others shut down" visually too
        def set_checked(widget, value):
            block = widget.blockSignals(True)
            widget.setChecked(bool(value))
            widget.blockSignals(block)
        # Modes
        set_checked(self.btn_manual, self.state["b2"])
        set_checked(self.btn_interval, self.state["b3"])
        set_checked(self.btn_vis, self.state["b4"])
        # Manual
        set_checked(self.btn_pump, self.state["b5"])
        set_checked(self.btn_z1,   self.state["b6"])
        set_checked(self.btn_z2,   self.state["b7"])
        set_checked(self.btn_z3,   self.state["b8"])
        # Interval enables
        set_checked(self.cb_it_z1, self.state["b9"])
        set_checked(self.cb_it_z2, self.state["b10"])
        set_checked(self.cb_it_z3, self.state["b11"])
        # Interval times
        for spin, key in [(self.sp_z1_on,"b12"),(self.sp_z2_on,"b13"),(self.sp_z3_on,"b14"),
                          (self.sp_z1_off,"b15"),(self.sp_z2_off,"b16"),(self.sp_z3_off,"b17")]:
            block = spin.blockSignals(True); spin.setValue(int(self.state[key])); spin.blockSignals(block)
        # Visibility
        set_checked(self.cb_vs_start, self.state["b18"])
        block = self.sp_vs_tgt.blockSignals(True); self.sp_vs_tgt.setValue(int(self.state["b19"])); self.sp_vs_tgt.blockSignals(block)

    # ===== Sending (identical approach to rain) =====
    def _build_payload(self):
        order = ["b1","b2","b3","b4","b5","b6","b7","b8","b9","b10","b11","b12","b13","b14","b15","b16","b17","b18","b19"]
        return {k: self.state[k] for k in order}
    def send_json(self):
        if not self.connected: return
        try:
            payload = self._build_payload()
            self.sock.sendall((json.dumps(payload) + "\n").encode())
        except Exception as e:
            print("⚠️ Send failed:", e)

    # ===== Stream JSON extractor (same as rain) =====
    def _extract_json_objects(self, buffer: str):
        objs, depth, start = [], 0, None
        for i, ch in enumerate(buffer):
            if ch == '{':
                if depth == 0: start = i
                depth += 1
            elif ch == '}':
                depth -= 1
                if depth == 0 and start is not None:
                    objs.append(buffer[start:i+1]); start = None
        tail = "" if depth == 0 else buffer[start:]
        return objs, tail

    # ===== TCP loop (same pattern as rain) =====
    def tcp_loop(self):
        buffer = ""
        while True:
            try:
                if not self.connected:
                    try:
                        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                        self.sock.connect((self.tcp_ip, self.tcp_port))
                        self.connected = True; self.c.update_connection.emit(True)
                        print(f"✅ Connected to {self.tcp_ip}:{self.tcp_port}")
                    except:
                        self.connected = False; self.c.update_connection.emit(False)
                        time.sleep(0.3); continue
                data = self.sock.recv(4096)
                if not data:
                    self.connected = False; self.c.update_connection.emit(False); continue
                buffer += data.decode(errors="ignore")
                objs, buffer = self._extract_json_objects(buffer)
                for raw in objs:
                    try:
                        obj = json.loads(raw); self.c.update_status.emit(obj)
                    except Exception as e:
                        print("⚠️ Parse error:", e, "\nRaw:", raw)
            except Exception as e:
                print("⚠️ TCP loop error:", e); self.connected = False; self.c.update_connection.emit(False); time.sleep(0.3)

    # ===== UI updates from r1..r8 =====
    def update_status_panel(self, status: dict):
        if "r1" in status: self.val_boxes["r1"].setText(f"{float(status['r1']):.1f}")
        if "r2" in status: self.val_boxes["r2"].setText(f"{float(status['r2']):.1f}")
        if "r5" in status: self.val_boxes["r5"].setText(f"{float(status['r5']):.0f}")
        def set_lamp(key):
            if key in status:
                v = status[key]; on = bool(v) if not isinstance(v,(int,float)) else v != 0
                self.status_indicators[key].setColor("green" if on else "red")
        for k in ("r3","r4","r6","r7","r8"): set_lamp(k)
    def update_connection_status(self, ok: bool): self.conn_box.set_connected(ok)

# ---------- main ----------
if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = FogSystemGUI()
    gui.show()
    sys.exit(app.exec())
