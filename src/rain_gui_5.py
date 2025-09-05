import sys, json, socket, threading
from PyQt6.QtWidgets import (
    QApplication, QWidget, QPushButton, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QGridLayout, QFrame, QCheckBox, QSizePolicy, QSpacerItem
)
from PyQt6.QtGui import QColor, QPainter, QFont
from PyQt6.QtCore import Qt, pyqtSignal, QObject, QSize, QTimer


# ---------- Small round status LED ----------
class CircleIndicator(QLabel):
    def __init__(self, size=16, color="grey"):
        super().__init__()
        self._color = QColor(color)
        self._size = size
        self.setMinimumSize(QSize(size, size))
        self.setMaximumSize(QSize(size, size))
        self.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
    def setColor(self, color):
        self._color = QColor(color); self.update()
    def paintEvent(self, e):
        p = QPainter(self); p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.setBrush(self._color); p.setPen(Qt.PenStyle.NoPen)
        p.drawEllipse(0, 0, self._size, self._size)


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

        lay = QVBoxLayout(self)
        lay.setContentsMargins(8, 6, 8, 6)
        lay.setSpacing(2)

        self.status_lbl = QLabel("● Disconnected")
        self.status_lbl.setStyleSheet("color:red;font-weight:bold;font-size:13px;")
        self.ip_lbl = QLabel(f"IP: {ip}   Port: {port}")
        self.ip_lbl.setStyleSheet("color:#ccc;font-size:11px;")

        lay.addWidget(self.status_lbl)
        lay.addWidget(self.ip_lbl)

    def set_connected(self, ok: bool):
        if ok:
            self.status_lbl.setText("● Connected")
            self.status_lbl.setStyleSheet("color:green;font-weight:bold;font-size:13px;")
        else:
            self.status_lbl.setText("● Disconnected")
            self.status_lbl.setStyleSheet("color:red;font-weight:bold;font-size:13px;")


# ======================= Main GUI =======================
class RainSystemGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Rain System Control Panel")
        self.resize(400, 400)  # starts here, fully resizable

        # TCP config
        self.tcp_ip = "127.0.0.1"
        self.tcp_port = 5055
        self.sock = None
        self.connected = False

        # Control state (Python -> LabVIEW)
        self.state = {"com":0,"v1":0,"v2":0,"v3":0,"v4":0,"v5":0,"v6":0,"cv":0,"wv":0,"int":0}

        SP = 10  # one place to control spacing/margins

        # ---------- Root two-column layout ----------
        root = QHBoxLayout(self)
        root.setContentsMargins(SP, SP, SP, SP)
        root.setSpacing(SP)

        # ===== Left column: 3x3 grid + bottom bar =====
        left_col = QVBoxLayout()
        left_col.setContentsMargins(0, 0, 0, 0)
        left_col.setSpacing(SP)

        grid = QGridLayout()
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setHorizontalSpacing(SP)
        grid.setVerticalSpacing(SP)

        def add_btn(text, key, r, c):
            b = QPushButton(text)
            b.setCheckable(True)
            b.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
            b.setMinimumSize(90, 58)   # keeps them usable when very small
            b.setStyleSheet("background:lightgray;color:black;font-size:12px;font-weight:bold;")
            b.clicked.connect(lambda chk, k=key, btn=b: self.toggle_state(k, btn, chk))
            grid.addWidget(b, r, c)
            return b

        self.btn_com = add_btn("ON/OFF", "com", 0, 0)
        self.btn_cv  = add_btn("Suplen", "cv", 0, 1)
        self.btn_wv  = add_btn("Wasser", "wv", 0, 2)

        self.btn_v1 = add_btn("Seg 1", "v1", 1, 0)
        self.btn_v2 = add_btn("Seg 2", "v2", 1, 1)
        self.btn_v3 = add_btn("Seg 3", "v3", 1, 2)

        self.btn_v4 = add_btn("Seg 4", "v4", 2, 0)
        self.btn_v5 = add_btn("Seg 5", "v5", 2, 1)
        self.btn_v6 = add_btn("Seg 6", "v6", 2, 2)

        # make 3x3 cells share space equally
        for i in range(3):
            grid.setColumnStretch(i, 1)
            grid.setRowStretch(i, 1)

        # bottom bar (intensity + Send)
        bottom = QHBoxLayout()
        bottom.setContentsMargins(0, 0, 0, 0)
        bottom.setSpacing(SP)

        self.intensity_input = QLineEdit()
        self.intensity_input.setPlaceholderText("Intensity (mm/h)")
        self.intensity_input.setMinimumHeight(34)
        self.intensity_input.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.intensity_input.setStyleSheet("font-size:12px;")

        self.send_btn = QPushButton("Send")
        self.send_btn.setMinimumHeight(34)
        self.send_btn.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
        self.send_btn.setStyleSheet("background:steelblue;color:white;font-weight:bold;font-size:12px;")
        self.send_btn.clicked.connect(self.send_json)

        bottom.addWidget(self.intensity_input, 3)  # give line edit more stretch
        bottom.addWidget(self.send_btn, 1)

        left_col.addLayout(grid, 6)   # grid takes more vertical room
        left_col.addLayout(bottom, 1)

        root.addLayout(left_col, 3)   # left column wider than right

        # ===== Right column: connection box + status + blue values =====
        right_col = QVBoxLayout()
        right_col.setContentsMargins(0, 0, 0, 0)
        right_col.setSpacing(SP)

        self.conn_box = ConnectionBox(self.tcp_ip, self.tcp_port)
        right_col.addWidget(self.conn_box)

        # status names + dots in a grid so left edge == blue bars
        status_grid = QGridLayout()
        status_grid.setContentsMargins(0, 0, 0, 0)
        status_grid.setHorizontalSpacing(SP)
        status_grid.setVerticalSpacing(SP)

        self.status_indicators = {}
        name_font = QFont("Arial", 12)

        for r, key in enumerate(["test_start", "pump1", "pump2", "pump3"]):
            label_text = "TestStart:" if key == "test_start" else f"{key.capitalize()}:"
            name = QLabel(label_text)
            name.setFont(name_font)
            name.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
            name.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)

            dot = CircleIndicator(size=16)

            status_grid.addWidget(name, r, 0)
            status_grid.addWidget(dot,  r, 1)
            self.status_indicators[key] = dot

        # allow column 0 (names) to grow; column 1 (dots) stays minimal
        status_grid.setColumnStretch(0, 1)
        status_grid.setColumnStretch(1, 0)

        # Blue value bars under the indicators, spanning both columns
        self.labels = {}
        pretty_map = {"int1":"Intensity1", "int2":"Intensity2", "int3":"Intensity3", "main_tank":"MainTank"}
        row_start = 4
        for i, key in enumerate(["int1", "int2", "int3", "main_tank"]):
            lbl = QLabel(f"{pretty_map[key]}: --")
            lbl.setStyleSheet("background:lightblue;font-size:12px;color:black;")
            lbl.setMinimumHeight(36)
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            lbl.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
            status_grid.addWidget(lbl, row_start + i, 0, 1, 2)  # span both columns
            self.labels[key] = lbl

        right_col.addLayout(status_grid, 7)

        # Auto-broadcast at bottom; push it down with a spacer for nice balance
        right_col.addItem(QSpacerItem(0, 0, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding))
        self.auto_cb = QCheckBox("Auto broadcast (1 Hz)")
        self.auto_cb.setStyleSheet("font-size:12px;")
        right_col.addWidget(self.auto_cb)

        root.addLayout(right_col, 2)

        # ---------- Signals/threads ----------
        self.c = Communicate()
        self.c.update_status.connect(self.update_status_panel)
        self.c.update_connection.connect(self.update_connection_status)
        threading.Thread(target=self.tcp_loop, daemon=True).start()

        # 1 Hz auto-broadcast
        self.timer = QTimer(self)
        self.timer.setInterval(1000)
        self.timer.timeout.connect(self.send_json)
        self.auto_cb.toggled.connect(lambda on: (self.timer.start() if on else self.timer.stop()))

    # ---------- Control behaviour ----------
    def toggle_state(self, key, btn, checked):
        self.state[key] = 1 if checked else 0
        btn.setStyleSheet(("background:green;" if checked else "background:lightgray;")
                          + "color:black;font-size:12px;font-weight:bold;")

    def send_json(self):
        if not self.connected:
            return
        try:
            self.state["int"] = int(self.intensity_input.text()) if self.intensity_input.text() else 0
            self.sock.sendall((json.dumps(self.state) + "\n").encode())
            print("📤 Sent:", json.dumps(self.state, indent=2))
        except Exception as e:
            print("⚠️ Send failed:", e)

    # ---------- Stream JSON extractor (works with or without '\n') ----------
    def _extract_json_objects(self, buffer: str):
        objs, depth, start = [], 0, None
        for i, ch in enumerate(buffer):
            if ch == '{':
                if depth == 0:
                    start = i
                depth += 1
            elif ch == '}':
                depth -= 1
                if depth == 0 and start is not None:
                    objs.append(buffer[start:i+1])
                    start = None
        tail = "" if depth == 0 else buffer[start:]
        return objs, tail

    # ---------- TCP loop ----------
    def tcp_loop(self):
        buffer = ""
        while True:
            try:
                if not self.connected:
                    try:
                        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                        self.sock.connect((self.tcp_ip, self.tcp_port))
                        self.connected = True
                        self.c.update_connection.emit(True)
                        print(f"✅ Connected to {self.tcp_ip}:{self.tcp_port}")
                    except:
                        self.connected = False
                        self.c.update_connection.emit(False)
                        continue

                data = self.sock.recv(4096)
                if not data:
                    self.connected = False
                    self.c.update_connection.emit(False)
                    continue

                buffer += data.decode(errors="ignore")
                objs, buffer = self._extract_json_objects(buffer)
                for raw in objs:
                    try:
                        obj = json.loads(raw)
                        self.c.update_status.emit(obj)
                        print("📥 Received:", json.dumps(obj, indent=2))
                    except Exception as e:
                        print("⚠️ Parse error:", e, "\nRaw:", raw)

            except Exception as e:
                print("⚠️ TCP loop error:", e)
                self.connected = False
                self.c.update_connection.emit(False)

    # ---------- UI updates ----------
    def update_status_panel(self, status: dict):
        for k, dot in self.status_indicators.items():
            if k in status and isinstance(status[k], bool):
                dot.setColor("green" if status[k] else "red")
        for k, lbl in self.labels.items():
            if k in status and isinstance(status[k], (int, float)):
                name = lbl.text().split(":")[0]
                lbl.setText(f"{name}: {status[k]}")

    def update_connection_status(self, ok: bool):
        self.conn_box.set_connected(ok)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = RainSystemGUI()
    gui.show()
    sys.exit(app.exec())