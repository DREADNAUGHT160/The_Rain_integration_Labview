# file: rain_GUI_newline.py
import sys, json, socket, threading
from PyQt6.QtWidgets import (
    QApplication, QWidget, QPushButton, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QGridLayout, QFrame, QCheckBox,
    QSizePolicy, QSpacerItem
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
        self._color = QColor(color)
        self.update()

    def paintEvent(self, e):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.setBrush(self._color)
        p.setPen(Qt.PenStyle.NoPen)
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

    def set_ip_port(self, ip, port):
        self.ip_lbl.setText(f"IP: {ip}   Port: {port}")


# ======================= Main GUI =======================
class RainSystemGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Rain System Control Panel")
        self.resize(900, 560)
        self.setMinimumSize(700, 420)

        # TCP configuration
        self.tcp_ip = "192.168.1.159"
        self.tcp_port = 5052
        self.sock = None
        self.connected = False

        # Control state
        self.state = {"com":0,"v1":0,"v2":0,"v3":0,"v4":0,"v5":0,"v6":0,"cv":0,"wv":0,"int":0}

        SP = 10  # spacing

        # ---------- Layout ----------
        root = QHBoxLayout(self)
        root.setContentsMargins(SP, SP, SP, SP)
        root.setSpacing(SP)

        # ===== Left column =====
        left_col = QVBoxLayout()
        left_col.setContentsMargins(0,0,0,0)
        left_col.setSpacing(SP)

        grid = QGridLayout()
        grid.setContentsMargins(0,0,0,0)
        grid.setHorizontalSpacing(SP)
        grid.setVerticalSpacing(SP)

        # store segment buttons
        self.seg_buttons = []

        def add_btn(text, key, r, c, is_segment=False):
            b = QPushButton(text)
            b.setCheckable(True)
            b.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
            b.setMinimumSize(120, 90)
            b.setStyleSheet("background:lightgray;color:black;font-size:13px;font-weight:bold;")
            b.clicked.connect(lambda chk, k=key, btn=b: self.toggle_state(k, btn, chk))
            grid.addWidget(b, r, c)
            if is_segment:
                self.seg_buttons.append((key, b))
            return b

        self.btn_com = add_btn("ON / OFF", "com", 0, 0)
        self.btn_cv  = add_btn("Cleaning", "cv",  0, 1)
        self.btn_wv  = add_btn("Water", "wv",  0, 2)
        add_btn("Seg 1", "v1", 1, 0, True)
        add_btn("Seg 2", "v2", 1, 1, True)
        add_btn("Seg 3", "v3", 1, 2, True)
        add_btn("Seg 4", "v4", 2, 0, True)
        add_btn("Seg 5", "v5", 2, 1, True)
        add_btn("Seg 6", "v6", 2, 2, True)

        for i in range(3):
            grid.setColumnStretch(i, 1)
            grid.setRowStretch(i, 1)

        bottom = QHBoxLayout()
        bottom.setContentsMargins(0,0,0,0)
        bottom.setSpacing(SP)

        self.intensity_input = QLineEdit()
        self.intensity_input.setPlaceholderText("Rain intensity (mm/h)")
        self.intensity_input.setMinimumHeight(36)
        self.intensity_input.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.intensity_input.setStyleSheet("font-size:13px;")

        self.send_btn = QPushButton("Send")
        self.send_btn.setMinimumHeight(36)
        self.send_btn.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
        self.send_btn.setStyleSheet("background:steelblue;color:white;font-weight:bold;font-size:13px;")
        self.send_btn.clicked.connect(self.send_json)

        bottom.addWidget(self.intensity_input, 3)
        bottom.addWidget(self.send_btn, 1)

        left_col.addLayout(grid, 6)
        left_col.addLayout(bottom, 1)
        root.addLayout(left_col, 3)

        # ===== Right column =====
        right_col = QVBoxLayout()
        right_col.setContentsMargins(0,0,0,0)
        right_col.setSpacing(SP)

        self.conn_box = ConnectionBox(self.tcp_ip, self.tcp_port)
        right_col.addWidget(self.conn_box)

        # --- IP/Port edit row ---
        ip_port_row = QHBoxLayout()
        self.ip_input = QLineEdit(self.tcp_ip)
        self.ip_input.setPlaceholderText("IP")
        self.ip_input.setMinimumHeight(30)
        self.ip_input.setStyleSheet("font-size:12px;")
        self.port_input = QLineEdit(str(self.tcp_port))
        self.port_input.setPlaceholderText("Port")
        self.port_input.setMinimumHeight(30)
        self.port_input.setStyleSheet("font-size:12px;")
        self.set_conn_btn = QPushButton("Update Connection")
        self.set_conn_btn.setMinimumHeight(30)
        self.set_conn_btn.setStyleSheet("background:orange;color:black;font-weight:bold;font-size:12px;")
        self.set_conn_btn.clicked.connect(self.update_ip_port)
        ip_port_row.addWidget(self.ip_input)
        ip_port_row.addWidget(self.port_input)
        ip_port_row.addWidget(self.set_conn_btn)
        right_col.addLayout(ip_port_row)

        status_grid = QGridLayout()
        status_grid.setContentsMargins(0,0,0,0)
        status_grid.setHorizontalSpacing(SP)
        status_grid.setVerticalSpacing(SP)

        self.status_indicators = {}
        name_font = QFont("Arial", 13)
        for r, key in enumerate(["test_start", "pump1", "pump2", "pump3"]):
            label_text = "Test Start:" if key == "test_start" else f"{key.capitalize()}:"
            name = QLabel(label_text)
            name.setFont(name_font)
            name.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
            name.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
            dot = CircleIndicator(size=16)
            status_grid.addWidget(name, r, 0)
            status_grid.addWidget(dot, r, 1)
            self.status_indicators[key] = dot

        status_grid.setColumnStretch(0, 1)
        status_grid.setColumnStretch(1, 0)

        self.labels = {}
        pretty_map = {"int1":"Intensity 1", "int2":"Intensity 2", "int3":"Intensity 3", "main_tank":"Main Tank"}
        row_start = 4
        for i, key in enumerate(["int1", "int2", "int3", "main_tank"]):
            lbl = QLabel(f"{pretty_map[key]}: 0.0")
            lbl.setStyleSheet("background:lightblue;font-size:13px;color:black;")
            lbl.setMinimumHeight(40)
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            lbl.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
            status_grid.addWidget(lbl, row_start + i, 0, 1, 2)
            self.labels[key] = lbl

        right_col.addLayout(status_grid, 7)
        right_col.addItem(QSpacerItem(0, 0, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding))
        self.auto_cb = QCheckBox("Automatic sending (1 Hz)")
        self.auto_cb.setStyleSheet("font-size:12px;")
        right_col.addWidget(self.auto_cb)

        root.addLayout(right_col, 2)

        # ---------- Signals / Threads ----------
        self.c = Communicate()
        self.c.update_status.connect(self.update_status_panel)
        self.c.update_connection.connect(self.update_connection_status)
        threading.Thread(target=self.tcp_loop, daemon=True).start()

        self.timer = QTimer(self)
        self.timer.setInterval(1000)
        self.timer.timeout.connect(self.send_json)
        self.auto_cb.toggled.connect(lambda on: (self.timer.start() if on else self.timer.stop()))

    # ---------- Control logic ----------
    def toggle_state(self, key, btn, checked):
        # Update state for the clicked button
        self.state[key] = 1 if checked else 0
        btn.setStyleSheet(("background:green;" if checked else "background:lightgray;") +
                          "color:black;font-size:13px;font-weight:bold;")

        # 🔹 Cleaning ON → turn all segments ON, Cleaning OFF → turn all segments OFF
        if key == "cv":
            for seg_key, seg_btn in self.seg_buttons:
                seg_btn.setChecked(checked)
                seg_btn.setStyleSheet(
                    "background:green;color:black;font-size:13px;font-weight:bold;"
                    if checked else
                    "background:lightgray;color:black;font-size:13px;font-weight:bold;"
                )
                self.state[seg_key] = 1 if checked else 0

    # ---------- SEND JSON (with newline) ----------
    def send_json(self):
        if not self.connected:
            return
        try:
            self.state["int"] = int(self.intensity_input.text()) if self.intensity_input.text() else 0
            payload = (json.dumps(self.state, separators=(",", ":")) + "\n").encode("utf-8")
            self.sock.sendall(payload)
            print("📤 Sending JSON object:")
            print(json.dumps(self.state, indent=2))
            print(f"• Bytes sent = {len(payload)}")
            print(f"• First 100 Bytes = {repr(payload[:100])}")
        except Exception as e:
            print("⚠️ Sending failed:", e)

    # ---------- TCP loop ----------
    def tcp_loop(self):
        buffer = b""
        while True:
            try:
                if not self.connected:
                    try:
                        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                        self.sock.connect((self.tcp_ip, self.tcp_port))
                        self.connected = True
                        self.c.update_connection.emit(True)
                    except:
                        self.connected = False
                        self.c.update_connection.emit(False)
                        continue

                data = self.sock.recv(4096)
                print("📡 Raw data from server:", data)
                if not data:
                    self.connected = False
                    self.c.update_connection.emit(False)
                    continue

                buffer += data
                messages = buffer.split(b"\n")
                buffer = messages.pop()

                for msg in messages:
                    if msg:
                        try:
                            obj = json.loads(msg.decode("utf-8"))
                            self.c.update_status.emit(obj)
                            print("📥 Received:", json.dumps(obj, indent=2))
                        except Exception as e:
                            print("⚠️ Parse error:", e, "\nRaw data:", msg)

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

    # --- Update IP/Port from GUI ---
    def update_ip_port(self):
        ip = self.ip_input.text().strip()
        port_text = self.port_input.text().strip()
        if not ip or not port_text.isdigit():
            print("⚠️ Invalid IP or port input")
            return
        self.tcp_ip = ip
        self.tcp_port = int(port_text)
        self.conn_box.set_ip_port(self.tcp_ip, self.tcp_port)
        print(f"🔄 Updating connection to {self.tcp_ip}:{self.tcp_port}")
        try:
            if self.sock:
                self.sock.close()
        except:
            pass
        self.connected = False  # tcp_loop will reconnect automatically


if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = RainSystemGUI()
    gui.show()
    sys.exit(app.exec())
