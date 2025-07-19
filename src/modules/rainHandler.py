import sys
import socket
import json
import threading
from PySide6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel,
    QProgressBar, QGridLayout, QGroupBox, QLineEdit
)
from PySide6.QtCore import Qt, Signal as pyqtSignal, QTimer

TCP_IP = '127.0.0.1'  # Change if needed
TCP_PORT = 5005

class Indicator(QLabel):
    def __init__(self, size=20, color="red"):
        super().__init__()
        self.setFixedSize(size, size)
        self.set_color(color)
    def set_color(self, color):
        self.setStyleSheet(f"""
            QLabel {{
                border-radius: {self.width() // 2}px;
                background-color: {color};
                border: 2px solid #333;
            }}
        """)

class RainWindow(QWidget):
    # Signals for thread-safe GUI updates
    update_tank_bars = pyqtSignal(list)
    update_start_test = pyqtSignal(bool)
    update_all = pyqtSignal(dict)
    update_conn_status = pyqtSignal(bool)
    send_intensity_values = pyqtSignal(list)
    send_control_status = pyqtSignal(dict)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("LabVIEW Control Panel")
        main_layout = QVBoxLayout()

        # Connection status
        conn_layout = QHBoxLayout()
        self.conn_status_label = QLabel("Connection: Disconnected")
        self.conn_status_indicator = Indicator(size=20, color="red")
        conn_layout.addWidget(self.conn_status_label)
        conn_layout.addWidget(self.conn_status_indicator)
        conn_layout.addStretch()
        main_layout.addLayout(conn_layout)

        # Master ON/OFF
        master_layout = QHBoxLayout()
        master_label = QLabel("Main Power:")
        self.master_btn = QPushButton("OFF")
        self.master_btn.setCheckable(True)
        self.master_indicator = Indicator(size=28, color="red")
        self.master_btn.clicked.connect(self.toggle_master)
        master_layout.addWidget(master_label)
        master_layout.addWidget(self.master_indicator)
        master_layout.addWidget(self.master_btn)
        master_layout.addStretch()
        main_layout.addLayout(master_layout)

        # Spulen & Wasser group
        spulen_group = QGroupBox("Systemsteuerung")
        spulen_layout = QHBoxLayout()
        self.spulen_btn = QPushButton("Spulen OFF")
        self.spulen_btn.setCheckable(True)
        self.spulen_indicator = Indicator()
        self.spulen_btn.clicked.connect(lambda: self.toggle_button(self.spulen_btn, self.spulen_indicator))
        spulen_layout.addWidget(QLabel("Spulen"))
        spulen_layout.addWidget(self.spulen_indicator)
        spulen_layout.addWidget(self.spulen_btn)
        self.wasser_btn = QPushButton("Wasser OFF")
        self.wasser_btn.setCheckable(True)
        self.wasser_indicator = Indicator()
        self.wasser_btn.clicked.connect(lambda: self.toggle_button(self.wasser_btn, self.wasser_indicator))
        spulen_layout.addSpacing(20)
        spulen_layout.addWidget(QLabel("Wasser"))
        spulen_layout.addWidget(self.wasser_indicator)
        spulen_layout.addWidget(self.wasser_btn)
        spulen_group.setLayout(spulen_layout)
        main_layout.addWidget(spulen_group)

        # Segments group
        self.seg_names = [
            "seg 1.1", "seg 1.2", "seg 2.1", "seg 2.2", "seg 3.1", "seg 3.2",
            "seg 4.1", "seg 4.2", "seg 5.1", "seg 5.2", "seg 6.1", "seg 6.2"
        ]
        seg_group = QGroupBox("Segment Steuerung")
        seg_grid = QGridLayout()
        self.seg_btns = []
        self.seg_inds = []
        for i, name in enumerate(self.seg_names):
            label = QLabel(name)
            btn = QPushButton("OFF")
            btn.setCheckable(True)
            ind = Indicator()
            btn.clicked.connect(self.make_seg_toggle(i))
            self.seg_btns.append(btn)
            self.seg_inds.append(ind)
            seg_grid.addWidget(label, i // 4, (i % 4) * 3)
            seg_grid.addWidget(ind, i // 4, (i % 4) * 3 + 1)
            seg_grid.addWidget(btn, i // 4, (i % 4) * 3 + 2)
        seg_group.setLayout(seg_grid)
        main_layout.addWidget(seg_group)

        # Intensity SEND Box
        send_intensity_layout = QHBoxLayout()
        self.intensity_input = QLineEdit()
        self.intensity_input.setPlaceholderText("Enter Intensity to Send")
        self.intensity_input.setFixedWidth(140)
        self.send_intensity_btn = QPushButton("Send to LabVIEW")
        self.send_intensity_btn.clicked.connect(self.start_sending)
        send_intensity_layout.addWidget(QLabel("Set Intensity (to LabVIEW):"))
        send_intensity_layout.addWidget(self.intensity_input)
        send_intensity_layout.addWidget(self.send_intensity_btn)
        send_intensity_layout.addStretch()
        main_layout.addLayout(send_intensity_layout)

        # Horizontal Tank Indicators (Progress Bars with Numbers)
        tank_layout = QHBoxLayout()
        self.tanks = []
        self.tank_labels = []
        for i in range(3):
            # Progress bar (tank)
            progress = QProgressBar()
            progress.setOrientation(Qt.Horizontal)
            progress.setMinimum(0)
            progress.setMaximum(100)
            progress.setValue(0)
            progress.setFixedHeight(20)  # Adjusted height for horizontal tank appearance
            tank_label = QLabel(f"Intensity {i+1}: 0", self)
            tank_label.setAlignment(Qt.AlignCenter)
            self.tanks.append(progress)
            self.tank_labels.append(tank_label)

            # Add tank and label to the horizontal layout
            tank_col = QVBoxLayout()
            tank_col.addWidget(progress)
            tank_col.addWidget(tank_label)
            tank_layout.addLayout(tank_col)

        main_layout.addLayout(tank_layout)

        # Big Tank for other variable (e.g., 'tank_value')
        self.big_tank = QProgressBar()
        self.big_tank.setOrientation(Qt.Horizontal)
        self.big_tank.setMinimum(0)
        self.big_tank.setMaximum(100)
        self.big_tank.setValue(0)
        self.big_tank.setFixedHeight(40)  # Larger height for "big tank"
        self.big_tank_label = QLabel("Big Tank Value: 0", self)
        self.big_tank_label.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(self.big_tank)
        main_layout.addWidget(self.big_tank_label)

        # Error/Status display
        self.status_label = QLabel("Status: Waiting for data...", self)
        self.status_label.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(self.status_label)

        # Start Test Indicator
        st_layout = QHBoxLayout()
        st_label = QLabel("Start Test:")
        self.start_test_indicator = Indicator(size=30, color="red")
        st_layout.addWidget(st_label)
        st_layout.addWidget(self.start_test_indicator)
        st_layout.addStretch()
        main_layout.addLayout(st_layout)

        self.setLayout(main_layout)

        # Signals for thread-safe updates
        self.send_intensity_values.connect(self.update_tank_bars)
        #self.send_control_status.connect(self.send_control_to_main)

        # Set up a timer to send data at 1-second intervals
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.send_all_states)

    def start_sending(self):
        """
        Start sending data to LabVIEW every second when the button is clicked.
        """
        self.timer.start(1000)  # Start the timer with a 1-second interval (1000 ms)
        print("Started sending data to LabVIEW every second.")

    def stop_sending(self):
        """
        Stop sending data when needed (this can be triggered by another button or condition).
        """
        self.timer.stop()
        print("Stopped sending data to LabVIEW.")

    # Button logic
    def toggle_master(self):
        if self.master_btn.isChecked():
            self.master_btn.setText("ON")
            self.master_indicator.set_color("green")
        else:
            self.master_btn.setText("OFF")
            self.master_indicator.set_color("red")

    def toggle_button(self, btn, ind):
        if btn.isChecked():
            btn.setText(btn.text().replace("OFF", "ON"))
            ind.set_color("green")
        else:
            btn.setText(btn.text().replace("ON", "OFF"))
            ind.set_color("red")

    def make_seg_toggle(self, idx):
        def toggle():
            btn = self.seg_btns[idx]
            ind = self.seg_inds[idx]
            if btn.isChecked():
                btn.setText("ON")
                ind.set_color("green")
            else:
                btn.setText("OFF")
                ind.set_color("red")
        return toggle

    # --- Send and immediately receive the LabVIEW reply ---
    def send_all_states(self):
        try:
            intensity_val = int(self.intensity_input.text())
        except ValueError:
            intensity_val = 0

        data = {
            "command": 1 if self.master_btn.isChecked() else 0,
            "sv": 1 if self.spulen_btn.isChecked() else 0,
            "wv": 1 if self.wasser_btn.isChecked() else 0,
            "int": intensity_val
        }
        for i in range(len(self.seg_btns)):
            data[f"v{i+1}"] = 1 if self.seg_btns[i].isChecked() else 0

        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.settimeout(2)
                sock.connect((TCP_IP, TCP_PORT))
                msg = json.dumps(data).encode('utf-8')
                sock.sendall(msg)
                response = sock.recv(4096)
                if response:
                    decoded = response.decode('utf-8')
                    print(f"Received from LabVIEW: {decoded}")
                    json_data = json.loads(decoded)
                    self.send_intensity1_value.emit(json_data["intensity1"]) #need to change the name with the name that coming from the LabVIEW
                    self.send_intensity2_value.emit(json_data["intensity2"]) #need to change the name with the name that coming from the LabVIEW
                    self.send_intensity3_value.emit(json_data["intensity3"]) #need to change the name with the name that coming from the LabVIEW

                    self.big_tank.setValue(json_data.get("tank_value", 0))  # Update the name of big tank
                    self.big_tank_label.setText(f"Big Tank Value: {json_data.get('tank_value', 0)}")
            self.update_conn_status.emit(True)
        except Exception as e:
            print(f"Error sending/receiving: {e}")
            self.update_conn_status.emit(False)

    def update_tank_bars(self, intensities):
        """
        Update the progress bars and labels with the intensities received from LabVIEW.
        """
        for i, intensity in enumerate(intensities):
            if 0 <= i < len(self.tanks):
                self.tanks[i].setValue(intensity)
                self.tank_labels[i].setText(f"Intensity {i+1}: {intensity}")

    def get_current_state(self, send_intensity1_value, send_intensity2_value, send_intensity3_value):
        tank_value = self.big_tank.value()

        # Define how "start test" is triggered — here's one option:
        start_test = self.start_test_indicator.styleSheet().find("green") != -1

        return {
            "intensity1": send_intensity1_value,
            "intensity2": send_intensity2_value,
            "intensity3": send_intensity3_value,
            "tank_value": tank_value,
            "start_test": start_test
        }


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RainWindow()
    window.show()
    sys.exit(app.exec())
