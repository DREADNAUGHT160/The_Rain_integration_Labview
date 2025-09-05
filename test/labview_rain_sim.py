# file: labview_sim.py
import sys, json, socket, threading
from PyQt6.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QLineEdit, QCheckBox,
    QTextEdit, QGridLayout, QHBoxLayout, QVBoxLayout, QMessageBox, QSpinBox
)
from PyQt6.QtCore import Qt, pyqtSignal, QObject, QTimer


class Signals(QObject):
    log_rx = pyqtSignal(str)
    conn_state = pyqtSignal(bool)


class JsonServer(threading.Thread):
    def __init__(self, host, port, signals, get_reply_fn):
        super().__init__(daemon=True)
        self.host=host; self.port=port; self.signals=signals
        self.get_reply=get_reply_fn
        self._stop=False; self.sock=None; self.client=None

    def stop(self):
        self._stop=True
        try:
            if self.client: self.client.close()
            if self.sock: self.sock.close()
        except: pass

    def send_reply(self):
        if not self.client: return
        reply=self.get_reply()
        try:
            self.client.sendall((json.dumps(reply)+"\n").encode())
        except: pass

    def run(self):
        while not self._stop:
            try:
                self.sock=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
                self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.sock.bind((self.host,self.port)); self.sock.listen(1)
                self.client,addr=self.sock.accept()
                self.signals.conn_state.emit(True)
                buffer=""
                while not self._stop:
                    data=self.client.recv(4096)
                    if not data: break
                    buffer+=data.decode(errors="ignore")
                    while "\n" in buffer:
                        line,buffer=buffer.split("\n",1)
                        msg=line.strip()
                        if not msg: continue
                        try:
                            obj=json.loads(msg); pretty=json.dumps(obj,indent=2)
                        except Exception as e:
                            pretty=f"(Invalid JSON)\n{msg}\nError: {e}"
                        self.signals.log_rx.emit(pretty)
                        self.send_reply()  # auto reply
                try: self.client.close()
                except: pass
                self.client=None; self.signals.conn_state.emit(False)
            except Exception:
                self.signals.conn_state.emit(False)
            finally:
                try: self.sock.close()
                except: pass


class LabVIEWSimulator(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("LabVIEW Simulator (TCP JSON)")
        self.setFixedSize(880, 540)
        self.signals=Signals()
        self.server=None

        # Controls
        self.ip_edit=QLineEdit("127.0.0.1"); self.port_edit=QLineEdit("5055")
        self.btn_start=QPushButton("Start"); self.btn_stop=QPushButton("Stop"); self.btn_stop.setEnabled(False)
        self.conn_lbl=QLabel("● Disconnected"); self.conn_lbl.setStyleSheet("color:red;font-weight:bold;")

        top=QHBoxLayout()
        top.addWidget(QLabel("IP:")); top.addWidget(self.ip_edit)
        top.addWidget(QLabel("Port:")); top.addWidget(self.port_edit)
        top.addStretch(1); top.addWidget(self.conn_lbl); top.addWidget(self.btn_start); top.addWidget(self.btn_stop)

        self.rx_box=QTextEdit(); self.rx_box.setReadOnly(True)

        # Reply setup
        self.cb_test=QCheckBox("Test Start"); self.cb_p1=QCheckBox("Pump1")
        self.cb_p2=QCheckBox("Pump2"); self.cb_p3=QCheckBox("Pump3")
        self.ed_int1=QLineEdit("0.0"); self.ed_int2=QLineEdit("0.0")
        self.ed_int3=QLineEdit("0.0"); self.ed_tank=QLineEdit("0.0")
        self.btn_send_now=QPushButton("Send Now")

        # Periodic broadcast
        self.cb_periodic=QCheckBox("Periodic broadcast")
        self.spin_period=QSpinBox(); self.spin_period.setRange(100,5000); self.spin_period.setValue(1000)
        self.lbl_ms=QLabel("ms")
        self.timer=QTimer(self); self.timer.timeout.connect(self.send_now)

        grid=QGridLayout(); r=0
        grid.addWidget(QLabel("Reply booleans:"),r,0,1,2); r+=1
        grid.addWidget(self.cb_test,r,0); grid.addWidget(self.cb_p1,r,1); r+=1
        grid.addWidget(self.cb_p2,r,0); grid.addWidget(self.cb_p3,r,1); r+=1
        grid.addWidget(QLabel("Reply numbers:"),r,0,1,2); r+=1
        grid.addWidget(QLabel("Intensity 1"),r,0); grid.addWidget(self.ed_int1,r,1); r+=1
        grid.addWidget(QLabel("Intensity 2"),r,0); grid.addWidget(self.ed_int2,r,1); r+=1
        grid.addWidget(QLabel("Intensity 3"),r,0); grid.addWidget(self.ed_int3,r,1); r+=1
        grid.addWidget(QLabel("Main Tank"),r,0); grid.addWidget(self.ed_tank,r,1); r+=1
        grid.addWidget(self.cb_periodic,r,0); grid.addWidget(self.spin_period,r,1); r+=1
        grid.addWidget(self.lbl_ms,r-1,1,Qt.AlignmentFlag.AlignRight)
        grid.addWidget(self.btn_send_now,r,0,1,2)

        left=QVBoxLayout(); left.addLayout(top); left.addWidget(self.rx_box,1)
        right=QVBoxLayout(); right.addLayout(grid); right.addStretch(1)
        root=QHBoxLayout(self); root.setContentsMargins(10,10,10,10); root.setSpacing(12)
        root.addLayout(left,3); root.addLayout(right,2)

        # Signals
        self.btn_start.clicked.connect(self.start_server)
        self.btn_stop.clicked.connect(self.stop_server)
        self.btn_send_now.clicked.connect(self.send_now)
        self.signals.log_rx.connect(self.on_rx)
        self.signals.conn_state.connect(self.on_conn_change)
        self.cb_periodic.toggled.connect(self.on_periodic_toggled)
        self.spin_period.valueChanged.connect(lambda v: self.timer.setInterval(v))

    # Build reply JSON
    def current_reply(self)->dict:
        def f(ed): 
            try: return float(ed.text())
            except: return 0.0
        return {
            "test_start": self.cb_test.isChecked(),
            "pump1": self.cb_p1.isChecked(),
            "pump2": self.cb_p2.isChecked(),
            "pump3": self.cb_p3.isChecked(),
            "int1": f(self.ed_int1), "int2": f(self.ed_int2),
            "int3": f(self.ed_int3), "main_tank": f(self.ed_tank)
        }

    # UI slots
    def on_rx(self, text:str):
        self.rx_box.append(f"▼ Received:\n{text}\n")
    def on_conn_change(self, ok:bool):
        if ok:
            self.conn_lbl.setText("● Connected"); self.conn_lbl.setStyleSheet("color:green;font-weight:bold;")
        else:
            self.conn_lbl.setText("● Disconnected"); self.conn_lbl.setStyleSheet("color:red;font-weight:bold;")
    def start_server(self):
        if self.server: QMessageBox.information(self,"Server","Already running."); return
        host=self.ip_edit.text().strip() or "127.0.0.1"
        try: port=int(self.port_edit.text())
        except: QMessageBox.warning(self,"Port","Invalid port."); return
        self.server=JsonServer(host,port,self.signals,self.current_reply); self.server.start()
        self.btn_start.setEnabled(False); self.btn_stop.setEnabled(True)
    def stop_server(self):
        if self.server: self.server.stop(); self.server=None
        self.btn_start.setEnabled(True); self.btn_stop.setEnabled(False); self.timer.stop()
    def send_now(self):
        if not self.server or not self.server.client:
            QMessageBox.information(self,"Send","No client connected."); return
        rep=self.current_reply()
        try:
            self.server.client.sendall((json.dumps(rep)+"\n").encode())
            self.rx_box.append(f"▲ Sent (manual/periodic):\n{json.dumps(rep,indent=2)}\n")
        except Exception as e:
            QMessageBox.warning(self,"Send",f"Failed: {e}")
    def on_periodic_toggled(self,on:bool):
        if on: self.timer.start(self.spin_period.value())
        else: self.timer.stop()


if __name__ == "__main__":
    app=QApplication(sys.argv)
    w=LabVIEWSimulator(); w.show()
    sys.exit(app.exec())