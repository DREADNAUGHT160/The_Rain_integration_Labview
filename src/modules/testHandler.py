import PySide6.QtWidgets as qtw
class TestHandler(qtw.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("LabVIEW Control Panel")
        main_layout = qtw.QVBoxLayout()
        