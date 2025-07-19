# gui
import PySide6.QtWidgets as qtw
# control center functions
from modules import guiQt as gui
# config class
from modules.config import Config

# initialization
config = Config()

# initialises GUI
app = qtw.QApplication([]) # import sys -> QApplication(sys.argv) -> allow command line arguments
window = gui.mainWindowSetUp(config)
window.show()
app.exec()