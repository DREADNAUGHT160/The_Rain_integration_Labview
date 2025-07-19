import math
import numpy as np
import socket
import time
from datetime import datetime

class PlatformRobotHandler:
    # variables
    def __init__(self, timeOutPR):
        self.name = 'platform robot'
        self.connected = False
        self.timeOut = timeOutPR
        # change shape
        self.shape = np.array([[3, 0, 0, 3, 3, 4, 3, 4, 4, 3], [0, 0, 2, 2, 0, 1, 2, 2, 0, 0]])
        self.shape[0, :] -= 2; self.shape[1, :] -= 1
        # scale shape
        self.shape = self.shape*0.75