import math
import numpy as np
import socket
import time
from datetime import datetime

class DrivingRobotHandler:
    # variables
    def __init__(self, timeOutDR):
        self.name = 'driving robot'
        self.connected = False
        self.timeOut = timeOutDR
        self.shape = np.array([[3, 0, 0, 3, 3, 4, 3, 4, 4, 3], [0, 0, 2, 2, 0, 1, 2, 2, 0, 0]])
        self.shape[0, :] -= 2; self.shape[1, :] -= 1

    def connectToRobot(self, window):
        if not self.connected:            
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            try:
                self.sock.connect((window.config.host.strip(), window.config.port))
                self.connected = True
                window.displayedMsg += [['info///Connected to the driving robot.']]
            except:
                window.displayedMsg += [['error///Connection to driving robot failed.']]
    
    def disconnectFromRobot(self, window):
        if self.connected and not window.testRunning:
            self.sock.shutdown(socket.SHUT_RDWR)
            self.sock.close()
            self.connected = False
            window.displayedMsg += [['info///Driving robot is disconnected.']]

    def startDrivingRobotTest(self, window):
        if self.connected:
            window.displayedMsg += [['info///Waypoint mode is initialized.']]
            # change mode of the driving robot
            self.drivRobot.akCommand = bytes('STDA K0 1 ' + str(self.timeOut), 'utf-8')
            self.sock.sendall(self.createAsciiArray())
            # display replay of the driving robot
            dataRecv = self.sock.recv(5000)
            window.displayedMsg += [['info///Long: ' + str(round(float(dataRecv.decode().split()[3]), 2)) +
                                    ' Lat: ' + str(round(float(dataRecv.decode().split()[4]), 2)) +
                                    ' Alt: ' + str(round(float(dataRecv.decode().split()[5]), 2))]]
            window.displayedMsg += [['info///Position of driving robot in LLH coordinates:']]
            # extract waypoints
            self.waypointsToSend = np.delete(self.waypoints , 1, axis=1)
            if len(self.waypoints) > 20:
                self.createSTDDMessage(self.waypointsToSend[0:20], window.config.ENU2Local)
            else:
                self.createSTDDMessage(self.waypointsToSend)
            # passes the waypoints to the driving robot
            self.idxWaypoint = 1
            self.sendWaypoints(self, window)
            # test starts successfully
            self.testRunning = True
            # count to avoid that driving robot get stack on lasts waypoints
            self.countWaypointIdx = 0

    def updateWaypoints(self, window):
        if len(self.waypointsToSend) > 20:
            if int(self.idxWaypoint) > 6:
                # deletes old waypoints
                indx = int(self.idxWaypoint) - 6
                for i in range(indx): self.waypointsToSend = np.delete(self.waypointsToSend, i, 0)
                self.createSTDDMessage(self.waypointsToSend[0:20], window.config.ENU2Local)                                
            else:
                self.createSTDDMessage(self.waypointsToSend, window.config.ENU2Local)
                # checks that the driving robot does not get stack on lasts waypoints
                if int(self.idxWaypoint) > len(self.waypointsToSend) - 2:
                    self.countWaypointIdx += 1
            # terminates passing waypoints        
            if int(self.idxWaypoint) == len(self.waypointsToSend) or self.countWaypointIdx > 10:
                # change states
                window.config.isTerminate = True
                window.displayedMsg += [['info///All waypoints have been successfully passed.The cycle has ended.']]

            self.sendWaypoints(self, window)
            
            if window.config.isTerminate:
                self.sock.sendall(self.createAsciiArray(self.akCommand + b'/BR100'))
                self.sock.recv(5000)
                time.sleep(0.2)
                self.sock.sendall(self.createAsciiArray(b'STDBY K0'))
                # remove variables
                del self.idxWaypoint; del self.waypointsToSend
                # change test state
                window.testRunning = False; window.config.isTerminate = False

    # creates the STDD command which allows controlling the driving robot with waypoints
    def createSTDDMessage(self, waypoints, ENU2Local):
        # note: the rotation between ENU and local is 107.16 -> the trajectory is 
        #       created in the local frame -> the rotation matrix needs is therefore 
        #       inverted
        R_z = np.array([[math.cos(math.radians(ENU2Local)),-math.sin(math.radians(ENU2Local))],
                    [math.sin(math.radians(ENU2Local)), math.cos(math.radians(ENU2Local))]])
        # AK-Command
        self.akCommand = "STDD K0 " + str(len(waypoints))
        # loop through waypoint list
        for i in range(len(waypoints)):
            # transformation of Waypoints
            coordXY = R_z.dot(waypoints[i,1:3])
            theta = waypoints[i,5] - (360 + math.radians(ENU2Local))
            self.akCommand += " " + str(i + 1) + " " + str(round(coordXY[0], 5)) + " " + str(round(coordXY[1], 5)) + " " + str(waypoints[i,3]) + " " + str(waypoints[i,4]) + " " + str(round(theta, 5)) + " " + str(waypoints[i,6])
        # converts string to bytes
        self.akCommand = bytes(self.akCommand, 'utf-8')
    
    # transforms the string data into binary format
    def createAsciiArray(self):
        # Variables 
        stx = b'\x02'
        dontCare = b'\x20'
        etx = b'\x03'

        # Byte sequence message
        asciiData = stx + dontCare + self.akCommand + etx

        return asciiData

    # Sends data to driving robot and receives response
    def sendWaypoints(self, window):
        self.sock.sendall(self.createAsciiArray(self.akCommand))
        #dataRecv = self.sock.recv(5000)
        self.STDDMsgExtraction(self.sock.recv(5000))
        self.saveLog(window.config)

    # extracts values from the STDD message, which is received as replay from the driving robot
    def STDDMsgExtraction(self, dataRecv):

        if not dataRecv:
            self.msgArray = np.zeros(26)
        else:
            self.msgArray = self.extractDataFromMsg(dataRecv) # test without extracting data -> ATDV with a higher time cylce 1 s
            self.idxWaypoint = self.msgArray[13]

    def extractDataFromMsg(msg_b):

        # extracts received message and store content in array
        msg_s = msg_b.decode("utf-8")
        msg_s = msg_s.split()

        # removes unnecessary bytes/ strings
        delPos = np.concatenate([np.array([0, 1, 2, 13, 23]), np.arange(msg_s.index('-'), len(msg_s))])
        for i in sorted(delPos, reverse=True): del msg_s[i]
        # converts data in number format
        msg_f = np.float64(msg_s)

        return msg_f

    # stores the values of the driving robot replay
    def saveLog(self, config):

        # initialises the file name
        if config.newLog:
            config.prefixName = datetime.now().strftime('%y%m%d_%H%M%S') + '_log_file_'  # log file
            config.idxCSV = 1; config.idxRow = 0; config.newLog = False
        # initialises the matrix array
        if not bool(len(config.logArray)):
            config.logArray = np.zeros((1000, len(self.msgArray)+1), dtype='<U21')
        # adds the system time and the driving robot's reply to the logfile.
        config.logArray[config.idxRow,:] = np.concatenate((np.array([datetime.now().strftime('%H:%M:%S.%f')[:-4]]), self.msgArray))
        if config.idxRow < 999 and not config.isTerminate:
            config.idxRow += 1
        elif config.isTerminate:
            fileName = config.prefixName + str(config.idxCSV) + '.csv'
            config.logArray = np.delete(config.logArray, np.arange(config.idxRow + 1,len(config.logArray)),0)
            np.savetxt('logs/' + fileName, (config.logArray), delimiter=',', fmt='%s')
            # reset variables
            config.logArray = np.array([]); config.newLog = True
        else:
            fileName = config.prefixName + str(config.idxCSV) + '.csv'
            np.savetxt('logs/' + fileName, (config.logArray), delimiter=',', fmt='%s')
            # reset variables 
            config.idxCSV += 1; config.idxRow = 0; config.logArray = np.array([])