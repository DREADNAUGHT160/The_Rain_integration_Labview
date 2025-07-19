# gui
import PySide6.QtCore as qt
import PySide6.QtWidgets as qtw
import PySide6.QtGui as qtg
import pyqtgraph as pg
# functions
import math
import numpy as np
from modules import functions as fun
from modules import trajectory as traj ### provisional ###
# classes
from modules.drivingRobotHandler import DrivingRobotHandler
from modules.platformRobotHandler import PlatformRobotHandler
from modules.rainHandler import RainWindow
# class for displaying tabular data
class TableConfig(qt.QAbstractTableModel):
    def __init__(self, data, mode):
        super(TableConfig, self).__init__()
        self._data = data
        if mode == 'live data':
            self.mode = mode
            self.columns = ['Data', 'Value']
        elif mode == 'msg data':
            self.mode = mode
            self.columns = ['Message']

    def data(self, index, role):
        # set color
        if role == qt.Qt.ItemDataRole.ForegroundRole:
            position = self._data[index.row()][0]
            if self.mode == 'msg data':
                if position[:position.index("///")] == 'error':
                    return qtg.QColor('red')
                elif position[:position.index("///")] == 'warning':
                    return qtg.QColor('orange')
        # set text
        if role == qt.Qt.ItemDataRole.DisplayRole:
            if self.mode == 'live data':
                displayData = self._data[index.row()][index.column()]
            elif self.mode == 'msg data':
                displayData = self._data[index.row()][index.column()]
                displayData = displayData[displayData.index("///") + 3:]
            return displayData

    def rowCount(self, _):
        return len(self._data)

    def columnCount(self, _):
        return len(self._data[0])
    
    def headerData(self, section, orientation, role):
        # section is the index of the column/row.
        if role == qt.Qt.ItemDataRole.DisplayRole:
            if orientation == qt.Qt.Orientation.Horizontal:
                return str(self.columns[section])

            if orientation == qt.Qt.Orientation.Vertical:
                return ''

# class for displaying data
class PlotConfig(pg.PlotWidget):
    def __init__(self, window, mode):
        super().__init__()
        self.setMenuEnabled(False)
        self.window = window
        self.mode = mode
    
    def contextMenuEvent(self, event):
        if self.mode == 'dynamic':
            menu = qtw.QMenu(self)
            # context menu for actors
            if len(self.window.actorInUseList) > 1:
                # creates context options: driving robot and platform robot
                for actor in self.window.actorInUseList:
                    match actor:
                        case 'driving robot':
                            menu.dRContextMenu = menu.addMenu('driving robot')
                            self.addMenuOptions(menu.dRContextMenu)
                        case 'platform robot':
                            menu.pRContextMenu = menu.addMenu('platform robot')
                            self.addMenuOptions(menu.pRContextMenu)
                menu.reset = menu.addAction('reset view')
                # evaluates action
                action = menu.exec_(event.globalPos())
                # actions
                if action == menu.dRContextMenu.zoomIn:
                    deltaZoomValue = -5
                    actor = 'driving robot'
                elif action == menu.pRContextMenu.zoomIn:
                    deltaZoomValue = -5
                    actor = 'platform robot'
                elif action == menu.dRContextMenu.zoomOut:
                    deltaZoomValue = 5
                    actor = 'driving robot'
                elif action == menu.pRContextMenu.zoomOut:
                    deltaZoomValue = 5
                    actor = 'platform robot'
                # resets zoom option
                elif action == menu.reset:
                    deltaZoomValue = 0
                    actor = ''
            else:
                self.addMenuOptions(menu)
                action = menu.exec_(event.globalPos())
                if action == menu.zoomIn:
                    deltaZoomValue = -5
                elif action == menu.zoomOut:
                    deltaZoomValue = 5
                actor = self.window.actorInUseList[0]
            # changes zoom
            if action is not None:
                self.changeZoom(deltaZoomValue, actor)

    def addMenuOptions(self, menuSpecific):
        # specific options for the actors
        menuSpecific.zoomIn = menuSpecific.addAction('zoom in')
        menuSpecific.zoomOut = menuSpecific.addAction('zoom out')

    # change the zoom and center the view to the vehicle of the live plot
    def changeZoom(self, deltaZoomValue, actor = ''):
        newZoomValue = self.window.zoomValue + deltaZoomValue
        if actor == 'driving robot':
            actor = self.window.drivRobot
            self.window.vehSel = 'driving robot'
        elif actor == 'platform robot':
            actor = self.window.platRobot
            self.window.vehSel = 'platform robot'
        else:
            self.window.vehSel = ''
        # reset zoom
        if deltaZoomValue == 0 and actor == '':
            self.window.plotLive.setXRange(-10, 10, padding=0); self.window.plotLive.setYRange(-20, 40, padding=0)
            self.window.zoomValue = 10
        # change zoom  or center view
        elif newZoomValue >= 5 and newZoomValue <= 40:
            limValues = self.calcLimPlot(actor, newZoomValue)
            self.window.plotLive.setYRange(limValues[0], limValues[1], padding=0); self.window.plotLive.setXRange(limValues[2], limValues[3], padding=0)
            self.window.zoomValue = newZoomValue

    def calcLimPlot(self,actor, zoomValue):
        limValues = []
        cg_x, cg_y = fun.calculateCg(actor)
        limValues.append(-zoomValue + cg_x); limValues.append(zoomValue + cg_x)
        limValues.append(-zoomValue + cg_y); limValues.append(zoomValue + cg_y)

        return limValues

# class for bulding the main window
class mainWindowSetUp(qtw.QMainWindow):
    def __init__(self, config):
        super().__init__()
        # variables
        self.config = config
        self.testRunning = False
        self.notMsgSet = True
        self.checkStatus = np.zeros(1)
        self.zoomValue = 10
        self.vehSel = ''
        self.updateRateRobots = 20  # in milliseconds
        self.updateRateData = 100   # in milliseconds

        # classes
        self.test = RainWindow()
        self.drivRobot = DrivingRobotHandler(self.config.timeOutDR)
        self.platRobot = PlatformRobotHandler(self.config.timeOutDR)

        # window label
        self.setWindowTitle("Control Center")

        # left column of the window (trajectory and test configurations)
        # trajectory
        labelTraj = qtw.QLabel('Trajectory')
        buttonNew = qtw.QPushButton('new')
        buttonNew.clicked.connect(self.createTrajectory)
        buttonLoad = qtw.QPushButton('load')
        buttonLoad.clicked.connect(self.openTrajectory)
        # driving robot
        labelRobot = qtw.QLabel('Driving robot')
        buttonConnect = qtw.QPushButton('connect')
        buttonConnect.clicked.connect(self.connectWithRobot)
        buttonDisconnect = qtw.QPushButton('disconnect')
        buttonDisconnect.clicked.connect(self.disconnectRobot)
        # testing
        labelTest = qtw.QLabel('Test')
        buttonStart = qtw.QPushButton('start')
        buttonStart.clicked.connect(self.startTest)
        buttonStop = qtw.QPushButton('stop')
        buttonStop.clicked.connect(self.stopTest)
        self.checkboxSim = qtw.QCheckBox('simualton')
        self.checkboxSim.setCheckState(qt.Qt.CheckState.Unchecked)
        self.checkboxSim.stateChanged.connect(self.onCheckBox)
        # rain test button
        buttonRain = qtw.QPushButton('rain')
        buttonRain.clicked.connect(self.openWindow)









        # main column of the window (plot)
        # determins the test participants
        self.actorList = []
        self.actorInUseList = []
        try:
            for actorUsed in ['driving robot', 'platform robot']:
                fileContent = open('./src/init/load.traj_conf','r')
                foundElement = fun.searchForText(fileContent, actorUsed)
                if foundElement != '':
                     match actorUsed:
                        case 'driving robot':
                            self.actorList.append([self.drivRobot])
                            self.actorInUseList.append('driving robot')
                        case 'platform robot':
                            self.actorList.append([self.platRobot])
                            self.actorInUseList.append('platform robot')
            if  self.actorList == []:
               raise Exception('no valid test loaded')
        except:
            raise Exception('load.traj_conf file not found')

        # static data
        # plot configuration (static data) -> trajectory, velocity, acceleration, orientation and curvature
        self.plotTraj = PlotConfig(self, ''); self.plotVel = PlotConfig(self, ''); self.plotAcc = PlotConfig(self, '')
        self.plotOrie = PlotConfig(self, ''); self.plotCurv = PlotConfig(self, '')
        # (live data)
        self.plotLive = PlotConfig(self, 'dynamic')
        # plot list
        plotList = [self.plotTraj, self.plotLive, self.plotVel, self.plotAcc, self.plotOrie, self.plotCurv]
        # general plot configuration
        for plot in plotList:
            if plot == self.plotLive or plot == self.plotTraj:
                plot.showGrid(x = True, y = True)
                plot.invertX(True)     # note: inversion of x axis for rotation needed
                plot.setAspectLocked(lock=True, ratio=1)
                plot.setXRange(-self.zoomValue, self.zoomValue, padding=0), plot.setYRange(-20, 40, padding=0)
            else:
                plot.setMenuEnabled(False)      # disables the context menu
                plot.setLimits(xMin = 0)        # set limits of plot
            plot.setBackground("w")             # set background color
            plot.addLegend()
                     
        # labels of the axis
        labelList = [['y coordinate in m','x coordinate in m'], ['y coordinate in m','x coordinate in m'],
                     ['waypoint','velocity in m/s'], ['waypoint','acceleration in m/s^2'],
                     ['waypoint','orientation in deg'], ['waypoint','curvature in 1/m']]
        # sets axis labelS
        for i in range(len(plotList)):
            plotList[i].setLabel('bottom', labelList[i][0])
            plotList[i].setLabel('left', labelList[i][1])

        self.createPlotDataSet()

        # loop through the actors
        for actor in  self.actorList:
            actor = actor[0]
            plotActorList, legendActorList = self.returnPlotActorList(actor)
            fileContent = open('./src/init/load.traj_conf','r')
            actor.waypoints = self.parseTrajConfigFile('read', actor.name, fileContent)

            # displays data in plots
            self.setPlotData(plotActorList, legendActorList, actor)
            # shows vehicle shapes
            actor.onlineXVehData, actor.onlineYVehData = self.calculatePosVehicleInFrame(actor, 0, 0, 0)
            plotActorList[3][0].setData(actor.onlineYVehData, actor.onlineXVehData)

        # right column of the window (display live data and messages)
        self.liveDataTable = qtw.QTableView() 
        self.liveData = [['x position [m]', ''], ['y position [m]', ''], ['orientation [deg]', ''], ['velocity [km/h]', ''],
                         ['waypoint index', '']]
        model = TableConfig(self.liveData, 'live data')
        self.liveDataTable.setModel(model)
        self.liveDataTable.horizontalHeader().setStretchLastSection(True)
        self.liveDataTable.horizontalHeader().setDefaultAlignment(qt.Qt.AlignmentFlag.AlignLeft)
        
        # message display
        self.msgTable = qtw.QTableView()
        self.displayedMsg = [['///']]
         # handels error msgs
        if hasattr(config, 'errorMsg'):
            self.displayedMsg = [['error///' + config.errorMsg]]
            self.notMsgSet = False
        model = TableConfig(self.displayedMsg, 'msg data')
        self.msgTable.setModel(model)
        self.msgTable.horizontalHeader().setStretchLastSection(True)
        self.msgTable.horizontalHeader().setDefaultAlignment(qt.Qt.AlignmentFlag.AlignLeft)

        # layout
        self.layoutMain = qtw.QHBoxLayout()
        self.layoutL = qtw.QVBoxLayout()
        self.layoutC = qtw.QVBoxLayout()
        self.layoutR = qtw.QVBoxLayout()

        # left column
        labelList = [labelTraj, buttonNew, buttonLoad, labelRobot, buttonConnect, buttonDisconnect, buttonRain]
        for label in labelList:
            self.layoutL.addWidget(label, alignment=qt.Qt.AlignmentFlag.AlignTop)
        self.layoutL.addStretch()
        labelList = [labelTest, buttonStart, buttonStop, self.checkboxSim]
        for label in labelList:
            self.layoutL.addWidget(label, alignment=qt.Qt.AlignmentFlag.AlignBottom)

        # centre column
        tabWidget = qtw.QTabWidget()
        # trajectory data
        tabWidgetTraj = qtw.QTabWidget()
        plotNameList = ['Trajectory', 'Velocity', 'Acceleration', 'Orientation', 'Curvature']
        i_plotName = 0
        for plot in plotList:
            if plot != self.plotLive:
                tabWidgetTraj.addTab(plot, plotNameList[i_plotName])
                i_plotName += 1 
        # static plots
        tabWidget.addTab(tabWidgetTraj, 'Trajectory data')
        # live data
        tabWidget.addTab(self.plotLive, 'Live Data')
        # main tab
        self.layoutC.addWidget(tabWidget)

        # right column
        self.layoutSplitter = qtw.QSplitter(qt.Qt.Vertical)
        # live data
        self.layoutSplitter.addWidget(self.liveDataTable)
        self.layoutSplitter.setStretchFactor(0, 2)      # index of element and strech factor
        # messages
        self.layoutSplitter.addWidget(self.msgTable)
        self.layoutSplitter.setStretchFactor(1, 0)      # index of element and strech factor
        self.layoutR.addWidget(self.layoutSplitter)

        # builds layout
        self.layoutMain.addLayout(self.layoutL)
        self.layoutMain.addLayout(self.layoutC,stretch=1)
        self.layoutMain.addLayout(self.layoutR)
        widget = qtw.QWidget()
        widget.setLayout(self.layoutMain)
        self.setCentralWidget(widget)

        # updates gui with given rate
        self.timerRobots = qt.QTimer(); self.timerData = qt.QTimer()
        timerConfList = [[self.timerRobots, self.updateRobots, self.updateRateRobots],
                         [self.timerData, self.updateData, self.updateRateData]]
        for timer in timerConfList:
            timer[0].timeout.connect(timer[1])
            timer[0].start(timer[2])

    # opens new window to create the trajectory
    def createTrajectory(self):
        1
        #if not self.window.isVisible():
        #    self.window.show()
        #self.drivRobot.waypoints, fileName = traj.initTrajectory(True)    # for debugging
        # updates GUI data
        #self.updateTrajectoryValues(fileName)   # change
    
    # function to control the rain and fog
    def openWindow(self):
        if not self.test.isVisible():
            self.test.show()

    # allows the selection of existing trajectories
    def openTrajectory(self):
        fileName, _ = qtw.QFileDialog.getOpenFileName(self,"Open File", "./trajectories","All Files (*.traj)")
        if fileName != '':
            if not 'driving robot' in self.actorInUseList:
                self.drivRobot.waypoints = np.loadtxt(fileName, delimiter=',', dtype=float)
                # updates GUI data
                plotActorList, legendActorList  = self.returnPlotActorList(self.drivRobot)
                self.updateTrajectoryValues(plotActorList, legendActorList, self.drivRobot, fileName)
                self.actorList.append([self.drivRobot])
                # updates contex menu
                self.actorInUseList.append('driving robot')
            else:
                self.displayedMsg += [['warning///Driving robot already exists!']]
        else:
            self.displayedMsg += [['warning///No file selected!']]

    def createPlotDataSet(self):
        # color of actors: driving robot and platform robot
        self.penList = [[pg.mkPen(color=(255, 150, 0), width = 3), pg.mkPen(color=(255, 150, 0), width = 1.5), pg.mkPen(color=(20, 20, 255), width = 1, style=qt.Qt.DashLine)],
                        [pg.mkPen(color=(255, 220, 0), width = 3), pg.mkPen(color=(255, 220, 0), width = 1.5), pg.mkPen(color=(0, 0, 0), width = 1, style=qt.Qt.DashLine)]]

        # driving robot plot
        self.drivTrajData = self.plotTraj.plot([], [], pen = self.penList[0][0]); self.drivVehData = self.plotTraj.plot([], [], pen = self.penList[0][1])
        self.drivLiveTrajData = self.plotLive.plot([], [], pen = self.penList[0][2]); self.drivLiveVehData = self.plotLive.plot([], [], pen = self.penList[0][1])
        self.drivVelData = self.plotVel.plot([], [], pen = self.penList[0][0]); self.drivAccData = self.plotAcc.plot([], [], pen = self.penList[0][0])
        self.drivOrieData = self.plotOrie.plot([], [], pen = self.penList[0][0]); self.drivCurvData = self.plotCurv.plot([], [], pen = self.penList[0][0])
        # driving robot legend
        actorName = 'driving robot'
        self.drivTrajLegend = pg.PlotDataItem(name = actorName, pen = self.penList[0][1]); self.drivLiveTrajLegend = pg.PlotDataItem(name = actorName, pen = self.penList[0][1])
        self.drivVelLegend = pg.PlotDataItem(name = actorName, pen = self.penList[0][1]); self.drivAccLegend = pg.PlotDataItem(name = actorName, pen = self.penList[0][1])
        self.drivOrieLegend = pg.PlotDataItem(name = actorName, pen = self.penList[0][1]); self.drivCurvLegend = pg.PlotDataItem(name = actorName, pen = self.penList[0][1])

        # platform robot plot
        self.platTrajData = self.plotTraj.plot([], [], pen = self.penList[1][0]); self.platVehData = self.plotTraj.plot([], [], pen = self.penList[1][1])
        self.platLiveTrajData = self.plotLive.plot([], [], pen = self.penList[1][2]); self.platLiveVehData = self.plotLive.plot([], [], pen = self.penList[1][1])
        self.platVelData = self.plotVel.plot([], [], pen = self.penList[1][0]); self.platAccData = self.plotAcc.plot([], [], pen = self.penList[1][0])
        self.platOrieData = self.plotOrie.plot([], [], pen = self.penList[1][0]); self.platCurvData = self.plotCurv.plot([], [], pen = self.penList[1][0])
        # driving robot legend
        actorName = 'platform robot'
        self.platTrajLegend = pg.PlotDataItem(name = actorName, pen = self.penList[1][1]); self.platLiveTrajLegend = pg.PlotDataItem(name = actorName, pen = self.penList[1][1])
        self.platVelLegend = pg.PlotDataItem(name = actorName, pen = self.penList[1][1]); self.platAccLegend = pg.PlotDataItem(name = actorName, pen = self.penList[1][1])
        self.platOrieLegend = pg.PlotDataItem(name = actorName, pen = self.penList[1][1]); self.platCurvLegend = pg.PlotDataItem(name = actorName, pen = self.penList[1][1])

    def returnPlotActorList(self, actor):

        drivRobotUsed = False; platRobotUsed = False
        
        if actor == self.drivRobot:
            drivRobotUsed = True
        # platform robot
        elif actor == self.platRobot:
            platRobotUsed = True

        if drivRobotUsed and not platRobotUsed:
            plotActorList = [[self.drivTrajData], [self.drivVehData], [self.drivLiveTrajData],
                             [self.drivLiveVehData], [self.drivVelData], [self.drivAccData], 
                             [self.drivOrieData], [self.drivCurvData]]
            legendActorList = [[self.drivTrajLegend], [self.drivLiveTrajLegend], [self.drivVelLegend],
                               [self.drivAccLegend], [self.drivOrieLegend], [self.drivCurvLegend]]
        elif not drivRobotUsed and platRobotUsed:
            plotActorList = [[self.platTrajData], [self.platVehData], [self.platLiveTrajData], 
                             [self.platLiveVehData], [self.platVelData], [self.platAccData], 
                             [self.platOrieData], [self.platCurvData]]
            legendActorList = [[self.platTrajLegend], [self.platLiveTrajLegend], [self.platVelLegend],
                               [self.platAccLegend], [self.platOrieLegend], [self.platCurvLegend]]

        return plotActorList, legendActorList

    def setPlotData(self, plotActorList, legendActorList, actor):
        # set values trajectory (static plot)
        plotActorList[0][0].setData(actor.waypoints[:,3], actor.waypoints[:,2])
        # transform vehicle
        orie = np.round(np.rad2deg(actor.waypoints[0,6]),2); x_point = actor.waypoints[0,2]; y_point = actor.waypoints[0,3]
        vehXData, vehYData = self.calculatePosVehicleInFrame(actor, orie, x_point, y_point)
        # set data vehicle shape
        plotActorList[1][0].setData(vehYData, vehXData)
        # set values trajectory (dynamic plot)
        plotActorList[2][0].setData(actor.waypoints[:,3], actor.waypoints[:,2])
        # y data of velocity, acceleration, orientation and curvature static plots
        plotYDataList  = [actor.waypoints[:,4], actor.waypoints[:,5], np.round(np.rad2deg(actor.waypoints[:,6]),2), actor.waypoints [:,7]]
        # set data vehicle shape
        actor.onlineXVehData, actor.onlineYVehData = self.calculatePosVehicleInFrame(actor, 0, 0, 0)
        plotActorList[3][0].setData(actor.onlineYVehData, actor.onlineXVehData)

        i = 0
        for plot in plotActorList[4:]:
            plot[0].setData(actor.waypoints[:,0], plotYDataList[i])
            i += 1
        # sets legend information
        self.addLegendForActor(legendActorList)    

    def updateTrajectoryValues(self, plotActorList, legendActorList, actor, fileName):
        # updates plots data
        self.setPlotData(plotActorList, legendActorList, actor)
        
        # save file name in traj_conf file
        if actor == self.drivRobot:
            actorName = 'driving robot'
        elif actor == self.platRobot:
            actorName = 'platform robot'
        lineText = actorName + ': ' + fileName[fileName.rindex('/')+1:]
        # reads file content
        with open('./src/init/load.traj_conf','r')  as fileContent:
            fileData = self.parseTrajConfigFile('write', actorName, fileContent)
        fileData.append(lineText)
        with open('./src/init/load.traj_conf','w')  as fileContent:
            if not (fileData[len(fileData)-2][-1:] == '\n'):
                fileData[len(fileData)-2] += '\n'
            fileContent.write(''.join(fileData))


    def addLegendForActor(self, legendActorList):
        plotList = [self.plotTraj, self.plotLive, self.plotVel, self.plotAcc, self.plotOrie, self.plotCurv]
        # general plot configuration
        for i in range(len(plotList)):
            plotList[i].removeItem(legendActorList[i][0])
            plotList[i].addItem(legendActorList[i][0])  

    def calculatePosVehicleInFrame(self, actor, orie, x_point, y_point):
        # rotation
        xTranPoints, yTranPoints = fun.transformPoints(orie, actor.shape[0,:], actor.shape[1,:])
        # translation
        xTranPoints = xTranPoints + x_point; yTranPoints = yTranPoints + y_point

        return xTranPoints, yTranPoints

    def parseTrajConfigFile(self, mode, actorName, fileContent):
        match mode:
            case 'read': 
                item = fun.searchForText(fileContent, actorName)
                if item != '':
                    idxStr = item.index(': ') + 1
                    try:
                        idxEnd = item.index('\n')
                    except:
                        idxEnd = len(item)
                    waypoints = np.loadtxt('src/trajectories/'+ item[idxStr:idxEnd].strip(), delimiter=',', dtype=float)
                            
                    return waypoints
                    
            case 'write':
                fileData = []
                for row in fileContent.readlines():
                    try: 
                        row.index(actorName)
                    except:
                        fileData.append(row)
                
                return fileData

    # builds the connection with the driving robot
    def connectWithRobot(self):
        self.drivRobot.connectToRobot(self)

    # disconnects from the driving robot
    def disconnectRobot(self):
        self.drivRobot.disconnectFromRobot(self)

    # start the test
    def startTest(self):
        if not self.testRunning:
            self.drivRobot.connectToRobot(self)
        
        # activates the driving robot waypoint mode
        self.drivRobot.startDrivingRobotTest(self)
        # activates the platform robot waypoint mode
        # self.platRobot.start...(self)

    # stops the test
    def stopTest(self):
        if self.testRunning:
            self.testRunning = self.config.isTerminate
            self.displayedMsg += [['info///Test was stopped manually']]

    # activate the simulation mode
    def onCheckBox(self):
        self.checkStatus = [self.checkboxSim.checkState().value]

    # updates the values of the GUI
    def updateRobots(self):
        # update the waypoints that are send to the driving robot
        if self.testRunning:
            # updates waypoints of driving robot
            self.drivRobot.updateWaypoints(self)
            # updates communication msg, waypoints etc. of platform robot (if needed)
            #self.platRobot.update...()

    def updateData(self):
        # updates live data
        # driving robot
        if self.drivRobot.connected:
            # request data of driving robot to update plot and table
            # the zoom of the plot needs to be updated
            # dynamic values
            self.drivRobot.akCommand = bytes('ATDV K0', 'utf-8')
            self.drivRobot.sock.sendall(self.drivRobot.createAsciiArray())
            dataRecv_dyn = self.drivRobot.sock.recv(5000).split()[3:]
            self.drivRobot.akCommand = bytes('AWRT K0', 'utf-8')
            self.drivRobot.sock.sendall(self.drivRobot.createAsciiArray())
            dataRecv_vel = self.drivRobot.sock.recv(5000).split()[3:]
            # x and y position of cg in ENU
            # rotation
            try:
                # x and y position of cg in local frame
                x, y = fun.transformPoints(math.radians(-self.config.ENU2Local), float(dataRecv_dyn[11]), float(dataRecv_dyn[12]))
                self.liveData[0][1] = round(x, 4); self.liveData[1][1] = round(y, 4)
                # orientation
                # note: orientation is given to the North in ENU and not to the "real" x
                #       corresponding to the East
                alpha = self.config.ENU2Local - 90.0
                self.liveData[2][1] = 360-round((float(dataRecv_dyn[5]) + alpha)%360, 3) 
                # velocity
                self.liveData[3][1] = round((float(dataRecv_vel[2])), 2)
                # update live
                ori = np.deg2rad(self.liveData[2][1])
                self.drivRobot.onlineXVehData, self.drivRobot.onlineYVehData = self.calculatePosVehicleInFrame(self.drivRobot, ori, x, y)
                self.drivLiveVehData.setData(self.drivRobot.onlineYVehData, self.drivRobot.onlineXVehData)
            except:
                self.liveData[0][1] = ''; self.liveData[1][1] = ''; self.liveData[2][1] = ''; self.liveData[3][1] = ''

            # waypoints
            if self.testRunning:
                self.liveData[4][1] = int(self.drivRobot.idxWaypoint)
            else:
                self.liveData[4][1] = ''    
        else:
            # sets the values to default
            for i in range(len(self.liveData)):
                self.liveData[i][1] = ''
        # changes view of plot to follow selected vehicle
        if self.vehSel != '' or len(self.actorInUseList) == 1:
            if self.vehSel == '':
                actor = self.actorInUseList[0]
            else:
                actor = self.vehSel
            match actor:
                case 'driving robot':
                    actor = self.drivRobot
                case 'platform robot':
                    actor = self.platRobot
                    
            limValues = self.plotLive.calcLimPlot(actor, self.zoomValue)
            self.plotLive.setYRange(limValues[0], limValues[1], padding=0); self.plotLive.setXRange(limValues[2], limValues[3], padding=0)
        # updates tablular data
        model = TableConfig(self.liveData, 'live data')
        self.liveDataTable.setModel(model)    
        # update messages
        if self.notMsgSet:
           if len(self.displayedMsg) > 1:
                self.displayedMsg = self.displayedMsg[1:]
                self.notMsgSet = False
        model = TableConfig(self.displayedMsg, 'msg data')
        self.msgTable.setModel(model)

# class for bulding the secondary window (trajectory)
#class test(qtw.QWidget):
#    def __init__(self):
#        super().__init__()
#        self.rainWindow.gui()
        