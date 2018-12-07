#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
Main GUI File

Zukünftige Features:
    - Flüssiges Nachverfolgen der Bewegungen des Kopfes (Linien zwischen Punkten berechnen und diesen folgen lassen)
    - Taster an den Seiten checken um die Motoren nicht kaputt zu machen
    - RobotState, readBuffer, writeBuffer in GUI anzeigen lassen und state zurücksetzen können für Fehlerzurücksetzung zur Laufzeit
    - Im Arduino einen Puffer aufbauen um noch flüssiger zu Plotten
    - Den GCode nicht zuerst in eine File schreiben, sondern XYRobot und SerialMonitor über ein Attribut kommunizieren lassen

Bugs:
    - keine.
"""

import logging, collections

from PyQt5 import QtCore, QtGui, QtWidgets, QtSvg
from PyQt5.QtGui import QPainter, QColor, QBrush, QFont
from PyQt5.QtWidgets import QWidget, QApplication, QMainWindow, QInputDialog, QMessageBox
from PyQt5.QtWidgets import QPushButton, QComboBox, QGroupBox, QLabel
from PyQt5.QtWidgets import QGraphicsItem, QGraphicsScene, QGraphicsView
from PyQt5.QtSvg import QGraphicsSvgItem
from PyQt5.QtCore import QRect, QPoint, QRectF
from PyQt5.QtCore import QObject, pyqtSignal

from Controls import *
from XYRobot import RobotState, PrintMode
from enum import Enum

class CanvasControl():
    ''' A wrapper for attributes describing the canvas. Point is a Class. It can be used to construct Point-like objects
    that can be accessed using x and y attributes on the object. '''
    width = 0                               # width of canvas
    height = 0                              # height of canvas
    roboOrigin = None                       # origin of the XY-Robot on the canvas / expressed in GUI base
    picOrigin = None                        # origin of the plotted picture on the canvas / expressed in GUI base
    initialPictureSize = None               # the initial size that picture is scaled to
    placePicture = None                     # use PicPos-Enum here / decides about the placement of the picture
    Point = collections.namedtuple('Point', ['x', 'y']) # Class / described above
    
    def __init__(self):
        self.roboOrigin = self.Point(0,0)   # Robot origin in absolute coordinates on the scene
        self.picOrigin = self.Point(0,0)    # Picture origin in absolute coordinates on the scene
        
        # Parameters: 
        self.initialPictureSize = self.Point(130, 130)
        self.width = 400                    # has to be bigger than roboWidth
        self.height = 400                   # has to be bigger than roboHeight
        self.placePicture = self.PicPos.MIDDLE
        # Parameters end 
    
    class PicPos(Enum):
        ''' Decides about the placement of the picture inside the robot body. '''
        NONE = 0
        MIDDLE = 1
        #BOTTOM_RIGHT = 2
        

class GUI(QMainWindow):
    ''' Builds up the GUI, connects signals and handles the following interaction with the user. Most of the methods -
    maybe after some UI relevant calculations - eventually call a method from Controls to delegate the task to. ''' 
    
    controls = None                     # reference to Controls object
    canvasControl = None                # CanvasControl Object instance 
    log = logging.getLogger(__name__)   # the logger
    
    def __init__(self, cont):
        super().__init__()
        self.controls = cont
        self.canvasControl = CanvasControl()
        self.initUI()
        self.setupSignals()

    def initUI(self):
        ''' Inits all the UI elements on the main form . '''
        # Params start 
        canvasSize = QPoint(400,400)        # canvas to draw on
        formSize = (850,600)                # main form
        buttonWidth = 250                   # width of big button
        # Params end 
        
        # Main Window
        self.resize(formSize[0], formSize[1])
        self.setGeometry(QtWidgets.QStyle.alignedRect(QtCore.Qt.LeftToRight, QtCore.Qt.AlignCenter,
                                                      self.size(), QApplication.desktop().availableGeometry()))
        self.setWindowTitle('LaserGUI v0.2')
        self.show()
        
        # Status Bar
        self.statusBar()
        
        # Canvas for drawing
        size = QRect(10, 10, canvasSize.x()+10, canvasSize.y()+10)
        self.canvas = QGraphicsScene(self)
        self.canvas.setSceneRect(QRectF(0,0,canvasSize.x(),canvasSize.y()))
        self.canvasView = QGraphicsView(self.canvas, self)
        self.canvasView.setGeometry(size)
        self.canvasView.setRenderHints(QPainter.Antialiasing | QPainter.TextAntialiasing)
        self.canvasView.show()
        self.log.info("Canvas loaded.")
        self.log.debug("Canvas size: %4.1fx%4.1f" % (self.canvas.sceneRect().width(), self.canvas.sceneRect().height()))
                
        # --- Buttons ---
        size = QRect(canvasSize.x()+50, 10, buttonWidth, 30)
        self.connectButton = QPushButton(self)
        self.connectButton.setGeometry(size)
        self.connectButton.setText("Connect plotter")
        self.connectButton.show()
        
        size = QRect(canvasSize.x()+50, 40, buttonWidth/2, 30)
        self.loadImgButton = QPushButton(self)
        self.loadImgButton.setGeometry(size)
        self.loadImgButton.setText("Load img")
        self.loadImgButton.show()
        
        size = QRect(canvasSize.x()+50+buttonWidth/2, 40, buttonWidth/2, 30)
        self.resizeButton = QPushButton(self)
        self.resizeButton.setGeometry(size)
        self.resizeButton.setText("Resize img")
        self.resizeButton.show()
        
        size = QRect(canvasSize.x()+50, 70, buttonWidth, 30)
        self.measureButton = QPushButton(self)
        self.measureButton.setGeometry(size)
        self.measureButton.setText("Measure XY area")
        self.measureButton.show()
        
        size = QRect(canvasSize.x()+50, 110, buttonWidth, 30)
        self.startButton = QPushButton(self)
        self.startButton.setGeometry(size)
        self.startButton.setText("Start/pause plotting")
        self.startButton.show()
        
        size = QRect(canvasSize.x()+50, 160, buttonWidth/2, 30)
        self.statsButton = QPushButton(self)
        self.statsButton.setGeometry(size)
        self.statsButton.setText("Get stats")
        self.statsButton.show()
        
        size = QRect(canvasSize.x()+50+buttonWidth/2, 160, buttonWidth/2, 30)
        self.speedButton = QPushButton(self)
        self.speedButton.setGeometry(size)
        self.speedButton.setText("Set speed")
        self.speedButton.show()
        
        size = QRect(canvasSize.x()+50, 200, buttonWidth, 30)
        self.printModeButton = QPushButton(self)
        self.printModeButton.setGeometry(size)
        self.printModeButton.setText("Change print mode")
        self.printModeButton.show()
        
        size = QRect(canvasSize.x()+50, 230, buttonWidth, 30)
        self.returnButton = QPushButton(self)
        self.returnButton.setGeometry(size)
        self.returnButton.setText("Return to origin")
        self.returnButton.show()
        
        # --- Stats display ---
        size = QRect(canvasSize.x()+50, 260, buttonWidth, 230)
        self.optionsGroupBox = QGroupBox(self)
        self.optionsGroupBox.setGeometry(size)
        self.optionsGroupBox.show()

        size = QRect(10, 10, 200, 200)
        self.optionsLabel = QLabel(self.optionsGroupBox)
        self.optionsLabel.setGeometry(size)
        self.optionsLabel.setWordWrap(True)
        self.optionsLabel.setText("Stats: \n\n---")
        self.optionsLabel.setAlignment(QtCore.Qt.AlignTop)
        self.optionsLabel.show()
        
        # --- Points display ---
        size = QRect(canvasSize.x()+60+buttonWidth, 10, 180, 410)
        self.pointsBox = QGroupBox(self)
        self.pointsBox.setGeometry(size)
        self.pointsBox.show()
        
        size = QRect(10, 10, 160, 20)
        self.pointsCaption = QLabel(self.pointsBox)
        self.pointsCaption.setGeometry(size)
        self.pointsCaption.setText("Next commands:")
        self.pointsCaption.show()
        
        size = QRect(5, 40, 140, 410)
        self.pointsLabel = QLabel(self.pointsBox)
        self.pointsLabel.setGeometry(size)
        self.pointsLabel.setWordWrap(True)
        self.pointsLabel.setText("")
        self.pointsLabel.setAlignment(QtCore.Qt.AlignTop)
        self.pointsLabel.show()
        
        font = QFont("Verdana", 8, QFont.Normal, False)
        
        # --- Extended GUI ---
        if self.controls.extendedGUI:
            firstLine = 40
            spacingV = 70
            spacingH = 210
            spacingText = 20
            formSize = (formSize[0]+80, formSize[1] + 110)
            self.resize(formSize[0], formSize[1])
            
            size = QRect(10, canvasSize.y()+110, canvasSize.x()+510, 180)
            self.extendedStatsBox = QGroupBox(self)
            self.extendedStatsBox.setGeometry(size)
            self.extendedStatsBox.show()
            
            size = QRect(10, 10, 250, 20)
            self.extendedStatsCaption = QLabel(self.extendedStatsBox)
            self.extendedStatsCaption.setGeometry(size)
            self.extendedStatsCaption.setText("Detailed plotter data:")
            self.extendedStatsCaption.show()
            
            # --- Description labels ---
            size = QRect(0*spacingH+10, firstLine, spacingH-10, 20)
            self.extendedStatsDesc1 = QLabel(self.extendedStatsBox)
            self.extendedStatsDesc1.setGeometry(size)
            self.extendedStatsDesc1.setText("Position [steps]:")
            self.extendedStatsDesc1.show()
            
            size = QRect(1*spacingH, firstLine, spacingH, 20)
            self.extendedStatsDesc2 = QLabel(self.extendedStatsBox)
            self.extendedStatsDesc2.setGeometry(size)
            self.extendedStatsDesc2.setText("Target [steps]:")
            self.extendedStatsDesc2.show()
            
            size = QRect(2*spacingH, firstLine, spacingH, 20)
            self.extendedStatsDesc3 = QLabel(self.extendedStatsBox)
            self.extendedStatsDesc3.setGeometry(size)
            self.extendedStatsDesc3.setText("stepdelay [ms]:")
            self.extendedStatsDesc3.show()

            size = QRect(3*spacingH+20, firstLine, spacingH+spacingH/2, 20)
            self.extendedStatsDesc4 = QLabel(self.extendedStatsBox)
            self.extendedStatsDesc4.setGeometry(size)
            self.extendedStatsDesc4.setText("stepdelay +aux delay [ms]:")
            self.extendedStatsDesc4.show()
            
            size = QRect(0*spacingH+10, firstLine+spacingV, spacingH-10, 20)
            self.extendedStatsDesc5 = QLabel(self.extendedStatsBox)
            self.extendedStatsDesc5.setGeometry(size)
            self.extendedStatsDesc5.setText("max stepdelay [ms]:")
            self.extendedStatsDesc5.show()

            size = QRect(1*spacingH, firstLine+spacingV, spacingH, 20)
            self.extendedStatsDesc6 = QLabel(self.extendedStatsBox)
            self.extendedStatsDesc6.setGeometry(size)
            self.extendedStatsDesc6.setText("min stepdelay [ms]:")
            self.extendedStatsDesc6.show()
            
            size = QRect(2*spacingH, firstLine+spacingV, spacingH, 20)
            self.extendedStatsDesc7 = QLabel(self.extendedStatsBox)
            self.extendedStatsDesc7.setGeometry(size)
            self.extendedStatsDesc7.setText("Limit Pins [X1 X2 Y1 Y2]:")
            self.extendedStatsDesc7.show()
            
            size = QRect(3*spacingH+20, firstLine+spacingV, spacingH, 20)
            self.extendedStatsDesc8 = QLabel(self.extendedStatsBox)
            self.extendedStatsDesc8.setGeometry(size)
            self.extendedStatsDesc8.setText("laser on?:")
            self.extendedStatsDesc8.show()
            
            # --- content labels ---
            size = QRect(0*spacingH+10, firstLine+spacingText, spacingH-10, 20)
            self.extendedStatsContent1 = QLabel(self.extendedStatsBox)
            self.extendedStatsContent1.setGeometry(size)
            self.extendedStatsContent1.setText("---")
            self.extendedStatsContent1.show()
            
            size = QRect(1*spacingH, firstLine+spacingText, spacingH, 20)
            self.extendedStatsContent2 = QLabel(self.extendedStatsBox)
            self.extendedStatsContent2.setGeometry(size)
            self.extendedStatsContent2.setText("---")
            self.extendedStatsContent2.show()
            
            size = QRect(2*spacingH, firstLine+spacingText, spacingH, 20)
            self.extendedStatsContent3 = QLabel(self.extendedStatsBox)
            self.extendedStatsContent3.setGeometry(size)
            self.extendedStatsContent3.setText("---")
            self.extendedStatsContent3.show()

            size = QRect(3*spacingH+20, firstLine+spacingText, spacingH+spacingH/2, 20)
            self.extendedStatsContent4 = QLabel(self.extendedStatsBox)
            self.extendedStatsContent4.setGeometry(size)
            self.extendedStatsContent4.setText("---")
            self.extendedStatsContent4.show()
            
            size = QRect(0*spacingH+10, firstLine+spacingV+spacingText, spacingH-10, 20)
            self.extendedStatsContent5 = QLabel(self.extendedStatsBox)
            self.extendedStatsContent5.setGeometry(size)
            self.extendedStatsContent5.setText("---")
            self.extendedStatsContent5.show()

            size = QRect(1*spacingH, firstLine+spacingV+spacingText, spacingH, 20)
            self.extendedStatsContent6 = QLabel(self.extendedStatsBox)
            self.extendedStatsContent6.setGeometry(size)
            self.extendedStatsContent6.setText("---")
            self.extendedStatsContent6.show()
            
            size = QRect(2*spacingH, firstLine+spacingV+spacingText, spacingH, 20)
            self.extendedStatsContent7 = QLabel(self.extendedStatsBox)
            self.extendedStatsContent7.setGeometry(size)
            self.extendedStatsContent7.setText("---")
            self.extendedStatsContent7.show()
            
            size = QRect(3*spacingH+20, firstLine+spacingV+spacingText, spacingH, 20)
            self.extendedStatsContent8 = QLabel(self.extendedStatsBox)
            self.extendedStatsContent8.setGeometry(size)
            self.extendedStatsContent8.setText("---")
            self.extendedStatsContent8.show()
            
            # --- fonts ---
            self.extendedStatsCaption.setFont(font) 
            self.extendedStatsDesc1.setFont(font)
            self.extendedStatsDesc2.setFont(font)
            self.extendedStatsDesc3.setFont(font)
            self.extendedStatsDesc4.setFont(font)
            self.extendedStatsDesc5.setFont(font)
            self.extendedStatsDesc6.setFont(font)
            self.extendedStatsDesc7.setFont(font)
            self.extendedStatsDesc8.setFont(font)
            self.extendedStatsContent1.setFont(font)
            self.extendedStatsContent2.setFont(font)
            self.extendedStatsContent3.setFont(font)
            self.extendedStatsContent4.setFont(font)
            self.extendedStatsContent5.setFont(font)
            self.extendedStatsContent6.setFont(font)
            self.extendedStatsContent7.setFont(font)
            self.extendedStatsContent8.setFont(font)

        # --- Fonts ---
        font = QFont("Verdana", 9, QFont.Normal, False)
        self.startButton.setFont(font)
        self.statsButton.setFont(font)
        self.connectButton.setFont(font)
        self.loadImgButton.setFont(font)
        self.resizeButton.setFont(font)
        self.measureButton.setFont(font)
        self.optionsLabel.setFont(font)
        self.speedButton.setFont(font)
        self.pointsCaption.setFont(font)
        self.pointsLabel.setFont(font)
        self.printModeButton.setFont(font)
        self.returnButton.setFont(font)
        
    def setupSignals(self):
        ''' Set up signals for communication inside GUI class. '''
        # Buttons:
        self.connectButton.clicked.connect(self.connectButton_clicked)
        self.loadImgButton.clicked.connect(self.loadImgButton_clicked)
        self.resizeButton.clicked.connect(self.resizeButton_clicked)
        self.startButton.clicked.connect(self.startButton_clicked)
        self.statsButton.clicked.connect(self.statsButton_clicked)
        self.speedButton.clicked.connect(self.speedButton_clicked)
        self.printModeButton.clicked.connect(self.printModeButton_clicked)
        self.returnButton.clicked.connect(self.returnButton_clicked)
        self.measureButton.clicked.connect(self.measureButton_clicked)
        
    def plotterConnected(self):
        ''' Wrapper method for GUI. Returns true if there is an active
        connection between Controls and XY-Plotter. False otherwise.'''
        return self.controls.serialMonitor.isPlotterConnected()
    
    def plotterPlotting(self):
        ''' Wrapper method for GUI. Returns true if cond 1 and 2 are met,
        false otherwise. 
        1. there is an active connection between Controls and XY-Plotter
        2. the plotter is currently plotting. '''
        return (self.controls.serialMonitor.isPlotterConnected() and
                not self.controls.serialMonitor.isInWritePause())
    
    def measureButton_clicked(self):
        ''' Make plotter measure the area it uses and return those values. '''
        if self.plotterConnected() and not self.plotterPlotting():
            self.controls.i_measureArea()
        else:
            self.controls.infoLog("Cannot measure field. Plotter has to be connected and paused.")
        
    def returnButton_clicked(self):
        ''' Make Plotter go back to the origin in (0,0). The plotter does not have to be paused
        for that. '''
        if self.plotterConnected():
            self.controls.i_plotterReturn()
        else:
            self.controls.infoLog("Cannot return. Plotter has to be connected.")
        
    def showLimitPinsPressed(self, msg):
        ''' Used as slot only when some limit pins were pressed on the plotter. Show warning. '''
        self.limitPinsWarning = QMessageBox()
        self.limitPinsWarning.setText("The plotter just hit the limit pins and stopped.")
        self.limitPinsWarning.setInformativeText(msg)
        self.limitPinsWarning.setModal(False)
        self.limitPinsWarning.show()
        
        if self.controls.extendedGUI:
            #"Move not allowed. PINS X1 X2 Y1 Y2: 1111" is the msg from the plotter
            lpx1 = msg[36:37]
            lpx2 = msg[37:38]
            lpy1 = msg[38:39]
            lpy2 = msg[39:40]
            self.extendedStatsContent7.setText(lpx1 + " " + lpx2 + " " + lpy1 + " " + lpy2)
        
        self.log.info("The plotter just hit the limit pins and stopped.")
    
    def printModeButton_clicked(self):
        ''' Make plotter change its print mode. Possible options are LASER and PEN. '''
        if self.plotterConnected() and not self.plotterPlotting():
            self.controls.i_changePrintMode()
            self.controls.i_getStats()
        else:
            self.controls.infoLog("Cannot change printMode. Plotter has to be connected and paused.")
            self.log.info("connected? " + str(self.plotterConnected()) + " not plotting? " + str(self.plotterPlotting()))
        self.drawPlotList()
    
    def loadImgButton_clicked(self, tpicPath=""): 
        ''' Load an image into the canvas. '''
        self.controls.loadPicture(tpicPath)
    
    def statsButton_clicked(self):
        ''' When the statsButton is clicked, the M10 command is sent to the plotter
        asking for the current stats, that are set in the plotter's EEPROM. '''
        if self.plotterConnected():
            self.controls.i_getStats()
        else:
            self.controls.infoLog("Cannot get stats. Plotter has to be connected.")    
        
    def startButton_clicked(self):
        ''' Start/continue/pause the writing process. '''
        if not self.plotterConnected():
            self.controls.infoLog("Cannot start/pause plotting. Plotter not connected.")
            return
        
        if self.controls.serialMonitor.getWbLength() <= 0:
            self.controls.i_startPlotting()
        elif self.controls.serialMonitor.isInWritePause():
            self.controls.i_contPlotting()
            self.controls.infoLog("Continue writing")
        else:
            self.controls.i_pausePlotting()
            self.controls.infoLog("Writing paused")
        
    def speedButton_clicked(self):
        ''' Open a dialog window asking for a speed value to set the plotter to. '''
        
        if not self.plotterConnected() or self.plotterPlotting():
            self.controls.infoLog("Cannot set plotter speed. Plotter has to be connected and paused.")
            return
        
        text, ok = QInputDialog.getText(self, 'Set plotter speed', 
            'Enter speed. (Possible range 0-100): ')
        
        if ok:
            try:
                speed = int(text)
                if speed > 100 or speed < 0:
                    speed = min(100, max(0, speed))
                    self.controls.infoLog("Speed constrained to fit into the range")
            except:
                speed = self.controls.robot.speed
                self.controls.infoLog("Invalid value. Speed remains unchanged (Speed: " + str(speed) + ")")
            if speed != self.controls.robot.speed:
                self.controls.i_setSpeed(speed)
            else:
                self.controls.infoLog("Entered speed is already the currently set speed value.")
        
    def resizeButton_clicked(self):
        ''' Open a dialog window asking for a factor to scale the picture with.
        The scaling process only operates on the initialPictureSize, so for
        the changes to take effect the loadPicture() method (that paints and
        generates neccessary data for the plotter) is called again. '''
        if not self.controls.pic:
            self.controls.infoLog("Cannot resize picture. No picture loaded.")
            return
        
        text, ok = QInputDialog.getText(self, 'Resize picture', 
            'Enter scale factor (1.0 = 100%):')
        
        if ok:
            try:
                scale = float(text)
            except:
                scale = 1
                self.controls.infoLog("Invalid value. Enter a float number")
            
            w = self.controls.pic.width
            h = self.controls.pic.height
            w = w * scale
            h = h * scale
            if w < self.controls.robot.width and h < self.controls.robot.height:
                self.canvasControl.initialPictureSize = self.canvasControl.initialPictureSize._replace(x=int(w), y=int(h))
            else:
                 self.canvasControl.initialPictureSize = (
                    self.canvasControl.initialPictureSize._replace(x=self.controls.robot.width, y=self.controls.robot.height))
            
            self.controls.loadPicture(self.controls.picPath)
            self.controls.infoLog("Picture resized")
        
    def connectButton_clicked(self):
        ''' Change text of the button and call appropriate interface methods
        in Controls for connecting / disconnecting plotter. Tries to connect
        or disconnect for 'tries' times. '''
        tries = 3
        if not self.controls.serialMonitor or not self.controls.serialMonitor.isPlotterConnected():
            i = 0
            connected = False
            while i < tries:
                if self.controls.i_connectPlotter():
                    connected = True
                    break
                else:
                    i += 1
                    if i != tries:
                        self.log.warning("Trying again to connect to plotter...")
            if connected:
                self.connectButton.setText("Disconnect plotter")
        else:
            i = 0
            disconnected = False
            while i < tries:
                if self.controls.i_disconnectPlotter():
                    disconnected = True
                    break
                else:
                    i += 1
                    if i != tries:
                        self.log.warning("Trying again to disconnect from plotter...")
            if disconnected:
                self.connectButton.setText("Connect plotter")
                
    def transformToGuiSys(self, coord):
        ''' Takes a Command of subtype Coordinate as input. Assumes that the
        coordinates are expressed using the standard base and transforms them to be
        expressed using the gui base.'''
        x = coord.x
        y = coord.y
        
        (x, y) = (x - self.controls.pic.width, y - self.controls.pic.height)
        (x, y) = (-x, -y)
        
        coord.x = self.canvasControl.picOrigin.x + x
        coord.y = self.canvasControl.picOrigin.y + y

    def drawPlotList(self):
        ''' Draws the current plotList's string representation in the pointsLabel.
        For perfomance reasons due to the synchronization, not the plotList itself is
        drawn but the visualList which is a deep copy of the plotList. '''
        self.pointsLabel.setText(str(self.controls.robot.head.visualList))
        #self.pointsLabel.setText(str(self.controls.plotList))