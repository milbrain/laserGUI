import sys, time, logging, threading

from PyQt5 import QtCore, QtGui
from PyQt5.QtGui import*
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import QPen, QColor
from math import *

from enum import Enum
from PlotList import Command, CommandType, CommandStatus

class RobotState(Enum):
    '''
    XYRobot States:
        uninitialized           -> NONE
        !(moving || plotting)   -> IDLE
        (moving || plotting)    -> BUSY
    '''
    NONE = 0
    IDLE = 1
    BUSY = 2
    #MOVING = 2
    #WORKING = 3
    
class PrintMode(Enum):
    NONE = 0
    LASER = 1
    PEN = 2
        
class XYRobot(QGraphicsItem):
    ''' This class represents the XY-Plotter. It is used by Controls for plotter-relevant tasks like conversion
    of coordinates using a plotter-base or transformation of Commands into G-Code. '''
    log = logging.getLogger(__name__)
    
    state = RobotState.IDLE         # is beeing accessed from different threads (e.g. serial_monitor_thread)
    stateLock = threading.Lock()
    
    def __init__(self, control, parent=None):
        super().__init__(parent)

        self.control = control
        # Robot params also stored in EEPROM
        self.width = 310            # Standard XY-Robot construction is 310x355
        self.height = 355
        self.speed = 80             # Range: 0-100
        self.motoADir = 0           # 0 is Standard when Origin is in left bottom
        self.motoBDir = 0           # 0 is Standard when Origin is in left bottom
        
        # stored, but stored values are not used. Instead we use these only
        self.penUpPos = 140         # Position of motor in "Up"-Position
        self.penDownPos = 10        # Position of motor in "Down"-Position
        
        # Not stored in EEPROM
        self.printMode = PrintMode.LASER
        self.printStatusOn = False  # True means Laser/Pen is writing/on; False means it's off: used by Server
        self.auxDelay = 0           # additional Delay
        self.laserPower = 255       # Range: 0-255
        self.laserBurnDelay = 0     # Time that the laser should wait to start (after the head starts moving)
        self.setZValue(-1)          # For drawing: Robot gets lowest zValue, picture gets 0, head gets 1
        ''' Robot params end '''
        
        self.goIdle()
        self.head = RobotHead(self)
    
    def boundingRect(self):
        ''' Bounding rect using for Qt paint-Event. '''
        return  QRectF(0,0,self.width,self.height)

    def paint(self, painter, option, widget=None):        
        ''' Qt paint method. '''
        painter.setBrush(Qt.white)
        painter.setPen(QPen(Qt.darkGray))
        painter.drawRect(0, 0, self.width, self.height)
        
    def cmdToGCode(self, cmd):
        ''' Takes an object of type Command as input and transforms it into a
        byte-string. This is getting returned. No coordinate system conversion
        is done.
        The order is very important in this if-structure! Do not change anything
        unless you're sure what you're doing.'''
        
        if cmd.type == CommandType.RETURN:
            msg = 'G28' # used to be 'G1 X0 Y0'
        elif cmd.type == CommandType.MEASURE:
            msg = 'M12'
        elif cmd.isCoordinate():
            msg = 'G1 X%.2f Y%.2f' % (cmd.x, cmd.y)
        elif cmd.type in {CommandType.DRAW, CommandType.WEB_DRAW}:
            if self.printMode == PrintMode.LASER:
                msg = 'M4 255'
            elif self.printMode == PrintMode.PEN:
                msg = 'M1 %d' % self.penDownPos
        elif cmd.type in {CommandType.DONT_DRAW, CommandType.WEB_DONT_DRAW}:
            if self.printMode == PrintMode.LASER:
                msg = 'M4 0'
            elif self.printMode == PrintMode.PEN:
                msg = 'M1 %d' % self.penUpPos
        elif cmd.type == CommandType.M10:
            msg = 'M10'
        elif cmd.type == CommandType.M5:
            msg = 'M5 W%d H%d S%s' % (int(cmd.x), int(cmd.y), str(cmd.misc))
        else:
            self.log.critical("Command structure not recognized. Command: " + repr(cmd))
            
        msg += '\n'
        return msg.encode()
        
    def transfToPlotterSys(self, coord):
        ''' Takes a Command of subtype Coordinate as input. Assumes that the
        coordinates are expressed using the standard base and transforms them to be
        expressed using the plotter base. '''
        x = coord.x
        y = coord.y
        
        if coord.isWebOriginated():
            # transform a web-coordinate
            (x, y) = (-x, y)
            
        else:
            # transform a picture-coordinate
        
            # move to center of robot-draw-space
            (x, y) = (x + (self.width - self.control.pic.width)/2, y + (self.height - self.control.pic.height)/2)
        
            # Adapt to robot head starting position beeing in bottom right corner
            # (left was the standard position used by Makeblock)
            (x, y) = (-x, y)
        
        coord.x = x
        coord.y = y
        
    def goBusy(self):
        ''' Change into BUSY robot state. '''
        self.stateLock.acquire()
        self.state = RobotState.BUSY
        self.stateLock.release()
    
    def goIdle(self):
        ''' Change into IDLE robot state. '''
        self.stateLock.acquire()
        self.state = RobotState.IDLE
        self.stateLock.release()
    
    def getState(self):
        ''' Get the robot state. '''
        ret = RobotState.NONE
        self.stateLock.acquire()
        ret = self.state
        self.stateLock.release()
        return ret


class RobotHead(QGraphicsItem):
    ''' Represents the plotter head. Has a paint-event implemented to be drawn on the GUI, a method for
    coordinate system conversion into the head-base and a visualList which is a copy of plotList used for
    performance reasons. '''
       
    parent = None
    
    # Drawing params start
    radius = 10             # px
    opacity = 0.5           # 0 - 1.0
    zVal = 1                # zValue for drawing. Head is on top of picture and robot
    # Drawing params end
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent
        self.setZValue(self.zVal)
        self.x = 0.0                # current relative position of robot head (inside robot body!) (0.0) is bottom right
        self.y = 0.0                # current relative position of robot head (inside robot body!)
        self.cmdOrigin = CommandType.PICTURE        # origin of the current self.x and self.y values
        self.visualList = None      # Clone of the plotList for performance reasons. Synchronized manually.
        
    def updatePosition(self, lastSent):            # used to be called visitNext()
        ''' Sets the current head position using lastSent which contains the ID of the element
        that was last sent from the plotList. '''
        if self.visualList.count() > 0:
            cmd = self.visualList.find(lastSent)
            
            if cmd and cmd.isCoordinate():
                if cmd.isWebOriginated():
                    self.cmdOrigin = CommandType.WEB_COORDINATE
                else:
                    self.cmdOrigin = CommandType.PICTURE
                
                self.x = cmd.x
                self.y = cmd.y

    def boundingRect(self):
        ''' Returns the bouding rect for the Qt paint event. '''
        return  QRectF(0-self.radius/2, 0-self.radius/2, self.radius, self.radius)

    def paint(self, painter, option, widget=None):
        ''' Qt paint event implementation. '''
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setBrush(Qt.cyan)
        painter.setPen(QPen(Qt.cyan))
        painter.setOpacity(self.opacity)
        
        cmd = Command(0, self.x, self.y, CommandStatus.PENDING, self.cmdOrigin)
        self.transformToHeadSys(cmd)
        
        if (self.x == 0.0 and self.y == 0.0 and
                ((self.parent.control and self.parent.control.plotList.count() <= 1) or
                not self.parent.control)):
            # nicer output. Draw blue dot in aboslute bottom right corner
            # if there are problems with the output when commands come from web, set <= 1 to == 0
            newX = self.parent.width - self.radius/2
            newY = self.parent.height - self.radius/2    
        else:
            newX = cmd.x
            newY = cmd.y
        
        painter.drawEllipse(newX, newY, self.radius, self.radius)
    
    def transformToHeadSys(self, coord):
        ''' Takes a Command of subtype Coordinate as input. Assumes that the
        coordinates are expressed using the standard base and transforms them to be
        expressed using the head base.'''
        
        if not self.parent:
            return
        
        x = coord.x
        y = coord.y
        
        if coord.isWebOriginated():
            # we don't do that much transformation. we just invert the axes and do a translation
            # bc (0,0) should be in bottom right, not top left
            newX = self.parent.width - x - self.radius/2
            newY = self.parent.height - y - self.radius/2
        else:
            if not self.parent.control.pic:
                return
            # put picture in the middle, but calculate the margin expressed from
            # the right side (bc origin is bottom right in head base)
            lRoboToPicX = self.parent.control.gui.canvasControl.picOrigin.x - self.parent.control.gui.canvasControl.roboOrigin.x
            lRoboToPicY = self.parent.control.gui.canvasControl.picOrigin.y - self.parent.control.gui.canvasControl.roboOrigin.y
            rMarginX = self.parent.width  - (lRoboToPicX + self.parent.control.pic.width)
            rMarginY = self.parent.height - (lRoboToPicY + self.parent.control.pic.height)
        
            if lRoboToPicX < 0:
                lRoboToPicX = 0
            if lRoboToPicY < 0:
                lRoboToPicY = 0
            
            #self.parent.log.debug("Head: self.x=%d, self.y=%d", round(self.x), round(self.y))
            
            newX = round(x + self.radius/2) + rMarginX
            newY = round(y + self.radius/2) + rMarginY
            
            (newX, newY) = (self.parent.width - newX, self.parent.height - newY)
            
            self.parent.log.log(5, "Head: newX=%d, newY=%d", newX, newY)
        
        coord.x = newX
        coord.y = newY

