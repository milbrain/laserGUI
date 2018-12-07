from PyQt5 import QtCore
from PyQt5.QtCore import QObject, pyqtSignal

import collections, logging, copy
from enum import Enum
from RWLock import RWLock

class CommandType(Enum):
    ''' Used to name a command's type. '''
    NONE = 0
    # values 1 to 10 reserved for coordinate-type commands
    PICTURE = 1
    WEB_PRIO = 2
    WEB_ORDERED = 3
    # values 11 to 30 reserved for non-coordinate-type commands
    M5 = 13                 # change plotter's stored values (speed)
    M10 = 14                # status query: give me stats stored in EEPROM of plotter
    MEASURE = 16            # makes plotter measure it's field (internally handled as G28)
    RETURN = 15             # return (to the origin) message
    WEB_COORDINATE = 18     # Describes some web-input coordinate (only used in XYRobot internally)
    DRAW = 19               # supertype for start-drawing commands (non-prio)
    DONT_DRAW = 20          # supertype for stop-drawing commands (non-prio)
    WEB_DRAW = 21           # supertype for start-drawing commands (prio)
    WEB_DONT_DRAW = 22      # supertype for stop-drawing commands (prio)
    
    
class CommandStatus(Enum):
    ''' Names a Command's sending status. '''
    PENDING = 0             # standard-status. means a cmd is enqueued in a plotList. nothing more.
    #TO_BE_SENT = 1         # a cmd that was priorized and is now marked to be sent asap (even in a writePause)
    SENT = 2                # a cmd that was already sent and is waiting for the confirming OK from the plotter. it's id is saved in plotList.lastSent
    CONFIRMED = 3           # a cmd for which an OK message was already received. cmd to be deleted when looked at in decideWhatToPop

class Command():
    ''' Coordinate to be plotted or other command to be sent to plotter.
    id should be unique. A Command's x and y field are always expressed
    using the standard base. Misc is used in commands where the given
    fields are not sufficient (i.e. M5 commands). Which commands are handled
    with priority when it comes to sending is decided by isPriorized method. '''
    
    def __init__(self, id = 0, x = 0.0, y = 0.0, status = CommandStatus.PENDING, type = CommandType.NONE, misc=""):
        self.id = id
        self.x = x
        self.y = y
        self.status = status
        self.type = type
        self.misc = misc
        
    def __repr__(self):
        ret = "([%d] (%3.0f, %3.0f); status: %s; type: %s, misc: %s)" % (self.id, self.x, self.y, self.status.name, self.type.name, self.misc)
        return ret
    
    def __str__(self):
        if not self._nameToBePrinted():
            ret = "(%3.0f, %3.0f) [%d]" % (self.x, self.y, self.id)
        else:
            ret = self.type.name
        
        if self.isPriorized():
            ret += " !"
        if self.status == CommandStatus.SENT:
            ret += " s"
        if self.status == CommandStatus.CONFIRMED:
            ret += " c"
        return ret
    
    def isCoordinate(self):
        ''' Decides if the commands x and y values are to be interpreted. '''
        if self.type in {CommandType.PICTURE, CommandType.WEB_ORDERED, CommandType.WEB_PRIO, CommandType.WEB_COORDINATE,
                         CommandType.RETURN, CommandType.MEASURE}:
            return True
        else:
            return False
    
    def _nameToBePrinted(self):
        ''' Decides about the output in the str() method. Only used internally.'''
        if not self.isCoordinate() or self.type in {CommandType.RETURN, CommandType.MEASURE}:
            return True
        else:
            return False
    
    def isPriorized(self):
        ''' Commands to be sent also in a writePause. '''
        if self.type in {CommandType.M10, CommandType.M5, CommandType.WEB_DONT_DRAW, CommandType.WEB_DRAW,
                         CommandType.WEB_PRIO, CommandType.WEB_ORDERED, CommandType.RETURN, CommandType.MEASURE}:
            return True
        else:
            return False

    def isWebOriginated(self):
        ''' Commands that have their coordinates expressed using the WEB base.
        Includes RETURN and MEASURE so their (0, 0)-Coordinates are interpreted correctly
        for the update of the robot position. '''
        if self.type in {CommandType.WEB_DRAW, CommandType.WEB_DONT_DRAW, CommandType.WEB_PRIO,
                         CommandType.WEB_ORDERED, CommandType.WEB_COORDINATE, CommandType.RETURN,
                         CommandType.MEASURE}: 
            return True
        else:
            return False
    
    def deepCopy(self):
        ''' Returns a deep copy of the command on which it is executed. Useful for
        base transformation methods because they change the object on wich they are
        called. '''
        return Command(self.id, self.x, self.y, self.status, self.type, self.misc)
    
class PlotList(QObject):
    ''' Contains a list of Commands and guarantees access to these commands without
    read/write conflicts between threads. Left part of the deque represents the front
    while the right part represents the end. All operations on this class are thread
    safe. '''

    #frontUpdated = pyqtSignal()     # only emitted when something is inserted
    #endUpdated = pyqtSignal()       # only emitted when something is inserted
    completeRefill = pyqtSignal()   # only emitted manually
    
    def __init__(self, plotL = None):
        ''' Construct a new plotList. plotL can be specified and is another plotList,
        making this constructor a copyconstructor for a deepcopy as well. ''' 
        super().__init__()
        if plotL:
            buf = plotL.buffering
            idC = plotL.idCounter
            lsent = copy.deepcopy(plotL.lastSent)
            plotL = copy.deepcopy(plotL.plotList)
            src = plotL
        else:
            buf = 2
            idC = 0
            lsent = 0
            plotL = None
            src = None
        
        self.buffering = buf                    # number of commands to be buffered in write buffer of serialMonitor
        self.idCounter = idC                    # next ID to be given to a newly inserted command
        self.lock = RWLock()                    # very efficient lock for Multiple-Readers-One-Writer-Problem
        self.lastSent = lsent                   # ID of the command that was lastSent (determined by markCommandAsSent())
        if plotL:
            self.plotList = collections.deque(plotL)
        else:
            self.plotList = collections.deque()
        self.source = src                       # If used as copy constructor: link to the original plotList
    
    def __str__(self):
        ''' String representation of plotList. outputPointNr can be changed
        to adjust the number of points displayed. '''
        self.lock.reader_acquire()
        ret = ""
        if len(self.plotList) == 0:
            self.lock.reader_release()
            return ret
        outputPointNr = 10
        try:
            count = 0
            ret += "---plotList---\n"
            for coord in self.plotList:
                ret += str(coord) + "\n"
                count += 1
                if count == outputPointNr:
                    break
            if len(self.plotList) > outputPointNr:
                ret += "..." + "\n"
            ret += "--------------\n"
        finally:
            self.lock.reader_release()
            return ret

    def count(self):
        ''' Returns the amount of elements in the list. '''
        self.lock.reader_acquire()
        ret = None
        try:
            ret = len(self.plotList)
        finally:
            self.lock.reader_release()
            return ret
    
    def peekFirst(self):
        ''' Returns the first Command of plotList. '''
        self.lock.reader_acquire()
        ret = None
        try:
            if len(self.plotList) == 0:
                ret = None
            else:
                ret = self.plotList[0]
        finally:
            self.lock.reader_release()
            return ret
    
    def peekLast(self):
        ''' Returns the last Command of plotList. '''
        self.lock.reader_acquire()
        ret = None
        try:
            if len(self.plotList) == 0:
                ret = None
            else:
                ret = self.plotList[len(self.plotList)-1]
        finally:
            self.lock.reader_release()
            return ret

    def popFirst(self):
        ''' Removes the first Command from plotList and returns that element.
        Do not emit a frontUpdated signal here. It may lead to a deadlock.'''
        self.lock.writer_acquire()
        ret = None
        try:
            if len(self.plotList) == 0:
                ret = None
            else:
                ret = self.plotList.popleft()
        finally:
            self.lock.writer_release()
            return ret
   
    def popLast(self):
        ''' Removes the last Command from plotList and returns that element.
        Don't emit any endUpdated signal here. It may lead to a deadlock. '''
        self.lock.writer_acquire()
        ret = None
        try:
            if len(self.plotList) == 0:
                ret = None
            else:
                ret = self.plotList.pop()
        finally:
            self.lock.writer_release()
            return ret
    
    def addFront(self, command):
        ''' Inserts Command at the beginning of plotList and returns
        that commands new id. '''
        self.lock.writer_acquire()
        ret = None
        try:
            self.idCounter += 1
            coord = Command(self.idCounter, command.x, command.y, command.status, command.type, command.misc)
            self.plotList.appendleft(coord)
            ret = self.idCounter
        finally:
            self.lock.writer_release()
            #self.frontUpdated.emit()
            return ret
    
    def addEnd(self, command):
        ''' Appends Command at the end of plotList and returns that
        command's new id. '''
        self.lock.writer_acquire()
        ret = None
        try:
            self.idCounter += 1
            coord = Command(self.idCounter, command.x, command.y, command.status, command.type, command.misc)
            self.plotList.append(coord)
            
            ret = self.idCounter
        finally:
            self.lock.writer_release()
            #self.endUpdated.emit()
            return ret
    
    def peek(self, index):
        ''' Returns the element at the given index. '''
        self.lock.reader_acquire()
        ret = None
        try:
            if index < len(self.plotList):
                ret = self.plotList[index]
        finally:
            self.lock.reader_release()
            return ret
    
    def find(self, id):
        ''' Returns the element with the given id. '''
        self.lock.reader_acquire()
        ret = None
        try:
            for coord in self.plotList:
                if coord.id == id:
                    ret = coord
                    break
        finally:
            self.lock.reader_release()
            return ret
        
    def clear(self):
        ''' Deletes all elements from plotList. '''
        self.lock.writer_acquire()
        try:
            self.idCounter = 0
            self.plotList.clear()
        finally:
            self.lock.writer_release()
    
    def markCommandAsSent(self, id):
        ''' Changes a command's status to SENT. '''
        ret = False
        lSent = -1
        try:
            i = 0
            self.lock.writer_acquire()
            
            while i < len(self.plotList):
                if self.plotList[i].id == id:
                    self.plotList[i].status = CommandStatus.SENT
                    #logging.debug("Command %d marked as sent", id)
                    ret = True
                    lSent = id
                    break
                i += 1
        finally:
            self.lock.writer_release()
            if lSent != -1:
                self.lastSent = lSent
            return ret
        
    def markCommandAsConfirmed(self, id):
        ''' Changes a command's status to CONFIRMED. '''
        ret = False
        try:
            i = 0
            self.lock.writer_acquire()
            
            while i < len(self.plotList):
                if self.plotList[i].id == id:
                    self.plotList[i].status = CommandStatus.CONFIRMED
                    ret = True
                    break
                i += 1
        finally:
            self.lock.writer_release()
            return ret
    
    def getLastSent(self):
        ''' Returns the ID of the last element sent. '''
        lSent = -1
        
        self.lock.reader_acquire()
        lSent = self.lastSent
        self.lock.reader_release()
        
        return lSent
        

