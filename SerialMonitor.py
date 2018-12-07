from PyQt5.QtCore import QObject, pyqtSignal
import sys, serial, threading, logging, time, inspect

from XYRobot import *

class SerialMonitor(QObject):
    '''
    This class is threadsafe. It is a low-level interface for communication with the XY-Plotter through a serial
    port. It's most important method is serial_monitor_thread which runs in a separate thread and has two purposes:
    First it is responsible for receiving all information from the plotter and notifying handleSerialInput through the
    bufferUpdated signal. Secondly it handles the sending of the content of the writeBuffer.
    '''

    # public variables
    bufferUpdated = pyqtSignal(str)
    commandSent   = pyqtSignal(int)
    port = ''                               # serial port to connect to. OS-dependent

    # private variables
    writePause = True                       # pause sending to the plotter while receiving still works.
    writePauseLock = threading.Lock()       # writePause lock
    writeBuffer = []                        # buffer for commands. Size to be set through in plotList.buffering
    wbLock = threading.Lock()               # writeBuffer lock
    plotterConnected = False                # contains connection status
    plotterConnectedLock = threading.Lock() # lock for plotterConnected
    messagesToSend = 0                      # count messages that shall be sent by the plotter. -1 == infinity, 0 == writePause
    messagesToSendLock = threading.Lock()   # lock for messagesToSend
    wbSize = 0                              # holds the maximum size of the writeBuffer (same value as plotList.buffering)
    log = logging.getLogger(__name__)       # the logger
    
    # references
    robot = None                            # reference to robot for access to the state-variable

    def __init__(self, robo, wbSize):
        super(SerialMonitor, self).__init__()
        self.robot = robo
        self.threadRunning = False
        self.thread = threading.Thread(target=self.serial_monitor_thread)
        self.thread.daemon = True
        self.wbSize = wbSize

    def sendMessages(self, x, behave='inc'):
        ''' Make serial_monitor_thread send x messages. x = -1 implies that
        there is no limit on the messages to be send. behave decides how the
        new x value will be set. If
            (messagesToSend = -1, behave == 'replace')  and x = 3 => messagesToSend =  3
            (messagesToSend = -1, behave == 'limit')    and x = 3 => messagesToSend = -1
            (messagesToSend = -1, behave == 'inc')      and x = 3 => messagesToSend = -1
            (messagesToSend =  0, behave == 'inc')      and x = 3 => messagesToSend =  3
            (messagesToSend =  2, behave == 'inc')      and x = 3 => messagesToSend =  5 '''
        self.messagesToSendLock.acquire()
        if behave == 'limit':
            if self.messagesToSend != -1:
                self.messagesToSend = x
        elif behave == 'replace':
            self.messagesToSend = x
        elif behave == 'inc':
            if self.messagesToSend != -1:
                self.messagesToSend += x
        else:
            self.log.critical("sendMessages did not recognize behave argument")
        #self.log.debug("messagesToSend was just set to %d by %s", self.messagesToSend, str(inspect.stack()[1][3]))
        self.messagesToSendLock.release()
            
    def getMessagesToSend(self):
        ''' Getter for messsagesToSend. '''
        ret = 0
        self.messagesToSendLock.acquire()
        ret = self.messagesToSend
        self.messagesToSendLock.release()
        return ret
    
    def isPlotterConnected(self):
        ''' Getter for plotterConnected. '''
        ret = False
        self.plotterConnectedLock.acquire()
        ret = self.plotterConnected
        self.plotterConnectedLock.release()
        return ret
    
    def setPlotterConnected(self, bool):
        ''' Sets the value of the plotterConnected variable. '''
        self.plotterConnectedLock.acquire()
        self.plotterConnected = bool
        self.plotterConnectedLock.release()
    
    def getWbAt(self, index):
        ''' Returns the element at the position of index inside of the writebuffer
        or None if there is no element at the requested position.'''
        ret = None
        try:
            self.wbLock.acquire()
            if index < len(self.writeBuffer):
                ret = self.writeBuffer[index]
        finally:
            self.wbLock.release()
        return ret
    
    def getWbLength(self):
        ''' Returns the length of the writeBuffer. '''
        ret = 0
        self.wbLock.acquire()
        ret = len(self.writeBuffer)
        self.wbLock.release()
        return ret
    
    def clearWb(self):
        ''' Deletes the contents of the writebuffer. '''
        self.wbLock.acquire()
        self.writeBuffer[:] = []
        self.wbLock.release()
        
    def enqueueWb(self, pair):
        '''
        Receives a pair (cmd_id, byte-string) of a command id and a byte-string
        ready to be sent and appends it to the write buffer.
        '''
        self.wbLock.acquire()
        if len(self.writeBuffer) > self.wbSize:
            self.log.error("writeBuffer overflow. Current content: " + str(self.writeBuffer))
            self.writeBuffer[self.wbSize-1] = pair
        else:
            self.writeBuffer.append(pair)
        self.wbLock.release()

    def startThread(self):
        ''' Starts the serial_monitor_thread responsible for listening and sending to the XY-Plotter. '''
        self.threadRunning = True
        self.thread.start()

    def stopThread(self):
        ''' Stops serial_monitor_thread again. '''
        self.threadRunning = False
    
    def isInWritePause(self):
        ''' Returns True if we are currently in a writePause (i.e. messagesToSend != 0). '''
        ret = True
        self.messagesToSendLock.acquire()
        if self.messagesToSend != 0:
            ret = False
        self.messagesToSendLock.release()
        return ret
    
    def pauseWriting(self):
        ''' Pauses sending of the writeBuffer messages. '''
        self.messagesToSendLock.acquire()
        self.messagesToSend = 0
        self.messagesToSendLock.release()
        
    def contWriting(self):
        ''' Stops a write pause and continues writing (an indefinite amount of messages). '''
        self.messagesToSendLock.acquire()
        self.messagesToSend = -1
        self.messagesToSendLock.release()   

    def serial_monitor_thread(self):
        '''
        This method is run in a separate thread. All accessed variables have to be accessed
        by thread-safe methods using Locks or be private variables.
        
        - Try opening serial connection to plotter
        - While connection established:
            - read message from plotter (if existent)
            - send next message from writeBuffer to plotter
        '''
        
        try:
            ser = serial.Serial(self.port, 115200, timeout=None)
            self.setPlotterConnected(True)
        except:
            self.setPlotterConnected(False)
            ser = None
            self.log.debug("Connection not established")
        
        # Listen to serial if serial was created properly
        while self.threadRunning and ser:
            
            if ser.in_waiting > 0:
                msg = ser.readline()
            else:
                msg = None
            if msg:
                try:
                    self.bufferUpdated.emit(msg.decode())
                except ValueError:
                    self.log.error('Wrong data received')
            else:
                # writing if not in writePause mode
                if (self.robot.getState() == RobotState.IDLE and (not self.isInWritePause())
                        and self.getWbLength() > 0 and self.getMessagesToSend() != 0):
                    self.wbLock.acquire()
                    cmdId, tmp = self.writeBuffer.pop(0)
                    self.wbLock.release()
                    ser.write(tmp)
                    self.messagesToSendLock.acquire()
                    if self.messagesToSend > 0:
                        self.messagesToSend -= 1
                    self.messagesToSendLock.release()
                    self.commandSent.emit(cmdId)
                    self.log.info("Sending " + str(tmp))
                    self.robot.goBusy()     # makes robot send the next message only after an answer to the last one was received
                
                if self.getWbLength() <= 0 and not self.isInWritePause():
                    self.pauseWriting()
                    
        
        # Close serial again in case it was created
        try:
            if ser:
                ser.close()

            self.log.debug("Closed port")
        except:
            self.log.debug("Error closing")

        self.setPlotterConnected(False)
