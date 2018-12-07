from PyQt5 import QtCore
from http.server import BaseHTTPRequestHandler, HTTPServer
from PyQt5.QtCore import QObject, pyqtSignal

from XYRobot import *
from SvgParser import *
from CsvParser import *
from SerialMonitor import *
from PlotList import *
import Server
from GUI import *
#import parseAndTransform
import sys, logging, threading, argparse, copy

'''
Coordinate conventions:
    - If not stated otherwise, no margin is included in any xy values

PlotList (neutral):
    - Origin:           bottom right
    - Quadrant:         top left
    - Transformation:   none
    - Overview:
        ^
     ab |
     cd |
   <----X----
        |
        |

Plotter:
    - Origin:           bottom right
    - Quadrant:         top left
    - Transformation:   xy-translation (into the plotter center)
                        y-mirroring
    - Overview:
        ^       
     ab |       
     cd |       
    ----X---->  
        |       
        |        
    
GUI:
    - Origin:           top left
    - Quadrant:         bottom right
    - Transformation:   xy-translation
                        xy-mirroring
    - Overview:
        |
        |
    ----X---->
        | ab
        | cd
        v
        
RobotHead:
The RobotHead needs coordinates relative to the robot body.
    - Origin:           bottom right
    - Quadrant:         top left
    - Transformation:   xy-translation, margin from pic to robot (= picOrigin - roboOrigin)
    - Overview:
        ^
     ab |
     cd |
   <----X----
        |
        |
    
Webapp:
Does not use explicit method to handle transformations because no pictures are plotted yet.
Conversions are done inside of drawOnCanvas function.
    - Origin:           top left
    - Quadrant:         bottom right
    - Transformation:   xy-mirroring
                        xy-translation
    - Overview:
        |
        |
    ----X---->
        | ..
        | ..
        v
    
'''

class Controls(QObject):
    ''' Central class of the plotter control. Handles various tasks and connects the individual
    functionality. Handles:
        - Parsing of .svg files using the SvgParser class. Saves the picture in pic
        - Building of the GUI using the GUI class. GUI reference to be accessed through gui
        - Management of the digital clone of the Plotter using the XYRobot class. Reference
          saved in robot
        - Command based communication with the plotter:
            - Handling the responses that come from the SerialMonitor class (can be accessed
              through serialMonitor) using the handleSerialInput(str) slot
            - Management of the plotList (PlotList class) that contains all Commands to be
              plotted
            - Appropriate (re)filling of the writeBuffer (in serialMonitor) with commands from
              plotList
        - Communication with the webapp using the server attribute (MyServer class)
    Be aware that the server and the important part of serialMonitor run in separate threads,
    so synchronization is neccessary when accessing shared values.
    Also Controls contains various high-level interface methods (i_*) that are called by the
    GUI class and also from the webapp. Using those methods is preferrable. '''
    
    # Public attributes
    plotList = None         # Central list of objects to be plotted
    platform = 'windows'    # Possible: 'linux' or 'windows', may be changed by command line params
    extendedGUI = False     # Set by command line argument. Changes display behavior of GUI
    
    # Private attributes
    gui = None
    server = None
    pic = None
    robot = None
    serialMonitor = None
    log = logging.getLogger('Controls')
    plotListDeletedByInterrupt = False  # Was the plotList deleted before the last command finished (e.g. by G28-command)? Used in decideWhatToPop
    
    picPath = ""            # if set, picture is loaded from there without opening the InputDialog


    def __init__(self):
        super().__init__()
        self.initFromCommandLine()
        
        self.plotList = PlotList()
        self.gui = GUI(self)
        
        self.initRobot()
        self.initServer()
        self.serialMonitor = SerialMonitor(self.robot, self.plotList.buffering)
    
    def initFromCommandLine(self):
        ''' Handles setting of parameters given to the program from the command line. '''
        parser = argparse.ArgumentParser()
        parser.add_argument("-w", "--windows", help="Use windows platform specific code",
                            action="store_true")
        parser.add_argument("-l", "--linux", help="Use linux platform specific code",
                            action="store_true")
        parser.add_argument("-d", "--debug", help="Set the debug level to DEBUG",
                            action="store_true")
        parser.add_argument("-v", "--verbose", help="Set the debug level to VERBOSE",
                            action="store_true")
        parser.add_argument("-e", "--extended", help="Show extended GUI that displays detailed plotter data",
                            action="store_true")
        args = parser.parse_args()
        
        if args.windows and args.linux:
            self.log.error("You cannot use --windows (-w) and --linux (-l) options together.")
            sys.exit()
        if args.verbose and args.debug:
            self.log.error("You cannot use --debug (-d) and --verbose (-v) options together.")
            sys.exit()
            
        if args.windows:
            self.platform = 'windows'
        if args.linux:
            self.platform = 'linux'
        if args.debug:
            logging.basicConfig(level=logging.DEBUG)
        elif args.verbose:
            logging.basicConfig(level=5)
        else:
            logging.basicConfig(level=logging.INFO)
            
        if args.extended:
            self.extendedGUI = True
            
    def initServer(self):
        ''' Inits the server that is used to communicate with the webapp, sets up
        the signals needed for the communication and starts the server thread. '''
        port = 8000
        server_address = ('', port)
        self.server = Server.MyServer(server_address, Server.RequestHandler)
        self.server.handler = Server.RequestHandler
        self.server.handler.guiref = self
        
        # signals for safe inter-thread communication
        self.server.returnSignal.connect(self.i_plotterReturn)
        self.server.changePrintModeSignal.connect(self.i_changePrintMode)
        self.server.plotterDrawSignal.connect(self.i_plotterDraw)
        self.server.plotterGotoSignal.connect(self.i_plotterGotoXY)
        
        self.server.startThread()
        self.log.info("Server started.")
        self.log.debug("Server listening on port: " + str(port))
  
    def initRobot(self):
        ''' Inits the robot if it doesn't exist yet. Also used for re-setting
        of roboOrigin in case robot measurements change (e.g. through M10
        command). '''
        if not self.robot:
            self.robot = XYRobot(self)
            # performance reasons copy of plotList
            self.robot.head.visualList = PlotList(self.plotList)
            self.plotList.completeRefill.connect(self.synchronizePlotLists)
        
        self.gui.canvasControl.roboOrigin = CanvasControl.Point((self.gui.canvas.width() - self.robot.width)/2,
            (self.gui.canvas.height() - self.robot.height)/2)
        self.robot.setPos(self.gui.canvasControl.roboOrigin.x, self.gui.canvasControl.roboOrigin.y)
        
        # Copy PlotList for performance reasons (less synchronisation needed)
        self.robot.head.visualList.completeRefill.emit()
        
        if self.robot not in self.gui.canvas.items():
            self.gui.canvas.addItem(self.robot)
        
    def enableTestMode(csv):
        ''' Takes a the path to a csv file as argument. Then the csv file is parsed
        and it's cooridantes are saved into the plotList. In addidion an svg file has
        to be loaded that looks the same as the image from the csv coordinates. This
        is obviously a hackish solution.
        Future work:
        Save the points form the csv file in a pathList (similar to when an svg file
        is parsed using the SvGParser class. This pathList is used by loadPicture to
        plot the an image on the canvas.'''
        # TODO
        pass
    
    def loadPicture(self, tpicPath=""):
        ''' Loads a picture, draws it on the canvas and fills the plotList using that
        information. tpicPath can be used to pass a fixed image to the loadPicture method
        once (e.g. for the resize-method) while still beeing able to open a QFileDialog
        at some other point. 
        '''
        if not tpicPath:
            fnameTuple = QFileDialog.getOpenFileName(self.gui, 'Open file', '', "Plotter Image files (*.svg *.csv)")
            tpicPath = fnameTuple[0]
            if tpicPath == "":
                self.infoLog("Choose a valid picture to continue")
                return
            else:
                self.picPath = tpicPath
        vals = self.picPath.split('.')
        ending = vals.pop()
        
        # Delete: Previous picture from scene (if existent), writeBuffer, plotList
        if self.pic:
            for item in self.pic.ptrList:
                self.gui.canvas.removeItem(item)
        self.serialMonitor.pauseWriting()
        self.serialMonitor.clearWb()
        self.plotList.clear()
        
        # Load picture
        self.log.debug("Recognized %s-file", ending)
        if ending == 'csv':
            self.pic = CsvParser(self.picPath, self.gui.canvas)
            self.pic.adaptOrigin()
        else:
            self.pic = SvgParser(self.picPath, self.gui.canvas)
        
        # Place and resize picture
        w = self.gui.canvasControl.initialPictureSize.x
        h = self.gui.canvasControl.initialPictureSize.y
        (w,h) = self.pic.resize((0,0,w,h))
        self.pic.width = w
        self.pic.height = h
        
        # Set picture origin
        self.gui.canvasControl.picOrigin = CanvasControl.Point(self.gui.canvasControl.roboOrigin.x + (self.robot.width - w)/2,
                                                               self.gui.canvasControl.roboOrigin.y + (self.robot.height - h)/2)
        
        # Fill plotList (using coordinates)
        self.loadPicIntoPlotList(self.pic.pathList)
        self.plotList.completeRefill.emit()     # completeRefill-handler needs set picOrigin and roboOrigin
        #translate picture
        self.pic.resize((self.gui.canvasControl.picOrigin.x,self.gui.canvasControl.picOrigin.y,w,h))
        
        # Plot picture
        self.pic.createPtrList()                # Create the pointers to be able to reference individual paths
        self.gui.canvasView.viewport().update()
        
        # Logging
        self.log.debug("Item size is %.1f x %.1f" % (w, h))
        self.log.debug("Picture consists of %d items" % (len(self.gui.canvas.items())-2))    # -2 parts because robot consists of Frame and Head (2 parts)
        if (len(self.pic.ptrList)) != (len(self.gui.canvas.items())-2):
            self.log.warning("pic.ptrList contains entries: %d, but canvas contains %d" % (len(self.pic.ptrList), len(self.gui.canvas.items())-2))
        self.log.debug("roboOrigin: " + str(round(self.gui.canvasControl.roboOrigin.x,1)) + " " + str(round(self.gui.canvasControl.roboOrigin.y,1)))
        self.log.debug("picOrigin: " + str(round(self.gui.canvasControl.picOrigin.x,1)) + " " + str(round(self.gui.canvasControl.picOrigin.y,1)))
        self.infoLog("Picture loaded")

    def loadPicIntoPlotList(self, pathList):
        ''' Takes a pic.pathList as argument and fills the central plotList structure
        with the coordinates taken from pathList.
        - pathList contains a list of paths.
        - a path consists of a list of points. '''
        
        if not pathList:
            return
        
        pathListLen = len(pathList)
        pathCnt = 0
        for path in pathList:
            # Loop through all points of every path
            for i in range(len(path)):
                (x,y) = path[i]
                
                # Turn laser power down when performing a transition
                if i == 0:
                    comm = Command(0, 0, 0, CommandStatus.PENDING, CommandType.DONT_DRAW)
                    self.plotList.addEnd(comm)
                
                # transformation [GUI (without borders) base] -> [plotList base]
                # only to be used once
                (x, y) = (x - self.pic.width, y - self.pic.height)
                (x, y) = (-x, -y)
                # transform end
                    
                comm = Command(0, x, y, CommandStatus.PENDING, CommandType.PICTURE)
                self.plotList.addEnd(comm)
                
                # Turn laser power back up when performing a transition
                if i == 0:
                    comm = Command(0, 0, 0, CommandStatus.PENDING, CommandType.DRAW)
                    self.plotList.addEnd(comm)
                    
            pathCnt += 1   
        # Turn laser off and send goback message
        comm = Command(0, 0, 0, CommandStatus.PENDING, CommandType.DONT_DRAW)
        self.plotList.addEnd(comm)
        comm = Command(0, 0, 0, CommandStatus.PENDING, CommandType.RETURN)
        self.plotList.addEnd(comm)
        self.log.debug("Finished adding elements to plotList. Elemen count is %d" % self.plotList.count()) 
    
    def updateWriteBuffer(self, type='total'):
        ''' Updates the writeBuffer with commands from plotList. Also works in writePause
        - Total refresh is recommended after something was inserted at the front of plotList. All
          kinds of elements are inserted. (type == 'total')
        - Total refresh can also be made with only the priorized commands inserted into the wb.
          (type == 'prio')
        - Partial refresh is more efficient than total refresh. It checks which commands are not
          in the write buffer yet and inserts only those into the wb. (type = 'partial') '''
              
        messagesToSend = self.serialMonitor.getMessagesToSend()
        
        if type == 'total' or type == 'prio':
            # Typically when sth was inserted at the front of plotList
            self.log.debug("Doing complete wb refresh (type: " + type + ")")
            
            self.serialMonitor.pauseWriting()
            self.serialMonitor.clearWb()
            
            if type == 'total':
                i = 0
                cmd = self.plotList.peek(i)
                while cmd and self.serialMonitor.getWbLength() < self.serialMonitor.wbSize:
                    if cmd.status == CommandStatus.PENDING:
                        cmd = cmd.deepCopy()
                        msg_bytes = self.prepCmdForBuffer(cmd)
                        self.serialMonitor.enqueueWb(msg_bytes)
                    i += 1
                    cmd = self.plotList.peek(i)
                    
            elif type == 'prio':
                i = 0
                cmd = self.plotList.peek(i)
                while cmd and cmd.isPriorized() and self.serialMonitor.getWbLength() < self.serialMonitor.wbSize:
                    if cmd.status == CommandStatus.PENDING:
                        cmd = cmd.deepCopy()
                        msg_bytes = self.prepCmdForBuffer(cmd)
                        self.serialMonitor.enqueueWb(msg_bytes)
                    i += 1
                    cmd = self.plotList.peek(i)
                    
            if messagesToSend != 0:
                self.serialMonitor.sendMessages(messagesToSend, behave='replace')
        else:
            # Just append the next message(s) to be plotted into the wb
            # Typically after a message was sent
            self.log.debug("Doing partial wb refresh")
            
            if messagesToSend != 0:
                i = 0
                cmd = self.plotList.peek(i)
                
                while cmd and self.serialMonitor.getWbLength() < self.serialMonitor.wbSize:
                    CmdInWb = False
                        
                    j = 0
                    wbElem = self.serialMonitor.getWbAt(0)
                    while wbElem:
                        cmdId, buf = wbElem
                        wbElem = self.serialMonitor.getWbAt(j)

                        if cmd.id == cmdId:
                            CmdInWb = True
                            break
                        j += 1
                            
                    if not CmdInWb and cmd.status == CommandStatus.PENDING:
                        cmd = cmd.deepCopy()
                        msg_bytes = self.prepCmdForBuffer(cmd)
                        self.serialMonitor.enqueueWb(msg_bytes)
                    
                    i += 1
                    cmd = self.plotList.peek(i)
        
    @pyqtSlot(str)
    def handleSerialInput(self, msg):
        ''' Handles the responses gotten from the plotter. '''
        if msg.find("M10") >= 0:
            # getStats command (M10)
            # Value H gotten from plotter is not used by XY-Plotter, but only other Makeblock robots
            msg = msg.strip()
            c = msg.split(' ')
            msg = msg.replace('M10 ', 'Stats:')
            self.infoLog(msg)
            self.robot.width = int(c[2])
            self.robot.height = int(c[3])
            self.robot.motoADir = int(c[6][1])
            self.robot.motoBDir = int(c[7][1])
            self.robot.speed = int(c[9][1:])
            self.gui.optionsLabel.setText("Stats:\n\n"
                                      + "Width: " + c[2] + "\n"
                                      + "Height: " + c[3] + "\n"
                                      + "XY-Pos: (" + str(round(self.robot.head.x)) + ", " + str(round(self.robot.head.y)) + ")\n"
                                      + "MotorADir: " + c[6][1] + "\n"
                                      + "MotorBDir: " + c[7][1] + "\n"
                                      + "Speed: " + c[9][1:] + "\n"
                                      + "Write Mode: " + self.robot.printMode.name + "\n")
            
            # Refresh Canvas
            self.initRobot()            # in case Robot coordinates changed
        elif msg.find("RoboSize") >= 0:
            # Response from Measure XY-area command (MEASURE)
            msg = msg.strip()
            c = msg.split(' ')
            stepsY = int(c[1])
            stepsX = int(c[2])
            stepsPerMM = float(c[3])
            self.log.info("RoboSize Message received\n"
                          "StepsY: %d, StepsX: %d, stepsPerMM: %.2f",
                          stepsY, stepsX, stepsPerMM);
        elif msg.find("UpdFeed") >= 0:
            # Update Feed from plotter. Is only interpreted when in extendedGUI-mode
            if not self.extendedGUI:
                return
            
            msg = msg.strip()
            c = msg.split(' ')
            posA = c[1]
            posB = c[2]
            tarA = c[3]
            tarB = c[4]
            mDelay = c[5]
            temp_delay = c[6]
            stepdelay_max = c[7]
            stepdelay_min = c[8]
            lpx1 = c[9]
            lpx2 = c[10]
            lpy1 = c[11]
            lpy2 = c[12]
            laserOn = c[13]
            if c[14] != "UpdFeed":
                self.log.warning("Wrong composition of Update Feed message from plotter.")
            
            self.gui.extendedStatsContent1.setText("(" + str(posA) + ",\t" + str(posB) + ")")
            self.gui.extendedStatsContent2.setText("(" + str(tarA) + ",\t" + str(tarB) + ")")
            self.gui.extendedStatsContent3.setText(mDelay)
            self.gui.extendedStatsContent4.setText(temp_delay)
            self.gui.extendedStatsContent5.setText(stepdelay_max)
            self.gui.extendedStatsContent6.setText(stepdelay_min)
            self.gui.extendedStatsContent7.setText(lpx1 + " " + lpx2 + " " + lpy1 + " " + lpy2)
            self.gui.extendedStatsContent8.setText(laserOn)
        elif msg.find("OK") >= 0:
            # Sent message is removed from plotList and new ones enqueued into the write buffer
            
            self.decideWhatToPop()
            self.updateWriteBuffer('partial')
            self.robot.goIdle()
            self.log.info("OK")
            self.log.debug("Current RobotStatus is: %s, wbLength is: %d, getMessagesToSend(): %d",
                          self.robot.getState().name, self.serialMonitor.getWbLength(), self.serialMonitor.getMessagesToSend())
            
        elif msg.find("Move not allowed") >= 0:
            stri = msg[17:].strip()
            self.gui.showLimitPinsPressed(stri)
            self.server.i_setLimitPinMsg(stri)
            self.server.i_setShowLimitPins()
            self.infoLog("Plotter head hit limit pins. Message: " + stri)
            
        elif msg.find("distance") >= 0:
            # Logging distance while printing
            #self.robot.head.visitNext() #<-- handled by decideWhatToPop()
            self.log.info(msg.strip())
        else:
            self.log.warning("Unrecognized message: " + msg)
        
        # Update GUI    
        self.gui.canvasView.viewport().update()
        self.gui.drawPlotList()
        
        if self.serialMonitor.getWbLength() <= 0 and not self.serialMonitor.isInWritePause():
            self.serialMonitor.pauseWriting()
            self.log.info("Writing finished because nothing to write left.")

    def prepCmdForBuffer(self, cmd):
        ''' High-level method which does all the conversion needed to transform
        a Coordinate from plotList into a form ready to be sent to the plotter. It
        does a coordinate system transformation (if neccessary) and returns a 
        '\\n'-terminated bytestring along with that commands ID as tuple (id, String).
        
        This method does changes on the given command. If that's not intended, use
        Command's deepCopy() method.'''
        
        self.log.debug("PrepCmdForBuffer called. Transormed " + repr(cmd))
        
        if cmd.isCoordinate():
            self.robot.transfToPlotterSys(cmd)
        
        self.log.debug("into " + repr(cmd))
        
        gcode = self.robot.cmdToGCode(cmd)
        idAndGCode = (cmd.id, gcode)
        return idAndGCode
    
    def infoLog(self, msg):
        ''' Helper method for logging. '''
        self.gui.statusBar().showMessage(msg)
        self.log.info(msg)

    def synchronizePlotLists(self):
        self.robot.head.visualList = PlotList(self.plotList)
        self.gui.drawPlotList()
        
    def decideWhatToSend(self):
        ''' Decides if a command from the front of the plotList shall be sent.
        To be used while robotState != IDLE (because a correctly filled write
        buffer is required for this method). Otherwise a call to updateWriteBuffer
        is required beforehand. This method will initiate the writing process if
        a priorized command was found at the front. '''
        actionDone = False
        
        if self.serialMonitor.getMessagesToSend() < 0:
            # Nothing to be done since sending is enabled permanently
            return actionDone
        
        firstCmd  = self.plotList.peekFirst()
        secondCmd = self.plotList.peek(1)

        if firstCmd and firstCmd.isPriorized():
            self.serialMonitor.sendMessages(1, 'inc')
            if (secondCmd and secondCmd.isPriorized() and
                    ( (firstCmd.type == CommandType.M5      and secondCmd.type == CommandType.M10) or
                      (firstCmd.type == CommandType.RETURN  and secondCmd.type == CommandType.M10)    )):
                self.serialMonitor.sendMessages(1, 'inc')
            actionDone = True
            
        return actionDone
    
    def decideWhatToPop(self):
        ''' Does the following:
        Optionally: Remove 0-n confirmed elements from the front of the plotList
        Mandatory (only one of the two):
            - Pop a sent element (if sent element = first element)
            - Mark a sent element as confirmed (if sent element != first element)
        If a RETURN command was issued and interrupted another command, the first
        OK (the OK of the interrupted command) is to be ignored.'''
        
        if self.plotListDeletedByInterrupt:
            self.plotListDeletedByInterrupt = False
            return
        
        visualListAsWell = True
        actionDone = False          # Makes sure that exactly one action gets done

        while not actionDone:
            firstCmd = self.plotList.peekFirst();

            # remove all confirmed elements from the front
            while firstCmd and firstCmd.status == CommandStatus.CONFIRMED:
                firstCmd = self.plotList.popFirst()
                
                if visualListAsWell:
                    self.robot.head.visualList.popFirst()
                firstCmd = self.plotList.peekFirst();
            
            # standard case: pop one element after one OK message.
            if firstCmd and firstCmd.status == CommandStatus.SENT:
                firstCmd = self.plotList.popFirst()
                if visualListAsWell:
                    self.robot.head.visualList.popFirst()
                
                actionDone = True
                self.log.debug("PlotList: Popped command: %s", repr(firstCmd))

            
            # look for an element to mark as confirmed

            if firstCmd and not actionDone:
                self.plotList.markCommandAsConfirmed(self.plotList.getLastSent())
                actionDone = True
                self.log.debug("PlotList: Marked command as confirmed: %s", repr(firstCmd))

            
        # position: needed for pos-update of points sent from web
        if firstCmd:
            self.robot.head.updatePosition(firstCmd.id)

    def i_connectPlotter(self):
        ''' Establish a connection to the given serial port.
        Returns True    if after the execution the plotter is connected for sure.
                False   if the plotter is either still disconnected or it can't be said
                        what the connections status is. '''
                        
        if self.platform == "windows":
            port = "COM3"           # Windows port
        elif self.platform == "linux":
            port = "/dev/ttyUSB0"   # Linux port
        
        if not self.serialMonitor or not self.serialMonitor.isPlotterConnected():
            # (Re)create serialMonitor because dead threads cannot be revived
            self.serialMonitor = SerialMonitor(self.robot, self.plotList.buffering)
            self.serialMonitor.port = port
        
        if self.serialMonitor.isPlotterConnected():
            self.log.debug("Plotter already connected")
            return True
        else:
            self.log.debug("Connecting to plotter on port: " + port)
            self.serialMonitor.bufferUpdated.connect(self.handleSerialInput)
            self.serialMonitor.commandSent.connect(self.plotList.markCommandAsSent)
            self.serialMonitor.commandSent.connect(self.robot.head.updatePosition)  # needed for fast update of GOTO-Point
            self.serialMonitor.startThread()
            
            time.sleep(1)                   # Wait for the connection to be established
            if self.serialMonitor.isPlotterConnected():
                self.infoLog("Connection established")
                return True
            else:
                self.log.error("Could not connect to port " + port)
                self.serialMonitor.stopThread()   # Stop thread if connection to serial port does not work
                return False
            
    def i_disconnectPlotter(self):
        ''' Disconnect from plotter and kill the serialMonitor thread.
        Returns True    if after the execution the plotter is disconnected for sure.
                False   if the plotter is either still connected or it can't be said
                        what the connections status is. '''
        if self.serialMonitor:
            if self.serialMonitor.isPlotterConnected():
                self.log.debug("Disconnecting from plotter...")
                self.serialMonitor.stopThread()
                self.serialMonitor.bufferUpdated.disconnect(self.handleSerialInput)
                self.serialMonitor.commandSent.disconnect(self.plotList.markCommandAsSent)
                
                time.sleep(1)                   # wait for the thread to disconnect from serial
                if not self.serialMonitor.isPlotterConnected():
                    self.infoLog("Disconnected from plotter")
                    return True
                else:
                    self.log.warning("May still be connected to serial port")
                    return False
            else:
                self.log.debug("Already disconnected from plotter")
                return True
        else:
            self.log.warning("You cannot disconnect because serialMonitor does not exist.")
            return True
    
    def i_plotterGotoXY(self, x, y, ordering="prio"):
        ''' x and y have to be ints. Enqueue a message into the plotList that tells
        the plotter to go to destination XY. Ordering decides if the msg is
        enqueued at the front or back of the plotList. '''
        self.log.info("plotterGotoXY received msg. x=%d, y=%d, ordering=%s", x,y,ordering)
        
        if isinstance(x, int) and isinstance(y, int):
            cmd = Command(x=x, y=y)
            if "prio" in ordering:
                cmd.type = CommandType.WEB_PRIO
                self.plotList.addFront(cmd)  
            else:
                cmd.type = CommandType.WEB_ORDERED
                self.plotList.addEnd(cmd)
            
            self.updateWriteBuffer('prio')
            self.decideWhatToSend()
        self.synchronizePlotLists()
                
    def i_plotterDraw(self, to="yes"):
        ''' Turns laser or pen on or off immediately (as priorized Command). '''
        
        if to == "yes":
            cmd = Command(type=CommandType.WEB_DRAW)
        else:
            cmd = Command(type=CommandType.WEB_DONT_DRAW)
        
        self.plotList.addFront(cmd)
        
        self.updateWriteBuffer('prio')
        self.decideWhatToSend()
        self.synchronizePlotLists()
              
    def i_setSpeed(self, speed):
        ''' Send a command to the plotter that sets its speed to speed. Expects
        speed to be of type Integer. After that an M10 command is automatically
        enqueued as well to update the robot's stats in the GUI. '''
        
        if isinstance(speed, int):
            spd = min(100, max(0, speed))
            spdstr = str(spd)
            cmdM5 = Command(0, self.robot.width, self.robot.height, CommandStatus.PENDING, CommandType.M5, spdstr)
            cmdM10 = Command(0, 0, 0, CommandStatus.PENDING, CommandType.M10)
            
            self.plotList.addFront(cmdM10)
            self.plotList.addFront(cmdM5)
            self.updateWriteBuffer('prio')
              
            self.decideWhatToSend()
            self.synchronizePlotLists() #Speed setting theoretically doesn't need to be displayed
                
        else:
            self.log.error("i_setSpeed needs a parameter of type int")
        
        self.infoLog("Speed change issued (to " + str(speed) + ")")
        
    def i_startPlotting(self):
        ''' Clear wb and enqueue some fresh commands from plotList into
        the wb. Make the plotter start plotting by exiting writePause.
        '''
        
        # Enqueue some messages into the write buffer
        self.updateWriteBuffer('total')
        self.serialMonitor.contWriting()
        
    def i_pausePlotting(self):
        ''' Pause the sending of the messages and delete the write Buffer. '''
        self.serialMonitor.pauseWriting()
        self.serialMonitor.clearWb()
        
    def i_contPlotting(self):
        ''' So far: same functinality as i_startPlotting(). Maybe different in the
        future. '''
        self.i_startPlotting()
        
    def i_getStats(self):
        ''' Get the stats that are saved in the plotter (as priorized message). '''
        command = self.plotList.peekFirst()
        if command and command.type == CommandType.M10:
            self.log.info("An M10 Command is already inserted")
            return
        else:
            cmd = Command(0, 0, 0, CommandStatus.PENDING, CommandType.M10)
            self.plotList.addFront(cmd)
            self.updateWriteBuffer('prio')
            self.decideWhatToSend()
            
        self.synchronizePlotLists()
    
    def i_changePrintMode(self, to=PrintMode.NONE):
        ''' Changes the PrintMode of the plotter. Only to be used while laser is
        off or pen is in not-writing position. Also better to be only used in
        write pause. '''
        if to != PrintMode.NONE:
            self.robot.printMode = to
            return
        
        pm = self.robot.printMode
        if pm == PrintMode.LASER:
            self.robot.printMode = PrintMode.PEN
            self.infoLog("Changed printMode to PEN")
        else:
            self.robot.printMode = PrintMode.LASER
            self.infoLog("Changed printMode to LASER")
    
    def i_plotterReturn(self):
        ''' Uses a command of type CommandType.RETURN to make the Head
        return to the plotter origin. Makes sure Laser is turned off and
        pen is up before that (handled by plotter firmware).'''
        cmdRet = Command(0, 0, 0, CommandStatus.PENDING, CommandType.RETURN)
        cmdM10 = Command(0, 0, 0, CommandStatus.PENDING, CommandType.M10)
        
        if self.pic:
            for item in self.pic.ptrList:
                self.gui.canvas.removeItem(item)
            self.pic = None
        
        if not self.serialMonitor.isInWritePause():
            self.plotListDeletedByInterrupt = True    # used for correct popping of commands
        self.serialMonitor.pauseWriting()
        self.serialMonitor.clearWb()
        self.plotList.clear()
        
        self.plotList.addFront(cmdM10)
        self.plotList.addFront(cmdRet)
        self.updateWriteBuffer('prio')
        
        self.robot.goIdle()
        self.decideWhatToSend()
        self.synchronizePlotLists()
        self.infoLog("Plotter returning to origin");
        
    def i_measureArea(self):
        ''' Uses a command of type CommandType.MEASURE to make the head
        measure the size of the field that can be used. '''
        cmdMeasure = Command(0, 0, 0, CommandStatus.PENDING, CommandType.MEASURE)
        
        self.plotList.addFront(cmdMeasure)
        self.updateWriteBuffer('prio')
              
        self.decideWhatToSend()
        self.synchronizePlotLists()
        self.infoLog("Plotter measuring the area size.");
            
        
if __name__ == '__main__':
    app = QApplication(sys.argv)
    controls = Controls()
    sys.exit(app.exec_())