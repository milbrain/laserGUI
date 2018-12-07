from http.server import BaseHTTPRequestHandler, HTTPServer
from PyQt5.QtCore import QObject, pyqtSignal
import urllib.parse
import json
import threading
import logging
import XYRobot
from collections import namedtuple

class RequestHandler(BaseHTTPRequestHandler):
    ''' Class reacts to the following requests:
    GET:
        - get=fielddata
    POST:
        - action=move&gotoX=<val>&gotoY=<val>   (goto XY)
        - action=write                          (drawing on / drawing off)
        - action=changePrintMode                (draw with Pen / draw with Laser)
        - action=returnToOrigin                 (return to Origin)
        - action=pause                          (pause plotting)    TODO
        - action=cont                           (continue plotting) TODO
    Signals are used for communication with the GUI methods because RequestHandler
    runs in a separate thread. Otherwise racing conditions occur.
    '''
    guiref = None
    log = logging.getLogger(__name__)
    
    #returnSignal = pyqtSignal()
    
    def _set_headers(self, str):
        self.send_response(200)
        self.send_header('Content-type', str)
        self.end_headers()

    def do_GET(self):
        parsed_path = urllib.parse.urlparse(self.path)
        
        # Set path separator depending on platform
        if self.guiref.platform == 'windows':
            ps = "\\"
        elif self.guiref.platform == 'linux':
            ps = "/"
        
        # GET-Parameter
        if parsed_path.path == "/" and "get=fielddata" in parsed_path.query:
            output = {}
            if not self.guiref:
                robodata = {}
            elif not self.guiref.robot:
                robodata = {}
            else:
                pm = self.guiref.robot.printMode
                if pm == XYRobot.PrintMode.LASER:
                    ps = 'Laser on' if self.guiref.robot.printStatusOn else 'Laser off'
                elif pm == XYRobot.PrintMode.PEN:
                    ps = 'Pen down' if self.guiref.robot.printStatusOn else 'Pen up'
                else:
                    ps = 'Print Status not recognized'
                
                showLp = self.server.i_getShowLimitPins()
                if showLp:
                    self.server.i_setShowLimitPins(False)
                    lpMsg = self.server.i_getLimitPinMsg()
                else:
                    lpMsg = "0"
                
                
                robodata = {
                    'roboHeight':       self.guiref.robot.height,
                    'roboWidth':        self.guiref.robot.width,
                    'headX':            self.guiref.robot.head.x,
                    'headY':            self.guiref.robot.head.y,
                    'speed':            self.guiref.robot.speed,
                    'laserPower':       self.guiref.robot.laserPower,
                    'printMode':        pm.name,
                    'printStatus':      ps,
                    'limitPins':        lpMsg,
                    'cmdOriginIsPic':   (self.guiref.robot.head.cmdOrigin == XYRobot.CommandType.PICTURE),
                    'picOrigin':        (round(self.guiref.gui.canvasControl.picOrigin.x - self.guiref.gui.canvasControl.roboOrigin.x),
                                         round(self.guiref.gui.canvasControl.picOrigin.y - self.guiref.gui.canvasControl.roboOrigin.y))
                }
                
            output.update(robodata)
            self._set_headers('application/json')
            dump = json.dumps(output)
            dump2 = dump.encode('utf-8')
            self.wfile.write(dump2)
        
        # Implizite Anfragen
        elif parsed_path.path == "/":
            self._set_headers('text/html')
            file = open(r'res' + ps + r'indexHTML.html', "rb")
            self.wfile.write(file.read())
            file.close()
        elif parsed_path.path == "/DiK_Logo.jpg":
            self._set_headers('image/jpeg')
            file = open(r'res' + ps + r'DiK_Logo.jpg', "rb")
            self.wfile.write(file.read())
            file.close()
        elif parsed_path.path == "/jquery-3.1.0.js":
            self._set_headers('application/javascript')
            file = open(r'res' + ps + r'jquery-3.1.0.js', "rb")
            self.wfile.write(file.read())
            file.close()
        elif parsed_path.path == "/favicon.ico":
            self._set_headers('image/x-icon')
            file = open(r'res' + ps + r'favicon.ico', "rb")
            self.wfile.write(file.read())
            file.close()

    def do_HEAD(self):
        self._set_headers('text/html')
        
    def do_POST(self):
        content_length = int(self.headers['Content-Length']) # <--- Gets the size of data
        post_data = bytes.decode(self.rfile.read(content_length)) # <--- Gets the data itself
        
        self.log.info("post_data: " + post_data)
        post_args = post_data.split("&")
        alert = ""                            # Cumulative error string
        if "action=move" in post_data:
            # Nachricht splitten, Werte extrahieren und übergeben
            xI = -1
            yI = -1
            mI = -1
            i = -1
            for arg in post_args:
                i += 1
                if "X=" in arg:
                    xI = i
                if "Y=" in arg:
                    yI = i
                if "moveOrder=" in arg:
                    mI = i
                    
            posX = post_args[xI][2:]
            posY = post_args[yI][2:]
            moveOrder = post_args[mI][10:]
            
            self.server.plotterGotoSignal.emit(int(posX), int(posY), moveOrder)
        elif "action=write" in post_data:
            changeTo = ""
            for arg in post_args:
                if "changeTo" in arg:
                    changeTo = arg[9:]
            self.server.plotterDrawSignal.emit(changeTo)
        elif "action=changePrintMode" in post_data:
            self.server.changePrintModeSignal.emit()
        elif "action=returnToOrigin" in post_data:
            self.server.returnSignal.emit()
        elif "action=pause" in post_data:
            pass
        elif "action=cont" in post_data:
            pass
        else:
            alert += "Alert: Wrong action parameter"
        
        self._set_headers("text/html")
        if alert != "":
            wdata = alert
        else:
            wdata = "Status: OK"
        self.wfile.write(wdata.encode("utf-8"))
    
    def log_message(self, format, *args):
        ''' Decides about the form of the logged messages. Everything is logged
        except for get=fielddata requests (they happen periodically, aprox. every
        2 seconds). '''
        if not args or args[0][6:19] != "get=fielddata":
            self.log.info("%s - - [%s] %s" % (self.address_string(),self.log_date_time_string(),format%args))
        
    def log_error(self, format, *args):
        self.log.error("%s - - [%s] %s" % (self.address_string(),self.log_date_time_string(),format%args))
        
class MyServer(HTTPServer, QObject):
    ''' Server Class used for communication with the webapp. Signals are used for communication
    with Controls. Direct method calls are not possible because the server runs in it's own
    separate thread. '''
    thread = None
    handler = None
    
    returnSignal = pyqtSignal()
    changePrintModeSignal = pyqtSignal()
    plotterDrawSignal = pyqtSignal(str)
    plotterGotoSignal = pyqtSignal(int, int, str)
    
    def startThread(self):
        ''' Server starts listening (in own thread). '''
        self.thread = threading.Thread(target=self.serve_forever)
        self.thread.daemon = True
        self.thread.start()
    
    def __init__(self, server_address, RequestHandlerClass):
        #super().__init__(server_address, RequestHandlerClass)
        HTTPServer.__init__(self, server_address, RequestHandlerClass)
        QObject.__init__(self)

        # limit pins forwarding at next get request and only once
        self.showlimitPins = False
        self.limitPinLock = threading.Lock()
        self.limitPinMsg = "initial-not-important-value"
        self.limitPinMsgLock = threading.Lock()
        
    def i_setShowLimitPins(self, to=True):
        ''' Change limit-showing behavior. True means that on next get-request
        the webapp shows a (one-time) message that the pins were hit. Setter
        needed for synchronisation reasons. '''
        self.limitPinLock.acquire()
        self.showlimitPins = to
        self.limitPinLock.release()
        
    def i_getShowLimitPins(self):
        ''' Getter for showlimitPins for synchronization reasons. '''
        ret = False
        self.limitPinLock.acquire()
        ret = self.showlimitPins
        self.limitPinLock.release()
        return ret
    
    def i_setLimitPinMsg(self, submsg):
        ''' Used to insert current limit-Pin values into the message to be sent. '''
        msg = "The plotter just hit the limit pins and stopped.\n\n" + submsg
        self.limitPinMsgLock.acquire()
        self.limitPinMsg = msg
        self.limitPinMsgLock.release()
    
    def i_getLimitPinMsg(self):
        ''' used to get the current limitPinMsg (should be set using setLimitPinMsg first). '''
        ret = ""
        self.limitPinMsgLock.acquire()
        ret = self.limitPinMsg
        self.limitPinMsgLock.release()
        return ret
        
def run(server_class=HTTPServer, handler_class=RequestHandler, port=8000):
    server_address = ('', port)
    self.httpd = server_class(server_address, handler_class)
    self.httpd.serve_forever()