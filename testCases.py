from PyQt5 import QtCore
from PyQt5.QtWidgets import QApplication
import unittest, sys, asyncio, aiounittest, time, logging
import Controls, GUI, XYRobot


#class TestOfflineStuff(unittest.TestCase):
#    pass

class TestPlotterCommunication(unittest.TestCase):
    
    test = True
    
    @unittest.skipIf(test, "")
    def testConnection(self):
        self.assertTrue(self.c.gui.plotterConnected())
        self.assertTrue(self.c.i_disconnectPlotter())
        self.assertTrue(self.c.i_connectPlotter())

    def test_movingReturningAndMeasuring(self):
        self.assertEqual(self.c.serialMonitor.getMessagesToSend(), 0)
        self.assertEqual(self.c.plotList.count(), 0)
        self.assertTrue(self.c.robot.head.x == 0)
        self.assertTrue(self.c.robot.head.y == 0)
        self.assertTrue(self.c.robot.getState() == XYRobot.RobotState.IDLE)

        self.c.i_plotterGotoXY(20,20)
        self.assertEqual(self.c.serialMonitor.getMessagesToSend(), 1)
        self.assertEqual(self.c.plotList.count(), 1)
        self.c.i_startPlotting()
        self.assertEqual(self.c.serialMonitor.getMessagesToSend(), -1)
        self.app.processEvents()
        self.c.serialMonitor.bufferUpdated.emit("OK")
        self.app.processEvents()

        self.assertEqual(self.c.plotList.count(), 0)
        self.assertTrue(self.c.robot.head.x == 20)
        self.assertTrue(self.c.robot.head.y == 20)
        self.assertTrue(self.c.robot.getState() == XYRobot.RobotState.IDLE)
        
        logging.info(self.c.plotList)
        
        self.c.i_plotterReturn()
        self.assertEqual(self.c.plotList.count(), 2)
        #self.c.serialMonitor.bufferUpdated.emit("OK")
        self.c.handleSerialInput("OK")
        self.c.handleSerialInput("OK")
        #self.c.serialMonitor.bufferUpdated.emit("OK")
        #self.app.processEvents()
        self.app.processEvents()
        
        logging.info(self.c.plotList)
        
        self.assertEqual(self.c.plotList.count(), 0)
        self.assertTrue(self.c.robot.head.x == 0)
        self.assertTrue(self.c.robot.head.y == 0)
        self.assertTrue(self.c.robot.getState() == XYRobot.RobotState.IDLE)
        
    
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication(sys.argv)
        cls.c = Controls.Controls()
        cls.c.platform = 'linux'
        cls.c.i_connectPlotter()
        #logging.basicConfig(level=logging.DEBUG)

    @classmethod
    def tearDownClass(cls):
        cls.c.i_disconnectPlotter()
        cls.c = None


        #self.c.i_plotterReturn()
        #self.c.log.setLevel(logging.DEBUG)
        #self.c.serialMonitor.log.setLevel(logging.DEBUG)


class TestServerCommunication(unittest.TestCase):
    # existing Server
    pass

class TestOfflineFunctionality(unittest.TestCase):
    # init
        # existing Robot
        # Robot in scene
        # roboOrigin pos
    # loadPicture (is every object from pathList in the scene)
        # picOrigin pos (dependent on PicPos.BOTTOM or PicPos.MIDDLE)
        # no more objects in scene than objects from pathList and robot
        # amount of elements in plotList equal to pathList - 2 (loadPicIntoPlotList)
    # resize picture by factor 2
        # check is width is aprox * 2
        # resize again by factor 0.5. check again
    # updateWriteBuffer
        # total
        # partial
        # prio
    
    # handleSerialInput tested implicitly
        # M10
        # RoboSize
        # UpdFeed
        # OK
        # Move not allowed
        # distance
        # plotting stops when wb empty
    # prepCmdForBuffer tested implicitly
    # decideWhatToSend tested implicitly
    # decideWhatToPop tested implicitly
    pass

if __name__ == '__main__':
    unittest.main()