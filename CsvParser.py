from PyQt5.QtGui import*
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from math import *
import logging

class CsvParser():
    ''' Contains all the neccessary tools to parse an image from a csv file and
    plot it afterwards onto a QGraphicsScene object. '''

    log = logging.getLogger(__name__)
    
    def __init__(self, filename, scene):
        self.pathList = []
        self.ptrList = []
        self.scene = scene
        self.width = 0
        self.height = 0
        self.parse(filename)
    
    def parse(self, filename):
        ''' Parses the csv file. filename is expected to be a file with 3 comma
        separated values per line (float, float, int/float). A line is drawn if
        the third value is not equal 0.'''
        
        file = open(filename, 'r')
        
        self.pathList = []
        tmpPath = []
        
        for line in file:
            vals = line.split(",")
            x = round(float(vals[0]),2)
            y = round(float(vals[1]),2)
            penDown = float(vals[2])
            draw = (penDown != 0)
            
            if not draw and len(tmpPath) > 0:
                self.pathList.append(tmpPath)
                tmpPath = []
            
            tmpPath.append((x,y))
            
        self.pathList.append(tmpPath)
        file.close()
            
    def resize(self,drawRect=(150,150,150,150)):
        ''' Resizes the image represented by pathList to fit into drawRect.
        Keeps the ratio of the image. '''
        # Find min and max positions of picture
        min_y, min_x, max_y, max_x = 0, 0, 0, 0
        for path in self.pathList:
            for p in path:
                if p[0] < min_x:
                    min_x = p[0]
                if p[0] > max_x:
                    max_x = p[0]
                if p[1] < min_y:
                    min_y = p[1]
                if p[1] > max_y:
                    max_y = p[1]
        
        dx = max_x - min_x
        dy = max_y - min_y
        ratio = dx / dy
        
        # scaling without transformations only. thats why only 1 scaler exists
        scaler = min(drawRect[2] / dx, drawRect[3] / dy)
        
        # Apply scaling
        for i in range(len(self.pathList)): # paths
            for j in range(len(self.pathList[i])): # points
                x = self.pathList[i][j][0]
                y = self.pathList[i][j][1]
                x = (x - min_x) * scaler + drawRect[0]                    
                y = (y - min_y) * scaler + drawRect[1]
                self.pathList[i][j] = (x,y)
        
        return (dx * scaler, dy * scaler)
    
    def adaptOrigin(self):
        ''' When reading from .csv files, we need to do a transformation to adapt
        to where the origin is. Depends on origin used inside of CSV files. '''
        for i in range(len(self.pathList)): # paths
            for j in range(len(self.pathList[i])): # points
                x = self.pathList[i][j][0]
                y = self.pathList[i][j][1]
                
                # Transformation:
                # 1. translation to origin
                # 2. mirroring on y-axis
                # 3. move back (theoretically not needed bc of resize method)
                (x, y) = (x - self.width, y - self.height)
                (x, y) = (x, -y)
                (x, y) = (x + self.width, y + self.height)
                self.pathList[i][j] = (x,y)
       
    def createPtrList(self):
        self.ptrList = []
        pen = QPen(QColor(124, 124, 124))
        for line in self.pathList:
            tmpPath = QPainterPath()
            for i in range(len(line)):
                point = line[i]
                if i==0:
                    tmpPath.moveTo(point[0],point[1])
                else:
                    tmpPath.lineTo(point[0],point[1])
                    
            ptr = self.scene.addPath(tmpPath, pen=pen)
            self.ptrList.append(ptr)
                
                