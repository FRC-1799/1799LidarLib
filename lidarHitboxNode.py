
from constants import constants
from lidarLib.lidarMeasurement import lidarMeasurement


class lidarHitboxNode:
    def __init__(self, x:float, y:float, sideLen:float = constants.mapNodeSizeMeters, isLegal:bool=True, isOpen:bool=True):
        self.x, self.y = x, y
        self.sideLen=sideLen
        self.readings=[]
        self.isLegal=isLegal
        self.isOpen=isOpen
        self.hasBeenTouched=False

    
#     """Takes the measurment to add to this node. Will return false if the measurment is outside the nodes scope and was theirfor not added, true otherwise"""
#     def addReading(self, reading:lidarMeasurement)->bool:
            
#         #if (reading.getX()>self.x and reading.getX()<self.x+self.sideLen) and (reading.getY()>self.y and reading.getY()<self.y+self.sideLen):
#         self.readings.append(reading)
#         self.isOpen=False
#         return True
#         print("lidarHiboxNode append failure")
#         return False
    

#     def setLegality(self, isLegal:bool):
#         self.isLegal=isLegal