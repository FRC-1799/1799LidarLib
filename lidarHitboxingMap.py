
import math
from constants import constants
from lidarHitboxNode import lidarHitboxNode
from lidarLib.lidarMeasurement import lidarMeasurement
from lidarLib.lidarMap import lidarMap
from wpimath.geometry import Pose2d
class lidarHitboxMap:
    
    adjecencyList=[[0, 1],[0, -1],[1, 0],[-1, 0]]    

    def __init__(self, seed:list[list[bool]] =constants.map, xHeight:float= constants.mapHeightMeters, yWidth:float = constants.mapWidthMeters, nodeSideLen:float = constants.mapNodeSizeMeters):
        self.xHeight=xHeight
        self.yWidth=yWidth
        self.nodeSideLen=nodeSideLen
        self.nodeMap:list[list[lidarHitboxNode]]= []
        self.clumps=0
        for y in range(math.ceil(yWidth/nodeSideLen)):
            self.nodeMap.append([])
            for x in range(math.ceil(xHeight/nodeSideLen)):
                
                self.nodeMap[-1].append(lidarHitboxNode(x*nodeSideLen, y*nodeSideLen, sideLen=self.nodeSideLen))


#         self.seed=seed
#         for y in range(0, len(seed)):
#             for x in range(0, len(seed[y])):
                
#                 self.nodeMap[y][x].setLegality(seed[y][x])
        

    def getAtMeters(self, x:int, y:int)->lidarHitboxNode:
        if x>0 and x<constants.mapHeightMeters and y>0 and y<constants.mapWidthMeters:
            return self.nodeMap[math.floor(y/constants.mapNodeSizeMeters)][math.floor(x/constants.mapNodeSizeMeters)]
        else:
            return None

    def getAs1DList(self)->list[lidarHitboxNode]:
        returnList=[]
        for arr in self.nodeMap:
            returnList.extend(arr)

        return returnList


    def addVal(self, reading:lidarMeasurement)->bool:
        if self.getAtMeters(reading.getX(), reading.getY()):
            self.getAtMeters(reading.getX(), reading.getY()).addReading(reading)
            return True
        return False

#     def addMap(self, map:lidarMap):
#         for reading in map.getPoints():
#             self.addVal(reading)


#     def clumpify(self):
#         self.clumps=[]
#         for row in self.nodeMap:
#             for node in row:
#                 node.hasBeenTouched=False

#         for row in self.nodeMap:
#             for node in row:
#                 if not node.hasBeenTouched and not node.isOpen:
                    
#                     self.clumps.append([])
                    

                    que=[node]
                    while que.len!=0:
                        current = que[0]
                        self.clumps[-1].append(current)
                        for side in self.adjecencyList:
                            try:
                                new:lidarHitboxNode = self.nodeMap[current.y+side[0]][current.x+side[1]]
                                new.hasBeenTouched=True
                            except IndexError:
                                continue

                            if not new.isOpen:
                                que.append(node)

    @staticmethod
    def findCenter(clump:list[lidarMeasurement])->Pose2d:
        topHigh:lidarMeasurement=list[0]
        bottomHigh:lidarMeasurement=list[0]
        leftHigh:lidarMeasurement=list[0]
        rightHigh:lidarMeasurement=list[0]
        for measurment in list:
            topHigh = lidarHitboxMap.findExtreme(topHigh, measurment)
            bottomHigh= lidarHitboxMap.findExtreme(bottomHigh, measurment)
            leftHigh = lidarHitboxMap.findExtreme(leftHigh, measurment)
            rightHigh = lidarHitboxMap.findExtreme(rightHigh, measurment)

        if ((topHigh.x-bottomHigh.x)**2+(topHigh.y-bottomHigh.y)**2)>((leftHigh.x-rightHigh.x)**2+(leftHigh.y-rightHigh.y)**2):
            pass
        else:
            pass

        
#     @staticmethod
#     def findExtreme(first:lidarMeasurement, second:lidarMeasurement, isGreater:bool, isX:bool)->lidarMeasurement:
#         firstX, firstY = first.getX(), first.getY()
#         secondX, secondY = second.getX(), second.getY()
#         if isX:
#             if isGreater:
#                 if firstX>secondX:
#                     return first
#                 elif firstX<secondX:
#                     return second
#                 else:
#                     if (firstY>secondY):
#                         return first
#                     else:
#                         return second
            
#             else:
#                 if firstX>secondX:
#                     return second
#                 elif firstX<secondX:
#                     return first
#                 else:
#                     if (firstY<secondY):
#                         return second
#                     else:
#                         return first
                    
#         else:
#             if isGreater:
#                 if firstY>secondY:
#                     return first
#                 elif firstY<secondY:
#                     return second
#                 else:
#                     if (firstX>secondX):
#                         return first
#                     else:
#                         return second
            
#             else:
#                 if firstY>secondY:
#                     return second
#                 elif firstY<secondY:
#                     return first
#                 else:
#                     if (firstX<secondX):
#                         return second
#                     else:
#                         return first
                    
            






