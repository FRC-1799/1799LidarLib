from time import time
from lidarLib.translation import translation
from typing import TYPE_CHECKING, Any
if TYPE_CHECKING:
    from lidarLib import Lidar
    from lidarLib.lidarMeasurement import lidarMeasurement


class lidarMap:
    """
        Class for handling a full 360 scan of lidar data
    """
    def __init__(self, hostLidar:"Lidar.Lidar", mapID:int=0, deadband:list[float]=None, sensorThetaOffset:float=0): # type: ignore
        """
            <h2>Initializes a lidarMap Map"</h2>
            Host lidar should be set to the lidar object responsible for populating the map.
            This value is only used if the point map is still being updated and so is unnecessary if the map is done being filled.\n
            MapId is a id for the map and should be unique to the lidar. However this value need not be unique from mapIDs on other lidar units. \n
            Deadband is a range of angles that should be dropped.
            The proper format is a list of 2 integers in which the first value is the start of the deadband and the second value is the end. \n
            sensorThetaOffset will be added to the point angle before the deadband is calculated however it is not permanently applied to the point. This should instead be done by the translation argument to addVal
        """
        self.points:dict[float, lidarMeasurement]={}
        self.deadband:list[float]=deadband
        self.deadbandRaps:bool= deadband != None and deadband[0]>deadband[1] # type: ignore
        self.sensorThetaOffset=sensorThetaOffset
        self.hostLidar:"Lidar.Lidar"=hostLidar
        self.isFinished=False
        self.mapID=mapID
        self.len=0
        self.startTime:float=None # type: ignore
        self.endTime:float=None # type: ignore
        

    def __array__(self):
        return self.getPoints()
    
    def __getstate__(self):
        
        state = self.__dict__.copy()
        
        if state.get('hostLidar'):
            del state['hostLidar']
        
        return state
        
    
    def __setstart__(self, state:dict[str,Any]):
        self.__dict__.update(state)
        self.hostLidar=None # type: ignore


    def addVal(self, point:"lidarMeasurement", translation: translation)->None:
        """
            <h2>Adds a value to the lidars point list.</h2> 
            If the imputed point shares an angle with a point already recorded by the lidar the old point will be replaced.
            The point will automatically be discarded if it is within the deadband or if its quality or distance values are both 0. 
            If the point has startFlag as true the point will not be logged and instead the lidar will be told to start a new map.
            The translation argument will be added to the point before the point is added to the map.

        """
        if self.startTime==None: # type: ignore
            self.startTime=point.timeStamp

        if point.start_flag:
            self.endTime:float=point.timeStamp
            self.hostLidar._mapIsDone() # type: ignore
            return

        if point.quality==0 or point.distance==0:
            return
        
        if self.deadband:
            if self.deadbandRaps:
                if (point.angle+self.sensorThetaOffset)%360>self.deadband[0] or (point.angle+self.sensorThetaOffset)%360<self.deadband[1]:
                    return
            else:
                if (point.angle+self.sensorThetaOffset)%360>self.deadband[0] and (point.angle+self.sensorThetaOffset)%360<self.deadband[1]:
                    return

        if translation !=None: # type: ignore
            translation.applyTranslation(point)

        # if printFlag:
        #     print("valHasBeenAdded", point)

             

        
        self.len+=1
        self.points[point.angle]=point

        
    

    def fetchPointAtClosestAngle(self, angle:float, tolerance:int=360)->"lidarMeasurement":
        """
            <h2>Returns the point in the map with the closest angle to the imputed angle.</h2> 
            If tolerance is set and the distance between the closest points angle and the requested angle is greater than tolerance None will be returned.
            This search will not loop around 360. Additionally None will be returned if the map is empty 
        """
        if len(self.points)==0:
            return None # type: ignore
        
        foundPoint= self.points[min([key for key, value in self.points.items()], key=lambda value: abs(value - angle))] # type: ignore
        if (foundPoint.angle-angle)>tolerance:
            return None  # type: ignore
        return foundPoint
    
    def getDistanceBetweenClosestAngle(self, angle:float)->float:
        """
            <h2>Fetches the difference between the specified angle and the closest angle within the map. </h2>
            If the map is empty None will be returned
        """
        if len(self.points)==0:
            return None # type: ignore
        return abs(self.fetchPointAtClosestAngle(angle).angle-angle)
    
    def getPoints(self)->list["lidarMeasurement"]:
        """<h2>Returns a list of all the points within the map</h2>"""
        return list(self.points.values())
    

    def printMap(self)->None:
        """<h2>Prints all points in the map in order of angle from least to greatest.</h2>"""
        print("current map:")
        #self.thisFuncDoesNothing()
        for point in self.getPoints():
            print(point)
            pass

    def getRange(self)->float:
        """<h2>Returns the range of angles from the points within the map.</h2>"""
        if len(self.points)==0:
            return 0
        return abs(self.fetchPointAtClosestAngle(0).angle - self.fetchPointAtClosestAngle(360).angle)
    
    def getPeriod(self)->float:
        """
            <h2>Returns the time period of measurements in this lidar map</h2>
            If the map is finished this function will return the time in seconds between the first value and last last value in the map.
            If the map is still in progress the function will instead return the time period between the first reading and the current timestamp. 
        """
        if self.startTime and self.endTime:
            return self.endTime-self.startTime
        
        if self.startTime:
            return time()-self.startTime
        
        # print(self.startTime, self.endTime)
        return 0
    

    def getHz(self)->float:
        """
            <h2>Returns the estimated amount of scans completed in one second (or the Hz of the lidar). </h2>
            WARNING this value is not the actual Hz of the lidar. Instead the functions estimates the Hz based on how long this scan took. 
            While this reading is mostly accurate there is some inaccuracy.
        """
        if self.startTime and self.endTime:
            return 1/self.getPeriod()
        return 0


    def setOffset(self, theta:float)->None:
        """
            <h2>Sets the internal offset used by this map.</h2>
            Theta will be added to the point angle before the deadband is calculated however it is not permanently applied to the point. This should instead be done by the translation argument to addVal
            WARNING this function throws a error if theta is greater than 360 or less than 0
        """
        if theta>=0 and theta<360:
            self.sensorThetaOffset=theta
        
        else:
            raise ValueError("attempted to set a sensor offset that is not a degree value: ", theta)
        
    def setDeadband(self, deadband:list[float])->None:
        """
            <h2> Sets the deadband used by this map. </h2>
            Deadband is a range of angles that should be dropped. The proper format is a list of 2 integers in which the first value is the start of the deadband and the second value is the end
            Any set sensor offset will be added to the point angle before the deadband is calculated.
        """
        self.deadband=deadband
        self.deadbandRaps= deadband != None and deadband[0]>deadband[1] # type: ignore