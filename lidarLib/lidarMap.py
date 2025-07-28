from lidarLib import lidarMeasurement
import lidarLib.Lidar
from lidarLib.translation import translation

class lidarMap:
    """
        Class for handling a full 360 scan of lidar data
    """
    def __init__(self, hostLidar:"Lidar.Lidar", mapID=0, deadband=None, sensorThetaOffset=0):
        """
            Initializes a lidarMap scan
            host lidar should be set to the lidar object responsible for populating the map. 
            This value is only used if the point map is still being updated and so is unnecessary if the map is done being filled.
            MapId is a id for the map and should be unique. however if the map does not need to be identified the field can be left blank
            Deadband is a range of angles that should be dropped. the proper format is a list of 2 integers in which the first value is the start of the deadband and the second value is the end
            sensorThetaOffset will be added to the point angle before the deadband is calculated however it is not permanently applied to the point. This should instead be done by the translation argument to addVal
        """
        self.points={}
        self.deadband=deadband
        self.deadbandRaps= deadband != None and deadband[0]>deadband[1]
        self.sensorThetaOffset=sensorThetaOffset
        self.hostLidar=hostLidar
        self.isFinished=False
        self.mapID=mapID
        self.len=0
        self.startTime=None
        self.endTime=None
        

    def __array__(self):
        return self.getPoints()
    
    def __getstate__(self):
        
        state = self.__dict__.copy()
        
        if state.get('hostLidar'):
            del state['hostLidar']
        
        return state
        
    
    def __setstart__(self, state):
        self.__dict__.update(state)
        self.hostLidar=None


    def addVal(self, point:lidarMeasurement, translation: translation, printFlag=False)->None:
        """
            INTERNAL FUNCTION, NOT FOR OUTSIDE USE
            Adds a value to the lidars point list. if the imputed point shares an angle with a point already recorded by the lidar the old point will be replaced
            The point will automatically be discarded if it is within the deadband or if its quality or distance values are 0. 
            If the point has startFlag as true the point will not be logged and instead the lidar will be told to start a new map.
            The translation argument will be added to the point before the point is added to the map
            if printFlag is set to true information about the point will also be printed after all translation and validity checks. This means that invalid points will not be printed

        """
        if self.startTime==None:
            self.startTime=point.timeStamp

        if point.start_flag:
            self.endTime=point.timeStamp
            self.hostLidar._mapIsDone()
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

        if translation !=None:
            translation.applyTranslation(point)

        # if printFlag:
        #     print("valHasBeenAdded", point)

             

        
        self.len+=1
        self.points[point.angle]=point

        
    

    def fetchPointAtClosestAngle(self, angle:float, tolerance=360)->lidarMeasurement:
        """
            Returns the point in the map with the closest angle to the imputed angle. This search will not loop around 360.
            If tolerance is set and the distance between the closest points angle and the requested angle is greater than tolerance None will be returned
            Additionally None will be returned if the map is empty 
        """
        if len(self.points)==0:
            return None
        foundPoint= self.points[min([key for key, value in self.points.items()], key=lambda value: abs(value - angle))]
        if (foundPoint.angle-angle)>tolerance:
            return 0
        return foundPoint
    
    def getDistanceBetweenClosestAngle(self, angle:float)->float:
        """
            fetches the difference between the specified angle and the closest angle within the map. 
            If the map is empty None will be returned
        """
        if len(self.points)==0:
            return None
        return abs(self.fetchPointAtClosestAngle(angle).angle-angle)
    
    def getPoints(self)->list[lidarMeasurement.lidarMeasurement]:
        """Returns a list of all the points within the map"""
        return list(self.points.values())
    

    def printMap(self)->None:
        """prints the map"""
        print("current map:")
        #self.thisFuncDoesNothing()
        for point in self.getPoints():
            print(point)
            pass

    def getRange(self)->float:
        """Returns the range of angles from the points within the map. aka the angle closest to 360 -  the angle closest to 0 """
        if len(self.points)==0:
            return 0
        return abs(self.fetchPointAtClosestAngle(0).angle - self.fetchPointAtClosestAngle(180).angle)*2
    
    def getPeriod(self)->float:
        """returns the time in seconds between the first value and the value with a true start_flag"""
        if self.startTime and self.endTime:
            return self.endTime-self.startTime
        print(self.startTime, self.endTime)
        return 0
    

    def getHz(self)->float:
        """
            Returns the estimated Hz of the scanning based off how long this scan took.
            WARNING this value may be inaccurate and should not be relied on. it estimates Hz based on how many of this scan could be run in a second however this is not fully accurate and should be used as such
        """
        if self.startTime and self.endTime:
            return 1/self.getPeriod()
        return 0


    def setOffset(self, theta:float)->None:
        """
            Sets the internal offset used by this map.
            Theta will be added to the point angle before the deadband is calculated however it is not permanently applied to the point. This should instead be done by the translation argument to addVal
            WARNING this function throws a error if theta is greater than 360 or less than 0
        """
        if theta>=0 and theta<360:
            self.sensorThetaOffset=theta
        
        else:
            raise ValueError("attempted to set a sensor offset that is not a degree value: ", theta)
        
    def setDeadband(self, deadband:list)->None:
        """
            Sets the deadband used by this map
            Deadband is a range of angles that should be dropped. The proper format is a list of 2 integers in which the first value is the start of the deadband and the second value is the end
            Any set sensor offset will be added to the point angle before the deadband is calculated.
        """
        self.deadband=deadband
        self.deadbandRaps= deadband != None and deadband[0]>deadband[1]