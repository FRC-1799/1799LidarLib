import time

from lidarLib.util import polarToX, polarToY, polarToCart
from lidarLib.lidarProtocol import LidarMeasurementHQ

class lidarMeasurement:
    """
        <h2>Class to handle a single lidar measurement.</h2>
        Coordinates can be gotten in polar with the get distance and get angle methods or in cartesian form using the getX, getY, and getCart methods.
        """
    def __init__(self, rawBytes:bytes=None, measurementHQ:LidarMeasurementHQ=None): # type: ignore
        """<h2>Initializes a lidar measurement using a package from the lidar or a measurementHQ from a lidar.</h2>"""
        self.timeStamp=time.time()

        if rawBytes is not None: # type: ignore
            self.start_flag = bool(rawBytes[0] & 0x1)
            self.quality:int = rawBytes[0] >> 2
            self.angle:float = ((rawBytes[1] >> 1) + (rawBytes[2] << 7)) / 64.0
            self.distance:float = ((rawBytes[3] + (rawBytes[4] << 8)) / 4.0)/1000
            
        elif measurementHQ is not None: # type: ignore
            self.start_flag=True if measurementHQ.startFlag==0x1 else False
            self.quality=measurementHQ.quality
            self.angle = ((measurementHQ.angleZQ14*90)>>8)/64.0
            self.distance= ((measurementHQ.distMmQ2)/4.0)/1000

    @classmethod
    def default(cls, startFlag:bool, quality:int, angle:float, distance:float, isInMM:bool=True)->"lidarMeasurement":
        """
            <h2>Initializes a lidarMeasurement using the values specified.</h2> 
            This method is only intended for debugging purposes. For creating measurements from a real lidar use the standard constructor.
        """
        new = cls()
        new.start_flag=startFlag
        new.quality=quality
        new.angle=angle
        if (isInMM):
            new.distance=distance/1000
        else:
            new.distance=distance

            
        return new
                
    def __str__(self):
        data = { # type: ignore
            "start_flag" : self.start_flag,
            "quality" : self.quality,
            "angle" : self.angle,
            "distance" : self.distance,
            "timestamp" : self.timeStamp
        }
        return str(data) # type: ignore

    def getAngle(self)->float:
        """<h2>Returns the measurements angle(or omega) as a float</h2>"""
        return self.angle

    def getDistance(self)->float:
        """<h2>Returns the measurements distance(or r) as a float</h2>"""
        return self.distance

    def getX(self)->float:
        """<h2>Returns the X of the measurement.</h2>"""
        return polarToX(self.distance, self.angle)

    def getY(self)->float:
        """<h2>Returns the Y of the measurement.</h2>"""
        return polarToY(self.distance, self.angle)
    
    def getCart(self)->tuple[float, float]:
        """<h2>Returns the x and y of the measurement as a tuple.</h2>"""
        return polarToCart(self.distance, self.angle)
