import time
from lidarLib.util import polarToX, polarToY, polarToCart

class lidarMeasurement:
    """Class to handle a single lidar measurement, coordinates are normally stored in polar but may be gotten in cartesian form using the getX, getY, and getCat methods"""
    def __init__(self, raw_bytes:bytes=None, measurement_hq=None): # type: ignore
        """
            initializes a lidar measurement using a package from the lidar
            while measurement_hq objects are accepted by this function the class is currently deprecated and should not be used
        """
        self.timeStamp=time.time()

        if raw_bytes is not None: # type: ignore
            self.start_flag = bool(raw_bytes[0] & 0x1)
            self.quality:int = raw_bytes[0] >> 2
            self.angle:float = ((raw_bytes[1] >> 1) + (raw_bytes[2] << 7)) / 64.0
            self.distance:float = ((raw_bytes[3] + (raw_bytes[4] << 8)) / 4.0)/1000
            
        elif measurement_hq is not None:
            self.start_flag=True if measurement_hq.start_flag==0x1 else False
            self.quality=measurement_hq.quality
            self.angle = ((measurement_hq.angle_z_q14*90)>>8)/64.0
            self.distance= ((measurement_hq.dist_mm_q2)/4.0)/1000

    @classmethod
    def default(cls, start_flag:bool, quality:int, angle:float, distance:float, isInMM:bool=True)->"lidarMeasurement":
        """initializes a lidarMeasurement using the values specified. this method is only intended for debugging purposes. For creating measurements from a lidar use the standard constructor"""
        new = cls()
        new.start_flag=start_flag
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
        """returns the measurements angle as a float"""
        return self.angle

    def getDistance(self)->float:
        """returns the measurements distance as a float"""
        return self.distance

    def getX(self)->float:
        """returns the X of the measurement. This value is not directly stored and is instead calculated whenever the function is called"""
        return polarToX(self.distance, self.angle)

    def getY(self)->float:
        """returns the Y of the measurement. This value is not directly stored and is instead calculated whenever the function is called"""
        return polarToY(self.distance, self.angle)
    
    def getCart(self)->tuple[float, float]:
        """returns the x and y of the measurement as a tuple. This value is not directly stored and is instead calculated whenever the function is called """
        return polarToCart(self.distance, self.angle)
