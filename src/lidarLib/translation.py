from lidarLib.util import polarToCart, cartToPolar
from wpimath.geometry import Pose2d
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from lidarLib.lidarMeasurement import lidarMeasurement

class translation:
    """
        <h2>Class to translate lidarMeasurements from one 0,0 to another</h2>
        This is used to convert coordinates from robot centric or lidar centric to a world centric system. 
    """
    def __init__(self, r:float, theta:float, rotation:float):
        """
            <h2>Creates a translation using polar coordinates.</h2>
            If cartesian coordinates are preferred use the translation from Cart helper method.
        """
        self.r=r
        self.theta=theta
        self.rotation=rotation
        self.x, self.y = polarToCart(self.r, self.theta)

    def __str__(self):
        return "Translation with coordinates: x = " + str(self.x) + ", y = " + str(self.y) + ", rotation = " + str(self.rotation) + ", r = " + str(self.r) + ", theta = " + str(self.theta)


    @classmethod 
    def default(cls)->"translation":
        """
            <h2>Creates a translation that will translate a point to itself.</h2>
            Good for values where a translation object is needed but an actual translation is not.
        """
        return cls(0,0,0)
    
    @classmethod
    def fromCart(cls, x:float, y:float, rotation:float)->"translation":
        """<h2>Creates a translation from cartesian coordinates</h2>"""
        r, theta = cartToPolar(x, y)
        return cls(r, theta, rotation)
    
    @classmethod
    def fromPose2d(cls, pose:Pose2d)->"translation":
        """<h2>Creates a translation object from WPILibs pose2d objects</h2>"""
        return cls.fromCart(pose.X(), pose.Y(), pose.rotation().degrees())

        
        


    def applyTranslation(self, lidarPoint:"lidarMeasurement")->None:
        """<h2>Applies a translation to the given point, the translation will be applied in place.</h2>"""
        lidarPoint.angle=(lidarPoint.angle-self.rotation)%360
        
        lidarPoint.distance, lidarPoint.angle = cartToPolar(lidarPoint.getX()-self.x, lidarPoint.getY()-self.y)


    def combineTranslation(self, addTranslation:"translation")->"translation":
        """
            <h2>Combines two translations.</h2>
            The composite translation will be returned.
        """
        return translation.fromCart(self.x+addTranslation.x, self.y+addTranslation.y, (self.rotation+addTranslation.rotation)%360)


