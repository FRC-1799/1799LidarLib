import cmath
from lidarLib import lidarMeasurement
from lidarLib.util import polarToCart, cartToPolar
from wpimath.geometry import Pose2d
class translation:
    """class to translate lidarMeasurements from one 0,0 to another"""
    def __init__(self, r:float, theta:float, rotation:float):
        """creates a translation using polar coordinates. if cartesian coordinates are preferred use the translation from Cart helper method"""
        self.r=r
        self.theta=theta
        self.rotation=rotation
        self.x, self.y = polarToCart(self.r, self.theta)

    def __str__(self):
        return "Translation with coordinates: x = " + str(self.x) + ", y = " + str(self.y) + ", rotation = " + str(self.rotation) + ", r = " + str(self.r) + ", theta = " + str(self.theta)


    @classmethod 
    def default(self)->"translation":
        """creates a translation that will translate a point from that point back to itself. good for values where a translation object is needed but a actual translation is not"""
        return self(0,0,0)
    
    @classmethod
    def fromCart(self, x:float, y:float, rotation:float)->"translation":
        """creates a translation from cartesian coordinates"""
        r, theta = cartToPolar(x, y)
        return self(r, theta, rotation)
    
    @classmethod
    def fromPose2d(self, pose:Pose2d)->"translation":
        return self.fromCart(pose.X(), pose.Y(), pose.rotation().degrees())

        
        


    def applyTranslation(self, lidarPoint:lidarMeasurement)->None:
        """Applies a translation to the given point, the translation will be applied in place"""
        lidarPoint.angle=(lidarPoint.angle-self.rotation)%360
        
        lidarPoint.distance, lidarPoint.angle = cartToPolar(lidarPoint.getX()-self.x, lidarPoint.getY()-self.y)


    def combineTranslation(self, addTranslation:"translation")->"translation":
        """Combines to translations. the composite translation will be returned"""
        return translation.fromCart(self.x+addTranslation.x, self.y+addTranslation.y, (self.rotation+addTranslation.rotation)%360)


