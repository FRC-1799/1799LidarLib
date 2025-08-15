import math

def polarToCart(r:float,theta:float)->tuple[float, float]:
    """
        <h2>Translates polar coordinates to cartesian coordinates.</h2> 
        Returns a tuple with values x, y.
    """
    return polarToX(r, theta), polarToY(r, theta)

def cartToPolar(x:float, y:float)->tuple[float, float]:
    """
        <h2>Translates cartesian coordinates to polar ones. </h2>
        Returns a tuple with values r, theta.
    """
    deg=math.degrees(math.atan2(y,x))
    if deg<0:
        deg+=360
    return math.sqrt(x**2+ y**2), deg

def polarToX(r:float, theta:float)->float:
    """
        <h2>Determines the x of a point based off polar coordinates.</h2>
        Returns x as a float.
    """
    return r*math.cos(math.radians(theta))
def polarToY(r:float, theta:float)->float:
    """
        <h2>Determines the y of a point based off polar coordinates.</h2>
        Returns y as a float.
    """
    return r*math.sin(math.radians(theta))