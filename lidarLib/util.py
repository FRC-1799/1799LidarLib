import math

def polarToCart(r:float,theta:float)->tuple:
    """translates polar coordinates to cartesian coordinates. returns a tuple with values x, y"""
    return polarToX(r, theta), polarToY(r, theta)

def cartToPolar(x:float, y:float)->tuple:
    """translates cartesian coordinates to polar ones. returns a tuple with values r, theta"""
    deg=math.degrees(math.atan2(y,x))
    if deg<0:
        deg+=360
    return math.sqrt(x**2+ y**2), deg

def polarToX(r:float, theta:float)->float:
    """determines the x of a point based off polar coordinates. returns x as a float"""
    return r*math.cos(math.radians(theta))
def polarToY(r:float, theta:float)->float:
    """determines the y of a point based off polar coordinates. returns y as a float"""
    return r*math.sin(math.radians(theta))