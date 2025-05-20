import math
from .constants import *

def EarthRadiusFunction(latitude):
    radi=0
    latitude=abs(latitude)
    if latitude==math.pi*0.5:
        radi=6356752.0
    elif latitude<math.pi*0.5:
        radi=math.sqrt(((1.0+(math.tan(latitude))**2)*(eartha*earthb)**2)/((earthb)**2+(eartha*math.tan(latitude))**2))  
    return(radi)

def EllipseGravity(height,latitude):
    latitude=abs(latitude)
    g0=-9.7803*(1.0+(0.0053*(math.sin(latitude))**2)-(0.0000058*(math.sin(2.0*latitude))**2))
    return(g0*((EarthRadiusFunction(latitude))**2/(EarthRadiusFunction(latitude)+height)**2))
