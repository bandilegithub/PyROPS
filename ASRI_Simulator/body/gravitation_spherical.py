import math
from .constants import *

def SphericalGravity(alti):
    AccelerationGravity=(EarthGravitationalConstant*EarthMass)/((EarthRadius+alti)**2)
    return AccelerationGravity
