import numpy as np
import pandas as pd
import scipy.interpolate as interpolate
import scipy.misc
import scipy.integrate as integrate
from scipy.spatial.transform import Rotation
import math

class transform_frame:
    def __init__(self,offset):
        self.offset=np.array([offset[0],offset[1],offset[2]],dtype=float)
        self.transform=np.array([0,0,0],dtype=float)

    def transformBO(self,vector,panga):
        self.vector=np.array([vector[0],vector[1],vector[2]],dtype=float)
        self.panga=panga
        self.transform[0]=(self.vector[0]*( (math.cos(self.panga[1]+self.offset[1])*math.cos(self.panga[2]+self.offset[2])                                                                                                                       )))-(self.vector[1]*( (math.cos(self.panga[1]+self.offset[1])*math.sin(self.panga[2]+self.offset[2])                                       )                                                                                ))+(self.vector[2]*math.sin(self.panga[1]+self.offset[1]                                       ))
        self.transform[1]=(self.vector[0]*( (math.sin(self.panga[0]+self.offset[0])*math.sin(self.panga[1]+self.offset[1])*math.cos(self.panga[2]+self.offset[2]))+(math.cos(self.panga[0]+self.offset[0])*math.sin(self.panga[2]+self.offset[2]))))+(self.vector[1]*(-(math.sin(self.panga[0]+self.offset[0])*math.sin(self.panga[1]+self.offset[1])*math.sin(self.panga[2]+self.offset[2]))+(math.cos(self.panga[0]+self.offset[0])*math.cos(self.panga[2]+self.offset[2]))))-(self.vector[2]*math.sin(self.panga[0]+self.offset[0])*math.cos(self.panga[1]+self.offset[1]))
        self.transform[2]=(self.vector[0]*(-(math.cos(self.panga[0]+self.offset[0])*math.sin(self.panga[1]+self.offset[1])*math.cos(self.panga[2]+self.offset[2]))+(math.sin(self.panga[0]+self.offset[0])*math.sin(self.panga[2]+self.offset[2]))))+(self.vector[1]*( (math.cos(self.panga[0]+self.offset[0])*math.sin(self.panga[1]+self.offset[1])*math.sin(self.panga[2]+self.offset[2]))+(math.sin(self.panga[0]+self.offset[0])*math.cos(self.panga[2]+self.offset[2]))))+(self.vector[2]*math.cos(self.panga[0]+self.offset[0])*math.cos(self.panga[1]+self.offset[1]))
##        self.transform=Rotation.from_euler('zyx',[self.panga[2],self.panga[1],self.panga[0]],degrees=False).apply(vector)
        return np.array([self.transform[0],self.transform[1],self.transform[2]],dtype=float)

    def transformOB(self,vector,panga):
        self.vector=np.array([vector[0],vector[1],vector[2]],dtype=float)
        self.panga=panga
        self.transform[0]=(self.vector[0]*( math.cos(self.panga[2]+self.offset[2])*math.cos(self.panga[1]+self.offset[1]))+self.vector[1]*( math.sin(self.panga[2]+self.offset[2])*math.cos(self.panga[0]+self.offset[0])+math.cos(self.panga[2]+self.offset[2])*math.sin(self.panga[1]+self.offset[1])*math.sin(self.panga[0]+self.offset[0]))+self.vector[2]*( math.sin(self.panga[2]+self.offset[2])*math.sin(self.panga[0]+self.offset[0])-math.cos(self.panga[2]+self.offset[2])*math.sin(self.panga[1]+self.offset[1])*math.cos(self.panga[0]+self.offset[0]) ))
        self.transform[1]=(self.vector[0]*(-math.sin(self.panga[2]+self.offset[2])*math.cos(self.panga[1]+self.offset[1]))+self.vector[1]*( math.cos(self.panga[2]+self.offset[2])*math.cos(self.panga[0]+self.offset[0])-math.sin(self.panga[2]+self.offset[2])*math.sin(self.panga[1]+self.offset[1])*math.sin(self.panga[0]+self.offset[0]))+self.vector[2]*( math.cos(self.panga[2]+self.offset[2])*math.sin(self.panga[0]+self.offset[0])+math.sin(self.panga[2]+self.offset[2])*math.sin(self.panga[1]+self.offset[1])*math.cos(self.panga[0]+self.offset[0]) ))
        self.transform[2]=(self.vector[0]*( math.sin(self.panga[1]+self.offset[1])                                       )+self.vector[1]*(-math.cos(self.panga[1]+self.offset[1])*math.sin(self.panga[0]+self.offset[0])                                                                                                                     )+self.vector[2]*( math.cos(self.panga[1]+self.offset[1])*math.cos(self.panga[0]+self.offset[0])                                                                                                                      ))
##        self.transform=Rotation.from_euler('xyz',[-self.panga[0],-self.panga[1],-self.panga[2]],degrees=False).apply(vector)
        return np.array([self.transform[0],self.transform[1],self.transform[2]],dtype=float)
