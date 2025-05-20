import os
import math
import csv
import pandas as pd
import numpy as np
"""Sum of least squares fit:"""
from numpy.polynomial import polynomial
"""z-series:"""
from numpy.polynomial import chebyshev
from numpy.polynomial import hermite
from numpy.polynomial import laguerre
from numpy.polynomial import legendre

class thrust_curve:
    def __init__(self,ThrustCurveFitDegree,Directory):
        self.ThrustCurveFitDegree=ThrustCurveFitDegree
        self.Directory=Directory
    
    def fit(self,time):
        ThrustDataframe=pd.read_excel(r'{}\inputs\thrust_curve.xlsx'.format(self.Directory),header=0)
        ThrustDataframe.columns=['Time (s)','Momentum Thrust (N)','Exit Pressure (Pa)']

        x=ThrustDataframe.iloc[:,0]
        y=ThrustDataframe.iloc[:,1]

        poly=np.polynomial.Polynomial.fit(x, y, self.ThrustCurveFitDegree, window=[0, 10])
        cheb=np.polynomial.Chebyshev.fit(x, y, self.ThrustCurveFitDegree, window=[0, 10])
        herm=np.polynomial.Hermite.fit(x, y, self.ThrustCurveFitDegree, window=[0, 10])
        lagu=np.polynomial.Laguerre.fit(x, y, self.ThrustCurveFitDegree, window=[0, 10])
        lege=np.polynomial.Legendre.fit(x, y, self.ThrustCurveFitDegree, window=[0, 10])

        Poly=poly.convert()
        Cheb=cheb.convert()
        Herm=herm.convert()
        Lagu=lagu.convert()
        Lege=lege.convert()

        return Cheb(time)
