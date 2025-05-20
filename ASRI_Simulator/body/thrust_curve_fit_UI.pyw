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

def ThrustCurveFitUI(ThrustCurveFitDegree,Directory):
    ThrustCurveExist=os.path.isfile(r'{}\Outputs\thrust_curve_fit_coefficients.csv'.format(Directory))
    if ThrustCurveExist==True:
        os.remove(r'{}\Outputs\thrust_curve_fit_coefficients.xlsx'.format(Directory))

    ThrustDataframe=pd.read_excel(r'{}\inputs\thrust_curve.xlsx'.format(Directory),header=0)
    ThrustDataframe.columns=['Time (s)','Momentum Thrust (N)','Exit Pressure (Pa)']

    x=ThrustDataframe['Time (s)']
    y=ThrustDataframe['Momentum Thrust (N)']

    poly=np.polynomial.Polynomial.fit(x, y, ThrustCurveFitDegree, window=[0, 10])
    cheb=np.polynomial.Chebyshev.fit(x, y, ThrustCurveFitDegree, window=[0, 10])
    herm=np.polynomial.Hermite.fit(x, y, ThrustCurveFitDegree, window=[0, 10])
    lagu=np.polynomial.Laguerre.fit(x, y, ThrustCurveFitDegree, window=[0, 10])
    lege=np.polynomial.Legendre.fit(x, y, ThrustCurveFitDegree, window=[0, 10])

    Poly=poly.convert()
    Cheb=cheb.convert()
    Herm=herm.convert()
    Lagu=lagu.convert()
    Lege=lege.convert()
    
    print("Polynomial:" ,Poly)
    print("Chebyshev:"  ,Cheb)
    print("Hermite:"    ,Herm)
    print("Laguerre:"   ,Lagu)
    print("Legendre:"   ,Lege)
    arg=1
    print('using time=1:',"   Polynomial:",round(Poly(arg),6),"   Chebyshev:",round(Cheb(arg),6),"   Hermite:",round(Herm(arg),6),"   Laguerre:",round(Lagu(arg),6),"   Legendre:",round(Lege(arg),6))
    arg=2
    print('using time=2:',"   Polynomial:",round(Poly(arg),6),"   Chebyshev:",round(Cheb(arg),6),"   Hermite:",round(Herm(arg),6),"   Laguerre:",round(Lagu(arg),6),"   Legendre:",round(Lege(arg),6))
    arg=3
    print('using time=3:',"   Polynomial:",round(Poly(arg),6),"   Chebyshev:",round(Cheb(arg),6),"   Hermite:",round(Herm(arg),6),"   Laguerre:",round(Lagu(arg),6),"   Legendre:",round(Lege(arg),6))
    pd.set_option('display.max_columns', None)
    
    ThrustCurveFitHeadings=["degree 0","degree 1","degree 2","degree 3","degree 4","degree 5","et cetera..."]
    ThrustCurveFitNotate=Cheb
    with open(r'{}\Outputs\thrust_curve_fit_coefficients.csv'.format(Directory), 'a', newline='') as ThrustCurveFitWrite:
        writer = csv.writer(ThrustCurveFitWrite)
        writer.writerow(ThrustCurveFitHeadings)
        writer.writerow(ThrustCurveFitNotate)
    ThrustCurveFitWrite.close()
    print('Momentum thrust approximation curve polynomial coefficients successfully exported.')
    return
