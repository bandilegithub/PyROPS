import pandas as pd
import time
import math
import csv

Gg=6.67428e-11
earthmass=5.9736e24
earthradi=6370000.0
gr0=9.81
omgt=0.00007292
gamma=1.4
Rair=287.0
ThA=-0.0065
ThB=0.0000
ThC=0.001
ThD=0.0028
ThE=0.0000
ThF=-0.0028
ThG=-0.0020

P0=101325
T0=288.16
rho0=1.225
  
P11=22631.7
T11=216.66
rho11=0.3639
  
P20=5474.72
T20=216.66
rho20=0.08803
  
P32=867.98
T32=228.66
rho32=0.01322

P47=110.91
T47=270.65
rho47=0

P51=66.939
T51=270.65
rho51=0

P71=3.9564
T71=214.65
rho71=0

P84=0.3734
T84=186.95
rho84=0

Gg=6.67428e-11
earthmass=5.9736e24
earthradi=6370000.0
earposx=0
earposy=0
earposz=6370000.0

pearax=earposx
pearay=earposy
pearaz=earposz

global height
height=math.sqrt(earposx*earposx+earposy*earposy+earposz*earposz)-earthradi
omgt=7.292e-5

"""Atmospheric Temperature:"""
global Temp
Temp=0
"""Atmospheric Pressure:"""
global Pres
Pres=0
"""Atmospheric Density:"""
global Dens
Dens=0

while height<=84000:
    """Gravitational acceleration:"""
    gravity = (Gg*earthmass)/(earthradi+height)**2
    if height<=11000:
        Temp=T0+(ThA*height)
        Pres=P0*((T0+(ThA*height))/T0)**(-gravity/(Rair*ThA))
        Dens=rho0*((T0+(ThA*height))/T0)**(-gravity/(Rair*ThA)-1)
    if height >11000 and height <=20000:
        Temp=T11
        Pres=P11*math.e**((-gravity*(height-11000))/(Rair*T11))
        Dens=rho11*math.e**((-gravity*(height-11000))/(Rair*T11))
    if height > 20000 and height <=32000:
        Temp=T20+(ThC*(height-20000))
        Pres=P20*((1+(ThC*(height-20000))/T20)**(-gravity/(Rair*ThC)))
        Dens=rho20*((1+((ThC*(height-20000))/T20))**(-gravity/(Rair*ThC)-1))
    if height > 32000 and height <=47000:
        Temp=T32+(ThD*(height-32000))
        Pres=P32*((1+(ThD*(height-32000))/T32)**(-gravity/(Rair*ThD)))
        Dens=rho32*(1+((ThD*(height-32000))/T32))**(-gravity/(Rair*ThD)-1)
    if height > 47000 and height <=51000:
        Temp=T47
        Pres=P47*math.e**((-gr0*(height-47000))/(Rair*T47))
        Dens=rho47*math.e**((-gr0*(height-47000))/(Rair*T47))
    if height > 51000 and height <=71000:
        Temp=T51+(ThF*(height-51000))
        Pres=P51*((1+(ThF*(height-51000))/T51)**(-gravity/(Rair*ThF)))
        Dens=rho51*((1+((ThF*(height-51000))/T51))**(-gravity/(Rair*ThF)-1))
    if height > 71000 and height <=84000:
        Temp=T71+(ThG*(height-71000))
        Pres=P71*((1+(ThG*(height-71000))/T71)**(-gravity/(Rair*ThG)))
        Dens=rho71*((1+((ThG*(height-71000))/T71))**(-gravity/(Rair*ThG)-1))

    atmosphere_row =[height,Temp,Pres,Dens]
    with open(r'atmosphere_data.csv','a', newline='') as atmosphere_file:
        writer = csv.writer(atmosphere_file)
        writer.writerow(atmosphere_row)
    
    """print (height,Temp,Pres,Dens)"""
    height+=100
