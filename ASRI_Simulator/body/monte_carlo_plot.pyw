import os
import webbrowser
from tkinter import *
from tkinter import ttk
from tkinter import filedialog
from tkinter import Entry
import tkinter as tk
import tkinter.messagebox
from PIL import Image,ImageTk
import math
import time
import csv
import sys
import colorsys
import subprocess
import random as random
import pandas as pd
import numpy as np
from numpy import array
import statistics
import scipy.ndimage
import scipy.stats as st
from scipy import stats
from scipy.stats import gaussian_kde
from scipy.stats import norm
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
import matplotlib.transforms as transforms
from matplotlib.pyplot import figure
from matplotlib.patches import Ellipse

def MonteCarloPlot(PlotScalingFactor,LocationOffset,BoundaryThickness,NominalSplashdown,CampaignSplashdown,RefreshRate,PlotZoom,Directory):
    SettingsPlotList=[PlotScalingFactor,LocationOffset,BoundaryThickness,NominalSplashdown[0],NominalSplashdown[1],CampaignSplashdown[0],CampaignSplashdown[1],RefreshRate]
    df = pd.read_excel(r'{}\Outputs\Monte Carlo Map.xlsx'.format(Directory),header=0)
    MonteCarloPoints=len(df)

    #dfWind = pd.read_excel(r'{}\Outputs\Monte Carlo Map Wind.xlsx'.format(Directory),header=0)
    #MonteCarloPointsWind=len(dfWind)
    
    #df.columns=['East (metres)','North (metres)','Apogee (metres)']
    df.columns=["East (metres)","North (metres)","Apogee (metres)","Simulation completed","Thrust Misalignment (Yawing) (deg):","Thrust Misalignment (Pitching) (deg):","Thrust Magnitude Variation (%):","Wind Magnitude Variation (%):","Wind Direction Variation (deg):","Aerodynamic Drag Coefficient Variation (%):","Aerodynamic Lift Coefficient Variation (%):","Aerodynamic Moment Coefficient Variation (%):","Centre-Of-Pressure Variation (%):","Fin Cant Angle Variation (deg):","Launch Altitude (m):","Launch Elevation (deg):","Launch Azimuth (deg):","Burnout Time (s):"]

    #x=df.loc[0,'East (metres)']
    x=df.loc[:,'East (metres)'].values
    #y=df.loc[0,'North (metres)']
    y=df.loc[:,'North (metres)'].values

    xmin,xmax,ymin,ymax=0.5,PlotZoom,0.5,PlotZoom
    fig, ax = plt.subplots()
    fig.patch.set_facecolor('black')

    OTR_map=plt.imread(r"body\OTR\OTR_map.tif")
    ##    OTR1=plt.imread(r"body\OTR\OTR1.bmp")
    ##    OTR2=plt.imread(r"body\OTR\OTR2.bmp")
    ##    OTR3=plt.imread(r"body\OTR\OTR3.bmp")
    ##    OTR4=plt.imread(r"body\OTR\OTR4.bmp")
    ##    OTR1=plt.imread(r"body\OTR\BL.png")
    ##    OTR2=plt.imread(r"body\OTR\TL.png")
    ##    OTR3=plt.imread(r"body\OTR\TR.png")
    ##    OTR4=plt.imread(r"body\OTR\BR.png")

    x=df.loc[:,'East (metres)'].values
    y=df.loc[:,'North (metres)'].values

    #x7=dfWind.loc[:,'East (metres)'].values
    #y7=dfWind.loc[:,'North (metres)'].values
    
    ax.set_facecolor('black')
    ax.tick_params(colors='white', which='both')
    ax.xaxis.label.set_color('white')
    ax.yaxis.label.set_color('white')

    """Map offset (metres, West):"""
    OTR_map_x1=-17890#-17500
    """Map offset (metres, East):"""
    OTR_map_x2=72110#72500
    """Map offset (metres, South):"""
    OTR_map_y1=-29270#-29000
    """Map offset (metres, North):"""
    OTR_map_y2=30730#31000

    ax.imshow(OTR_map,extent=[OTR_map_x1,OTR_map_x2,OTR_map_y1,OTR_map_y2])
    ax.xaxis.set_label_position('top')
    ax.yaxis.set_label_position('left')
    plt.axis('on')
    plt.xlabel('East (metres)')
    plt.ylabel('North (metres)')

    number_ticks=10

    x_array=np.linspace(-17500,72500,18+1)
    #x_array.round(decimals=3)
    y_array=np.linspace(-29000,31000,12+1)
    #y_array.round(decimals=3)
    plt.xticks(x_array)
    plt.yticks(y_array)

    tick_x_rounded=abs(round(((xmax)/number_ticks),-1))
    #plt.xticks(np.linspace(-PlotZoom, PlotZoom, 11))
    tick_y_rounded=abs(round(((ymax)/number_ticks),-1))
    #plt.yticks(np.linspace(-PlotZoom, PlotZoom, 11))
    ax.tick_params(labeltop=True, labelright=True)

    plot_manager = plt.get_current_fig_manager()
    plot_manager.window.state('zoomed')
    plt.draw()

    def encircle(x,y,ax=None,**kw):
        if not ax: ax=plt.gca()
        p = np.c_[x,y]
        hull = ConvexHull(p)
        poly = plt.Polygon(p[hull.vertices,:], **kw)
        return poly

    Counter=0
    DisplayMap=0
    while DisplayMap<1:
        plt.ion()

        """Change plot settings while code is running"""
        SettingsPlot=pd.read_excel(r'{}\Inputs\SettingsPlot.xlsx'.format(Directory),header=0)

        """nominal simulation splashdown coordinates:"""
        x01=SettingsPlotList[3]
        y01=SettingsPlotList[4]
        
        """OTR Campaign splashdown coordinates:"""
        x0=SettingsPlotList[5]
        y0=SettingsPlotList[6]

        NominalSplashdown=np.array([SettingsPlot.iloc[0,3],SettingsPlot.iloc[0,4]])
        CampaignSplashdown=np.array([SettingsPlot.iloc[0,5],SettingsPlot.iloc[0,6]])

        if NominalSplashdown[0] or NominalSplashdown[1]!=0:
            x01=NominalSplashdown[0]
            y01=NominalSplashdown[1]

        if CampaignSplashdown[0] or CampaignSplashdown[1]!=0:
            x0=CampaignSplashdown[0]
            y0=CampaignSplashdown[1]

        number_ticks=13    
        line=ax.plot([0,x01], [0,y01],c='yellow',linewidth=1,zorder=2)
        plot1=ax.scatter(x0, y0, facecolors='none',edgecolors='lime', marker='o', s=0.0000001,linewidths=2,zorder=4) #s=0.1*((xmax+ymax)/(2*number_ticks))
        plot2=ax.scatter(x01, y01, facecolors='none',c='orange', marker='x', s=0.1*((xmax+ymax)/(2*number_ticks)),linewidths=2,zorder=3)
        plot3=ax.scatter(x, y, c='dodgerblue', marker='x',s=0.1*((xmax+ymax)/(2*number_ticks)),linewidths=1,zorder=1) #c='cornflowerblue' #c='dodgerblue' #c='lime' #dodgerblue
        #plot7=ax.scatter(x7, y7, c='green', marker='x',s=0.1*((xmax+ymax)/(2*number_ticks)),linewidths=1,zorder=1) #c='cornflowerblue' #c='dodgerblue' #c='lime' #dodgerblue
        plt.legend((plot1,plot2,plot3),('OTR splashdown', 'Nominal simulation splashdown', 'Monte Carlo splashdown', 'Average'),loc='center right',ncol=1,fontsize=12)
        #plt.legend((plot1,plot2,plot3,plot7),('OTR splashdown', 'Nominal simulation splashdown', 'Monte Carlo splashdown','Splashdown Wind 12h00', 'Average'),loc='center right',ncol=1,fontsize=12)
        
        RefreshRate=SettingsPlotList[7]
        if SettingsPlot.iloc[0,7]!=0:
            RefreshRate=SettingsPlot.iloc[0,7]
        
        plt.pause(RefreshRate)
        
        plot1.remove()
        plot2.remove()
        plot3.remove()
        #plot7.remove()
        ax.lines[0].remove()

        df = pd.read_excel(r'{}\Outputs\Monte Carlo Map.xlsx'.format(Directory),header=0)

        """If new splashdown point detected, update footprint."""
        if len(df)>MonteCarloPoints:
            x_value=np.array([df.loc[(len(df)-1),'East (metres)']])
            y_value=np.array([df.loc[(len(df)-1),'North (metres)']])

            x=np.concatenate((x,x_value),axis = 0)
            y=np.concatenate((y,y_value),axis = 0)
            plot3.set_offsets(np.c_[x,y])#plot3=ax.scatter(x, y)
            MonteCarloPoints+=1

##        if len(dfWind)>MonteCarloPointsWind:
##            x7_value=np.array([df.loc[(len(dfWind)-1),'East (metres)']])
##            y7_value=np.array([df.loc[(len(dfWind)-1),'North (metres)']])
##
##            x7=np.concatenate((x7,x7_value),axis = 0)
##            y7=np.concatenate((y7,y7_value),axis = 0)
##            plot7.set_offsets(np.c_[x7,y7])#plot3=ax.scatter(x, y)
##            MonteCarloPointsWind+=1

        """Clear boundaries:"""
        if Counter>=1:
            c1.remove()
            c2.remove()

        """Use points to create a symmetric safety envelope."""
        x_mirror=np.array([])
        y_mirror=np.array([])
        counter=0
        m=np.mean(y)/np.mean(x)
        counterlength=len(x)
        while counter<(counterlength):
            """Gradient of line fron launch pad to mean splashdown point coordinates:"""
            x_new=2*((x[counter]+(y[counter])*m)/(1+m**2))-x[counter]
            y_new=2*((x[counter]+(y[counter])*m)/(1+m**2))*m-y[counter]
            xnew=np.array([x_new])
            ynew=np.array([y_new])
            x_mirror=np.concatenate((x_mirror,xnew),axis = 0)
            y_mirror=np.concatenate((y_mirror,ynew),axis = 0)
            counter+=1

        """Setting up the safety envelope boundary."""
        x_boundary=np.concatenate((x,x_mirror),axis = 0)
        y_boundary=np.concatenate((y,y_mirror),axis = 0)
        M=[np.mean(x_boundary),np.mean(y_boundary)]
        Factor1=max(abs(min(x_mirror)),abs(max(x_mirror)))-M[0]
        Factor2=max(abs(min(y_mirror)),abs(max(y_mirror)))-M[1]
        PlotScalingFactor1=max([Factor1/2000,Factor2/2000])
        PlotScalingFactor2=min([Factor1/2000,Factor2/2000])
        LocationOffset=np.linalg.norm(np.array([Factor1,Factor2]))/np.linalg.norm(np.array([M[0],M[1]]))
        PlotScalingFactor=PlotScalingFactor2

        PlotScalingFactor=SettingsPlotList[0]
        if SettingsPlot.iloc[0,0]!=0:
            PlotScalingFactor=SettingsPlot.iloc[0,0]

        LocationOffset=SettingsPlotList[1]
        if SettingsPlot.iloc[0,1]!=0:
            LocationOffset=SettingsPlot.iloc[0,1]

        BoundaryThickness=SettingsPlotList[2]
        if SettingsPlot.iloc[0,2]!=0:
            BoundaryThickness=SettingsPlot.iloc[0,2]

        """Contours:"""
        NumberRuns=len(x_boundary)
        row_number=1
        while row_number<(NumberRuns):
            boundary=BoundaryThickness*PlotScalingFactor
            resolution=16
            while resolution>0:
                row_write=[x_boundary[row_number]+boundary*math.sin(resolution*math.pi/8),y_boundary[row_number]+boundary*math.cos(resolution*math.pi/8)]
                with open(r'body\points.csv', 'a', newline='') as inner_enclosure:
                    writer = csv.writer(inner_enclosure)
                    writer.writerow(row_write)
                resolution-=1
            row_number+=1
        resolution=160
        while resolution>0:
            row_write_last=[boundary*math.sin(resolution*math.pi/80),boundary*math.cos(resolution*math.pi/80)]
            Locationx=np.mean(x)+np.mean(x)*LocationOffset
            Locationy=np.mean(y)+np.mean(y)*LocationOffset
            row_write_lastt=[Locationx+(PlotScalingFactor*(boundary+800-800/3))*math.sin(resolution*math.pi/80),Locationy+(PlotScalingFactor*(boundary+800-800/3))*math.cos(resolution*math.pi/80)]
            with open(r'body\points.csv', 'a', newline='') as outer_enclosure:
                writer = csv.writer(outer_enclosure)
                writer.writerow(row_write_last)
                writer.writerow(row_write_lastt)
            resolution-=1

        row_number=1
        while row_number<(NumberRuns):
            boundary=(BoundaryThickness)*PlotScalingFactor+BoundaryThickness
            resolution=16
            while resolution>0:
                row_write=[x_boundary[row_number]+boundary*math.sin(resolution*math.pi/8),y_boundary[row_number]+boundary*math.cos(resolution*math.pi/8)]
                with open(r'body\points2.csv', 'a', newline='') as inner_enclosure:
                    writer = csv.writer(inner_enclosure)
                    writer.writerow(row_write)
                resolution-=1
            row_number+=1
        resolution=160
        while resolution>0:
            row_write_last=[boundary*math.sin(resolution*math.pi/80),boundary*math.cos(resolution*math.pi/80)]
            row_write_lastt=[Locationx+(PlotScalingFactor*boundary)*math.sin(resolution*math.pi/80),Locationy+(PlotScalingFactor*boundary)*math.cos(resolution*math.pi/80)]
            with open(r'body\points2.csv', 'a', newline='') as outer_enclosure:
                writer = csv.writer(outer_enclosure)
                writer.writerow(row_write_last)
                writer.writerow(row_write_lastt)
            resolution-=1

        """Splashdown point coordinates:"""
        df = pd.read_csv(r'body\points.csv',header='infer')    
        df.columns=['x','y']
        x2=df.loc[:,"x"].values
        y2=df.loc[:,"y"].values
        encircle(x2, y2, ec="k", fc="gold", alpha=0.2)
        c1=ax.add_patch(encircle(x2, y2, ec="k", fc="gold", alpha=0.2))

        df = pd.read_csv(r'body\points2.csv',header='infer')    
        df.columns=['x','y']
        x3=df.loc[:,"x"].values
        y3=df.loc[:,"y"].values
        encircle(x3, y3, ec="k", fc="red", alpha=0.2)
        c2=ax.add_patch(encircle(x3, y3, ec="k", fc="red", alpha=0.2))

        os.remove(r'body\points.csv')
        os.remove(r'body\points2.csv')
        Counter+=1

    plt.waitforbuttonpress()


