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
import body as bulk

"""from fixed_step_solver import *"""

root = Tk()
root.geometry('486x464+540+200')
root.title("PyROPS v2.3.12")
x = 540
y = 200
os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (x,y)

def LoadFile():
    filepath = filedialog.askopenfilename(initialdir="C:\\Users\\",filetypes= (("text files","*.txt"),("all files","*.*")))
    file = open(filepath,'r')
    print(file.read())
    file.close()

def SaveFile():
    file = filedialog.asksaveasfile(initialdir="C:\\Users\\",filetypes= (("text files","*.txt"),("all files","*.*")))
    file.write(filetext)
    file.close()

def Exit():
    exit()

def Website():
    webbrowser.open('https://aerospace.ukzn.ac.za/')

def Support():
    print('<insert GitHub URL here>')

def Start():
    launch = Tk()
    launch.title("ASRI Propulsion and Automation Simulation Software (v2.3.12)")
    launch.geometry('1590x870')
    launch.state('zoomed')

    """Column 1"""

    label=Label(launch,text="Maximum Simulation Time",anchor='w',width=30).place(x=5,y=10+0*22)
    label=Label(launch,text="s",anchor='w',width=4).place(x=415,y=10+0*22)
    entry1=tk.Entry(launch,width=35,textvariable=tk.StringVar(),justify='right')
    entry1.insert(0, "{:.3f}".format(1200))
    entry1.place(x=190,y=10)

    label=Label(launch,text="Time Step Size",anchor='w',width=30).place(x=5,y=10+1*22)
    label=Label(launch,text="s",anchor='w',width=4).place(x=415,y=10+1*22)
    entry2=tk.Entry(launch,width=35,textvariable=tk.StringVar(),justify='right')
    entry2.insert(0, "{:.3f}".format(0.02))
    entry2.place(x=190,y=10+1*22)
    
    label=Label(launch,text="Launch Latitude",anchor='w',width=30).place(x=5,y=10+2*22)
    label=Label(launch,text="deg",anchor='w',width=4).place(x=415,y=10+2*22)
    entry3=tk.Entry(launch,width=35,textvariable=tk.StringVar(),justify='right')
    entry3.insert(0, "{:.3f}".format(-34.600))
    entry3.place(x=190,y=10+2*22)

    label=Label(launch,text="Launch Longitude",anchor='w',width=30).place(x=5,y=10+3*22)
    label=Label(launch,text="deg",anchor='w',width=4).place(x=415,y=10+3*22)
    entry4=tk.Entry(launch,width=35,textvariable=tk.StringVar(),justify='right')
    entry4.insert(0, "{:.3f}".format(20.300))
    entry4.place(x=190,y=10+3*22)
    
    label=Label(launch,text="Launch Altitude",anchor='w',width=30).place(x=5,y=10+4*22)
    label=Label(launch,text="m",anchor='w',width=4).place(x=415,y=10+4*22)
    entry5=tk.Entry(launch,width=35,textvariable=tk.StringVar(),justify='right')
    entry5.insert(0, "{:.3f}".format(0.000))
    entry5.place(x=190,y=10+4*22)
    
    label=Label(launch,text="Launch Elevation",anchor='w',width=30).place(x=5,y=10+5*22)
    label=Label(launch,text="deg",anchor='w',width=4).place(x=415,y=10+5*22)
    entry6=tk.Entry(launch,width=35,textvariable=tk.StringVar(),justify='right')
    entry6.insert(0, "{:.3f}".format(80.000))
    entry6.place(x=190,y=10+5*22)

    label=Label(launch,text="Launch Azimuth",anchor='w',width=30).place(x=5,y=10+6*22)
    label=Label(launch,text="deg",anchor='w',width=4).place(x=415,y=10+6*22)
    entry7=tk.Entry(launch,width=35,textvariable=tk.StringVar(),justify='right')
    entry7.insert(0, "{:.3f}".format(-100.000))
    entry7.place(x=190,y=10+6*22)
    
    label=Label(launch,text="Rocket Body Radius",anchor='w',width=30).place(x=5,y=10+7*22)
    label=Label(launch,text="m",anchor='w',width=4).place(x=415,y=10+7*22)
    entry8=tk.Entry(launch,width=35,textvariable=tk.StringVar(),justify='right')
    entry8.insert(0, "{:.3f}".format(0.087))
    entry8.place(x=190,y=10+7*22)
    
    label=Label(launch,text="Rocket Body Length",anchor='w',width=30).place(x=5,y=10+8*22)
    label=Label(launch,text="m",anchor='w',width=4).place(x=415,y=10+8*22)
    entry9=tk.Entry(launch,width=35,textvariable=tk.StringVar(),justify='right')
    entry9.insert(0, "{:.3f}".format(4.920))
    entry9.place(x=190,y=10+8*22)
    
    label=Label(launch,text="Launch Rail Length",anchor='w',width=30).place(x=5,y=10+9*22)
    label=Label(launch,text="m",anchor='w',width=4).place(x=415,y=10+9*22)
    entry10=tk.Entry(launch,width=35,textvariable=tk.StringVar(),justify='right')
    entry10.insert(0, "{:.3f}".format(7))
    entry10.place(x=190,y=10+9*22)
    
    label=Label(launch,text="Nozzle Exit Area",anchor='w',width=30).place(x=5,y=10+10*22)
    label=Label(launch,text="m2",anchor='w',width=4).place(x=415,y=10+10*22)
    entry11=tk.Entry(launch,width=35,textvariable=tk.StringVar(),justify='right')
    entry11.insert(0, "{:.6f}".format(0.007056))
    entry11.place(x=190,y=10+10*22)

    label=Label(launch,text="Thrust Polynomial Degree",anchor='w',width=30).place(x=5,y=10+11*22)
    label=Label(launch,text="-",anchor='w',width=4).place(x=415,y=10+11*22)
    entry12=tk.Entry(launch,width=35,textvariable=tk.StringVar(),justify='right')
    entry12.insert(0, "{:.0f}".format(6))
    entry12.place(x=190,y=10+11*22)
    
    label=Label(launch,text="MissileDATCOM Cards",anchor='w',width=30).place(x=5,y=10+12*22)
    label=Label(launch,text="-",anchor='w',width=4).place(x=415,y=10+12*22)
    entry13=tk.Entry(launch,width=35,textvariable=tk.StringVar(),justify='right')
    entry13.insert(0, "{:.0f}".format(20))
    entry13.place(x=190,y=10+12*22)
 
    label=Label(launch,text="CAD Mass (Dry)",anchor='w',width=30).place(x=5,y=10+13*22)
    label=Label(launch,text="kg",anchor='w',width=4).place(x=415,y=10+13*22)
    entry14=tk.Entry(launch,width=35,textvariable=tk.StringVar(),justify='right')
    entry14.insert(0, "{:.8f}".format(49.35224149))
    entry14.place(x=190,y=10+13*22)

    label=Label(launch,text="CAD COM abt. origin (Dry)",anchor='w',width=30).place(x=5,y=10+14*22)
    label=Label(launch,text="m",anchor='w',width=4).place(x=415,y=10+14*22)
    label=Label(launch,text="x",anchor='w',width=2).place(x=242,y=10+14*22)
    entry15=tk.Entry(launch,width=8,textvariable=tk.StringVar(),justify='right')
    entry15.insert(0, "{:.8f}".format(1.386632))
    entry15.place(x=190,y=10+14*22)
    label=Label(launch,text="y",anchor='w',width=2).place(x=317,y=10+14*22)
    entry16=tk.Entry(launch,width=8,textvariable=tk.StringVar(),justify='right')
    entry16.insert(0, "{:.8f}".format(0))
    entry16.place(x=265,y=10+14*22)
    label=Label(launch,text="z",anchor='w',width=2).place(x=392,y=10+14*22)
    entry17=tk.Entry(launch,width=8,textvariable=tk.StringVar(),justify='right')
    entry17.insert(0, "{:.8f}".format(0))
    entry17.place(x=340,y=10+14*22)

    label=Label(launch,text="CAD MOI abt. origin (Dry)",anchor='w',width=30).place(x=5,y=10+15*22)
    label=Label(launch,text="kgm2",anchor='w',width=4).place(x=415,y=10+15*22)
    label=Label(launch,text="x",anchor='w',width=2).place(x=242,y=10+15*22)
    entry18=tk.Entry(launch,width=8,textvariable=tk.StringVar(),justify='right')
    entry18.insert(0, "{:.8f}".format(0.04023116))
    entry18.place(x=190,y=10+15*22)
    label=Label(launch,text="y",anchor='w',width=2).place(x=317,y=10+15*22)
    entry19=tk.Entry(launch,width=8,textvariable=tk.StringVar(),justify='right')
    entry19.insert(0, "{:.8f}".format(180.8297))
    entry19.place(x=265,y=10+15*22)
    label=Label(launch,text="z",anchor='w',width=2).place(x=392,y=10+15*22)
    entry20=tk.Entry(launch,width=8,textvariable=tk.StringVar(),justify='right')
    entry20.insert(0, "{:.8f}".format(180.8297))
    entry20.place(x=340,y=10+15*22)

    label=Label(launch,text="Time Burn",anchor='w',width=30).place(x=5,y=10+16*22)
    label=Label(launch,text="s",anchor='w',width=4).place(x=415,y=10+16*22)
    entry21=tk.Entry(launch,width=35,textvariable=tk.StringVar(),justify='right')
    entry21.insert(0, "{:.3f}".format(17.1))
    entry21.place(x=190,y=10+16*22)

    label=Label(launch,text="Density Fuel",anchor='w',width=30).place(x=5,y=10+17*22)
    label=Label(launch,text="kg/m3",anchor='w',width=5).place(x=415,y=10+17*22)
    entry22=tk.Entry(launch,width=35,textvariable=tk.StringVar(),justify='right')
    entry22.insert(0, "{:.6f}".format(1065))
    entry22.place(x=190,y=10+17*22)

    label=Label(launch,text="Radius Fuel",anchor='w',width=30).place(x=5,y=10+18*22)
    label=Label(launch,text="m",anchor='w',width=4).place(x=415,y=10+18*22)
    entry23=tk.Entry(launch,width=35,textvariable=tk.StringVar(),justify='right')
    entry23.insert(0, "{:.6f}".format(0.0735))
    entry23.place(x=190,y=10+18*22)

    label=Label(launch,text="Thickness Fuel",anchor='w',width=30).place(x=5,y=10+19*22)
    label=Label(launch,text="m",anchor='w',width=4).place(x=415,y=10+19*22)
    entry24=tk.Entry(launch,width=35,textvariable=tk.StringVar(),justify='right')
    entry24.insert(0, "{:.6f}".format(0.0375))
    entry24.place(x=190,y=10+19*22)

    label=Label(launch,text="Thickness Fuel Initial",anchor='w',width=30).place(x=5,y=10+20*22)
    label=Label(launch,text="m",anchor='w',width=4).place(x=415,y=10+20*22)
    entry25=tk.Entry(launch,width=35,textvariable=tk.StringVar(),justify='right')
    entry25.insert(0, "{:.6f}".format(0.0375))
    entry25.place(x=190,y=10+20*22)

    label=Label(launch,text="COM Fuel",anchor='w',width=30).place(x=5,y=10+21*22)
    label=Label(launch,text="m",anchor='w',width=4).place(x=415,y=10+21*22)
    entry26=tk.Entry(launch,width=35,textvariable=tk.StringVar(),justify='right')
    entry26.insert(0, "{:.6f}".format(0.5395))
    entry26.place(x=190,y=10+21*22)

    label=Label(launch,text="Length Fuel",anchor='w',width=30).place(x=5,y=10+22*22)
    label=Label(launch,text="m",anchor='w',width=4).place(x=415,y=10+22*22)                                                                     
    entry27=tk.Entry(launch,width=35,textvariable=tk.StringVar(),justify='right')
    entry27.insert(0, "{:.6f}".format(0.510))
    entry27.place(x=190,y=10+22*22)

    #
    
    label=Label(launch,text="CD Parachute and Drogue",anchor='w',width=30).place(x=5,y=10+23.5*22)
    """label=Label(launch,text="-",anchor='w',width=4).place(x=415,y=10+23.5*22)"""
    label=Label(launch,text="-",anchor='w',width=4).place(x=280,y=10+23.5*22) #"(1)"
    label=Label(launch,text="-",anchor='w',width=4).place(x=415,y=10+23.5*22) #"(2)"
    label=Label(launch,text="Nose",anchor='w',width=4).place(x=530,y=10+23.5*22) #"(3)"
    label=Label(launch,text="Booster",anchor='w',width=6).place(x=655,y=10+23.5*22) #"(4)"
    label=Label(launch,text="Combined",anchor='w',width=8).place(x=780,y=10+23.5*22) #"(5)"
    entry28=tk.Entry(launch,width=14,textvariable=tk.StringVar(),justify='right')
    entry28.insert(0, "{:.3f}".format(2.2))
    entry28.place(x=190,y=10+23.5*22)
    entry123=tk.Entry(launch,width=14,textvariable=tk.StringVar(),justify='right')
    entry123.insert(0, "{:.3f}".format(1.6))
    entry123.place(x=315,y=10+23.5*22)
    entry202=tk.Entry(launch,width=14,textvariable=tk.StringVar(),justify='right')
    entry202.insert(0, "{:.3f}".format(1.2))
    entry202.place(x=440,y=10+23.5*22)
    entry203=tk.Entry(launch,width=14,textvariable=tk.StringVar(),justify='right')
    entry203.insert(0, "{:.3f}".format(1.2))
    entry203.place(x=565,y=10+23.5*22)
    entry206=tk.Entry(launch,width=14,textvariable=tk.StringVar(),justify='right')
    entry206.insert(0, "{:.3f}".format(1.2))
    entry206.place(x=690,y=10+23.5*22)

    label=Label(launch,text="Diameter Parachute and Drogue",anchor='w',width=30).place(x=5,y=10+24.5*22)
    """label=Label(launch,text="m",anchor='w',width=4).place(x=415,y=10+24.5*22)"""
    label=Label(launch,text="m",anchor='w',width=4).place(x=280,y=10+24.5*22) #"(1)"
    label=Label(launch,text="m",anchor='w',width=4).place(x=415,y=10+24.5*22) #"(2)"
    entry29=tk.Entry(launch,width=14,textvariable=tk.StringVar(),justify='right')
    entry29.insert(0, "{:.3f}".format(1.22052868353847))
    entry29.place(x=190,y=10+24.5*22)
    entry124=tk.Entry(launch,width=14,textvariable=tk.StringVar(),justify='right')
    entry124.insert(0, "{:.3f}".format(0.304871262610412))
    entry124.place(x=315,y=10+24.5*22)

    label=Label(launch,text="Deploy Parachute and Drogue",anchor='w',width=30).place(x=5,y=10+25.5*22)
    """label=Label(launch,text="s",anchor='w',width=4).place(x=415,y=10+25.5*22)"""
    """label=Label(launch,text="Nose",anchor='w',width=4).place(x=242,y=10+25.5*22)"""
    """label=Label(launch,text="Booster",anchor='w',width=6).place(x=317,y=10+25.5*22)"""
    label=Label(launch,text="m",anchor='w',width=4).place(x=280,y=10+25.5*22)
    label=Label(launch,text="s",anchor='w',width=4).place(x=415,y=10+25.5*22)
    entry30=tk.Entry(launch,width=14,textvariable=tk.StringVar(),justify='right')
    entry30.insert(0, "{:.3f}".format(13500))
    entry30.place(x=190,y=10+25.5*22)
    entry125=tk.Entry(launch,width=14,textvariable=tk.StringVar(),justify='right')
    entry125.insert(0, "{:.3f}".format(5))
    entry125.place(x=315,y=10+25.5*22)

    label=Label(launch,text="Mass Nose Separated",anchor='w',width=30).place(x=5,y=10+26.5*22)
    label=Label(launch,text="kg",anchor='w',width=4).place(x=415,y=10+26.5*22)
    entry31=tk.Entry(launch,width=35,textvariable=tk.StringVar(),justify='right')
    entry31.insert(0, "{:.8f}".format(4))
    entry31.place(x=190,y=10+26.5*22)

    label=Label(launch,text="COM Nose Separated",anchor='w',width=30).place(x=5,y=10+27.5*22)
    label=Label(launch,text="m",anchor='w',width=4).place(x=415,y=10+27.5*22)
    label=Label(launch,text="x",anchor='w',width=2).place(x=242,y=10+27.5*22)
    entry32=tk.Entry(launch,width=8,textvariable=tk.StringVar(),justify='right')
    entry32.insert(0, "{:.8f}".format(0.5))
    entry32.place(x=190,y=10+27.5*22)
    label=Label(launch,text="y",anchor='w',width=2).place(x=317,y=10+27.5*22)
    entry33=tk.Entry(launch,width=8,textvariable=tk.StringVar(),justify='right')
    entry33.insert(0, "{:.8f}".format(0))
    entry33.place(x=265,y=10+27.5*22)
    label=Label(launch,text="z",anchor='w',width=2).place(x=392,y=10+27.5*22)
    entry34=tk.Entry(launch,width=8,textvariable=tk.StringVar(),justify='right')
    entry34.insert(0, "{:.8f}".format(0))
    entry34.place(x=340,y=10+27.5*22)

    label=Label(launch,text="MOI Nose Separated",anchor='w',width=30).place(x=5,y=10+28.5*22)
    label=Label(launch,text="kgm2",anchor='w',width=4).place(x=415,y=10+28.5*22)
    label=Label(launch,text="x",anchor='w',width=2).place(x=242,y=10+28.5*22)
    entry35=tk.Entry(launch,width=8,textvariable=tk.StringVar(),justify='right')
    entry35.insert(0, "{:.8f}".format(0.02))
    entry35.place(x=190,y=10+28.5*22)
    label=Label(launch,text="y",anchor='w',width=2).place(x=317,y=10+28.5*22)
    entry36=tk.Entry(launch,width=8,textvariable=tk.StringVar(),justify='right')
    entry36.insert(0, "{:.8f}".format(25.64))
    entry36.place(x=265,y=10+28.5*22)
    label=Label(launch,text="z",anchor='w',width=2).place(x=392,y=10+28.5*22)
    entry37=tk.Entry(launch,width=8,textvariable=tk.StringVar(),justify='right')
    entry37.insert(0, "{:.8f}".format(25.64))
    entry37.place(x=340,y=10+28.5*22)

    label=Label(launch,text="Mass Booster Separated",anchor='w',width=30).place(x=5,y=10+29.5*22)
    label=Label(launch,text="kg",anchor='w',width=4).place(x=415,y=10+29.5*22)
    entry38=tk.Entry(launch,width=35,textvariable=tk.StringVar(),justify='right')
    entry38.insert(0, "{:.8f}".format(48))
    entry38.place(x=190,y=10+29.5*22)

    label=Label(launch,text="COM Booster Separated",anchor='w',width=30).place(x=5,y=10+30.5*22)
    label=Label(launch,text="m",anchor='w',width=4).place(x=415,y=10+30.5*22)
    label=Label(launch,text="x",anchor='w',width=2).place(x=242,y=10+30.5*22)
    entry39=tk.Entry(launch,width=8,textvariable=tk.StringVar(),justify='right')
    entry39.insert(0, "{:.8f}".format(1.3))
    entry39.place(x=190,y=10+30.5*22)
    label=Label(launch,text="y",anchor='w',width=2).place(x=317,y=10+30.5*22)
    entry40=tk.Entry(launch,width=8,textvariable=tk.StringVar(),justify='right')
    entry40.insert(0, "{:.8f}".format(0))
    entry40.place(x=265,y=10+30.5*22)
    label=Label(launch,text="z",anchor='w',width=2).place(x=392,y=10+30.5*22)
    entry41=tk.Entry(launch,width=8,textvariable=tk.StringVar(),justify='right')
    entry41.insert(0, "{:.8f}".format(0))
    entry41.place(x=340,y=10+30.5*22)    

    label=Label(launch,text="MOI Booster Separated",anchor='w',width=30).place(x=5,y=10+31.5*22)
    label=Label(launch,text="kgm2",anchor='w',width=4).place(x=415,y=10+31.5*22)
    label=Label(launch,text="x",anchor='w',width=2).place(x=242,y=10+31.5*22)
    entry42=tk.Entry(launch,width=8,textvariable=tk.StringVar(),justify='right')
    entry42.insert(0, "{:.8f}".format(0.039))
    entry42.place(x=190,y=10+31.5*22) 
    label=Label(launch,text="y",anchor='w',width=2).place(x=317,y=10+31.5*22)
    entry43=tk.Entry(launch,width=8,textvariable=tk.StringVar(),justify='right')
    entry43.insert(0, "{:.8f}".format(160))
    entry43.place(x=265,y=10+31.5*22) 
    label=Label(launch,text="z",anchor='w',width=2).place(x=392,y=10+31.5*22)
    entry44=tk.Entry(launch,width=8,textvariable=tk.StringVar(),justify='right')
    entry44.insert(0, "{:.8f}".format(160))
    entry44.place(x=340,y=10+31.5*22) 

    """Column 2"""

    label=Label(launch,text="Number Runs",anchor='w',width=10).place(x=465,y=10+0*22)
    """label=Label(launch,text="-",anchor='w',width=4).place(x=845,y=10+0*22)"""
    entry45=tk.Entry(launch,width=8,textvariable=tk.StringVar(),justify='right')
    entry45.insert(0, "{:.0f}".format(1))
    entry45.place(x=550,y=10+0*22)

    label=Label(launch,text="Body State",anchor='w',width=10).place(x=465,y=10+1*22)
    """label=Label(launch,text="-",anchor='w',width=4).place(x=845,y=10+0*22)"""
    entry201=tk.Entry(launch,width=8,textvariable=tk.StringVar(),justify='right')
    entry201.insert(0, "{:.0f}".format(1))
    entry201.place(x=550,y=10+1*22)

    label=Label(launch,text="Lower",anchor='w',width=6).place(x=758,y=10+1*22)
    label=Label(launch,text="Upper",anchor='w',width=6).place(x=818,y=10+1*22)

    label=Label(launch,text="Launch Elevation Uncertainty",anchor='w',width=30).place(x=465,y=10+2*22)
    label=Label(launch,text="deg",anchor='w',width=4).place(x=705,y=10+2*22)
    entry46=tk.Entry(launch,width=8,textvariable=tk.StringVar(),justify='right')
    entry46.insert(0, "{:.3f}".format(0))
    entry46.place(x=750,y=10+2*22)
    entry47=tk.Entry(launch,width=8,textvariable=tk.StringVar(),justify='right')
    entry47.insert(0, "{:.3f}".format(0))
    entry47.place(x=810,y=10+2*22)

    label=Label(launch,text="Launch Azimuth Uncertainty",anchor='w',width=30).place(x=465,y=10+3*22)
    label=Label(launch,text="deg",anchor='w',width=4).place(x=705,y=10+3*22)
    entry48=tk.Entry(launch,width=8,textvariable=tk.StringVar(),justify='right')
    entry48.insert(0, "{:.3f}".format(0))
    entry48.place(x=750,y=10+3*22)
    entry49=tk.Entry(launch,width=8,textvariable=tk.StringVar(),justify='right')
    entry49.insert(0, "{:.3f}".format(0))
    entry49.place(x=810,y=10+3*22)

    label=Label(launch,text="Thrust Misalignment (Yaw) Uncertainty",anchor='w',width=30).place(x=465,y=10+4*22)
    label=Label(launch,text="deg",anchor='w',width=4).place(x=705,y=10+4*22)
    entry50=tk.Entry(launch,width=8,textvariable=tk.StringVar(),justify='right')
    entry50.insert(0, "{:.3f}".format(0))
    entry50.place(x=750,y=10+4*22)
    entry51=tk.Entry(launch,width=8,textvariable=tk.StringVar(),justify='right')
    entry51.insert(0, "{:.3f}".format(0))
    entry51.place(x=810,y=10+4*22)

    label=Label(launch,text="Thrust Misalignment (Pitch) Uncertainty",anchor='w',width=30).place(x=465,y=10+5*22)
    label=Label(launch,text="deg",anchor='w',width=4).place(x=705,y=10+5*22)
    entry52=tk.Entry(launch,width=8,textvariable=tk.StringVar(),justify='right')
    entry52.insert(0, "{:.3f}".format(0))
    entry52.place(x=750,y=10+5*22)
    entry53=tk.Entry(launch,width=8,textvariable=tk.StringVar(),justify='right')
    entry53.insert(0, "{:.3f}".format(0))
    entry53.place(x=810,y=10+5*22)

    label=Label(launch,text="Thrust Magnitude Uncertainty",anchor='w',width=30).place(x=465,y=10+6*22)
    """Note: previously, thrust force magnitude variation (Newtons) was input here by the user"""
    label=Label(launch,text="%",anchor='w',width=4).place(x=705,y=10+6*22)
    entry54=tk.Entry(launch,width=8,textvariable=tk.StringVar(),justify='right')
    entry54.insert(0, "{:.3f}".format(0))
    entry54.place(x=750,y=10+6*22)
    entry55=tk.Entry(launch,width=8,textvariable=tk.StringVar(),justify='right')
    entry55.insert(0, "{:.3f}".format(0))
    entry55.place(x=810,y=10+6*22)

    label=Label(launch,text="Burn Time Uncertainty",anchor='w',width=30).place(x=465,y=10+7*22)
    label=Label(launch,text="s",anchor='w',width=4).place(x=705,y=10+7*22)
    entry56=tk.Entry(launch,width=8,textvariable=tk.StringVar(),justify='right')
    entry56.insert(0, "{:.3f}".format(0))
    entry56.place(x=750,y=10+7*22)
    entry57=tk.Entry(launch,width=8,textvariable=tk.StringVar(),justify='right')
    entry57.insert(0, "{:.3f}".format(0))
    entry57.place(x=810,y=10+7*22)

    label=Label(launch,text="Wind Magnitude Uncertainty",anchor='w',width=30).place(x=465,y=10+8*22)
    label=Label(launch,text="%",anchor='w',width=4).place(x=705,y=10+8*22)
    entry58=tk.Entry(launch,width=8,textvariable=tk.StringVar(),justify='right')
    entry58.insert(0, "{:.3f}".format(0))
    entry58.place(x=750,y=10+8*22)
    entry59=tk.Entry(launch,width=8,textvariable=tk.StringVar(),justify='right')
    entry59.insert(0, "{:.3f}".format(0))
    entry59.place(x=810,y=10+8*22)

    label=Label(launch,text="Wind Direction Uncertainty",anchor='w',width=30).place(x=465,y=10+9*22)
    label=Label(launch,text="deg",anchor='w',width=4).place(x=705,y=10+9*22)
    entry60=tk.Entry(launch,width=8,textvariable=tk.StringVar(),justify='right')
    entry60.insert(0, "{:.3f}".format(0))
    entry60.place(x=750,y=10+9*22)
    entry61=tk.Entry(launch,width=8,textvariable=tk.StringVar(),justify='right')
    entry61.insert(0, "{:.3f}".format(0))
    entry61.place(x=810,y=10+9*22)

    label=Label(launch,text="Aerodynamic Drag Coefficients",anchor='w',width=30).place(x=465,y=10+10*22)
    label=Label(launch,text="%",anchor='w',width=4).place(x=705,y=10+10*22)
    entry62=tk.Entry(launch,width=8,textvariable=tk.StringVar(),justify='right')
    entry62.insert(0, "{:.3f}".format(0))
    entry62.place(x=750,y=10+10*22)
    entry63=tk.Entry(launch,width=8,textvariable=tk.StringVar(),justify='right')
    entry63.insert(0, "{:.3f}".format(0))
    entry63.place(x=810,y=10+10*22)

    label=Label(launch,text="Aerodynamic Lift Coefficients",anchor='w',width=30).place(x=465,y=10+11*22)
    label=Label(launch,text="%",anchor='w',width=4).place(x=705,y=10+11*22)
    entry64=tk.Entry(launch,width=8,textvariable=tk.StringVar(),justify='right')
    entry64.insert(0, "{:.3f}".format(0))
    entry64.place(x=750,y=10+11*22)
    entry65=tk.Entry(launch,width=8,textvariable=tk.StringVar(),justify='right')
    entry65.insert(0, "{:.3f}".format(0))
    entry65.place(x=810,y=10+11*22)

    label=Label(launch,text="Aerodynamic Moment Coefficients",anchor='w',width=30).place(x=465,y=10+12*22)
    label=Label(launch,text="%",anchor='w',width=4).place(x=705,y=10+12*22)
    entry66=tk.Entry(launch,width=8,textvariable=tk.StringVar(),justify='right')
    entry66.insert(0, "{:.3f}".format(0))
    entry66.place(x=750,y=10+12*22)
    entry67=tk.Entry(launch,width=8,textvariable=tk.StringVar(),justify='right')
    entry67.insert(0, "{:.3f}".format(0))
    entry67.place(x=810,y=10+12*22)

    label=Label(launch,text="Centre of Pressure Variation",anchor='w',width=30).place(x=465,y=10+13*22)
    label=Label(launch,text="%",anchor='w',width=4).place(x=705,y=10+13*22)
    entry68=tk.Entry(launch,width=8,textvariable=tk.StringVar(),justify='right')
    entry68.insert(0, "{:.3f}".format(0))
    entry68.place(x=750,y=10+13*22)
    entry69=tk.Entry(launch,width=8,textvariable=tk.StringVar(),justify='right')
    entry69.insert(0, "{:.3f}".format(0))
    entry69.place(x=810,y=10+13*22)

    label=Label(launch,text="Fin Cant Angle Uncertainty",anchor='w',width=30).place(x=465,y=10+14*22)
    label=Label(launch,text="deg",anchor='w',width=4).place(x=705,y=10+14*22)
    entry70=tk.Entry(launch,width=8,textvariable=tk.StringVar(),justify='right')
    entry70.insert(0, "{:.3f}".format(0))
    entry70.place(x=750,y=10+14*22)
    entry71=tk.Entry(launch,width=8,textvariable=tk.StringVar(),justify='right')
    entry71.insert(0, "{:.3f}".format(0))
    entry71.place(x=810,y=10+14*22)

    label=Label(launch,text="Launch Altitude Uncertainty",anchor='w',width=30).place(x=465,y=10+15*22)
    label=Label(launch,text="m",anchor='w',width=4).place(x=705,y=10+15*22)
    entry72=tk.Entry(launch,width=8,textvariable=tk.StringVar(),justify='right')
    entry72.insert(0, "{:.3f}".format(0))
    entry72.place(x=750,y=10+15*22)
    entry73=tk.Entry(launch,width=8,textvariable=tk.StringVar(),justify='right')
    entry73.insert(0, "{:.3f}".format(0))
    entry73.place(x=810,y=10+15*22)

    #
    
    label=Label(launch,text="Calculator Step Size",anchor='w',width=20).place(x=465,y=10+16*22)
    label=Label(launch,text="s",anchor='w',width=4).place(x=845,y=10+16*22)
    entry74=tk.Entry(launch,width=35,textvariable=tk.StringVar(),justify='right')
    entry74.insert(0, "{:.3f}".format(0.005))
    entry74.place(x=620,y=10+16*22)

    label=Label(launch,text="Density Oxidiser",anchor='w',width=20).place(x=465,y=10+17*22)
    label=Label(launch,text="kg/m3",anchor='w',width=5).place(x=845,y=10+17*22)
    entry75=tk.Entry(launch,width=35,textvariable=tk.StringVar(),justify='right')
    entry75.insert(0, "{:.6f}".format(880))
    entry75.place(x=620,y=10+17*22)

    label=Label(launch,text="Radius Oxidiser",anchor='w',width=20).place(x=465,y=10+18*22)
    label=Label(launch,text="m",anchor='w',width=4).place(x=845,y=10+18*22)
    entry76=tk.Entry(launch,width=35,textvariable=tk.StringVar(),justify='right')
    entry76.insert(0, "{:.6f}".format(0.07633))
    entry76.place(x=620,y=10+18*22)

    label=Label(launch,text="Length Oxidiser",anchor='w',width=20).place(x=465,y=10+19*22)
    label=Label(launch,text="m",anchor='w',width=4).place(x=845,y=10+19*22)
    entry77=tk.Entry(launch,width=35,textvariable=tk.StringVar(),justify='right')
    entry77.insert(0, "{:.6f}".format(1.9869))
    entry77.place(x=620,y=10+19*22)

    label=Label(launch,text="Length Oxidiser Initial",anchor='w',width=20).place(x=465,y=10+20*22)
    label=Label(launch,text="m",anchor='w',width=4).place(x=845,y=10+20*22)
    entry78=tk.Entry(launch,width=35,textvariable=tk.StringVar(),justify='right')
    entry78.insert(0, "{:.6f}".format(1.9869))
    entry78.place(x=620,y=10+20*22)

    label=Label(launch,text="COM Oxidiser",anchor='w',width=20).place(x=465,y=10+21*22)
    label=Label(launch,text="m",anchor='w',width=4).place(x=845,y=10+21*22)
    entry79=tk.Entry(launch,width=35,textvariable=tk.StringVar(),justify='right')
    entry79.insert(0, "{:.6f}".format(2.20745))
    entry79.place(x=620,y=10+21*22)

    label=Label(launch,text="Time Burn Fuel",anchor='w',width=20).place(x=465,y=10+22*22)
    label=Label(launch,text="s",anchor='w',width=4).place(x=845,y=10+22*22) 
    entry80=tk.Entry(launch,width=35,textvariable=tk.StringVar(),justify='right')
    entry80.insert(0, "{:.3f}".format(17.1))
    entry80.place(x=620,y=10+22*22)
    
    #
    
    label=Label(launch,text="Number Stages",anchor='w',width=20).place(x=465,y=10+26.5*22)
    label=Label(launch,text="-",anchor='w',width=4).place(x=845,y=10+26.5*22)
    entry81=tk.Entry(launch,width=35,textvariable=tk.StringVar(),justify='right')
    entry81.insert(0, "{:.0f}".format(1))
    entry81.place(x=620,y=10+26.5*22)

    label=Label(launch,text="Stage Separation Time",anchor='w',width=20).place(x=465,y=10+27.5*22)
    label=Label(launch,text="m",anchor='w',width=4).place(x=845,y=10+27.5*22)
    entry82=tk.Entry(launch,width=35,textvariable=tk.StringVar(),justify='right')
    entry82.insert(0, "{:.3f}".format(45))
    entry82.place(x=620,y=10+27.5*22)

    label=Label(launch,text="Stage Separation Delay",anchor='w',width=20).place(x=465,y=10+28.5*22)
    label=Label(launch,text="m",anchor='w',width=4).place(x=845,y=10+28.5*22)
    entry83=tk.Entry(launch,width=35,textvariable=tk.StringVar(),justify='right')
    entry83.insert(0, "{:.3f}".format(0))
    entry83.place(x=620,y=10+28.5*22)

    label=Label(launch,text="Mass Stage Separated",anchor='w',width=20).place(x=465,y=10+29.5*22)
    label=Label(launch,text="m",anchor='w',width=4).place(x=845,y=10+29.5*22)
    entry84=tk.Entry(launch,width=35,textvariable=tk.StringVar(),justify='right')
    entry84.insert(0, "{:.8f}".format(48))
    entry84.place(x=620,y=10+29.5*22)
    
    label=Label(launch,text="COM Stage Separated",anchor='w',width=20).place(x=465,y=10+30.5*22)
    label=Label(launch,text="m",anchor='w',width=4).place(x=845,y=10+30.5*22)
    label=Label(launch,text="x",anchor='w',width=2).place(x=672,y=10+30.5*22)
    entry85=tk.Entry(launch,width=8,textvariable=tk.StringVar(),justify='right')
    entry85.insert(0, "{:.8f}".format(1.3))
    entry85.place(x=620,y=10+30.5*22)
    label=Label(launch,text="y",anchor='w',width=2).place(x=747,y=10+30.5*22)
    entry86=tk.Entry(launch,width=8,textvariable=tk.StringVar(),justify='right')
    entry86.insert(0, "{:.8f}".format(0))
    entry86.place(x=695,y=10+30.5*22)
    label=Label(launch,text="z",anchor='w',width=2).place(x=822,y=10+30.5*22)
    entry87=tk.Entry(launch,width=8,textvariable=tk.StringVar(),justify='right')
    entry87.insert(0, "{:.8f}".format(0))
    entry87.place(x=770,y=10+30.5*22)

    label=Label(launch,text="MOI Stage Separated",anchor='w',width=20).place(x=465,y=10+31.5*22)
    label=Label(launch,text="kgm2",anchor='w',width=4).place(x=845,y=10+31.5*22)
    label=Label(launch,text="x",anchor='w',width=2).place(x=672,y=10+31.5*22)
    entry88=tk.Entry(launch,width=8,textvariable=tk.StringVar(),justify='right')
    entry88.insert(0, "{:.8f}".format(0.039))
    entry88.place(x=620,y=10+31.5*22)
    label=Label(launch,text="y",anchor='w',width=2).place(x=747,y=10+31.5*22)
    entry89=tk.Entry(launch,width=8,textvariable=tk.StringVar(),justify='right')
    entry89.insert(0, "{:.8f}".format(160))
    entry89.place(x=695,y=10+31.5*22)
    label=Label(launch,text="z",anchor='w',width=2).place(x=822,y=10+31.5*22)
    entry90=tk.Entry(launch,width=8,textvariable=tk.StringVar(),justify='right')
    entry90.insert(0, "{:.8f}".format(160))
    entry90.place(x=770,y=10+31.5*22)

    """Column 3"""

    label=Label(launch,text="Nosecone Radius",anchor='w',width=20).place(x=895,y=10+0*22)
    label=Label(launch,text="m",anchor='w',width=4).place(x=1075,y=10+0*22)
    entry91=tk.Entry(launch,width=10,textvariable=tk.StringVar(),justify='right')
    entry91.insert(0, "{:.5f}".format(0.085))
    entry91.place(x=1000,y=10+0*22)

    label=Label(launch,text="Nosecone Length",anchor='w',width=20).place(x=895,y=10+1*22)
    label=Label(launch,text="m",anchor='w',width=4).place(x=1075,y=10+1*22)
    entry92=tk.Entry(launch,width=10,textvariable=tk.StringVar(),justify='right')
    entry92.insert(0, "{:.5f}".format(0.85))
    entry92.place(x=1000,y=10+1*22)

    label=Label(launch,text="Fin Root Chord",anchor='w',width=20).place(x=895,y=10+2*22)
    label=Label(launch,text="m",anchor='w',width=4).place(x=1075,y=10+2*22)
    entry93=tk.Entry(launch,width=10,textvariable=tk.StringVar(),justify='right')
    entry93.insert(0, "{:.5f}".format(0.39))
    entry93.place(x=1000,y=10+2*22)

    label=Label(launch,text="Fin Tip Chord",anchor='w',width=20).place(x=895,y=10+3*22)
    label=Label(launch,text="m",anchor='w',width=4).place(x=1075,y=10+3*22)
    entry94=tk.Entry(launch,width=10,textvariable=tk.StringVar(),justify='right')
    entry94.insert(0, "{:.5f}".format(0.12))
    entry94.place(x=1000,y=10+3*22)

    label=Label(launch,text="Fin Sweep",anchor='w',width=20).place(x=895,y=10+4*22)
    label=Label(launch,text="m",anchor='w',width=4).place(x=1075,y=10+4*22)
    entry95=tk.Entry(launch,width=10,textvariable=tk.StringVar(),justify='right')
    entry95.insert(0, "{:.5f}".format(0.279))
    entry95.place(x=1000,y=10+4*22)

    label=Label(launch,text="Fin Span",anchor='w',width=20).place(x=895,y=10+5*22)
    label=Label(launch,text="m",anchor='w',width=4).place(x=1075,y=10+5*22)
    entry96=tk.Entry(launch,width=10,textvariable=tk.StringVar(),justify='right')
    entry96.insert(0, "{:.5f}".format(0.17))
    entry96.place(x=1000,y=10+5*22)

    label=Label(launch,text="Fin Location",anchor='w',width=20).place(x=895,y=10+6*22)
    label=Label(launch,text="m",anchor='w',width=4).place(x=1075,y=10+6*22)
    entry97=tk.Entry(launch,width=10,textvariable=tk.StringVar(),justify='right')
    entry97.insert(0, "{:.5f}".format(0.141+0.123))
    entry97.place(x=1000,y=10+6*22)

    label=Label(launch,text="Fin Root Span",anchor='w',width=20).place(x=895,y=10+7*22)
    label=Label(launch,text="m",anchor='w',width=4).place(x=1075,y=10+7*22)
    entry98=tk.Entry(launch,width=10,textvariable=tk.StringVar(),justify='right')
    entry98.insert(0, "{:.5f}".format(0.0855))
    entry98.place(x=1000,y=10+7*22)

    label=Label(launch,text="Fin Cant Angle",anchor='w',width=20).place(x=895,y=10+8*22)
    label=Label(launch,text="deg",anchor='w',width=4).place(x=1075,y=10+8*22)
    entry99=tk.Entry(launch,width=10,textvariable=tk.StringVar(),justify='right')
    entry99.insert(0, "{:.5f}".format(0.35))
    entry99.place(x=1000,y=10+8*22)

    #

    label=Label(launch,text="Monte Carlo",anchor='w',width=20).place(x=895,y=10+9.5*22)
    label=Label(launch,text="on=1 off=0",anchor='w',width=20).place(x=1065,y=10+9.5*22)
    entry200=tk.Entry(launch,width=10,textvariable=tk.StringVar(),justify='right')
    entry200.insert(0, "{:.0f}".format(1))
    entry200.place(x=1000,y=10+9.5*22)
    """check1=tk.Checkbutton(launch,text='On/Off',variable=tk.IntVar(),onvalue=1,offvalue=0)"""
    """check1.place(x=1165,y=10+9.3*22)"""

    label=Label(launch,text="WGS Earth Model",anchor='w',width=20).place(x=895,y=10+10.5*22)
    label=Label(launch,text="on=1 off=0",anchor='w',width=20).place(x=1065,y=10+10.5*22)
    entry205=tk.Entry(launch,width=10,textvariable=tk.StringVar(),justify='right')
    entry205.insert(0, "{:.0f}".format(1))
    entry205.place(x=1000,y=10+10.5*22)
    """check1=tk.Checkbutton(launch,text='On/Off',variable=tk.IntVar(),onvalue=1,offvalue=0)"""
    """check1.place(x=1165,y=10+9.3*22)"""
    
    label=Label(launch,text="Wind Turbulence",anchor='w',width=20).place(x=895,y=11+11.5*22)
    label=Label(launch,text="on=1 off=0",anchor='w',width=20).place(x=1065,y=10+11.5*22)
    entry100=tk.Entry(launch,width=10,textvariable=tk.StringVar(),justify='right')
    entry100.insert(0, "{:.0f}".format(0))
    entry100.place(x=1000,y=10+11.5*22)
    """check1=tk.Checkbutton(launch,text='On/Off',variable=tk.IntVar(),onvalue=1,offvalue=0)"""
    """check1.place(x=1165,y=10+11.3*22)"""

    label=Label(launch,text="4-in-1 Simulation",anchor='w',width=20).place(x=895,y=10+12.5*22)
    label=Label(launch,text="on=1 off=0",anchor='w',width=20).place(x=1065,y=10+12.5*22)
    entry101=tk.Entry(launch,width=10,textvariable=tk.StringVar(),justify='right')
    entry101.insert(0, "{:.0f}".format(0))
    entry101.place(x=1000,y=10+12.5*22)
    """check2=tk.Checkbutton(launch,text='On/Off',variable=tk.IntVar(),onvalue=1,offvalue=0)"""
    """check2.place(x=1165,y=10+12.3*22)"""

    label=Label(launch,text="Staging",anchor='w',width=20).place(x=895,y=10+13.5*22)
    label=Label(launch,text="on=1 off=0",anchor='w',width=20).place(x=1065,y=10+13.5*22)
    entry102=tk.Entry(launch,width=10,textvariable=tk.StringVar(),justify='right')
    entry102.insert(0, "{:.0f}".format(0))
    entry102.place(x=1000,y=10+13.5*22)
    """check3=tk.Checkbutton(launch,text='On/Off',variable=tk.IntVar(),onvalue=1,offvalue=0)"""
    """check3.place(x=1165,y=10+13.3*22)"""
    
    label=Label(launch,text="Thrust Curve Fit",anchor='w',width=20).place(x=895,y=10+14.5*22)
    label=Label(launch,text="on=1 off=0",anchor='w',width=20).place(x=1065,y=10+14.5*22)
    entry103=tk.Entry(launch,width=10,textvariable=tk.StringVar(),justify='right')
    entry103.insert(0, "{:.0f}".format(0))
    entry103.place(x=1000,y=10+14.5*22)
    """check4=tk.Checkbutton(launch,text='On/Off',variable=tk.IntVar(),onvalue=1,offvalue=0)"""
    """check4.place(x=1165,y=10+14.3*22)"""

    label=Label(launch,text="Roll Control",anchor='w',width=20).place(x=895,y=10+15.5*22)
    label=Label(launch,text="on=1 off=0",anchor='w',width=20).place(x=1065,y=10+15.5*22)
    entry104=tk.Entry(launch,width=10,textvariable=tk.StringVar(),justify='right')
    entry104.insert(0, "{:.0f}".format(0))
    entry104.place(x=1000,y=10+15.5*22)
    """check5=tk.Checkbutton(launch,text='On/Off',variable=tk.IntVar(),onvalue=1,offvalue=0)"""
    """check5.place(x=1165,y=10+15.3*22)"""

    """
    label=Label(launch,text="Monte Carlo UI",anchor='w',width=20).place(x=895,y=10+16.5*22)
    label=Label(launch,text="on=1 off=0",anchor='w',width=20).place(x=1065,y=10+16.5*22)
    entry105=tk.Entry(launch,width=10,textvariable=tk.StringVar(),justify='right')
    entry105.insert(0, "{:.0f}".format(1))
    entry105.place(x=1000,y=10+16.5*22)
    check6=tk.Checkbutton(launch,text='On/Off',variable=tk.IntVar(),onvalue=1,offvalue=0)
    check6.place(x=1165,y=10+16.3*22)
    check6.select()
    """
    
    label=Label(launch,text="Stream Full Output",anchor='w',width=20).place(x=895,y=10+16.5*22)
    label=Label(launch,text="on=1 off=0",anchor='w',width=20).place(x=1065,y=10+16.5*22)
    entry106=tk.Entry(launch,width=10,textvariable=tk.StringVar(),justify='right')
    entry106.insert(0, "{:.0f}".format(1))
    entry106.place(x=1000,y=10+16.5*22)
    """check7=tk.Checkbutton(launch,text='On/Off',variable=tk.IntVar(),onvalue=1,offvalue=0)"""
    """check7.place(x=1165,y=10+16.3*22)"""
    
    label=Label(launch,text="2nd Order Solver",anchor='w',width=20).place(x=895,y=10+17.5*22)
    label=Label(launch,text="on=1 off=0",anchor='w',width=20).place(x=1065,y=10+17.5*22)
    entry107=tk.Entry(launch,width=10,textvariable=tk.StringVar(),justify='right')
    entry107.insert(0, "{:.0f}".format(0))
    entry107.place(x=1000,y=10+17.5*22)
    """check8=tk.Checkbutton(launch,text='On/Off',variable=tk.IntVar(),onvalue=1,offvalue=0)"""
    """check8.place(x=1165,y=10+17.3*22)"""
    
    label=Label(launch,text="4th Order Solver",anchor='w',width=20).place(x=895,y=10+18.5*22)
    label=Label(launch,text="on=1 off=0",anchor='w',width=20).place(x=1065,y=10+18.5*22)
    entry108=tk.Entry(launch,width=10,textvariable=tk.StringVar(),justify='right')
    entry108.insert(0, "{:.0f}".format(0))
    entry108.place(x=1000,y=10+18.5*22)
    """check9=tk.Checkbutton(launch,text='On/Off',variable=tk.IntVar(),onvalue=1,offvalue=0)"""
    """check9.place(x=1165,y=10+18.3*22)"""
    
    label=Label(launch,text="8th Order Solver",anchor='w',width=20).place(x=895,y=10+19.5*22)
    label=Label(launch,text="on=1 off=0",anchor='w',width=20).place(x=1065,y=10+19.5*22)
    entry109=tk.Entry(launch,width=10,textvariable=tk.StringVar(),justify='right')
    entry109.insert(0, "{:.0f}".format(1))
    entry109.place(x=1000,y=10+19.5*22)
    """check10=tk.Checkbutton(launch,text='On/Off',variable=tk.IntVar(),onvalue=1,offvalue=0)"""
    """check10.place(x=1165,y=10+19.3*22)"""
    """check10.select()"""

    label=Label(launch,text="MC Detailed Output",anchor='w',width=20).place(x=895,y=10+20.5*22)
    label=Label(launch,text="on=1 off=0",anchor='w',width=20).place(x=1065,y=10+20.5*22)
    entry207=tk.Entry(launch,width=10,textvariable=tk.StringVar(),justify='right')
    entry207.insert(0, "{:.0f}".format(1))
    entry207.place(x=1000,y=10+20.5*22)
    
    label=Label(launch,text="Relative Tolerance",anchor='w',width=20).place(x=895,y=10+21.5*22)
    label=Label(launch,text="-",anchor='w',width=6).place(x=1070,y=10+21.5*22)
    entry110=tk.Entry(launch,width=10,textvariable=tk.StringVar(),justify='right')
    entry110.insert(0, "{:.3f}".format(1))
    entry110.place(x=1000,y=10+21.5*22)

    label=Label(launch,text="Absolute Tolerance",anchor='w',width=20).place(x=895,y=10+22.5*22)
    label=Label(launch,text="-",anchor='w',width=6).place(x=1070,y=10+22.5*22)
    entry111=tk.Entry(launch,width=10,textvariable=tk.StringVar(),justify='right')
    entry111.insert(0, "{:.3f}".format(1))
    entry111.place(x=1000,y=10+22.5*22)

    label=Label(launch,text="First Step Size",anchor='w',width=20).place(x=895,y=10+23.5*22)
    label=Label(launch,text="s",anchor='w',width=6).place(x=1070,y=10+23.5*22)
    entry112=tk.Entry(launch,width=10,textvariable=tk.StringVar(),justify='right')
    entry112.insert(0, "{:.3f}".format(0.02))
    entry112.place(x=1000,y=10+23.5*22)

    label=Label(launch,text="Max Step Size",anchor='w',width=20).place(x=895,y=10+24.5*22)
    label=Label(launch,text="s",anchor='w',width=6).place(x=1070,y=10+24.5*22)
    entry113=tk.Entry(launch,width=10,textvariable=tk.StringVar(),justify='right')
    entry113.insert(0, "{:.3f}".format(0.02))
    entry113.place(x=1000,y=10+24.5*22)

    label=Label(launch,text="Lug(from boat tail)",anchor='w',width=20).place(x=895,y=10+25.5*22)
    label=Label(launch,text="m",anchor='w',width=6).place(x=1070,y=10+25.5*22)
    entry204=tk.Entry(launch,width=10,textvariable=tk.StringVar(),justify='right')
    entry204.insert(0, "{:.6f}".format(1.927))
    entry204.place(x=1000,y=10+25.5*22)

    
    #

    label=Label(launch,text="Roll Rate Threshold",anchor='w',width=20).place(x=895,y=10+26.5*22)
    label=Label(launch,text="Lower",anchor='w',width=6).place(x=1105,y=10+25.5*22)
    label=Label(launch,text="Upper",anchor='w',width=6).place(x=1230,y=10+25.5*22)
    label=Label(launch,text="rad/s",anchor='w',width=4).place(x=1170,y=10+26.5*22)
    entry114=tk.Entry(launch,width=14,textvariable=tk.StringVar(),justify='right')
    entry114.insert(0, "{:.3f}".format(3))
    entry114.place(x=1080,y=10+26.5*22)
    label=Label(launch,text="rad/s",anchor='w',width=4).place(x=1300,y=10+26.5*22)
    entry115=tk.Entry(launch,width=14,textvariable=tk.StringVar(),justify='right')
    entry115.insert(0, "{:.3f}".format(5))
    entry115.place(x=1205,y=10+26.5*22)

    label=Label(launch,text="Roll Control T-Initial, T-Final",anchor='w',width=30).place(x=895,y=10+27.5*22)
    label=Label(launch,text="s",anchor='w',width=4).place(x=1170,y=10+27.5*22)
    entry121=tk.Entry(launch,width=14,textvariable=tk.StringVar(),justify='right')
    entry121.insert(0, "{:.3f}".format(10))
    entry121.place(x=1080,y=10+27.5*22)
    label=Label(launch,text="s",anchor='w',width=4).place(x=1300,y=10+27.5*22)
    entry122=tk.Entry(launch,width=14,textvariable=tk.StringVar(),justify='right')
    entry122.insert(0, "{:.3f}".format(30))
    entry122.place(x=1205,y=10+27.5*22)
    
    label=Label(launch,text="Thrust Pulse Frequency",anchor='w',width=20).place(x=895,y=10+28.5*22)
    label=Label(launch,text="Hz",anchor='w',width=4).place(x=1300,y=10+28.5*22)
    entry116=tk.Entry(launch,width=35,textvariable=tk.StringVar(),justify='right')
    entry116.insert(0, "{:.3f}".format(1))
    entry116.place(x=1080,y=10+28.5*22)

    label=Label(launch,text="Thrust Force",anchor='w',width=20).place(x=895,y=10+29.5*22)
    label=Label(launch,text="N",anchor='w',width=4).place(x=1300,y=10+29.5*22)
    entry117=tk.Entry(launch,width=35,textvariable=tk.StringVar(),justify='right')
    entry117.insert(0, "{:.3f}".format(0))
    entry117.place(x=1080,y=10+29.5*22)
    
    label=Label(launch,text="OpenGL Graphic Scale",anchor='w',width=30).place(x=895,y=10+30.5*22)
    label=Label(launch,text="-",anchor='w',width=4).place(x=1300,y=10+30.5*22)
    entry118=tk.Entry(launch,width=35,textvariable=tk.StringVar(),justify='right')
    entry118.insert(0, "{:.3f}".format(101))
    entry118.place(x=1080,y=10+30.5*22)
    
    label=Label(launch,text="OpenGL Trajectory Resolution",anchor='w',width=30).place(x=895,y=10+31.5*22)
    label=Label(launch,text="-",anchor='w',width=4).place(x=1300,y=10+31.5*22)
    entry119=tk.Entry(launch,width=35,textvariable=tk.StringVar(),justify='right')
    entry119.insert(0, "{:.3f}".format(100))
    entry119.place(x=1080,y=10+31.5*22)

    label=Label(launch,text="Hybrid Rocket Thrust Curve",anchor='w',width=30).place(x=895,y=10+32.5*22)
    label=Label(launch,text="on=1 off=0",anchor='w',width=20).place(x=1300,y=10+32.5*22)
    entry208=tk.Entry(launch,width=35,textvariable=tk.StringVar(),justify='right')
    entry208.insert(0, "{:.0f}".format(1))
    entry208.place(x=1080,y=10+32.5*22)

    label=Label(launch,text="Max. Total Angle of Attack",anchor='w',width=30).place(x=895,y=10+33.5*22)
    label=Label(launch,text="deg",anchor='w',width=4).place(x=1300,y=10+33.5*22)
    entry209=tk.Entry(launch,width=35,textvariable=tk.StringVar(),justify='right')
    entry209.insert(0, "{:.0f}".format(25))
    entry209.place(x=1080,y=10+33.5*22)

    entry120=tk.Entry(launch,width=33,textvariable=tk.StringVar(),justify='left')
    entry120.insert(0,(open(r"Path.txt","r").readlines()[0]))
    entry120.place(x=1130,y=5+0*22)

    def Refresh():
        global TimeMax
        global TimeSize
        global LaunchLatitude
        global LaunchLongitude
        global LaunchAltitude
        global LaunchElevation
        global LaunchAzimuth
        global RocketBodyRadius
        global RocketBodyLength
        global LaunchRailLength
        global NozzleExitArea
        global ThrustPolynomialDegree
        global MissileDATCOMCards
        global SolidWorksMass
        global SolidWorksCOMx
        global SolidWorksCOMy
        global SolidWorksCOMz
        global SolidWorksMOIx
        global SolidWorksMOIy
        global SolidWorksMOIz
        global TimeBurn
        global FuelDensity
        global FuelRadius
        global FuelThickness
        global FuelThicknessInitial
        global FuelCOM
        global FuelLength
        global ParachuteCD
        global NoseCD
        global BoosterCD
        global ParachuteDiameter
        global ParachuteDelay
        global NoseMass
        global NoseCOMx
        global NoseCOMy
        global NoseCOMz
        global NoseMOIx
        global NoseMOIy
        global NoseMOIz
        global BoosterMass
        global BoosterCOMx
        global BoosterCOMy
        global BoosterCOMz
        global BoosterMOIx
        global BoosterMOIy
        global BoosterMOIz
        global NumberRuns
        global State
        global LaunchElevationLower
        global LaunchElevationUpper
        global LaunchAzimuthLower
        global LaunchAzimuthUpper
        global ThrustMisalignmentYawLower
        global ThrustMisalignmentYawUpper
        global ThrustMisalignmentPitchLower
        global ThrustMisalignmentPitchUpper
        global ThrustMagnitudeLower
        global ThrustMagnitudeUpper
        global TimeBurnLower
        global TimeBurnUpper
        global WindMagnitudeLower
        global WindMagnitudeUpper
        global WindDirectionLower
        global WindDirectionUpper
        global AerodynamicDragLower
        global AerodynamicDragUpper
        global AerodynamicLiftLower
        global AerodynamicLiftUpper
        global AerodynamicMomentLower
        global AerodynamicMomentUpper
        global CentreOfPressureLower
        global CentreOfPressuerUpper
        global FinCantAngleLower
        global FinCantAngleUpper
        global LaunchAltitudeLower
        global LaunchAltitudeUpper
        global CalculatorTimeSize
        global OxidiserDensity
        global OxidiserRadius
        global OxidiserLength
        global OxidiserLengthInitial
        global OxidiserCOM
        global TimeBurnFuel
        global NumberStages
        global StageTime
        global StageDelay
        global StageMass
        global StageCOMx
        global StageCOMy
        global StageCOMz
        global StageMOIx
        global StageMOIy
        global StageMOIz
        global NoseRadius
        global NoseLength
        global FinRootChord
        global FinTipChord
        global FinSweep
        global FinSpan
        global FinLocation
        global FinSpanRoot
        global FinCantAngle
        global CheckEarthModelWGS
        global CheckTurbulence
        global Check4in1
        global CheckStaging
        global CheckThrust
        global CheckRollControl
        global CheckMonteCarloUI
        global CheckStream
        global CheckOrder2
        global CheckOrder4
        global CheckOrder8
        global SolverRelative
        global SolverAbsolute
        global SolverTimeSizeFirst
        global SolverTimeSizeMax
        global RollControlLower
        global RollControlUpper
        global RollControlFrequency
        global RollControlForce
        global GraphicScale
        global TrajectoryResolution
        global Directory
        global RollControlTimeInitial
        global RollControlTimeFinal
        global DrogueCD
        global DrogueDiameter
        global DrogueDelay
        global Lug
        global CombinedCD
        global MCDetailed
        global ThrustHybrid
        global MaxTAOA
        
        TimeMax=float(entry1.get())
        TimeSize=float(entry2.get())
        LaunchLatitude=float(entry3.get())
        LaunchLongitude=float(entry4.get())
        LaunchAltitude=float(entry5.get())
        LaunchElevation=float(entry6.get())
        LaunchAzimuth=float(entry7.get())
        RocketBodyRadius=float(entry8.get())
        RocketBodyLength=float(entry9.get())
        LaunchRailLength=float(entry10.get())
        NozzleExitArea=float(entry11.get())
        ThrustPolynomialDegree=float(entry12.get())
        MissileDATCOMCards=float(entry13.get())
        SolidWorksMass=float(entry14.get())
        SolidWorksCOMx=float(entry15.get())
        SolidWorksCOMy=float(entry16.get())
        SolidWorksCOMz=float(entry17.get())
        SolidWorksMOIx=float(entry18.get())
        SolidWorksMOIy=float(entry19.get())
        SolidWorksMOIz=float(entry20.get())
        TimeBurn=float(entry21.get())
        FuelDensity=float(entry22.get())
        FuelRadius=float(entry23.get())
        FuelThickness=float(entry24.get())
        FuelThicknessInitial=float(entry25.get())
        FuelCOM=float(entry26.get())
        FuelLength=float(entry27.get())
        ParachuteCD=float(entry28.get())
        NoseCD=float(entry202.get())
        BoosterCD=float(entry203.get())
        ParachuteDiameter=float(entry29.get())
        ParachuteDelay=float(entry30.get())
        NoseMass=float(entry31.get())
        NoseCOMx=float(entry32.get())
        NoseCOMy=float(entry33.get())
        NoseCOMz=float(entry34.get())
        NoseMOIx=float(entry35.get())
        NoseMOIy=float(entry36.get())
        NoseMOIz=float(entry37.get())
        BoosterMass=float(entry38.get())
        BoosterCOMx=float(entry39.get())
        BoosterCOMy=float(entry40.get())
        BoosterCOMz=float(entry41.get())
        BoosterMOIx=float(entry42.get())
        BoosterMOIy=float(entry43.get())
        BoosterMOIz=float(entry44.get())
        NumberRuns=float(entry45.get())
        State=round(float(entry201.get()),0)
        LaunchElevationLower=float(entry46.get())
        LaunchElevationUpper=float(entry47.get())
        LaunchAzimuthLower=float(entry48.get())
        LaunchAzimuthUpper=float(entry49.get())
        ThrustMisalignmentYawLower=float(entry50.get())
        ThrustMisalignmentYawUpper=float(entry51.get())
        ThrustMisalignmentPitchLower=float(entry52.get())
        ThrustMisalignmentPitchUpper=float(entry53.get())
        ThrustMagnitudeLower=float(entry54.get())
        ThrustMagnitudeUpper=float(entry55.get())
        TimeBurnLower=float(entry56.get())
        TimeBurnUpper=float(entry57.get())
        WindMagnitudeLower=float(entry58.get())
        WindMagnitudeUpper=float(entry59.get())
        WindDirectionLower=float(entry60.get())
        WindDirectionUpper=float(entry61.get())
        AerodynamicDragLower=float(entry62.get())
        AerodynamicDragUpper=float(entry63.get())
        AerodynamicLiftLower=float(entry64.get())
        AerodynamicLiftUpper=float(entry65.get())
        AerodynamicMomentLower=float(entry66.get())
        AerodynamicMomentUpper=float(entry67.get())
        CentreOfPressureLower=float(entry68.get())
        CentreOfPressuerUpper=float(entry69.get())
        FinCantAngleLower=float(entry70.get())
        FinCantAngleUpper=float(entry71.get())
        LaunchAltitudeLower=float(entry72.get())
        LaunchAltitudeUpper=float(entry73.get())
        CalculatorTimeSize=float(entry74.get())
        OxidiserDensity=float(entry75.get())
        OxidiserRadius=float(entry76.get())
        OxidiserLength=float(entry77.get())
        OxidiserLengthInitial=float(entry78.get())
        OxidiserCOM=float(entry79.get())
        TimeBurnFuel=float(entry80.get())
        NumberStages=float(entry81.get())
        StageTime=float(entry82.get())
        StageDelay=float(entry83.get())
        StageMass=float(entry84.get())
        StageCOMx=float(entry85.get())
        StageCOMy=float(entry86.get())
        StageCOMz=float(entry87.get())
        StageMOIx=float(entry88.get())
        StageMOIy=float(entry89.get())
        StageMOIz=float(entry90.get())
        NoseRadius=float(entry91.get())
        NoseLength=float(entry92.get())
        FinRootChord=float(entry93.get())
        FinTipChord=float(entry94.get())
        FinSweep=float(entry95.get())
        FinSpan=float(entry96.get())
        FinLocation=float(entry97.get())
        FinSpanRoot=float(entry98.get())
        FinCantAngle=float(entry99.get())
        CheckEarthModelWGS=float(entry205.get())
        CheckTurbulence=float(entry100.get())
        Check4in1=float(entry101.get())
        CheckStaging=float(entry102.get())
        CheckThrust=float(entry103.get())
        CheckRollControl=float(entry104.get())
        CheckMonteCarloUI=float(entry200.get())
        CheckStream=float(entry106.get())
        CheckOrder2=float(entry107.get())
        CheckOrder4=float(entry108.get())
        CheckOrder8=float(entry109.get())
        SolverRelative=float(entry110.get())
        SolverAbsolute=float(entry111.get())
        SolverTimeSizeFirst=float(entry112.get())
        SolverTimeSizeMax=float(entry113.get())
        RollControlLower=float(entry114.get())
        RollControlUpper=float(entry115.get())
        RollControlFrequency=float(entry116.get())
        RollControlForce=float(entry117.get())
        GraphicScale=float(entry118.get())
        TrajectoryResolution=float(entry119.get())
        Directory=entry120.get()
        Path=open(r"Path.txt","w")
        Path.write(Directory)
        Path.close()
        RollControlTimeInitial=float(entry121.get())
        RollControlTimeFinal=float(entry122.get())
        DrogueCD=float(entry123.get())
        DrogueDiameter=float(entry124.get())
        DrogueDelay=float(entry125.get())
        Lug=float(entry204.get())
        CombinedCD=float(entry206.get())
        MCDetailed=float(entry207.get())
        ThrustHybrid=float(entry208.get())
        MaxTAOA=float(entry209.get())

        """print(TimeMax,TimeSize,LaunchLatitude,LaunchLongitude,LaunchAltitude,LaunchElevation,LaunchAzimuth,RocketBodyRadius,RocketBodyLength,LaunchRailLength,NozzleExitArea,ThrustPolynomialDegree,MissileDATCOMCards,SolidWorksMass,SolidWorksCOMx,SolidWorksCOMy,SolidWorksCOMz,SolidWorksMOIx,SolidWorksMOIy,SolidWorksMOIz,TimeBurn,FuelDensity,FuelRadius,FuelThickness,FuelThicknessInitial,FuelCOM,FuelLength,ParachuteCD,ParachuteDiameter,ParachuteDelay,NoseMass,NoseCOMx,NoseCOMy,NoseCOMz,NoseMOIx,NoseMOIy,NoseMOIz,BoosterMass,BoosterCOMx,BoosterCOMy,BoosterCOMz,BoosterMOIx,BoosterMOIy,BoosterMOIz,NumberRuns,LaunchElevationLower,LaunchElevationUpper,LaunchAzimuthLower,LaunchAzimuthUpper,ThrustMisalignmentYawLower,ThrustMisalignmentYawUpper,ThrustMisalignmentPitchLower,ThrustMisalignmentPitchUpper,ThrustMagnitudeLower,ThrustMagnitudeUpper,TimeBurnLower,TimeBurnUpper,WindMagnitudeLower,WindMagnitudeUpper,WindDirectionLower,WindDirectionUpper,AerodynamicDragLower,AerodynamicDragUpper,AerodynamicLiftLower,AerodynamicLiftUpper,AerodynamicMomentLower,AerodynamicMomentUpper,CentreOfPressureLower,CentreOfPressuerUpper,FinCantAngleLower,FinCantAngleUpper,LaunchAltitudeLower,LaunchAltitudeUpper,CalculatorTimeSize,OxidiserDensity,OxidiserRadius,OxidiserLength,OxidiserLengthInitial,OxidiserCOM,TimeBurnFuel,NumberStages,StageTime,StageDelay,StageMass,StageCOMx,StageCOMy,StageCOMz,StageMOIx,StageMOIy,StageMOIz,NoseRadius,NoseLength,FinRootChord,FinTipChord,FinSweep,FinSpan,FinLocation,FinSpanRoot,FinCantAngle,CheckTurbulence,Check4in1,CheckStaging,CheckThrust,CheckRollControl,CheckStream,CheckOrder2,CheckOrder4,CheckOrder8,SolverRelative,SolverAbsolute,SolverTimeSizeFirst,SolverTimeSizeMax,RollControlLower,RollControlUpper,RollControlFrequency,RollControlForce,GraphicScale,TrajectoryResolution,Directory)"""

    def LoadInputs():
        with open(r"Path.txt") as file:
            directory=file.read()
        LoadInputs=pd.read_excel(r'{}\Inputs\Settings.xlsx'.format(directory),header=0)
        def Load():
            entry1.delete(1,END)
            entry1.insert(0, " {:.3f}".format(LoadInputs.at[0,"Maximum Simulation Time"]))
            entry2.delete(1,END)
            entry2.insert(0, " {:.3f}".format(LoadInputs.at[0,"Time Step Size"]))
            entry3.delete(1,END)
            entry3.insert(0, " {:.3f}".format(LoadInputs.at[0,"Launch Latitude"]))
            entry4.delete(1,END)
            entry4.insert(0, " {:.3f}".format(LoadInputs.at[0,"Launch Longitude"]))
            entry5.delete(1,END)
            entry5.insert(0, " {:.3f}".format(LoadInputs.at[0,"Launch Altitude"]))
            entry6.delete(1,END)
            entry6.insert(0, " {:.3f}".format(LoadInputs.at[0,"Launch Elevation"]))
            entry7.delete(1,END)
            entry7.insert(0, " {:.3f}".format(LoadInputs.at[0,"Launch Azimuth"]))
            entry8.delete(1,END)
            entry8.insert(0, " {:.3f}".format(LoadInputs.at[0,"Rocket Body Radius"]))
            entry9.delete(1,END)
            entry9.insert(0, " {:.3f}".format(LoadInputs.at[0,"Rocket Body Length"]))
            entry10.delete(1,END)
            entry10.insert(0, " {:.3f}".format(LoadInputs.at[0,"Launch Rail Length"]))
            entry11.delete(1,END)
            entry11.insert(0, " {:.6f}".format(LoadInputs.at[0,"Nozzle Exit Area"]))
            entry12.delete(1,END)
            entry12.insert(0, " {:.3f}".format(LoadInputs.at[0,"Thrust Polynomial Degree"]))
            entry13.delete(1,END)
            entry13.insert(0, " {:.3f}".format(LoadInputs.at[0,"MissileDATCOM Cards"]))
            entry14.delete(1,END)
            entry14.insert(0, " {:.8f}".format(LoadInputs.at[0,"SolidWorks Mass"]))
            entry15.delete(1,END)
            entry15.insert(0, " {:.8f}".format(LoadInputs.at[0,"SolidWorks COMx"]))
            entry16.delete(1,END)
            entry16.insert(0, " {:.8f}".format(LoadInputs.at[0,"SolidWorks COMy"]))
            entry17.delete(1,END)
            entry17.insert(0, " {:.8f}".format(LoadInputs.at[0,"SolidWorks COMz"]))
            entry18.delete(1,END)
            entry18.insert(0, " {:.8f}".format(LoadInputs.at[0,"SolidWorks MOIx"]))
            entry19.delete(1,END)
            entry19.insert(0, " {:.8f}".format(LoadInputs.at[0,"SolidWorks MOIy"]))
            entry20.delete(1,END)
            entry20.insert(0, " {:.8f}".format(LoadInputs.at[0,"SolidWorks MOIz"]))
            entry21.delete(1,END)
            entry21.insert(0, " {:.3f}".format(LoadInputs.at[0,"Time Burn"]))
            entry22.delete(1,END)
            entry22.insert(0, " {:.6f}".format(LoadInputs.at[0,"Density Fuel"]))
            entry23.delete(1,END)
            entry23.insert(0, " {:.6f}".format(LoadInputs.at[0,"Radius Fuel"]))
            entry24.delete(1,END)
            entry24.insert(0, " {:.6f}".format(LoadInputs.at[0,"Thickness Fuel"]))
            entry25.delete(1,END)
            entry25.insert(0, " {:.6f}".format(LoadInputs.at[0,"Thickness Fuel Initial"]))
            entry26.delete(1,END)
            entry26.insert(0, " {:.6f}".format(LoadInputs.at[0,"COM Fuel"]))
            entry27.delete(1,END)
            entry27.insert(0, " {:.6f}".format(LoadInputs.at[0,"Length Fuel"]))
            entry28.delete(1,END)
            entry28.insert(0, " {:.3f}".format(LoadInputs.at[0,"CD Parachute"]))
            entry29.delete(1,END)
            entry29.insert(0, " {:.3f}".format(LoadInputs.at[0,"Diameter Parachute"]))
            entry30.delete(1,END)
            entry30.insert(0, " {:.3f}".format(LoadInputs.at[0,"Parachute Deployment Delay"]))
            entry31.delete(1,END)
            entry31.insert(0, " {:.8f}".format(LoadInputs.at[0,"Mass Nose Separated"]))
            entry32.delete(1,END)
            entry32.insert(0, " {:.8f}".format(LoadInputs.at[0,"COMx Nose Separated"]))
            entry33.delete(1,END)
            entry33.insert(0, " {:.8f}".format(LoadInputs.at[0,"COMy Nose Separated"]))
            entry34.delete(1,END)
            entry34.insert(0, " {:.8f}".format(LoadInputs.at[0,"COMz Nose Separated"]))
            entry35.delete(1,END)
            entry35.insert(0, " {:.8f}".format(LoadInputs.at[0,"MOIx Nose Separated"]))
            entry36.delete(1,END)
            entry36.insert(0, " {:.8f}".format(LoadInputs.at[0,"MOIy Nose Separated"]))
            entry37.delete(1,END)
            entry37.insert(0, " {:.8f}".format(LoadInputs.at[0,"MOIz Nose Separated"]))
            entry38.delete(1,END)
            entry38.insert(0, " {:.8f}".format(LoadInputs.at[0,"Mass Booster Separated"]))
            entry39.delete(1,END)
            entry39.insert(0, " {:.8f}".format(LoadInputs.at[0,"COMx Booster Separated"]))
            entry40.delete(1,END)
            entry40.insert(0, " {:.8f}".format(LoadInputs.at[0,"COMy Booster Separated"]))
            entry41.delete(1,END)
            entry41.insert(0, " {:.8f}".format(LoadInputs.at[0,"COMz Booster Separated"]))
            entry42.delete(1,END)
            entry42.insert(0, " {:.8f}".format(LoadInputs.at[0,"MOIx Booster Separated"]))
            entry43.delete(1,END)
            entry43.insert(0, " {:.8f}".format(LoadInputs.at[0,"MOIy Booster Separated"]))
            entry44.delete(1,END)
            entry44.insert(0, " {:.8f}".format(LoadInputs.at[0,"MOIz Booster Separated"]))
            entry45.delete(1,END)
            entry45.insert(0, " {:.0f}".format(LoadInputs.at[0,"Number Runs"]))
            entry46.delete(1,END)
            entry46.insert(0, " {:.3f}".format(LoadInputs.at[0,"Launch Elevation Uncertainty Lower"]))
            entry47.delete(1,END)
            entry47.insert(0, " {:.3f}".format(LoadInputs.at[0,"Launch Elevation Uncertainty Upper"]))
            entry48.delete(1,END)
            entry48.insert(0, " {:.3f}".format(LoadInputs.at[0,"Launch Azimuth Uncertainty Lower"]))
            entry49.delete(1,END)
            entry49.insert(0, " {:.3f}".format(LoadInputs.at[0,"Launch Azimuth Uncertainty Upper"]))
            entry50.delete(1,END)
            entry50.insert(0, " {:.3f}".format(LoadInputs.at[0,"Thrust Misalignment (Yaw) Uncertainty Lower"]))
            entry51.delete(1,END)
            entry51.insert(0, " {:.3f}".format(LoadInputs.at[0,"Thrust Misalignment (Yaw) Uncertainty Upper"]))
            entry52.delete(1,END)
            entry52.insert(0, " {:.3f}".format(LoadInputs.at[0,"Thrust Misalignment (Pitch) Uncertainty Lower"]))
            entry53.delete(1,END)
            entry53.insert(0, " {:.3f}".format(LoadInputs.at[0,"Thrust Misalignment (Pitch) Uncertainty Upper"]))
            entry54.delete(1,END)
            entry54.insert(0, " {:.3f}".format(LoadInputs.at[0,"Thrust Magnitude Lower"]))
            entry55.delete(1,END)
            entry55.insert(0, " {:.3f}".format(LoadInputs.at[0,"Thrust Magnitude Upper"]))
            entry56.delete(1,END)
            entry56.insert(0, " {:.3f}".format(LoadInputs.at[0,"Burn Time Uncertainty Lower"]))
            entry57.delete(1,END)
            entry57.insert(0, " {:.3f}".format(LoadInputs.at[0,"Burn Time Uncertainty Upper"]))
            entry58.delete(1,END)
            entry58.insert(0, " {:.3f}".format(LoadInputs.at[0,"Wind Magnitude Uncertainty Lower"]))
            entry59.delete(1,END)
            entry59.insert(0, " {:.3f}".format(LoadInputs.at[0,"Wind Magnitude Uncertainty Upper"]))
            entry60.delete(1,END)
            entry60.insert(0, " {:.3f}".format(LoadInputs.at[0,"Wind Direction Uncertainty Lower"]))
            entry61.delete(1,END)
            entry61.insert(0, " {:.3f}".format(LoadInputs.at[0,"Wind Direction Uncertainty Upper"]))
            entry62.delete(1,END)
            entry62.insert(0, " {:.3f}".format(LoadInputs.at[0,"Aerodynamic Drag Uncertainty Lower"]))
            entry63.delete(1,END)
            entry63.insert(0, " {:.3f}".format(LoadInputs.at[0,"Aerodynamic Drag Uncertainty Upper"]))
            entry64.delete(1,END)
            entry64.insert(0, " {:.3f}".format(LoadInputs.at[0,"Aerodynamic Lift Uncertainty Lower"]))
            entry65.delete(1,END)
            entry65.insert(0, " {:.3f}".format(LoadInputs.at[0,"Aerodynamic Lift Uncertainty Upper"]))
            entry66.delete(1,END)
            entry66.insert(0, " {:.3f}".format(LoadInputs.at[0,"Aerodynamic Moment Uncertainty Lower"]))
            entry67.delete(1,END)
            entry67.insert(0, " {:.3f}".format(LoadInputs.at[0,"Aerodynamic Moment Uncertainty Upper"]))
            entry68.delete(1,END)
            entry68.insert(0, " {:.3f}".format(LoadInputs.at[0,"Centre of Pressure Uncertainty Lower"]))
            entry69.delete(1,END)
            entry69.insert(0, " {:.3f}".format(LoadInputs.at[0,"Centre of Pressure Uncertainty Upper"]))
            entry70.delete(1,END)
            entry70.insert(0, " {:.3f}".format(LoadInputs.at[0,"Fin Cant Angle Uncertainty Lower"]))
            entry71.delete(1,END)
            entry71.insert(0, " {:.3f}".format(LoadInputs.at[0,"Fin Cant Angle Uncertainty Upper"]))
            entry72.delete(1,END)
            entry72.insert(0, " {:.3f}".format(LoadInputs.at[0,"Launch Altitude Uncertainty Lower"]))
            entry73.delete(1,END)
            entry73.insert(0, " {:.3f}".format(LoadInputs.at[0,"Launch Altitude Uncertainty Upper"]))
            entry74.delete(1,END)
            entry74.insert(0, " {:.3f}".format(LoadInputs.at[0,"Calculator Step Size"]))
            entry75.delete(1,END)
            entry75.insert(0, " {:.6f}".format(LoadInputs.at[0,"Density Oxidiser"]))
            entry76.delete(1,END)
            entry76.insert(0, " {:.6f}".format(LoadInputs.at[0,"Radius Oxidiser"]))
            entry77.delete(1,END)
            entry77.insert(0, " {:.6f}".format(LoadInputs.at[0,"Length Oxidiser"]))
            entry78.delete(1,END)
            entry78.insert(0, " {:.6f}".format(LoadInputs.at[0,"Length Oxidiser Initial"]))
            entry79.delete(1,END)
            entry79.insert(0, " {:.6f}".format(LoadInputs.at[0,"COM Oxidiser"]))
            entry80.delete(1,END)
            entry80.insert(0, " {:.3f}".format(LoadInputs.at[0,"Time Burn Fuel"]))
            entry81.delete(1,END)
            entry81.insert(0, " {:.3f}".format(LoadInputs.at[0,"Number Stages"]))
            entry82.delete(1,END)
            entry82.insert(0, " {:.3f}".format(LoadInputs.at[0,"Stage Separation Time"]))
            entry83.delete(1,END)
            entry83.insert(0, " {:.3f}".format(LoadInputs.at[0,"Stage Separation Delay"]))
            entry84.delete(1,END)
            entry84.insert(0, " {:.8f}".format(LoadInputs.at[0,"Mass Stage Separated"]))
            entry85.delete(1,END)
            entry85.insert(0, " {:.8f}".format(LoadInputs.at[0,"COMx Stage Separated"]))
            entry86.delete(1,END)
            entry86.insert(0, " {:.8f}".format(LoadInputs.at[0,"COMy Stage Separated"]))
            entry87.delete(1,END)
            entry87.insert(0, " {:.8f}".format(LoadInputs.at[0,"COMz Stage Separated"]))
            entry88.delete(1,END)
            entry88.insert(0, " {:.8f}".format(LoadInputs.at[0,"MOIx Stage Separated"]))
            entry89.delete(1,END)
            entry89.insert(0, " {:.8f}".format(LoadInputs.at[0,"MOIy Stage Separated"]))
            entry90.delete(1,END)
            entry90.insert(0, " {:.8f}".format(LoadInputs.at[0,"MOIz Stage Separated"]))
            entry91.delete(1,END)
            entry91.insert(0, " {:.5f}".format(LoadInputs.at[0,"Nosecone Radius"]))
            entry92.delete(1,END)
            entry92.insert(0, " {:.5f}".format(LoadInputs.at[0,"Nosecone Length"]))
            entry93.delete(1,END)
            entry93.insert(0, " {:.5f}".format(LoadInputs.at[0,"Fin Root Chord"]))
            entry94.delete(1,END)
            entry94.insert(0, " {:.5f}".format(LoadInputs.at[0,"Fin Tip Chord"]))
            entry95.delete(1,END)
            entry95.insert(0, " {:.5f}".format(LoadInputs.at[0,"Fin Sweep"]))
            entry96.delete(1,END)
            entry96.insert(0, " {:.5f}".format(LoadInputs.at[0,"Fin Span"]))
            entry97.delete(1,END)
            entry97.insert(0, " {:.5f}".format(LoadInputs.at[0,"Fin Location"]))
            entry98.delete(1,END)
            entry98.insert(0, " {:.5f}".format(LoadInputs.at[0,"Fin Root Span"]))
            entry99.delete(1,END)
            entry99.insert(0, " {:.5f}".format(LoadInputs.at[0,"Fin Cant Angle"]))
            entry205.delete(1,END)
            entry205.insert(0, " {:.0f}".format(LoadInputs.at[0,"WGS Earth Model"]))
            entry100.delete(1,END)
            entry100.insert(0, " {:.0f}".format(LoadInputs.at[0,"Wind Turbulence"]))
            entry101.delete(1,END)
            entry101.insert(0, " {:.0f}".format(LoadInputs.at[0,"4-in-1 Simulation"]))
            entry102.delete(1,END)
            entry102.insert(0, " {:.0f}".format(LoadInputs.at[0,"Staging"]))
            entry103.delete(1,END)
            entry103.insert(0, " {:.0f}".format(LoadInputs.at[0,"Thrust Curve Fit"]))
            entry104.delete(1,END)
            entry104.insert(0, " {:.0f}".format(LoadInputs.at[0,"Roll Control"]))

            """
            entry105.delete(1,END)
            entry105.insert(0, " {:.0f}".format(LoadInputs.at[0,"Monte Carlo UI Input"]))
            """
            
            entry106.delete(1,END)
            entry106.insert(0, " {:.0f}".format(LoadInputs.at[0,"Stream Full Output"]))
            entry107.delete(1,END)
            entry107.insert(0, " {:.0f}".format(LoadInputs.at[0,"2nd Order Solver"]))
            entry108.delete(1,END)
            entry108.insert(0, " {:.0f}".format(LoadInputs.at[0,"4th Order Solver"]))
            entry109.delete(1,END)
            entry109.insert(0, " {:.0f}".format(LoadInputs.at[0,"8th Order Solver"]))
            entry110.delete(1,END)
            entry110.insert(0, " {:.3f}".format(LoadInputs.at[0,"Relative Tolerance"]))
            entry111.delete(1,END)
            entry111.insert(0, " {:.3f}".format(LoadInputs.at[0,"Absolute Tolerance"]))
            entry112.delete(1,END)
            entry112.insert(0, " {:.3f}".format(LoadInputs.at[0,"First Step Size"]))
            entry113.delete(1,END)
            entry113.insert(0, " {:.3f}".format(LoadInputs.at[0,"Maximum Step Size"]))
            entry114.delete(1,END)
            entry114.insert(0, " {:.3f}".format(LoadInputs.at[0,"Roll Rate Threshold Lower"]))
            entry115.delete(1,END)
            entry115.insert(0, " {:.3f}".format(LoadInputs.at[0,"Roll Rate Threshold Upper"]))
            entry116.delete(1,END)
            entry116.insert(0, " {:.3f}".format(LoadInputs.at[0,"Thrust Pulse Frequency"]))
            entry117.delete(1,END)
            entry117.insert(0, " {:.3f}".format(LoadInputs.at[0,"Thrust Force"]))
            entry118.delete(1,END)
            entry118.insert(0, " {:.3f}".format(LoadInputs.at[0,"OpenGL Graphic Scale"]))
            entry119.delete(1,END)
            entry119.insert(0, " {:.3f}".format(LoadInputs.at[0,"OpenGL Trajectory Resolution"]))
            entry120.delete(0,END)
            entry120.insert(0, LoadInputs.at[0,"Directory"])
            entry121.delete(1,END)
            entry121.insert(0, " {:.3f}".format(LoadInputs.at[0,"Roll Control Time Initial"]))
            entry122.delete(1,END)
            entry122.insert(0, " {:.3f}".format(LoadInputs.at[0,"Roll Control Time Final"]))
            entry123.delete(1,END)
            entry123.insert(0, " {:.3f}".format(LoadInputs.at[0,"CD Drogue"]))
            entry124.delete(1,END)
            entry124.insert(0, " {:.3f}".format(LoadInputs.at[0,"Diameter Drogue"]))
            entry125.delete(1,END)
            entry125.insert(0, " {:.3f}".format(LoadInputs.at[0,"Drogue Deployment Delay"]))
            entry201.delete(1,END)
            entry201.insert(0, " {:.0f}".format(LoadInputs.at[0,"State"]))
            entry202.delete(1,END)
            entry202.insert(0, " {:.3f}".format(LoadInputs.at[0,"NoseCD"]))
            entry203.delete(1,END)
            entry203.insert(0, " {:.3f}".format(LoadInputs.at[0,"BoosterCD"]))
            entry206.delete(1,END)
            entry206.insert(0, " {:.6f}".format(LoadInputs.at[0,"CombinedCD"]))
            entry207.delete(1,END)
            entry207.insert(0, " {:.0f}".format(LoadInputs.at[0,"MCDetailed"]))
            entry200.delete(1,END)
            entry200.insert(0, " {:.0f}".format(LoadInputs.at[0,"CheckMonteCarloUI"]))
            entry204.delete(1,END)
            entry204.insert(0, " {:.6f}".format(LoadInputs.at[0,"Lug"]))
            entry208.delete(1,END)
            entry208.insert(0, " {:.0f}".format(LoadInputs.at[0,"ThrustHybrid"]))
            entry209.delete(1,END)
            entry209.insert(0, " {:.0f}".format(LoadInputs.at[0,"MaxTAOA"]))
        
        Load()
        Load()

    def SaveInputs():
        TimeMax=float(entry1.get())
        TimeSize=float(entry2.get())
        LaunchLatitude=float(entry3.get())
        LaunchLongitude=float(entry4.get())
        LaunchAltitude=float(entry5.get())
        LaunchElevation=float(entry6.get())
        LaunchAzimuth=float(entry7.get())
        RocketBodyRadius=float(entry8.get())
        RocketBodyLength=float(entry9.get())
        LaunchRailLength=float(entry10.get())
        NozzleExitArea=float(entry11.get())
        ThrustPolynomialDegree=float(entry12.get())
        MissileDATCOMCards=float(entry13.get())
        SolidWorksMass=float(entry14.get())
        SolidWorksCOMx=float(entry15.get())
        SolidWorksCOMy=float(entry16.get())
        SolidWorksCOMz=float(entry17.get())
        SolidWorksMOIx=float(entry18.get())
        SolidWorksMOIy=float(entry19.get())
        SolidWorksMOIz=float(entry20.get())
        TimeBurn=float(entry21.get())
        FuelDensity=float(entry22.get())
        FuelRadius=float(entry23.get())
        FuelThickness=float(entry24.get())
        FuelThicknessInitial=float(entry25.get())
        FuelCOM=float(entry26.get())
        FuelLength=float(entry27.get())
        ParachuteCD=float(entry28.get())
        NoseCD=float(entry202.get())
        BoosterCD=float(entry203.get())
        ParachuteDiameter=float(entry29.get())
        ParachuteDelay=float(entry30.get())
        NoseMass=float(entry31.get())
        NoseCOMx=float(entry32.get())
        NoseCOMy=float(entry33.get())
        NoseCOMz=float(entry34.get())
        NoseMOIx=float(entry35.get())
        NoseMOIy=float(entry36.get())
        NoseMOIz=float(entry37.get())
        BoosterMass=float(entry38.get())
        BoosterCOMx=float(entry39.get())
        BoosterCOMy=float(entry40.get())
        BoosterCOMz=float(entry41.get())
        BoosterMOIx=float(entry42.get())
        BoosterMOIy=float(entry43.get())
        BoosterMOIz=float(entry44.get())
        NumberRuns=float(entry45.get())
        State=round(float(entry201.get()),0)
        LaunchElevationLower=float(entry46.get())
        LaunchElevationUpper=float(entry47.get())
        LaunchAzimuthLower=float(entry48.get())
        LaunchAzimuthUpper=float(entry49.get())
        ThrustMisalignmentYawLower=float(entry50.get())
        ThrustMisalignmentYawUpper=float(entry51.get())
        ThrustMisalignmentPitchLower=float(entry52.get())
        ThrustMisalignmentPitchUpper=float(entry53.get())
        ThrustMagnitudeLower=float(entry54.get())
        ThrustMagnitudeUpper=float(entry55.get())
        TimeBurnLower=float(entry56.get())
        TimeBurnUpper=float(entry57.get())
        WindMagnitudeLower=float(entry58.get())
        WindMagnitudeUpper=float(entry59.get())
        WindDirectionLower=float(entry60.get())
        WindDirectionUpper=float(entry61.get())
        AerodynamicDragLower=float(entry62.get())
        AerodynamicDragUpper=float(entry63.get())
        AerodynamicLiftLower=float(entry64.get())
        AerodynamicLiftUpper=float(entry65.get())
        AerodynamicMomentLower=float(entry66.get())
        AerodynamicMomentUpper=float(entry67.get())
        CentreOfPressureLower=float(entry68.get())
        CentreOfPressuerUpper=float(entry69.get())
        FinCantAngleLower=float(entry70.get())
        FinCantAngleUpper=float(entry71.get())
        LaunchAltitudeLower=float(entry72.get())
        LaunchAltitudeUpper=float(entry73.get())
        CalculatorTimeSize=float(entry74.get())
        OxidiserDensity=float(entry75.get())
        OxidiserRadius=float(entry76.get())
        OxidiserLength=float(entry77.get())
        OxidiserLengthInitial=float(entry78.get())
        OxidiserCOM=float(entry79.get())
        TimeBurnFuel=float(entry80.get())
        NumberStages=float(entry81.get())
        StageTime=float(entry82.get())
        StageDelay=float(entry83.get())
        StageMass=float(entry84.get())
        StageCOMx=float(entry85.get())
        StageCOMy=float(entry86.get())
        StageCOMz=float(entry87.get())
        StageMOIx=float(entry88.get())
        StageMOIy=float(entry89.get())
        StageMOIz=float(entry90.get())
        NoseRadius=float(entry91.get())
        NoseLength=float(entry92.get())
        FinRootChord=float(entry93.get())
        FinTipChord=float(entry94.get())
        FinSweep=float(entry95.get())
        FinSpan=float(entry96.get())
        FinLocation=float(entry97.get())
        FinSpanRoot=float(entry98.get())
        FinCantAngle=float(entry99.get())
        CheckEarthModelWGS=float(entry205.get())
        CheckTurbulence=float(entry100.get())
        Check4in1=float(entry101.get())
        CheckStaging=float(entry102.get())
        CheckThrust=float(entry103.get())
        CheckRollControl=float(entry104.get())
        CheckMonteCarloUI=float(entry200.get())
        CheckStream=float(entry106.get())
        CheckOrder2=float(entry107.get())
        CheckOrder4=float(entry108.get())
        CheckOrder8=float(entry109.get())
        SolverRelative=float(entry110.get())
        SolverAbsolute=float(entry111.get())
        SolverTimeSizeFirst=float(entry112.get())
        SolverTimeSizeMax=float(entry113.get())
        RollControlLower=float(entry114.get())
        RollControlUpper=float(entry115.get())
        RollControlFrequency=float(entry116.get())
        RollControlForce=float(entry117.get())
        GraphicScale=float(entry118.get())
        TrajectoryResolution=float(entry119.get())
        Directory=entry120.get()
        RollControlTimeInitial=float(entry121.get())
        RollControlTimeFinal=float(entry122.get())
        DrogueCD=float(entry123.get())
        DrogueDiameter=float(entry124.get())
        DrogueDelay=float(entry125.get())
        CombinedCD=float(entry206.get())
        MCDetailed=float(entry207.get())
        CheckMonteCarloUI=float(entry200.get())
        Lug=float(entry204.get())
        ThrustHybrid=float(entry208.get())
        MaxTAOA=float(entry209.get())
        SaveColumns=["Maximum Simulation Time","Time Step Size","Launch Latitude","Launch Longitude","Launch Altitude","Launch Elevation","Launch Azimuth","Rocket Body Radius","Rocket Body Length","Launch Rail Length","Nozzle Exit Area","Thrust Polynomial Degree","MissileDATCOM Cards","SolidWorks Mass","SolidWorks COMx","SolidWorks COMy","SolidWorks COMz","SolidWorks MOIx","SolidWorks MOIy","SolidWorks MOIz","Time Burn","Density Fuel","Radius Fuel","Thickness Fuel","Thickness Fuel Initial","COM Fuel","Length Fuel","CD Parachute","Diameter Parachute","Parachute Deployment Delay","Mass Nose Separated","COMx Nose Separated","COMy Nose Separated","COMz Nose Separated","MOIx Nose Separated","MOIy Nose Separated","MOIz Nose Separated","Mass Booster Separated","COMx Booster Separated","COMy Booster Separated","COMz Booster Separated","MOIx Booster Separated","MOIy Booster Separated","MOIz Booster Separated","Number Runs","Launch Elevation Uncertainty Lower","Launch Elevation Uncertainty Upper","Launch Azimuth Uncertainty Lower","Launch Azimuth Uncertainty Upper","Thrust Misalignment (Yaw) Uncertainty Lower","Thrust Misalignment (Yaw) Uncertainty Upper","Thrust Misalignment (Pitch) Uncertainty Lower","Thrust Misalignment (Pitch) Uncertainty Upper","Thrust Magnitude Lower","Thrust Magnitude Upper","Burn Time Uncertainty Lower","Burn Time Uncertainty Upper","Wind Magnitude Uncertainty Lower","Wind Magnitude Uncertainty Upper","Wind Direction Uncertainty Lower","Wind Direction Uncertainty Upper","Aerodynamic Drag Uncertainty Lower","Aerodynamic Drag Uncertainty Upper","Aerodynamic Lift Uncertainty Lower","Aerodynamic Lift Uncertainty Upper","Aerodynamic Moment Uncertainty Lower","Aerodynamic Moment Uncertainty Upper","Centre of Pressure Uncertainty Lower","Centre of Pressure Uncertainty Upper","Fin Cant Angle Uncertainty Lower","Fin Cant Angle Uncertainty Upper","Launch Altitude Uncertainty Lower","Launch Altitude Uncertainty Upper","Calculator Step Size","Density Oxidiser","Radius Oxidiser","Length Oxidiser","Length Oxidiser Initial","COM Oxidiser","Time Burn Fuel","Number Stages","Stage Separation Time","Stage Separation Delay","Mass Stage Separated","COMx Stage Separated","COMy Stage Separated","COMz Stage Separated","MOIx Stage Separated","MOIy Stage Separated","MOIz Stage Separated","Nosecone Radius","Nosecone Length","Fin Root Chord","Fin Tip Chord","Fin Sweep","Fin Span","Fin Location","Fin Root Span","Fin Cant Angle","WGS Earth Model","Wind Turbulence","4-in-1 Simulation","Staging","Thrust Curve Fit","Roll Control","Monte Carlo UI Input","Stream Full Output","2nd Order Solver","4th Order Solver","8th Order Solver","Relative Tolerance","Absolute Tolerance","First Step Size","Maximum Step Size","Roll Rate Threshold Lower","Roll Rate Threshold Upper","Thrust Pulse Frequency","Thrust Force","OpenGL Graphic Scale","OpenGL Trajectory Resolution","Directory","Roll Control Time Initial","Roll Control Time Final","CD Drogue","Diameter Drogue","Drogue Deployment Delay","State","NoseCD","BoosterCD","CombinedCD","MCDetailed","CheckMonteCarloUI","Lug","ThrustHybrid","MaxTAOA"]
        SaveInputs=pd.DataFrame(columns=SaveColumns)
        SaveInputs.loc[1]=TimeMax,TimeSize,LaunchLatitude,LaunchLongitude,LaunchAltitude,LaunchElevation,LaunchAzimuth,RocketBodyRadius,RocketBodyLength,LaunchRailLength,NozzleExitArea,ThrustPolynomialDegree,MissileDATCOMCards,SolidWorksMass,SolidWorksCOMx,SolidWorksCOMy,SolidWorksCOMz,SolidWorksMOIx,SolidWorksMOIy,SolidWorksMOIz,TimeBurn,FuelDensity,FuelRadius,FuelThickness,FuelThicknessInitial,FuelCOM,FuelLength,ParachuteCD,ParachuteDiameter,ParachuteDelay,NoseMass,NoseCOMx,NoseCOMy,NoseCOMz,NoseMOIx,NoseMOIy,NoseMOIz,BoosterMass,BoosterCOMx,BoosterCOMy,BoosterCOMz,BoosterMOIx,BoosterMOIy,BoosterMOIz,NumberRuns,LaunchElevationLower,LaunchElevationUpper,LaunchAzimuthLower,LaunchAzimuthUpper,ThrustMisalignmentYawLower,ThrustMisalignmentYawUpper,ThrustMisalignmentPitchLower,ThrustMisalignmentPitchUpper,ThrustMagnitudeLower,ThrustMagnitudeUpper,TimeBurnLower,TimeBurnUpper,WindMagnitudeLower,WindMagnitudeUpper,WindDirectionLower,WindDirectionUpper,AerodynamicDragLower,AerodynamicDragUpper,AerodynamicLiftLower,AerodynamicLiftUpper,AerodynamicMomentLower,AerodynamicMomentUpper,CentreOfPressureLower,CentreOfPressuerUpper,FinCantAngleLower,FinCantAngleUpper,LaunchAltitudeLower,LaunchAltitudeUpper,CalculatorTimeSize,OxidiserDensity,OxidiserRadius,OxidiserLength,OxidiserLengthInitial,OxidiserCOM,TimeBurnFuel,NumberStages,StageTime,StageDelay,StageMass,StageCOMx,StageCOMy,StageCOMz,StageMOIx,StageMOIy,StageMOIz,NoseRadius,NoseLength,FinRootChord,FinTipChord,FinSweep,FinSpan,FinLocation,FinSpanRoot,FinCantAngle,CheckEarthModelWGS,CheckTurbulence,Check4in1,CheckStaging,CheckThrust,CheckRollControl,CheckMonteCarloUI,CheckStream,CheckOrder2,CheckOrder4,CheckOrder8,SolverRelative,SolverAbsolute,SolverTimeSizeFirst,SolverTimeSizeMax,RollControlLower,RollControlUpper,RollControlFrequency,RollControlForce,GraphicScale,TrajectoryResolution,Directory,RollControlTimeInitial,RollControlTimeFinal,DrogueCD,DrogueDiameter,DrogueDelay,State,NoseCD,BoosterCD,CombinedCD,MCDetailed,CheckMonteCarloUI,Lug,ThrustHybrid,MaxTAOA
        with open(r"Path.txt") as file:
            directory=file.read()
        SaveInputs.to_excel(r'{}\Inputs\Settings.xlsx'.format(directory),columns=SaveColumns,index=False)

    def Display():
        PlotScalingFactor=2.73
        LocationOffset=0.2
        BoundaryThickness=1000
        RefreshRate=20
        PlotZoom=18000
        
        """East (m):"""        
        x01=15263.5529603298
        
        """North (m):"""       
        y01=-2776.22241539613

        """NominalSplashdown=np.array([x01,y01])"""
        NominalSplashdown=np.array([x01,y01])
        
        """East (m):"""
        x0=14610.0546779195
        
        """North (m):"""
        y0=-2610.71051734057

        """CampaignSplashdown=np.array([x0,y0])"""
        CampaignSplashdown=np.array([x0,y0])

        bulk.MonteCarloPlot(PlotScalingFactor,LocationOffset,BoundaryThickness,NominalSplashdown,CampaignSplashdown,RefreshRate,PlotZoom,Directory)

    def Plot():
        DensityZoom=18000
        bulk.MonteCarloDensity(DensityZoom,Directory)

    def Trajectory():
        scale_user=float(entry118.get())
        interval=100
        trajectory=float(entry119.get())
        view=20000
        land=54000
        state=0
        bulk.GraphicsLibrary(scale_user,interval,trajectory,view,land,state)#,Directory)
        """Open PyOpenGL externally: subprocess.call(r'C:\ASRI_Simulator\body\GraphicsLibrary.pyw', shell=True)"""

    def MassPropertiesCalculator():
        endMass=SolidWorksMass
        endCOM=SolidWorksCOMx
        endMOIx=SolidWorksMOIx
        endMOIy=SolidWorksMOIy
        endMOIz=SolidWorksMOIz
        calculator_t=0
        calculator_t_burn=TimeBurn
        calculator_t_burn_fuel=TimeBurnFuel
        calculator_timestep=CalculatorTimeSize
        rhoFuel=FuelDensity
        RFuel=FuelRadius
        TFuelInitial=FuelThicknessInitial
        TFuel=FuelThickness
        LFuel=FuelLength
        COMFuel=FuelCOM
        rhoOxid=OxidiserDensity
        ROxid=OxidiserRadius
        LOxid=OxidiserLength
        LOxidInitial=OxidiserLengthInitial
        COMOxid=OxidiserCOM
        FactorTimestep=CalculatorTimeSize/0.005
        timestepsFinal=30000/FactorTimestep
        """Keep this value arbitrarily high."""
        """User can adjust this '30000' if more timesteps are needed by simulator."""
        """This is number of timesteps added to dataframe post- part shape variation."""
        """Number of timesteps in exported dataframe containing mass properties must always be > Number of timesteps of simulation."""
        """For full flight duration."""
        bulk.MassPropertiesCalculation(Directory,endMass,endCOM,endMOIx,endMOIy,endMOIz,calculator_t,calculator_t_burn,calculator_t_burn_fuel,calculator_timestep,rhoFuel,RFuel,TFuelInitial,TFuel,LFuel,COMFuel,rhoOxid,ROxid,LOxid,LOxidInitial,COMOxid,timestepsFinal)

    def ThrustCurveFit():
        """Thrust curve approximation. A thrust curve fit is created using a polynomial of degree six, and the polynomial coefficients are contained in the output file."""
        ThrustCurveFitDegree=6
        bulk.ThrustCurveFitUI(ThrustCurveFitDegree,Directory)

    def MissileDATCOM():
        """Missile DATCOM aerodynamic input file format uses 18 rows in each table:"""
        MissileDATCOMCardRows=18
        bulk.MissileDATCOM(MissileDATCOMCardRows,MissileDATCOMCards,Directory)

    def CreateWindFile():
        Wind=pd.read_excel(r'{}\Inputs\wind_from.xlsx'.format(Directory))
        Wind.iloc[:,2]=-(Wind.iloc[:,2]-180)
        Wind.to_excel(r'{}\Inputs\wind.xlsx'.format(Directory),index=False)
        print("PyROPS wind file created.")

    def Simulate():
        def Directory(**kwargs):
            return kwargs

        Inputs=Directory(TimeMax=float(entry1.get()),TimeSize=float(entry2.get()),LaunchLatitude=float(entry3.get()),LaunchLongitude=float(entry4.get()),LaunchAltitude=float(entry5.get()),LaunchElevation=float(entry6.get()),LaunchAzimuth=float(entry7.get()),RocketBodyRadius=float(entry8.get()),RocketBodyLength=float(entry9.get()),LaunchRailLength=float(entry10.get()),NozzleExitArea=float(entry11.get()),ThrustPolynomialDegree=float(entry12.get()),MissileDATCOMCards=float(entry13.get()),SolidWorksMass=float(entry14.get()),SolidWorksCOMx=float(entry15.get()),SolidWorksCOMy=float(entry16.get()),SolidWorksCOMz=float(entry17.get()),SolidWorksMOIx=float(entry18.get()),SolidWorksMOIy=float(entry19.get()),SolidWorksMOIz=float(entry20.get()),TimeBurn=float(entry21.get()),FuelDensity=float(entry22.get()),FuelRadius=float(entry23.get()),FuelThickness=float(entry24.get()),FuelThicknessInitial=float(entry25.get()),FuelCOM=float(entry26.get()),FuelLength=float(entry27.get()),ParachuteCD=float(entry28.get()),ParachuteDiameter=float(entry29.get()),ParachuteDelay=float(entry30.get()),NoseMass=float(entry31.get()),NoseCOMx=float(entry32.get()),NoseCOMy=float(entry33.get()),NoseCOMz=float(entry34.get()),NoseMOIx=float(entry35.get()),NoseMOIy=float(entry36.get()),NoseMOIz=float(entry37.get()),BoosterMass=float(entry38.get()),BoosterCOMx=float(entry39.get()),BoosterCOMy=float(entry40.get()),BoosterCOMz=float(entry41.get()),BoosterMOIx=float(entry42.get()),BoosterMOIy=float(entry43.get()),BoosterMOIz=float(entry44.get()),NumberRuns=float(entry45.get()),LaunchElevationLower=float(entry46.get()),LaunchElevationUpper=float(entry47.get()),LaunchAzimuthLower=float(entry48.get()),LaunchAzimuthUpper=float(entry49.get()),ThrustMisalignmentYawLower=float(entry50.get()),ThrustMisalignmentYawUpper=float(entry51.get()),ThrustMisalignmentPitchLower=float(entry52.get()),ThrustMisalignmentPitchUpper=float(entry53.get()),ThrustMagnitudeLower=float(entry54.get()),ThrustMagnitudeUpper=float(entry55.get()),TimeBurnLower=float(entry56.get()),TimeBurnUpper=float(entry57.get()),WindMagnitudeLower=float(entry58.get()),WindMagnitudeUpper=float(entry59.get()),WindDirectionLower=float(entry60.get()),WindDirectionUpper=float(entry61.get()),AerodynamicDragLower=float(entry62.get()),AerodynamicDragUpper=float(entry63.get()),AerodynamicLiftLower=float(entry64.get()),AerodynamicLiftUpper=float(entry65.get()),AerodynamicMomentLower=float(entry66.get()),AerodynamicMomentUpper=float(entry67.get()),CentreOfPressureLower=float(entry68.get()),CentreOfPressuerUpper=float(entry69.get()),FinCantAngleLower=float(entry70.get()),FinCantAngleUpper=float(entry71.get()),LaunchAltitudeLower=float(entry72.get()),LaunchAltitudeUpper=float(entry73.get()),CalculatorTimeSize=float(entry74.get()),OxidiserDensity=float(entry75.get()),OxidiserRadius=float(entry76.get()),OxidiserLength=float(entry77.get()),OxidiserLengthInitial=float(entry78.get()),OxidiserCOM=float(entry79.get()),TimeBurnFuel=float(entry80.get()),NumberStages=float(entry81.get()),StageTime=float(entry82.get()),StageDelay=float(entry83.get()),StageMass=float(entry84.get()),StageCOMx=float(entry85.get()),StageCOMy=float(entry86.get()),StageCOMz=float(entry87.get()),StageMOIx=float(entry88.get()),StageMOIy=float(entry89.get()),StageMOIz=float(entry90.get()),NoseRadius=float(entry91.get()),NoseLength=float(entry92.get()),FinRootChord=float(entry93.get()),FinTipChord=float(entry94.get()),FinSweep=float(entry95.get()),FinSpan=float(entry96.get()),FinLocation=float(entry97.get()),FinSpanRoot=float(entry98.get()),FinCantAngle=float(entry99.get()),CheckEarthModelWGS=float(entry205.get()),CheckTurbulence=float(entry100.get()),Check4in1=float(entry101.get()),CheckStaging=float(entry102.get()),CheckThrust=float(entry103.get()),CheckRollControl=float(entry104.get()),CheckMonteCarloUI=float(entry200.get()),CheckStream=float(entry106.get()),CheckOrder2=float(entry107.get()),CheckOrder4=float(entry108.get()),CheckOrder8=float(entry109.get()),SolverRelative=float(entry110.get()),SolverAbsolute=float(entry111.get()),SolverTimeSizeFirst=float(entry112.get()),SolverTimeSizeMax=float(entry113.get()),RollControlLower=float(entry114.get()),RollControlUpper=float(entry115.get()),RollControlFrequency=float(entry116.get()),RollControlForce=float(entry117.get()),GraphicScale=float(entry118.get()),TrajectoryResolution=float(entry119.get()),Directory=entry120.get(),RollControlTimeInitial=float(entry121.get()),RollControlTimeFinal=float(entry122.get()),DrogueCD=float(entry123.get()),DrogueDiameter=float(entry124.get()),DrogueDelay=float(entry125.get()),NoseCD=float(entry202.get()),BoosterCD=float(entry203.get()),CombinedCD=float(entry206.get()),MCDetailed=float(entry207.get()),Lug=float(entry204.get()),ThrustHybrid=float(entry208.get()),MaxTAOA=float(entry209.get()))
        CheckMonteCarloUI=float(entry200.get())
        if CheckMonteCarloUI==0:
            Inputs=Directory(TimeMax=float(entry1.get()),TimeSize=float(entry2.get()),LaunchLatitude=float(entry3.get()),LaunchLongitude=float(entry4.get()),LaunchAltitude=float(entry5.get()),LaunchElevation=float(entry6.get()),LaunchAzimuth=float(entry7.get()),RocketBodyRadius=float(entry8.get()),RocketBodyLength=float(entry9.get()),LaunchRailLength=float(entry10.get()),NozzleExitArea=float(entry11.get()),ThrustPolynomialDegree=float(entry12.get()),MissileDATCOMCards=float(entry13.get()),SolidWorksMass=float(entry14.get()),SolidWorksCOMx=float(entry15.get()),SolidWorksCOMy=float(entry16.get()),SolidWorksCOMz=float(entry17.get()),SolidWorksMOIx=float(entry18.get()),SolidWorksMOIy=float(entry19.get()),SolidWorksMOIz=float(entry20.get()),TimeBurn=float(entry21.get()),FuelDensity=float(entry22.get()),FuelRadius=float(entry23.get()),FuelThickness=float(entry24.get()),FuelThicknessInitial=float(entry25.get()),FuelCOM=float(entry26.get()),FuelLength=float(entry27.get()),ParachuteCD=float(entry28.get()),ParachuteDiameter=float(entry29.get()),ParachuteDelay=float(entry30.get()),NoseMass=float(entry31.get()),NoseCOMx=float(entry32.get()),NoseCOMy=float(entry33.get()),NoseCOMz=float(entry34.get()),NoseMOIx=float(entry35.get()),NoseMOIy=float(entry36.get()),NoseMOIz=float(entry37.get()),BoosterMass=float(entry38.get()),BoosterCOMx=float(entry39.get()),BoosterCOMy=float(entry40.get()),BoosterCOMz=float(entry41.get()),BoosterMOIx=float(entry42.get()),BoosterMOIy=float(entry43.get()),BoosterMOIz=float(entry44.get()),NumberRuns=float(1),LaunchElevationLower=float(entry46.get()),LaunchElevationUpper=float(entry47.get()),LaunchAzimuthLower=float(entry48.get()),LaunchAzimuthUpper=float(entry49.get()),ThrustMisalignmentYawLower=float(entry50.get()),ThrustMisalignmentYawUpper=float(entry51.get()),ThrustMisalignmentPitchLower=float(entry52.get()),ThrustMisalignmentPitchUpper=float(entry53.get()),ThrustMagnitudeLower=float(entry54.get()),ThrustMagnitudeUpper=float(entry55.get()),TimeBurnLower=float(entry56.get()),TimeBurnUpper=float(entry57.get()),WindMagnitudeLower=float(entry58.get()),WindMagnitudeUpper=float(entry59.get()),WindDirectionLower=float(entry60.get()),WindDirectionUpper=float(entry61.get()),AerodynamicDragLower=float(entry62.get()),AerodynamicDragUpper=float(entry63.get()),AerodynamicLiftLower=float(entry64.get()),AerodynamicLiftUpper=float(entry65.get()),AerodynamicMomentLower=float(entry66.get()),AerodynamicMomentUpper=float(entry67.get()),CentreOfPressureLower=float(entry68.get()),CentreOfPressuerUpper=float(entry69.get()),FinCantAngleLower=float(entry70.get()),FinCantAngleUpper=float(entry71.get()),LaunchAltitudeLower=float(entry72.get()),LaunchAltitudeUpper=float(entry73.get()),CalculatorTimeSize=float(entry74.get()),OxidiserDensity=float(entry75.get()),OxidiserRadius=float(entry76.get()),OxidiserLength=float(entry77.get()),OxidiserLengthInitial=float(entry78.get()),OxidiserCOM=float(entry79.get()),TimeBurnFuel=float(entry80.get()),NumberStages=float(entry81.get()),StageTime=float(entry82.get()),StageDelay=float(entry83.get()),StageMass=float(entry84.get()),StageCOMx=float(entry85.get()),StageCOMy=float(entry86.get()),StageCOMz=float(entry87.get()),StageMOIx=float(entry88.get()),StageMOIy=float(entry89.get()),StageMOIz=float(entry90.get()),NoseRadius=float(entry91.get()),NoseLength=float(entry92.get()),FinRootChord=float(entry93.get()),FinTipChord=float(entry94.get()),FinSweep=float(entry95.get()),FinSpan=float(entry96.get()),FinLocation=float(entry97.get()),FinSpanRoot=float(entry98.get()),FinCantAngle=float(entry99.get()),CheckEarthModelWGS=float(entry205.get()),CheckTurbulence=float(entry100.get()),Check4in1=float(entry101.get()),CheckStaging=float(entry102.get()),CheckThrust=float(entry103.get()),CheckRollControl=float(entry104.get()),CheckMonteCarloUI=float(entry200.get()),CheckStream=float(entry106.get()),CheckOrder2=float(entry107.get()),CheckOrder4=float(entry108.get()),CheckOrder8=float(entry109.get()),SolverRelative=float(entry110.get()),SolverAbsolute=float(entry111.get()),SolverTimeSizeFirst=float(entry112.get()),SolverTimeSizeMax=float(entry113.get()),RollControlLower=float(entry114.get()),RollControlUpper=float(entry115.get()),RollControlFrequency=float(entry116.get()),RollControlForce=float(entry117.get()),GraphicScale=float(entry118.get()),TrajectoryResolution=float(entry119.get()),Directory=entry120.get(),RollControlTimeInitial=float(entry121.get()),RollControlTimeFinal=float(entry122.get()),DrogueCD=float(entry123.get()),DrogueDiameter=float(entry124.get()),DrogueDelay=float(entry125.get()),NoseCD=float(entry202.get()),BoosterCD=float(entry203.get()),CombinedCD=float(entry206.get()),MCDetailed=float(entry207.get()),Lug=float(entry204.get()),ThrustHybrid=float(entry208.get()),MaxTAOA=float(entry209.get()))
        MonteCarloInputs=Directory(LaunchElevationLower=float(entry46.get()),LaunchElevationUpper=float(entry47.get()),LaunchAzimuthLower=float(entry48.get()),LaunchAzimuthUpper=float(entry49.get()),ThrustMisalignmentYawLower=float(entry50.get()),ThrustMisalignmentYawUpper=float(entry51.get()),ThrustMisalignmentPitchLower=float(entry52.get()),ThrustMisalignmentPitchUpper=float(entry53.get()),ThrustMagnitudeLower=float(entry54.get()),ThrustMagnitudeUpper=float(entry55.get()),TimeBurnLower=float(entry56.get()),TimeBurnUpper=float(entry57.get()),WindMagnitudeLower=float(entry58.get()),WindMagnitudeUpper=float(entry59.get()),WindDirectionLower=float(entry60.get()),WindDirectionUpper=float(entry61.get()),AerodynamicDragLower=float(entry62.get()),AerodynamicDragUpper=float(entry63.get()),AerodynamicLiftLower=float(entry64.get()),AerodynamicLiftUpper=float(entry65.get()),AerodynamicMomentLower=float(entry66.get()),AerodynamicMomentUpper=float(entry67.get()),CentreOfPressureLower=float(entry68.get()),CentreOfPressureUpper=float(entry69.get()),FinCantAngleLower=float(entry70.get()),FinCantAngleUpper=float(entry71.get()),LaunchAltitudeLower=float(entry72.get()),LaunchAltitudeUpper=float(entry73.get()))
        AerodynamicBallistic=bulk.aerodynamic_tables(r'{}\Inputs\RasAeroII.xlsx'.format(entry120.get()),r'{}\Inputs\RasAeroII15.xlsx'.format(entry120.get()))
        AerodynamicNose=bulk.aerodynamic_tables(r'{}\Inputs\RasAeroIINose.xlsx'.format(entry120.get()),r'{}\Inputs\RasAeroIINose15.xlsx'.format(entry120.get()))
        AerodynamicBooster=bulk.aerodynamic_tables(r'{}\Inputs\RasAeroIIBooster.xlsx'.format(entry120.get()),r'{}\Inputs\RasAeroIIBooster15.xlsx'.format(entry120.get()))
        TransformFrame=bulk.transform_frame(np.array([0,0,0],dtype=float))
        ThrustFit=bulk.thrust_curve(10,entry120.get())
        WindInput=bulk.wind_vector.read_excel(entry120.get())

        MonteCarloExcelInput=False
        """
        if CheckMonteCarloUI==1:
            MonteCarloExcelInput=False
        else:
            MonteCarloExcelInput=True
        """

        """Automated four-in-one 'recovery' body simulation:"""
        Check4in1=float(entry101.get())
        if Check4in1==1:
            FourInOneSimulation=True
            print("Four-In-One Simulation Selected")
        else:
            FourInOneSimulation=False

        RunNumber=1
        NumberRuns=float(entry45.get())
        if CheckMonteCarloUI==0:
            NumberRuns=1

        """If a previous Monte Carlo output file exists, delete it."""
        MonteCarloFile=os.path.isfile(r'{}\Outputs\Monte Carlo.xlsx'.format(Directory))
        if MonteCarloFile==True:
            os.remove(r'{}\Outputs\Monte Carlo.xlsx'.format(Directory))
        
        if (Inputs["CheckOrder2"]==False) and (Inputs["CheckOrder4"]==False) and (Inputs["CheckOrder8"]==False):
            print("Run Number:",RunNumber)
            """State=1"""
            State=round(float(entry201.get()),0)
            Dispersion=bulk.monte_carlo.random(MonteCarloInputs,r'{}\Inputs\monte_carlo.xlsx'.format(Directory),MonteCarloExcelInput)
            Simulation=bulk.fixed_step_solver(AerodynamicBallistic,TransformFrame,ThrustFit,Dispersion,WindInput,AerodynamicNose,AerodynamicBooster)
            Simulation.run(Inputs["TimeMax"],Inputs["TimeSize"],Inputs,State)
            RunNumber+=1
            while RunNumber<=NumberRuns:
                if FourInOneSimulation==True:
                    if State<4:
                        print("Run number:",RunNumber)
                        State+=1
                        Dispersion=bulk.monte_carlo.random(MonteCarloInputs,r'{}\Inputs\monte_carlo.xlsx'.format(Directory),MonteCarloExcelInput)
                        Simulation=bulk.fixed_step_solver(AerodynamicBallistic,TransformFrame,ThrustFit,Dispersion,WindInput,AerodynamicNose,AerodynamicBooster)
                        Simulation.run(Inputs["TimeMax"],Inputs["TimeSize"],Inputs,State)
                        MonteCarloPlotRun()
                        RunNumber+=1
                    elif State==4:
                        print("Run number:",RunNumber)
                        State=1
                        Dispersion=bulk.monte_carlo.random(MonteCarloInputs,r'{}\Inputs\monte_carlo.xlsx'.format(Directory),MonteCarloExcelInput)
                        Simulation=bulk.fixed_step_solver(AerodynamicBallistic,TransformFrame,ThrustFit,Dispersion,WindInput,AerodynamicNose,AerodynamicBooster)
                        Simulation.run(Inputs["TimeMax"],Inputs["TimeSize"],Inputs,State)
                        MonteCarloPlotRun()
                        RunNumber+=1
                else:
                    print("Run number:",RunNumber)
                    Dispersion=bulk.monte_carlo.random(MonteCarloInputs,r'{}\Inputs\monte_carlo.xlsx'.format(Directory),MonteCarloExcelInput)
                    Simulation=bulk.fixed_step_solver(AerodynamicBallistic,TransformFrame,ThrustFit,Dispersion,WindInput,AerodynamicNose,AerodynamicBooster)
                    Simulation.run(Inputs["TimeMax"],Inputs["TimeSize"],Inputs,State)
                    MonteCarloPlotRun()
                    RunNumber+=1
            print("Simulation Completed")

        else:
            print("Run Number:",RunNumber)
            """State=1"""
            State=round(float(entry201.get()),0)
            Dispersion=bulk.monte_carlo.random(MonteCarloInputs,r'{}\Inputs\monte_carlo.xlsx'.format(Directory),MonteCarloExcelInput)
            Simulation=bulk.main(AerodynamicBallistic,TransformFrame,ThrustFit,Dispersion,WindInput,AerodynamicNose,AerodynamicBooster)
            Simulation.run(Inputs,TimeMax,State)
            
            RunNumber+=1
            while RunNumber<=NumberRuns:
                if FourInOneSimulation==True:
                    if State<4:
                        print("Run number:",RunNumber)
                        #State+=1
                        Dispersion=bulk.monte_carlo.random(MonteCarloInputs,r'{}\Inputs\monte_carlo.xlsx'.format(Directory),MonteCarloExcelInput)
                        Simulation=bulk.main(AerodynamicBallistic,TransformFrame,ThrustFit,Dispersion,WindInput,AerodynamicNose,AerodynamicBooster)
                        Simulation.run(Inputs,TimeMax,State)
                        MonteCarloPlotRun()
                        RunNumber+=1
                    elif State==4:
                        print("Run number:",RunNumber)
                        #State=1
                        Dispersion=bulk.monte_carlo.random(MonteCarloInputs,r'{}\Inputs\monte_carlo.xlsx'.format(Directory),MonteCarloExcelInput)
                        Simulation=bulk.main(AerodynamicBallistic,TransformFrame,ThrustFit,Dispersion,WindInput,AerodynamicNose,AerodynamicBooster)
                        Simulation.run(Inputs,TimeMax,State)
                        MonteCarloPlotRun()
                        RunNumber+=1
                else:
                    print("Run number:",RunNumber)
                    Dispersion=bulk.monte_carlo.random(MonteCarloInputs,r'{}\Inputs\monte_carlo.xlsx'.format(Directory),MonteCarloExcelInput)
                    Simulation=bulk.main(AerodynamicBallistic,TransformFrame,ThrustFit,Dispersion,WindInput,AerodynamicNose,AerodynamicBooster)
                    Simulation.run(Inputs,TimeMax,State)
                    MonteCarloPlotRun()
                    RunNumber+=1
            print("Simulation Completed")

    def MonteCarloPlotRun():
        """Monte Carlo Map:"""
        print("<display OTR map showing splashdown points, updated after each Monte Carlo run>")

    button=Button(launch,text="Load Settings",width=13,height=2,bg='lightgray',fg='black',command=lambda:LoadInputs()).place(x=1130,y=30)
    button=Button(launch,text="Save Settings",width=13,height=2,bg='lightgray',fg='black',command=lambda:SaveInputs()).place(x=1235,y=30+0*45)
    button=Button(launch,text="Refresh",width=28,height=2,bg='lightgray',fg='black',command=lambda:Refresh()).place(x=1130,y=30+1*45)
    button=Button(launch,text="Simulate",width=28,height=2,bg='lightgray',fg='black',command=lambda:Simulate()).place(x=1130,y=30+2*45)
    button=Button(launch,text="Dispersion Map",width=28,height=2,bg='lightgray',fg='black',command=lambda:Display()).place(x=1130,y=30+3*45)
    button=Button(launch,text="Dispersion Plots",width=28,height=2,bg='lightgray',fg='black',command=lambda:Plot()).place(x=1130,y=30+4*45)
    button=Button(launch,text="Graphics",width=28,height=2,bg='lightgray',fg='black',command=lambda:Trajectory()).place(x=1130,y=30+5*45)
    button=Button(launch,text="Mass Properties Calculator",width=28,height=2,bg='lightgray',fg='black',command=lambda:MassPropertiesCalculator()).place(x=1130,y=30+6*45)
    button=Button(launch,text="Thrust Curve Fit",width=28,height=2,bg='lightgray',fg='black',command=lambda:ThrustCurveFit()).place(x=1130,y=30+7*45)
    button=Button(launch,text="Missile DATCOM",width=28,height=2,bg='lightgray',fg='black',command=lambda:MissileDATCOM()).place(x=1130,y=30+8*45)
    button=Button(launch,text="Create Wind File",width=28,height=2,bg='lightgray',fg='black',command=lambda:CreateWindFile()).place(x=1130,y=30+9*45)
    button=Button(launch,text="Quit",width=28,height=2,bg='lightgray',fg='black',command=lambda:Exit()).place(x=1130,y=30+10*45)
    launch.mainloop

menu = Menu(root)
root.config(menu=menu)
submenu = Menu(menu)
menu.add_cascade(label="File",menu=submenu)
submenu.add_command(label="Load",command=LoadFile)
submenu.add_command(label="Save",command=SaveFile)
submenu.add_command(label="Exit",command=Exit)
button=Button(root,text="Start",width=68,height=2,bg='lightgray',fg='black',command=Start).place(x=0,y=294)
button=Button(root,text="Website",width=68,height=2,bg='lightgray',fg='black',command=Website).place(x=0,y=337)
button=Button(root,text="Support",width=68,height=2,bg='lightgray',fg='black',command=Support).place(x=0,y=380)
button=Button(root,text="Quit",width=68,height=2,bg='lightgray',fg='black',command=exit).place(x=0,y=423)
photo=ImageTk.PhotoImage(Image.open(r"ASRI.jpg"))
image=Label(image=photo)
image.place(x=0,y=0)
root.mainloop()
