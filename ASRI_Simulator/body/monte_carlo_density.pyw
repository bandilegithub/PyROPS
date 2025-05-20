import math
from matplotlib.pyplot import figure
import pandas as pd
import numpy as np
from scipy import stats as st
from scipy.stats import gaussian_kde
import matplotlib.pyplot as plt

def MonteCarloDensity(DensityZoom,Directory):
     df = pd.read_excel(r'{}\Outputs\Monte Carlo.xlsx'.format(Directory),header=0)
     df.columns=['North (metres)','East (metres)','Apogee (metres)']
     x=df.loc[:,"North (metres)"].values
     y=df.loc[:,"East (metres)"].values
     xmin,xmax,ymin,ymax=-DensityZoom,DensityZoom,-DensityZoom,DensityZoom
     xx, yy = np.mgrid[xmin:xmax:100j, ymin:ymax:100j]
     positions = np.vstack([xx.ravel(), yy.ravel()])
     values = np.vstack([x, y])
     kernel = st.gaussian_kde(values)
     f = np.reshape(kernel(positions).T, xx.shape)
     fig = plt.figure(figsize=(10,10))
     """Alternatively, use .set_figwidth(30)#.set_figheight(30)"""
     ax = fig.gca()
     """ax.set_facecolor('black')"""
     cfset = ax.contourf(xx, yy, f, cmap= 'viridis',levels=30)

     plt.axis('off')
     plt.savefig(r'C:\ASRI_Simulator\body\MonteCarloDensity1.png', dpi=100,bbox_inches='tight',pad_inches=0)
     plt.savefig(r'C:\ASRI_Simulator\body\graphics\map2.png', dpi=100,bbox_inches='tight',pad_inches=0)
     plt.show()

     xy = np.vstack([x,y])
     """Calculate point density:"""
     z = gaussian_kde(xy)(xy)
     fig, ax2 = plt.subplots(figsize=(10,10))
     ax2.scatter(x, y, c=z, s=10)
     """fig.set_facecolor('black')"""

     plt.yticks(np.linspace(-DensityZoom, DensityZoom, 11))
     plt.xticks(np.linspace(-DensityZoom, DensityZoom, 11))
     plt.axis('off')
     plt.savefig(r'C:\ASRI_Simulator\body\MonteCarloDensity2.png', dpi=100,bbox_inches='tight',pad_inches=0,transparent=True)
     plt.savefig(r'C:\ASRI_Simulator\body\graphics\map3.png', dpi=100,bbox_inches='tight',pad_inches=0,transparent=True)
     plt.show()
