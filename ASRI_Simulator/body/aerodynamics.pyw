import pandas as pd
import numpy as np
import scipy.interpolate
import warnings
warnings.filterwarnings("ignore")

#A=aerodynamic_tables(r'C:\Users\user\Desktop\Inputs\RasAeroII.xlsx',r'C:\Users\user\Desktop\Inputs\RasAeroII15.xlsx')
#A.lookup15(2,0.02)

class aerodynamic_tables:
    def __init__(self,directory,directory15):
        self.directory=directory
        self.directory15=directory15

        df=pd.read_excel(self.directory,header=0,engine='openpyxl')
        ##array_CA=np.append(array_CA,array_CA[-1]) #size, for unedited .csv file having full Mach array
        ##array_CN=np.append(array_CN,array_CN[-1]) #size, for unedited .csv file having full Mach array
        ##array_cop=np.append(array_cop,array_cop[-1]) #size, for unedited .csv file having full Mach array
        array_mach=df.iloc[:,0].values
        array_mach=np.append(array_mach,array_mach[-1])
        self.array_mach_unique=np.unique(array_mach)
        #array_mach_unique=np.arange(0,5,0.01)
        array_alpha=df.iloc[:,1].values
        array_alpha=np.append(array_alpha,array_alpha[-1])
        array_alpha=np.array(array_alpha)*np.pi/180
        self.array_alpha_unique=np.unique(array_alpha)
        #array_alpha_unique=np.arange(0,6,2)

        array_CA=df.iloc[:,5].values
        array_CN=df.iloc[:,8].values
        array_cop=df.iloc[:,12].values
        array_cop=np.array(array_cop)*0.0254 #inches to metres
        self.CA_lookupmatrix=np.reshape(array_CA,(len(self.array_alpha_unique),len(self.array_mach_unique)))
        self.CN_lookupmatrix=np.reshape(array_CN,(len(self.array_alpha_unique),len(self.array_mach_unique)))
        self.cop_lookupmatrix=np.reshape(array_cop,(len(self.array_alpha_unique),len(self.array_mach_unique)))

        df15=pd.read_excel(self.directory15,header=0,engine='openpyxl')
        array_mach15=df15.iloc[:,0].values
        array_mach15=np.append(array_mach15,array_mach15[-1])
        self.array_mach_unique15=np.unique(array_mach15)
        array_alpha15=df15.iloc[:,1].values
        array_alpha15=np.append(array_alpha15,array_alpha15[-1])
        array_alpha15=np.array(array_alpha15)*np.pi/180
        self.array_alpha_unique15=np.unique(array_alpha15)

        array_CA15=df15.iloc[:,5].values
        array_CN15=df15.iloc[:,8].values
        array_cop15=df15.iloc[:,12].values
        array_cop15=np.array(array_cop15)*0.0254 #inches to metres
        self.CA_lookupmatrix15=np.reshape(array_CA15,(len(self.array_alpha_unique15),len(self.array_mach_unique15)))
        self.CN_lookupmatrix15=np.reshape(array_CN15,(len(self.array_alpha_unique15),len(self.array_mach_unique15)))
        self.cop_lookupmatrix15=np.reshape(array_cop15,(len(self.array_alpha_unique15),len(self.array_mach_unique15)))




    def lookup(self,mach,taoa):
        self.mach=mach
        self.taoa=taoa

        CA_lookup=scipy.interpolate.interp2d(self.array_mach_unique,self.array_alpha_unique,self.CA_lookupmatrix) #keyword argument: ,kind='linear')
        CN_lookup=scipy.interpolate.interp2d(self.array_mach_unique,self.array_alpha_unique,self.CN_lookupmatrix)
        cop_lookup=scipy.interpolate.interp2d(self.array_mach_unique,self.array_alpha_unique,self.cop_lookupmatrix)

        RASAero=np.array([CA_lookup(self.mach,self.taoa),CN_lookup(self.mach,self.taoa),cop_lookup(self.mach,self.taoa)],dtype=float)
        return RASAero
        
        #CA_lookup=scipy.interpolate.RectBivariateSpline(array_mach_unique,array_alpha_unique,CA_lookupmatrix) #,kind='linear')
        #CA_lookup=scipy.interpolate.RegularGridInterpolator((array_mach_unique,array_alpha_unique),CA_lookupmatrix)

    def lookup15(self,mach,taoa):
        self.mach=mach
        self.taoa=taoa

        CA_lookup15=scipy.interpolate.interp2d(self.array_mach_unique15,self.array_alpha_unique15,self.CA_lookupmatrix15)
        CN_lookup15=scipy.interpolate.interp2d(self.array_mach_unique15,self.array_alpha_unique15,self.CN_lookupmatrix15)
        cop_lookup15=scipy.interpolate.interp2d(self.array_mach_unique15,self.array_alpha_unique15,self.cop_lookupmatrix15)

        RASAero15=np.array([CA_lookup15(self.mach,self.taoa),CN_lookup15(self.mach,self.taoa),cop_lookup15(self.mach,self.taoa)],dtype=float)
        return RASAero15
