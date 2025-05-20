import os
import pandas as pd
import numpy as np
import math
import csv
import json
from scipy.spatial.transform import Rotation
from scipy import stats

class wind_vector:
    def __init__(self,altitude_array,magnitude1_array,magnitude2_array):
        self.altitude_array=altitude_array
        self.magnitude1_array=magnitude1_array
        self.magnitude2_array=magnitude2_array

    def interpolate(self,height):
        self.height=height
        self.wind_magnitude_1=(np.interp(self.height, self.altitude_array, self.magnitude1_array))
        self.wind_magnitude_2=(np.interp(self.height, self.altitude_array, self.magnitude2_array))
        self.WindAngle=math.atan2(self.wind_magnitude_2,self.wind_magnitude_1)
        return self.wind_magnitude_1,self.wind_magnitude_2,self.WindAngle

    @staticmethod
    def lookup(table):
        altitude_array_1=table.iloc[:,0].values
        magnitude_array_2=table.iloc[:,1].values
        magnitude_array_3=table.iloc[:,2].values
        return wind_vector(altitude_array_1,magnitude_array_2,magnitude_array_3)

    @staticmethod
    def coordinate_transform(vector):
        dataframe=pd.DataFrame()
        dataframe["altitude (m)"]=vector["altitude (m)"]
        dataframe["magnitude1"]=0
        dataframe["magnitude2"]=0
        wind_counter=0
        wind_counter_max=len(dataframe)
        while wind_counter<(wind_counter_max-1):
            dataframe.at[wind_counter,"magnitude1"]=vector.at[wind_counter,"magnitude (m/s)"]*math.cos(math.radians(vector.at[wind_counter,"bearing (degrees)"]))
            dataframe.at[wind_counter,"magnitude2"]=-vector.at[wind_counter,"magnitude (m/s)"]*math.sin(math.radians(vector.at[wind_counter,"bearing (degrees)"]))
            wind_counter+=1
        return wind_vector.lookup(dataframe)
    
    @staticmethod
    def read_excel(directory):
        dataframe=pd.read_excel(r'{}\Inputs\wind.xlsx'.format(directory),header=0) # Wind bearing input uses HYROPS launch azimuth convention (counter-clockwise is positive)
        return wind_vector.coordinate_transform(dataframe)

    def jetstream(self,alti,JetVelocity=30,JetDirection=90,JetFloor=10000,JetCeiling=15000):
        self.jetstream=np.array([0,0,0],dtype=float)
        if alti>=JetFloor and alti<=JetCeiling:
            self.jetstream=np.array([JetVelocity*(1-( (alti-(JetCeiling+JetFloor)/2)/((JetCeiling-JetFloor)/2) )**2),0,0],dtype=float)
            rotationJetStream=Rotation.from_euler('xyz',[0,0,JetDirection],degrees=True)
            self.jetstream=rotationJetStream.apply(self.jetstream)
        return np.array([self.jetstream[0],self.jetstream[1],self.jetstream[2]],dtype=float)

    def superposition(turb,alti,Vamag,TurbulenceLength=150,TurbulenceSigma=10):#superposition(turb,alti,Vamag,wind_vector,TurbulenceLength=150,TurbulenceSigma=10):
        tsigma=TurbulenceSigma
        tlengt=TurbulenceLength
        turvec=np.array([0,0,0],dtype=float)
        if Vamag>0:
            turvec[2]=turb[0]+( (math.sqrt(3)*(tlengt/Vamag))*turb[1] )
            turvec[2]=turvec[2]*(tsigma*math.sqrt(tlengt/2*math.pi*Vamag))
        Jet=True
        jetvec=np.array([0,0,0],dtype=float)
        if Jet==True:
            jetvec=wind_vector([],[],[]).jetstream(alti)
            turvec+=jetvec
        return np.array([turvec[0],turvec[1],turvec[2]],dtype=float)

    @staticmethod
    def parameters(dt,vamag,alti,TurbulenceLength=150):#parameters(dt,vamag,alti,wind_vector,TurbulenceLength=150):
        tlengt=TurbulenceLength
        if alti>0:
            a=(-0.5-0)/1.1
            b=(0.5-0)/1.1
            low,high,mean,stddev,num_pop=a,b,0,1.1,1
            normalised_wind=stats.truncnorm.rvs(low,high,loc=mean,scale=stddev,size=num_pop)
            TurbulenceRandom=normalised_wind[0]

            path=r"Turbulence.txt"
            exist = os.path.isfile(path)
            if exist==False:
                file1=open("Turbulence.txt","w")
                file1.write(str('{'+'"Turbulence[0]":{},"Turbulence[1]":{}'.format(0,0)+'}'))
                file1.close()         

            """Read previous turbulence point:"""
            file1=open("Turbulence.txt","r")
            TurbulencePrevious=json.loads(str(file1.readlines()[0]))
            file1.close()

            turb=np.array([TurbulencePrevious["Turbulence[0]"],TurbulencePrevious["Turbulence[1]"]],dtype=float)
            dtur=np.array([0,0],dtype=float)
            dtur[0]=turb[1]
            dtur[1]=-(((vamag/tlengt)**2)*turb[0])-(2*(vamag/tlengt)*turb[1])+(TurbulenceRandom*(vamag/tlengt)**2)
            turb[0]+=dt*dtur[0]
            turb[1]+=dt*dtur[1]
            
            """Write turbulence point:"""
            file1 = open("Turbulence.txt","w")
            file1.write(str('{'+'"Turbulence[0]":{},"Turbulence[1]":{}'.format(turb[0],turb[1])+'}'))
            file1.close()
        else:
            turb=np.array([0,0],dtype=float)
        return wind_vector.superposition(turb,alti,vamag)#wind_vector.superposition(turb,alti,vamag,wind_vector)




        
