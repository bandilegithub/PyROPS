from scipy.integrate import RK45, DOP853, solve_ivp, odeint
import scipy.interpolate
import numpy as np
import os
import time
import os
import csv
import sys
import json
import os.path
import time
import numpy as np
import pandas as pd
import scipy.interpolate as interpolate
import scipy.misc
import scipy.integrate as integrate
from scipy.spatial.transform import Rotation
from datetime import *
import math
import random as random
from subprocess import Popen, PIPE, STDOUT
from PIL import Image
import random as random
from scipy import stats
import statistics
from scipy.stats import norm
import matplotlib.pyplot as plt
import warnings
warnings.filterwarnings("ignore")
pd.set_option('display.max_columns', None)
pd.options.display.width=0

class fixed_step_solver:
    def __init__(self,aerodynamic_tables,transform_frame,thrust_curve,monte_carlo,wind_vector,aerodynamic_tables_nose,aerodynamic_tables_booster):
        self.aerodynamic_tables=aerodynamic_tables
        self.transform_frame=transform_frame
        self.thrust_curve=thrust_curve
        self.monte_carlo=monte_carlo
        self.wind_vector=wind_vector
        self.aerodynamic_tables_nose=aerodynamic_tables_nose
        self.aerodynamic_tables_booster=aerodynamic_tables_booster
        
    def run(self,TimeMax,TimeSize,input_values,BodyState):
        self.TimeMax=TimeMax
        self.TimeSize=TimeSize
        self.input_values=input_values
        self.BodyState=BodyState
        print("Body State: {:.0f}".format(self.BodyState))

        global KinematicVelocityMagnitude,DragAccelerationMagnitudeX,DragAccelerationMagnitudeY,DragAccelerationMagnitudeZ,Drag,Gravity,DragAccelerationX,DragAccelerationY,DragAccelerationZ
        global t,Mach,dt,LaunchLatitude,LaunchLongitude,LaunchAltitude,LaunchElevation,LaunchAzimuth,RocketBodyRadius,RocketBodyLength,raillength,RailLength,NozzleExitArea,TimeBurn,rbod,RocketLength,LaunchElevationLower,LaunchAzimuthLower,ThrustMisalignmentYawLower,ThrustMisalignmentPitchLower,ThrustMagnitudeLower,BurnTimeLower,WindMagnitudeLower,WindDirectionLower,AerodynamicDragCoefficientLower,AerodynamicLiftCoefficientLower,AerodynamicMomentCoefficientLower,CentreOfPressureLower,FinCantAngleLower,LaunchAltitudeLower,LaunchElevationUpper,LaunchAzimuthUpper,ThrustMisalignmentYawUpper,ThrustMisalignmentPitchUpper,ThrustMagnitudeUpper,BurnTimeUpper,WindMagnitudeUpper,WindDirectionUpper,AerodynamicDragCoefficientUpper,AerodynamicLiftCoefficientUpper,AerodynamicMomentCoefficientUpper,CentreOfPressureUpper,FinCantAngleUpper,LaunchAltitudeUpper,WindMagnitude,WindBearing,df_atmosphere,df_mass,df_thrust,atmosphere_altitude_array,atmosphere_temperature_array,atmosphere_pressure_array,atmosphere_density_array,timem_array,massm_array,ixxm_array,iyym_array,izzm_array,cogm_array,timey_array,thrusty_array,pressure_array,thrust_vector_ideal,header_wind,df4,df5,wind_counter,wind_counter_max,wind_temporary_header,altitude_array,magnitude1_array,magnitude2_array,ApogeeMessage,RailMessage,AngleOfAttackMessage,BurnoutMessage,Spin,NoseRadius,NoseLength,FinRootChord,FinTipChord,FinSweep,FinSpan,FinLocation,ThrusterPulseFrequency,ThrusterForce,RollRateThresholdLower,RollRateThresholdUpper,time,tfinal,Ry,Rz,EarthRadius,launchalt,Lengthpgeoa,maxsteps,cutoffsteps,nsteps,mach,railtime,step,windcuta,windgspd,jetflor,jetceil,jetvelo,jetdire,ApogeeTime,EarthGravitationalConstant,EarthMass,omgt,gamma,Rair,ixy,ixz,iyz,TLengt,TSigma,Cl,Clins,Cdp,Cltot,Cdptot,fincp,cna,pitdamp,yawdamp,J,K,L,D0,D1,D2,D3,gamk,xsik,a11pk,a12pk,a13pk,alpk,betk,gama,xsia,a11pa,a12pa,a13pa,alpa,beta,a11a,a21a,a31a,alpap,betap,phipp,taoap,muap,taoa,phip,mua,quant1,quant2,marm,alpaeq,betaeq,Macheq,taoaeq,ortherr,elevf,numer,denom,tlengt,tsigma,tran,AirSpeed,SpeedOfSound,DynamicPressure,StepNumber,TrackAltitude1,TrackAltitude2,ParachuteList,inpa,ilatl,ivgeo,vaer,aang,pang,pear,pgeo,pear,pear,pear,omeg
        global vgeo,ageo,ltlo,dcor,qang,dqan,turb,Ccof,Clinear,Cmom,VaB,inertia,altinit,fgeoa,vbody,omegt,omgtt,raild,raild,raild,conmom,Clin,omegeart,veara,dpgeo,dvgeo,domeg,dpear,dltlo,dqang,dtur,launchlatit,launchlongi,ivelx,Schar,alti,height,pang,panga,vaera,aanga,ageoa,pgeoa,peara,dcora,dqana,peara,Va2,Vk2,toth,vamag,rho,pres,time,mass,ixx,iyy,izz,cog,df,array_mach,array_mach,array_mach_unique
        global array_alpha,array_alpha,array_alpha,array_alpha_unique,array_CA,array_CN,array_cop,array_cop,CA_lookupmatrix,CN_lookupmatrix,cop_lookupmatrix,CA_lookup,CN_lookup,cop_lookup,df15,array_mach15,array_mach15,array_mach_unique15,array_alpha15,array_alpha15,array_alpha15,array_alpha_unique15,array_CA15,array_CN15,array_cop15,array_cop15,CA_lookupmatrix15,CN_lookupmatrix15,cop_lookupmatrix15,CA_lookup15,CN_lookup15,cop_lookup15,RASAero,cop,StabilityMargin,RelativeVelocityWind,ThrustMagnitude,thrust,gr
        global qanga,OutputColumns,dfOutput,ListOutput
        global dfOutputRow,dfOutput,Range,dfNose,array_mach_nose,array_mach_nose_unique,array_alpha_nose,array_alpha_nose_unique,array_CA_nose,array_CN_nose,array_cop_nose,CA_lookupmatrix_nose,CN_lookupmatrix_nose,cop_lookupmatrix_nose,CA_lookupNose,CN_lookupNose,cop_lookupNose,dfNose15,array_mach_nose15,array_mach_nose15_unique,array_alpha_nose15,array_alpha_nose15_unique,array_CA_nose15,array_CN_nose15,array_cop_nose15,CA_lookupmatrix_nose15,CN_lookupmatrix_nose15,cop_lookupmatrix_nose15,CA_lookupNose15,CN_lookupNose15,cop_lookupNose15,Nose,Booster,dfSideDamping,StabilityMarginMessage,SeparationApogeeMessage,vwnda,DrogueDeploymentMessage,ParachuteDeploymentMessage,CA_lookupBooster,CN_lookupBooster,cop_lookupBooster,CA_lookupBooster15,CN_lookupBooster15,cop_lookupBooster15
        t=0
        TimeMax=self.input_values["TimeMax"]
        TimeSize=self.input_values["TimeSize"]
        LaunchLatitude=self.input_values["LaunchLatitude"]
        LaunchLongitude=self.input_values["LaunchLongitude"]
        LaunchAltitude=self.input_values["LaunchAltitude"]
        LaunchElevation=self.input_values["LaunchElevation"]
        LaunchAzimuth=self.input_values["LaunchAzimuth"]
        RocketBodyRadius=self.input_values["RocketBodyRadius"]
        RocketBodyLength=self.input_values["RocketBodyLength"]
        LaunchRailLength=self.input_values["LaunchRailLength"]
        NozzleExitArea=self.input_values["NozzleExitArea"]
        ThrustPolynomialDegree=self.input_values["ThrustPolynomialDegree"]
        MissileDATCOMCards=self.input_values["MissileDATCOMCards"]
        SolidWorksMass=self.input_values["SolidWorksMass"]
        SolidWorksCOMx=self.input_values["SolidWorksCOMx"]
        SolidWorksCOMy=self.input_values["SolidWorksCOMy"]
        SolidWorksCOMz=self.input_values["SolidWorksCOMz"]
        SolidWorksMOIx=self.input_values["SolidWorksMOIx"]
        SolidWorksMOIy=self.input_values["SolidWorksMOIy"]
        SolidWorksMOIz=self.input_values["SolidWorksMOIz"]
        TimeBurn=self.input_values["TimeBurn"]
        FuelDensity=self.input_values["FuelDensity"]
        FuelRadius=self.input_values["FuelRadius"]
        FuelThickness=self.input_values["FuelThickness"]
        FuelThicknessInitial=self.input_values["FuelThicknessInitial"]
        FuelCOM=self.input_values["FuelCOM"]
        FuelLength=self.input_values["FuelLength"]
        ParachuteCD=self.input_values["ParachuteCD"]
        ParachuteDiameter=self.input_values["ParachuteDiameter"]
        ParachuteDelay=self.input_values["ParachuteDelay"]
        NoseMass=self.input_values["NoseMass"]
        NoseCOMx=self.input_values["NoseCOMx"]
        NoseCOMy=self.input_values["NoseCOMy"]
        NoseCOMz=self.input_values["NoseCOMz"]
        NoseMOIx=self.input_values["NoseMOIx"]
        NoseMOIy=self.input_values["NoseMOIy"]
        NoseMOIz=self.input_values["NoseMOIz"]
        BoosterMass=self.input_values["BoosterMass"]
        BoosterCOMx=self.input_values["BoosterCOMx"]
        BoosterCOMy=self.input_values["BoosterCOMy"]
        BoosterCOMz=self.input_values["BoosterCOMz"]
        BoosterMOIx=self.input_values["BoosterMOIx"]
        BoosterMOIy=self.input_values["BoosterMOIy"]
        BoosterMOIz=self.input_values["BoosterMOIz"]
        NumberRuns=self.input_values["NumberRuns"]
        LaunchElevationLower=self.input_values["LaunchElevationLower"]
        LaunchElevationUpper=self.input_values["LaunchElevationUpper"]
        LaunchAzimuthLower=self.input_values["LaunchAzimuthLower"]
        LaunchAzimuthUpper=self.input_values["LaunchAzimuthUpper"]
        ThrustMisalignmentYawLower=self.input_values["ThrustMisalignmentYawLower"]
        ThrustMisalignmentYawUpper=self.input_values["ThrustMisalignmentYawUpper"]
        ThrustMisalignmentPitchLower=self.input_values["ThrustMisalignmentPitchLower"]
        ThrustMisalignmentPitchUpper=self.input_values["ThrustMisalignmentPitchUpper"]
        ThrustMagnitudeLower=self.input_values["ThrustMagnitudeLower"]
        ThrustMagnitudeUpper=self.input_values["ThrustMagnitudeUpper"]
        TimeBurnLower=self.input_values["TimeBurnLower"]
        TimeBurnUpper=self.input_values["TimeBurnUpper"]
        WindMagnitudeLower=self.input_values["WindMagnitudeLower"]
        WindMagnitudeUpper=self.input_values["WindMagnitudeUpper"]
        WindDirectionLower=self.input_values["WindDirectionLower"]
        WindDirectionUpper=self.input_values["WindDirectionUpper"]
        AerodynamicDragLower=self.input_values["AerodynamicDragLower"]
        AerodynamicDragUpper=self.input_values["AerodynamicDragUpper"]
        AerodynamicLiftLower=self.input_values["AerodynamicLiftLower"]
        AerodynamicLiftUpper=self.input_values["AerodynamicLiftUpper"]
        AerodynamicMomentLower=self.input_values["AerodynamicMomentLower"]
        AerodynamicMomentUpper=self.input_values["AerodynamicMomentUpper"]
        CentreOfPressureLower=self.input_values["CentreOfPressureLower"]
        CentreOfPressuerUpper=self.input_values["CentreOfPressuerUpper"]
        FinCantAngleLower=self.input_values["FinCantAngleLower"]
        FinCantAngleUpper=self.input_values["FinCantAngleUpper"]
        LaunchAltitudeLower=self.input_values["LaunchAltitudeLower"]
        LaunchAltitudeUpper=self.input_values["LaunchAltitudeUpper"]
        CalculatorTimeSize=self.input_values["CalculatorTimeSize"]
        OxidiserDensity=self.input_values["OxidiserDensity"]
        OxidiserRadius=self.input_values["OxidiserRadius"]
        OxidiserLength=self.input_values["OxidiserLength"]
        OxidiserLengthInitial=self.input_values["OxidiserLengthInitial"]
        OxidiserCOM=self.input_values["OxidiserCOM"]
        TimeBurnFuel=self.input_values["TimeBurnFuel"]
        NumberStages=self.input_values["NumberStages"]
        StageTime=self.input_values["StageTime"]
        StageDelay=self.input_values["StageDelay"]
        StageMass=self.input_values["StageMass"]
        StageCOMx=self.input_values["StageCOMx"]
        StageCOMy=self.input_values["StageCOMy"]
        StageCOMz=self.input_values["StageCOMz"]
        StageMOIx=self.input_values["StageMOIx"]
        StageMOIy=self.input_values["StageMOIy"]
        StageMOIz=self.input_values["StageMOIz"]
        NoseRadius=self.input_values["NoseRadius"]
        NoseLength=self.input_values["NoseLength"]
        FinRootChord=self.input_values["FinRootChord"]
        FinTipChord=self.input_values["FinTipChord"]
        FinSweep=self.input_values["FinSweep"]
        FinSpan=self.input_values["FinSpan"]
        FinLocation=self.input_values["FinLocation"]
        FinSpanRoot=self.input_values["FinSpanRoot"]
        FinCantAngle=self.input_values["FinCantAngle"]
        CheckTurbulence=self.input_values["CheckTurbulence"]
        Check4in1=self.input_values["Check4in1"]
        CheckStaging=self.input_values["CheckStaging"]
        CheckThrust=self.input_values["CheckThrust"]
        CheckRollControl=self.input_values["CheckRollControl"]
        CheckMonteCarloUI=self.input_values["CheckMonteCarloUI"]
        CheckStream=self.input_values["CheckStream"]
        CheckOrder2=self.input_values["CheckOrder2"]
        CheckOrder4=self.input_values["CheckOrder4"]
        CheckOrder8=self.input_values["CheckOrder8"]
        SolverRelative=self.input_values["SolverRelative"]
        SolverAbsolute=self.input_values["SolverAbsolute"]
        SolverTimeSizeFirst=self.input_values["SolverTimeSizeFirst"]
        SolverTimeSizeMax=self.input_values["SolverTimeSizeMax"]
        RollControlLower=self.input_values["RollControlLower"]
        RollControlUpper=self.input_values["RollControlUpper"]
        RollControlFrequency=self.input_values["RollControlFrequency"]
        RollControlForce=self.input_values["RollControlForce"]
        GraphicScale=self.input_values["GraphicScale"]
        TrajectoryResolution=self.input_values["TrajectoryResolution"]
        Directory=self.input_values["Directory"]
        RollControlTimeInitial=self.input_values["RollControlTimeInitial"]
        RollControlTimeFinal=self.input_values["RollControlTimeFinal"]
        DrogueCD=self.input_values["DrogueCD"]
        DrogueDiameter=self.input_values["DrogueDiameter"]
        DrogueDelay=self.input_values["DrogueDelay"]

##        path=r"TimestepFirst.txt"
##        exist = os.path.isfile(path)
##        if exist==True:
##            os.remove("TimestepFirst.txt")

        path=r"VariablesFirst.txt"
        exist = os.path.isfile(path)
        if exist==True:
            os.remove("VariablesFirst.txt")

        """Transferring variables from UI to main:"""
        file1=open("VariablesFirst.txt","w")
        file1.write(str('{'+'"TimeMax":{},"TimeSize":{},"LaunchLatitude":{},"LaunchLongitude":{},"LaunchAltitude":{},"LaunchElevation":{},"LaunchAzimuth":{},"RocketBodyRadius":{},"RocketBodyLength":{},"LaunchRailLength":{},"NozzleExitArea":{},"ThrustPolynomialDegree":{},"MissileDATCOMCards":{},"SolidWorksMass":{},"SolidWorksCOMx":{},"SolidWorksCOMy":{},"SolidWorksCOMz":{},"SolidWorksMOIx":{},"SolidWorksMOIy":{},"SolidWorksMOIz":{},"TimeBurn":{},"FuelDensity":{},"FuelRadius":{},"FuelThickness":{},"FuelThicknessInitial":{},"FuelCOM":{},"FuelLength":{},"ParachuteCD":{},"ParachuteDiameter":{},"ParachuteDelay":{},"NoseMass":{},"NoseCOMx":{},"NoseCOMy":{},"NoseCOMz":{},"NoseMOIx":{},"NoseMOIy":{},"NoseMOIz":{},"BoosterMass":{},"BoosterCOMx":{},"BoosterCOMy":{},"BoosterCOMz":{},"BoosterMOIx":{},"BoosterMOIy":{},"BoosterMOIz":{},"NumberRuns":{},"LaunchElevationLower":{},"LaunchElevationUpper":{},"LaunchAzimuthLower":{},"LaunchAzimuthUpper":{},"ThrustMisalignmentYawLower":{},"ThrustMisalignmentYawUpper":{},"ThrustMisalignmentPitchLower":{},"ThrustMisalignmentPitchUpper":{},"ThrustMagnitudeLower":{},"ThrustMagnitudeUpper":{},"TimeBurnLower":{},"TimeBurnUpper":{},"WindMagnitudeLower":{},"WindMagnitudeUpper":{},"WindDirectionLower":{},"WindDirectionUpper":{},"AerodynamicDragLower":{},"AerodynamicDragUpper":{},"AerodynamicLiftLower":{},"AerodynamicLiftUpper":{},"AerodynamicMomentLower":{},"AerodynamicMomentUpper":{},"CentreOfPressureLower":{},"CentreOfPressuerUpper":{},"FinCantAngleLower":{},"FinCantAngleUpper":{},"LaunchAltitudeLower":{},"LaunchAltitudeUpper":{},"CalculatorTimeSize":{},"OxidiserDensity":{},"OxidiserRadius":{},"OxidiserLength":{},"OxidiserLengthInitial":{},"OxidiserCOM":{},"TimeBurnFuel":{},"NumberStages":{},"StageTime":{},"StageDelay":{},"StageMass":{},"StageCOMx":{},"StageCOMy":{},"StageCOMz":{},"StageMOIx":{},"StageMOIy":{},"StageMOIz":{},"NoseRadius":{},"NoseLength":{},"FinRootChord":{},"FinTipChord":{},"FinSweep":{},"FinSpan":{},"FinLocation":{},"FinSpanRoot":{},"FinCantAngle":{},"CheckTurbulence":{},"Check4in1":{},"CheckStaging":{},"CheckThrust":{},"CheckRollControl":{},"CheckMonteCarloUI":{},"CheckStream":{},"CheckOrder2":{},"CheckOrder4":{},"CheckOrder8":{},"SolverRelative":{},"SolverAbsolute":{},"SolverTimeSizeFirst":{},"SolverTimeSizeMax":{},"RollControlLower":{},"RollControlUpper":{},"RollControlFrequency":{},"RollControlForce":{},"GraphicScale":{},"TrajectoryResolution":{},"RollControlTimeInitial":{},"RollControlTimeFinal":{},"DrogueCD":{},"DrogueDiameter":{},"DrogueDelay":{}'.format(TimeMax,TimeSize,LaunchLatitude,LaunchLongitude,LaunchAltitude,LaunchElevation,LaunchAzimuth,RocketBodyRadius,RocketBodyLength,LaunchRailLength,NozzleExitArea,ThrustPolynomialDegree,MissileDATCOMCards,SolidWorksMass,SolidWorksCOMx,SolidWorksCOMy,SolidWorksCOMz,SolidWorksMOIx,SolidWorksMOIy,SolidWorksMOIz,TimeBurn,FuelDensity,FuelRadius,FuelThickness,FuelThicknessInitial,FuelCOM,FuelLength,ParachuteCD,ParachuteDiameter,ParachuteDelay,NoseMass,NoseCOMx,NoseCOMy,NoseCOMz,NoseMOIx,NoseMOIy,NoseMOIz,BoosterMass,BoosterCOMx,BoosterCOMy,BoosterCOMz,BoosterMOIx,BoosterMOIy,BoosterMOIz,NumberRuns,LaunchElevationLower,LaunchElevationUpper,LaunchAzimuthLower,LaunchAzimuthUpper,ThrustMisalignmentYawLower,ThrustMisalignmentYawUpper,ThrustMisalignmentPitchLower,ThrustMisalignmentPitchUpper,ThrustMagnitudeLower,ThrustMagnitudeUpper,TimeBurnLower,TimeBurnUpper,WindMagnitudeLower,WindMagnitudeUpper,WindDirectionLower,WindDirectionUpper,AerodynamicDragLower,AerodynamicDragUpper,AerodynamicLiftLower,AerodynamicLiftUpper,AerodynamicMomentLower,AerodynamicMomentUpper,CentreOfPressureLower,CentreOfPressuerUpper,FinCantAngleLower,FinCantAngleUpper,LaunchAltitudeLower,LaunchAltitudeUpper,CalculatorTimeSize,OxidiserDensity,OxidiserRadius,OxidiserLength,OxidiserLengthInitial,OxidiserCOM,TimeBurnFuel,NumberStages,StageTime,StageDelay,StageMass,StageCOMx,StageCOMy,StageCOMz,StageMOIx,StageMOIy,StageMOIz,NoseRadius,NoseLength,FinRootChord,FinTipChord,FinSweep,FinSpan,FinLocation,FinSpanRoot,FinCantAngle,CheckTurbulence,Check4in1,CheckStaging,CheckThrust,CheckRollControl,CheckMonteCarloUI,CheckStream,CheckOrder2,CheckOrder4,CheckOrder8,SolverRelative,SolverAbsolute,SolverTimeSizeFirst,SolverTimeSizeMax,RollControlLower,RollControlUpper,RollControlFrequency,RollControlForce,GraphicScale,TrajectoryResolution,RollControlTimeInitial,RollControlTimeFinal,DrogueCD,DrogueDiameter,DrogueDelay)+'}'))
        file1.close()

##        """Initialise variables:"""
##        path=r"TimestepFirst.txt"
##        exist = os.path.isfile(path)
##        if exist==False:
##            file1=open("TimestepFirst.txt","w")
##            file1.write(str(0))
##            file1.close()
        """Read variables:"""
        file1=open("VariablesFirst.txt","r")
        Variables=json.loads(str(file1.readlines()[0]))
        file1.close()

        if t==0:
            global list0,list1,list2,list3,list4,list5,list6,list7,list8,list9,list10,list11,list12,list13,list14,list15,list16,list17,list18,list19,list20,list21,list22,list23,list24,list25,list26,list27,list28,list29,list30,list31,list32,list33,list34,list35,list36,list37,list38,list39,list40,list41,list42,list43,list44,list45,list46,list47,list48,list49,list50,list51,list52,list53,list54,list55,list56,list57,list58,list59,list60,list61,list62,list63,list64,list65,list66
            list0=[]
            list1=[]
            list2=[]
            list3=[]
            list4=[]
            list5=[]
            list6=[]
            list7=[]
            list8=[]
            list9=[]
            list10=[]
            list11=[]
            list12=[]
            list13=[]
            list14=[]
            list15=[]
            list16=[]
            list17=[]
            list18=[]
            list19=[]
            list20=[]
            list21=[]
            list22=[]
            list23=[]
            list24=[]
            list25=[]
            list26=[]
            list27=[]
            list28=[]
            list29=[]
            list30=[]
            list31=[]
            list32=[]
            list33=[]
            list34=[]
            list35=[]
            list36=[]
            list37=[]
            list38=[]
            list39=[]
            list40=[]
            list41=[]
            list42=[]
            list43=[]
            list44=[]
            list45=[]
            list46=[]
            list47=[]
            list48=[]
            list49=[]
            list50=[]
            list51=[]
            list52=[]
            list53=[]
            list54=[]
            list55=[]
            list56=[]
            
            """Spare lists:"""
            list57=[]
            list58=[]
            list59=[]
            list60=[]
            list61=[]
            list62=[]
            list63=[]
            list64=[]
            list65=[]
            list66=[]

            """Additional output variables (optional):"""
            KinematicVelocityMagnitude=0
            DragAccelerationMagnitudeX=0
            DragAccelerationMagnitudeY=0
            DragAccelerationMagnitudeZ=0
            Drag=0
            Gravity=0
            DragAccelerationX=0
            DragAccelerationY=0
            DragAccelerationZ=0

            """Unused variables:"""

            """
            TimeSize=Variables["TimeSize"]
            NumberRuns=Variables["NumberRuns"]
            LaunchElevationLower=Variables["LaunchElevationLower"]
            LaunchElevationUpper=Variables["LaunchElevationUpper"]
            LaunchAzimuthLower=Variables["LaunchAzimuthLower"]
            LaunchAzimuthUpper=Variables["LaunchAzimuthUpper"]
            ThrustMisalignmentYawLower=Variables["ThrustMisalignmentYawLower"]
            ThrustMisalignmentYawUpper=Variables["ThrustMisalignmentYawUpper"]
            ThrustMisalignmentPitchLower=Variables["ThrustMisalignmentPitchLower"]
            ThrustMisalignmentPitchUpper=Variables["ThrustMisalignmentPitchUpper"]
            ThrustMagnitudeLower=Variables["ThrustMagnitudeLower"]
            ThrustMagnitudeUpper=Variables["ThrustMagnitudeUpper"]
            TimeBurnLower=Variables["TimeBurnLower"]
            TimeBurnUpper=Variables["TimeBurnUpper"]
            WindMagnitudeLower=Variables["WindMagnitudeLower"]
            WindMagnitudeUpper=Variables["WindMagnitudeUpper"]
            WindDirectionLower=Variables["WindDirectionLower"]
            WindDirectionUpper=Variables["WindDirectionUpper"]
            AerodynamicDragLower=Variables["AerodynamicDragLower"]
            AerodynamicDragUpper=Variables["AerodynamicDragUpper"]
            AerodynamicLiftLower=Variables["AerodynamicLiftLower"]
            AerodynamicLiftUpper=Variables["AerodynamicLiftUpper"]
            AerodynamicMomentLower=Variables["AerodynamicMomentLower"]
            AerodynamicMomentUpper=Variables["AerodynamicMomentUpper"]
            CentreOfPressureLower=Variables["CentreOfPressureLower"]
            CentreOfPressuerUpper=Variables["CentreOfPressuerUpper"]
            FinCantAngleLower=Variables["FinCantAngleLower"]
            FinCantAngleUpper=Variables["FinCantAngleUpper"]
            LaunchAltitudeLower=Variables["LaunchAltitudeLower"]
            LaunchAltitudeUpper=Variables["LaunchAltitudeUpper"]
            CalculatorTimeSize=Variables["CalculatorTimeSize"]
            OxidiserDensity=Variables["OxidiserDensity"]
            OxidiserRadius=Variables["OxidiserRadius"]
            OxidiserLength=Variables["OxidiserLength"]
            OxidiserLengthInitial=Variables["OxidiserLengthInitial"]
            OxidiserCOM=Variables["OxidiserCOM"]
            TimeBurnFuel=Variables["TimeBurnFuel"]
            CheckMonteCarloUI=Variables["CheckMonteCarloUI"]
            """
            
            TimePrevious=0
            
            TimeMax=                            self.TimeMax
            LaunchLatitude=                     Variables["LaunchLatitude"]
            LaunchLongitude=                    Variables["LaunchLongitude"]
            LaunchAltitude=                     Variables["LaunchAltitude"]
            LaunchElevation=                    Variables["LaunchElevation"]
            LaunchAzimuth=                      Variables["LaunchAzimuth"]
            RocketBodyRadius=                   Variables["RocketBodyRadius"]
            rbod=                               RocketBodyRadius
            RocketBodyLength=                   Variables["RocketBodyLength"]
            RocketLength=                       RocketBodyLength
            raillength=                         Variables["LaunchRailLength"]
            RailLength=                         raillength
            NozzleExitArea=                     Variables["NozzleExitArea"]
            TimeBurn=                           Variables["TimeBurn"]
            LaunchElevationLower=               0
            LaunchAzimuthLower=                 0
            ThrustMisalignmentYawLower=         0
            ThrustMisalignmentPitchLower=       0
            ThrustMagnitudeLower=               0
            BurnTimeLower=                      0
            WindMagnitudeLower=                 0
            WindDirectionLower=                 0
            AerodynamicDragCoefficientLower=    0
            AerodynamicLiftCoefficientLower=    0
            AerodynamicMomentCoefficientLower=  0
            CentreOfPressureLower=              0
            FinCantAngleLower=                  0
            LaunchAltitudeLower=                0
            LaunchElevationUpper=               0
            LaunchAzimuthUpper=                 0
            ThrustMisalignmentYawUpper=         0
            ThrustMisalignmentPitchUpper=       0
            ThrustMagnitudeUpper=               0
            BurnTimeUpper=                      0
            WindMagnitudeUpper=                 0
            WindDirectionUpper=                 0
            AerodynamicDragCoefficientUpper=    0
            AerodynamicLiftCoefficientUpper=    0
            AerodynamicMomentCoefficientUpper=  0
            CentreOfPressureUpper=              0
            FinCantAngleUpper=                  0
            LaunchAltitudeUpper=                0
            WindMagnitude=                      0
            WindBearing=                        0

            """Import atmosphere:"""                
            df_atmosphere=pd.read_excel(r'{}\inputs\atmosphere_data.xlsx'.format(Directory),header=None)
            df_mass=pd.read_excel(r'{}\inputs\mass_properties.xlsx'.format(Directory),header=0)
            df_thrust=pd.read_excel(r'{}\inputs\thrust_curve.xlsx'.format(Directory),header=0)
            atmosphere_altitude_array=df_atmosphere.iloc[:,0].values        
            atmosphere_temperature_array=df_atmosphere.iloc[:,1].values
            atmosphere_pressure_array=df_atmosphere.iloc[:,2].values
            atmosphere_density_array=df_atmosphere.iloc[:,3]#.values

            """Import mass properties:"""
            timem_array=df_mass.iloc[:,0].values
            massm_array=df_mass.iloc[:,1].values
            ixxm_array=df_mass.iloc[:,2].values
            iyym_array=df_mass.iloc[:,3].values
            izzm_array=df_mass.iloc[:,4].values
            cogm_array=df_mass.iloc[:,5].values

            """Import thrust:"""
            timey_array=df_thrust.iloc[:,0].values
            thrusty_array=df_thrust.iloc[:,1].values
            pressure_array=df_thrust.iloc[:,2].values
            thrust_vector_ideal=np.array([1,0,0])

            """Import wind. Will move to a separate file, wind.pyw."""
            df4=pd.read_excel(r'{}\Inputs\wind.xlsx'.format(Directory),header=0)
            df5=pd.DataFrame()
            df5["altitude (m)"]=df4["altitude (m)"]
            df5["magnitude1"]=0
            df5["magnitude2"]=0
            wind_counter=0
            wind_counter_max=len(df4)
            while wind_counter<(wind_counter_max-1):
                df5.at[wind_counter,"magnitude1"]=df4.at[wind_counter,"magnitude (m/s)"]*math.cos(math.radians(df4.at[wind_counter,"bearing (degrees)"]))
                df5.at[wind_counter,"magnitude2"]=-df4.at[wind_counter,"magnitude (m/s)"]*math.sin(math.radians(df4.at[wind_counter,"bearing (degrees)"]))
                wind_counter+=1
            wind_temporary_header=["altitude (m)", "magnitude1", "magnitude2"]
            altitude_array=df5.iloc[:,0].values
            magnitude1_array=df5.iloc[:,1].values
            magnitude2_array=df5.iloc[:,2].values

            """Message variables (while simulation is running):"""
            ApogeeMessage=                          False
            RailMessage=                            False
            AngleOfAttackMessage=                   False
            BurnoutMessage=                         False
            SeparationApogeeMessage=                False
            StabilityMarginMessage=                 False
            DrogueDeploymentMessage=                False
            ParachuteDeploymentMessage=             False

            """Rocket Body State:"""
            global Nose, Booster, Stage
            Nose=                                   False
            Booster=                                False
            Stage=                                  False
            
            """Body State 1: complete rocket assembly:"""
            """Body State 2: nosecone and payload with parachute:"""
            if self.BodyState==2:
                Nose=True
            """Body State 3: nosecone and payload without parachute:"""
            if self.BodyState==3:
                Nose=True
            """Body State 4: booster:"""
            if self.BodyState==4:
                Booster=True

            """Roll Control System variables:"""
            ThrusterPulseFrequency=                 0
            ThrusterForce=                          0
            RollRateThresholdLower=                 0
            RollRateThresholdUpper=                 0

##            """Input Quaternion -> Euler Angles:"""
##            elevf=0
##            numer=0
##            denom=0
##            numer=2*((qanga[1]*qanga[2])+(qanga[0]*qanga[3]))
##            denom=(qanga[0]*qanga[0])+(qanga[1]*qanga[1])-(qanga[2]*qanga[2])-(qanga[3]*qanga[3])
##            inpa=np.array([0,0,0],dtype=float) 
##            if numer>=0 and denom>0:
##                inpa[2]=math.atan(numer/denom)
##            if numer<=0 and denom<0:
##                inpa[2]=math.atan(abs(numer)/abs(denom))+math.pi
##            if numer>=0 and denom<0:
##                inpa[2]=math.pi-math.atan(abs(numer)/abs(denom))
##            if numer<=0 and denom>0:
##                inpa[2]=2*math.pi-math.atan(abs(numer)/abs(denom))
##            numer=2*((qanga[2]*qanga[3])+(qanga[0]*qanga[1]))
##            denom=(qanga[0]*qanga[0])-(qanga[1]*qanga[1])-(qanga[2]*qanga[2])+(qanga[3]*qanga[3])
##            if numer>=0 and denom>0:
##                inpa[0]=math.atan(numer/denom)
##            if numer<=0 and denom<0:
##                inpa[0]=math.atan(abs(numer)/abs(denom))+math.pi
##            if numer>=0 and denom<0:
##                inpa[0]=math.pi-math.atan(abs(numer)/abs(denom))
##            if numer<=0 and denom>0:
##                inpa[0]=2*math.pi-math.atan(abs(numer)/abs(denom))
##            elevf=-2*((qanga[1]*qanga[3])-(qanga[0]*qanga[2]))
##            if elevf>=-1 and elevf<=1:
##                inpa[1]=math.asin(elevf)
##            while(inpa[0]>(2*math.pi)):
##                inpa[0]-=2*math.pi
##            while(inpa[0]<(-2*math.pi)):
##                inpa[0]+=2*math.pi
##            while(inpa[1]>(2*math.pi)):
##                inpa[1]-=2*math.pi
##            while(inpa[1]<(-2*math.pi)):
##                inpa[1]+=2*math.pi
##            while inpa[2]>2*math.pi:
##                inpa[2]-=2*math.pi
##            while inpa[2]<(-2*math.pi):
##                inpa[2]+=2*math.pi

            """Initial vehicle orientation state vector:"""
            inpa=np.array([0,(-Variables["LaunchElevation"]-self.monte_carlo.outputs()[0])*(math.pi/180),(Variables["LaunchAzimuth"]+self.monte_carlo.outputs()[1])*(math.pi/180)],dtype=float)
            if CheckMonteCarloUI==0:
                inpa=np.array([0,(-Variables["LaunchElevation"])*(math.pi/180),(Variables["LaunchAzimuth"])*(math.pi/180)],dtype=float)
            """pang[1]*=MC_ALT in HYROPS, MC_ALT=1.0+(MC_LAUNCH_ALTITUDE*NormalRandom()*rad), MC_ALT=1 without uncertainty. In HYROPS: Elevation is called "Altitud" """
            """pang[2]*=MC_AZI in HYROPS, MC_AZI=1.0+(MC_LAUNCH_AZIMUTH*NormalRandom()*rad), MC_AZI=1 without uncertainty."""

            """Constants:"""
            time=                                   0
            Ry=                                     0
            Rz=                                     0
            EarthRadius=                            6378000
            launchalt=                              0
            Lengthpgeoa=                            0
            maxsteps=                               400000
            cutoffsteps=                            maxsteps-1
            nsteps=                                 cutoffsteps
            mach=                                   0
            railtime=                               0
            step=                                   0
            windcuta=                               20000
            windgspd=                               10
            jetflor=                                10000
            jetceil=                                15000
            jetvelo=                                30
            jetdire=                                90
            ApogeeTime=                             0
            EarthGravitationalConstant=             6.67428e-11
            EarthMass=                              5.9736e24
            omgt=                                   7.292e-5
            gamma=                                  1.4
            Rair=                                   287.0
            ixy=                                    0
            ixz=                                    0
            iyz=                                    0
            TLengt=                                 0
            TSigma=                                 0
            Cl=                                     0
            Clins=                                  0
            Cdp=                                    0
            Cltot=                                  0
            Cdptot=                                 0
            fincp=                                  0
            cna=                                    0
            pitdamp=                                0
            yawdamp=                                0
            J,K,L,D0,D1,D2,D3=                      0,0,0,0,0,0,0
            gamk,xsik,a11pk,a12pk,a13pk,alpk,betk=  0,0,0,0,0,0,0
            gama,xsia,a11pa,a12pa,a13pa,alpa,beta=  0,0,0,0,0,0,0
            a11a,a21a,a31a=                         0,0,0
            alpap,betap,phipp,taoap,muap=           0,0,0,0,0
            taoa,phip,mua=                          0,0,0
            quant1,quant2=                          0,0
            marm=                                   0
            alpaeq,betaeq,Macheq,taoaeq=            0,0,0,0
            ortherr=                                0
            elevf=                                  0
            numer=                                  0
            denom=                                  0
            tlengt,tsigma,tran=                     0,0,0
            AirSpeed=                               0
            SpeedOfSound=                           math.sqrt(gamma*Rair*np.interp(0,atmosphere_altitude_array,atmosphere_temperature_array))
            DynamicPressure=                        0
            StepNumber=                             0
            TrackAltitude1=                         0
            TrackAltitude2=                         0
            ParachuteList=                          []
            ilatl=                                  np.array([LaunchLatitude*(math.pi/180),LaunchLongitude*(math.pi/180)],dtype=float)
            ivgeo=                                  np.array([0,0,0],dtype=float)
            vaer=                                   np.array([0,0,0],dtype=float)                                      # Velocity Aerodynamic               #aerveloc
            aang=                                   np.array([0,0,0],dtype=float)                                      # Rocket: Acceleration Angular       #angaccel
            pang=                                   np.array([0,0,0],dtype=float)                                      # Rocket: Position Angular           #angposit
            pear=                                   np.array([0,0,0],dtype=float)                                      # Position Inertial                  #earposit
            pgeo=                                   np.array([0,0,0],dtype=float)                                      # Position Kinematic                 #geoposit #moved #*
            pear=                                   np.array([EarthRadius-pgeo[2]+LaunchAltitude+self.monte_carlo.outputs()[13],0,0])
            if CheckMonteCarloUI==0:
                pear=                                   np.array([EarthRadius-pgeo[2]+LaunchAltitude,0,0])
            """Altitude variable not in HYROPS. In HYROPS: Elevation is called "Altitud" """
            pear=                                   Rotation.from_euler('xyz',[0,-ilatl[0]*(180/math.pi),0 ],degrees=True).apply(pear)
            pear=                                   Rotation.from_euler('xyz',[0,0,ilatl[1]*(180/math.pi)  ],degrees=True).apply(pear)
            omeg=                                   np.array([0,0,0],dtype=float)                                      # Rocket: Velocity Angular           #angomega      
            vgeo=                                   np.array([0,0,0],dtype=float)                                      # Velocity Kinematic                 #geoveloc
            ageo=                                   np.array([0,0,0],dtype=float)                                      # Acceleration Kinematic             #geoaccel
            ltlo=                                   np.array([0,0],dtype=float)                                        # 'Position Coriolis'                #latl
            dcor=                                   np.array([0,0],dtype=float)                                        # 'Velocity Coriolis'
            qang=                                   np.array([0,0,0,0],dtype=float)                                    # Quarternion 'Orientation' Angular #angl, "Rotation Quaternion"
            dqan=                                   np.array([0,0,0,0],dtype=float)
            turb=                                   np.array([0,0],dtype=float)
            Ccof=                                   np.array([0,0,0],dtype=float)                                      # Aerodynamic Coefficients Linear
            Clinear=                                np.array([0,0,0],dtype=float)                                   # Aerodynamic Forces Linear
            Cmom=                                   np.array([0,0,0],dtype=float)                                      # Aerodynamic Moments Attitude       # Check Coefficients Attitude
            VaB=                                    np.array([0,0,0],dtype=float)                                       # BodyVeloc: Velocity Linear Bodyframe
            inertia=                                np.array([[0,0,0],[0,0,0],[0,0,0]],dtype=float)
            altinit=                                np.array([0,0,0,0],dtype=float)                                 # Altitude Initial. Quaternion containing initial altitudes for four separate runs
            fgeoa=                                  np.array([0,0,0],dtype=float)
            vbody=                                  np.array([0,0,0],dtype=float)
            omegt=                                  np.array([0,0,0],dtype=float)
            omgtt=                                  np.array([0,0,0],dtype=float)
            raild=                                  np.array([1,0,0],dtype=float)
            raild=Rotation.from_euler('xyz',[0,-inpa[1]*(180/math.pi),0],degrees=True).apply(raild)
            raild=Rotation.from_euler('xyz',[0,0,-inpa[2]*(180/math.pi)],degrees=True).apply(raild)
            conmom=                                 np.array([0,0,0],dtype=float)
            Clin=                                   np.array([0,0,0],dtype=float)                                      # Linear Aerodynamic Force Coefficients
            omegeart=                               np.array([0,0,0],dtype=float)
            veara=                                  np.array([0,0,0],dtype=float)
            dpgeo=                                  np.array([0,0,0],dtype=float)
            dvgeo=                                  np.array([0,0,0],dtype=float)
            domeg=                                  np.array([0,0,0],dtype=float)
            dpear=                                  np.array([0,0,0],dtype=float)
            dltlo=                                  np.array([0,0],dtype=float)
            dqang=                                  np.array([0,0,0,0],dtype=float)
            dtur=                                   np.array([0],dtype=float)
            launchlatit=                            ilatl[0] * (180/math.pi)
            launchlongi=                            ilatl[1] * (180/math.pi)
            ivelx=                                  ivgeo[0]
            Schar=                                  math.pi*rbod*rbod
            alti=                                   np.linalg.norm(pear)-EarthRadius
            height=                                 alti
            pang=                                   inpa
            panga=                                  pang
            vaera=                                  vaer
            aanga=                                  aang
            ageoa=                                  ageo
            pgeoa=                                  pgeo
            peara=                                  pear
            dcora=                                  dcor
            dqana=                                  dqan
            peara=                                  np.array([EarthRadius,0,0],dtype=float)
            Va2=                                    vaera[0]*vaera[0]+vaera[1]*vaera[1]+vaera[2]*vaera[2]

            vgeoa=np.array([0,0,0],dtype=float)
            omega=np.array([0,0,0],dtype=float)
            ltloa=np.array([0,0],dtype=float)
            qanga=np.array([0,0,0,0],dtype=float) 
            qanga[0]=math.cos(panga[2]/2)*math.cos(panga[1]/2)*math.cos(panga[0]/2)+math.sin(panga[2]/2)*math.sin(panga[1]/2)*math.sin(panga[0]/2)
            qanga[1]=qanga[1]=math.cos(panga[2]/2)*math.cos(panga[1]/2)*math.sin(panga[0]/2)-math.sin(panga[2]/2)*math.sin(panga[1]/2)*math.cos(panga[0]/2)
            qanga[2]=qanga[2]=math.cos(panga[2]/2)*math.sin(panga[1]/2)*math.cos(panga[0]/2)+math.sin(panga[2]/2)*math.cos(panga[1]/2)*math.sin(panga[0]/2)
            qanga[3]=qanga[3]=math.sin(panga[2]/2)*math.cos(panga[1]/2)*math.cos(panga[0]/2)-math.cos(panga[2]/2)*math.sin(panga[1]/2)*math.sin(panga[0]/2)
    
            Mach=0
            """State variable 1: velocity kinematic, North (m/s)"""
            vgeoa[0]=vgeoa[0]
            
            """State variable 2: velocity kinematic, East(m/s)"""                
            vgeoa[1]=vgeoa[1]
            
            """State variable 3: velocity kinematic, Downwards (m/s)"""  
            vgeoa[2]=vgeoa[2]

            """State variable 4: velocity angular, roll (rad/s)"""                
            omega[0]=omega[0]

            """State variable 5: velocity angular, pitch (rad/s)""" 
            omega[1]=omega[1]

            """State variable 6: velocity angular, yaw (rad/s)""" 
            omega[2]=omega[2]

            """State variable 7: latitude (degrees)"""                 
            ltloa[0]=ltloa[0]

            """State variable 8: longitude (degrees)"""   
            ltloa[1]=ltloa[1]

            """State variable 9: quaternion orientation 'w' (unit vector component)"""  
            qanga[0]=qanga[0]

            """State variable 10: quaternion orientation 'x' (unit vector component)""" 
            qanga[1]=qanga[1]

            """State variable 11: quaternion orientation 'y' (unit vector component)""" 
            qanga[2]=qanga[2]

            """State variable 12: quaternion orientation 'z' (unit vector component)""" 
            qanga[3]=qanga[3]
            
            Vk2=                                    vgeoa[0]*vgeoa[0]+vgeoa[1]*vgeoa[1]+vgeoa[2]*vgeoa[2]
            toth=                                   EarthRadius
            vamag=                                  np.linalg.norm(vaera)
            rho=                                    float(df_atmosphere.iloc[0,3])
            pres=                                   float(df_atmosphere.iloc[0,2])
            time=                                   float(df_mass.iloc[0,0])
            mass=                                   float(df_mass.iloc[0,1])
            ixx=                                    float(df_mass.iloc[0,2])
            iyy=                                    float(df_mass.iloc[0,3])
            izz=                                    float(df_mass.iloc[0,4])
            cog=                                    float(df_mass.iloc[0,5])

            """Phoenix-1B Mk II-R. From aerodynamics.pyw. Can ignore this:"""
            df=pd.read_excel(r'{}\Inputs\RasAeroII.xlsx'.format(Directory),header=0,engine='openpyxl')
            array_mach=df.iloc[:,0].values
            array_mach=np.append(array_mach,array_mach[-1])
            array_mach_unique=np.unique(array_mach)
            array_alpha=df.iloc[:,1].values
            array_alpha=np.append(array_alpha,array_alpha[-1])
            array_alpha=np.array(array_alpha)*np.pi/180
            array_alpha_unique=np.unique(array_alpha)
            array_CA=df.iloc[:,5].values
            array_CN=df.iloc[:,8].values
            array_cop=df.iloc[:,12].values
            array_cop=np.array(array_cop)*0.0254
            CA_lookupmatrix=np.reshape(array_CA,(len(array_alpha_unique),len(array_mach_unique)))
            CN_lookupmatrix=np.reshape(array_CN,(len(array_alpha_unique),len(array_mach_unique)))
            cop_lookupmatrix=np.reshape(array_cop,(len(array_alpha_unique),len(array_mach_unique)))
            CA_lookup=scipy.interpolate.interp2d(array_mach_unique,array_alpha_unique,CA_lookupmatrix)
            CN_lookup=scipy.interpolate.interp2d(array_mach_unique,array_alpha_unique,CN_lookupmatrix)
            cop_lookup=scipy.interpolate.interp2d(array_mach_unique,array_alpha_unique,cop_lookupmatrix)
            df15=pd.read_excel(r'{}\Inputs\RasAeroII15.xlsx'.format(Directory),header=0,engine='openpyxl')    
            array_mach15=df15.iloc[:,0].values
            array_mach15=np.append(array_mach15,array_mach15[-1])
            array_mach_unique15=np.unique(array_mach15)
            array_alpha15=df15.iloc[:,1].values
            array_alpha15=np.append(array_alpha15,array_alpha15[-1])
            array_alpha15=np.array(array_alpha15)*np.pi/180
            array_alpha_unique15=np.unique(array_alpha15)
            array_CA15=df15.iloc[:,5].values
            array_CN15=df15.iloc[:,8].values
            array_cop15=df15.iloc[:,12].values
            array_cop15=np.array(array_cop15)*0.0254
            CA_lookupmatrix15=np.reshape(array_CA15,(len(array_alpha_unique15),len(array_mach_unique15)))
            CN_lookupmatrix15=np.reshape(array_CN15,(len(array_alpha_unique15),len(array_mach_unique15)))
            cop_lookupmatrix15=np.reshape(array_cop15,(len(array_alpha_unique15),len(array_mach_unique15)))
            CA_lookup15=scipy.interpolate.interp2d(array_mach_unique15,array_alpha_unique15,CA_lookupmatrix15)
            CN_lookup15=scipy.interpolate.interp2d(array_mach_unique15,array_alpha_unique15,CN_lookupmatrix15)
            cop_lookup15=scipy.interpolate.interp2d(array_mach_unique15,array_alpha_unique15,cop_lookupmatrix15)
            
            """Nose and payload. From aerodynamics.pyw. Can ignore this:"""
            dfNose=pd.read_excel(r'{}\Inputs\RasAeroIINose.xlsx'.format(Directory),header=0,engine='openpyxl')
            array_mach_nose=dfNose.iloc[:,0].values
            array_mach_nose=np.append(array_mach_nose,array_mach_nose[-1])
            array_mach_nose_unique=np.unique(array_mach_nose)
            array_alpha_nose=dfNose.iloc[:,1].values
            array_alpha_nose=np.append(array_alpha_nose,array_alpha_nose[-1])
            array_alpha_nose=np.array(array_alpha_nose)*np.pi/180
            array_alpha_nose_unique=np.unique(array_alpha_nose)
            array_CA_nose=dfNose.iloc[:,5].values
            array_CN_nose=dfNose.iloc[:,8].values
            array_cop_nose=dfNose.iloc[:,12].values
            array_cop_nose=np.array(array_cop_nose)*0.0254
            CA_lookupmatrix_nose=np.reshape(array_CA_nose,(len(array_alpha_nose_unique),len(array_mach_nose_unique)))
            CN_lookupmatrix_nose=np.reshape(array_CN_nose,(len(array_alpha_nose_unique),len(array_mach_nose_unique)))
            cop_lookupmatrix_nose=np.reshape(array_cop_nose,(len(array_alpha_nose_unique),len(array_mach_nose_unique)))
            CA_lookupNose=scipy.interpolate.interp2d(array_mach_nose_unique,array_alpha_nose_unique,CA_lookupmatrix_nose)
            CN_lookupNose=scipy.interpolate.interp2d(array_mach_nose_unique,array_alpha_nose_unique,CN_lookupmatrix_nose)
            cop_lookupNose=scipy.interpolate.interp2d(array_mach_nose_unique,array_alpha_nose_unique,cop_lookupmatrix_nose)
            dfNose15=pd.read_excel(r'{}\Inputs\RasAeroIINose15.xlsx'.format(Directory),header=0,engine='openpyxl')
            array_mach_nose15=dfNose15.iloc[:,0].values
            array_mach_nose15=np.append(array_mach_nose15,array_mach_nose15[-1])
            array_mach_nose15_unique=np.unique(array_mach_nose15)
            array_alpha_nose15=dfNose15.iloc[:,1].values
            array_alpha_nose15=np.append(array_alpha_nose15,array_alpha_nose15[-1])
            array_alpha_nose15=np.array(array_alpha_nose15)*np.pi/180
            array_alpha_nose15_unique=np.unique(array_alpha_nose15)
            array_CA_nose15=dfNose15.iloc[:,5].values
            array_CN_nose15=dfNose15.iloc[:,8].values
            array_cop_nose15=dfNose15.iloc[:,12].values
            array_cop_nose15=np.array(array_cop_nose15)*0.0254
            CA_lookupmatrix_nose15=np.reshape(array_CA_nose15,(len(array_alpha_nose15_unique),len(array_mach_nose15_unique)))
            CN_lookupmatrix_nose15=np.reshape(array_CN_nose15,(len(array_alpha_nose15_unique),len(array_mach_nose15_unique)))
            cop_lookupmatrix_nose15=np.reshape(array_cop_nose15,(len(array_alpha_nose15_unique),len(array_mach_nose15_unique)))
            CA_lookupNose15=scipy.interpolate.interp2d(array_mach_nose15_unique,array_alpha_nose15_unique,CA_lookupmatrix_nose15)
            CN_lookupNose15=scipy.interpolate.interp2d(array_mach_nose15_unique,array_alpha_nose15_unique,CN_lookupmatrix_nose15)
            cop_lookupNose15=scipy.interpolate.interp2d(array_mach_nose15_unique,array_alpha_nose15_unique,cop_lookupmatrix_nose15)

            """Booster. From aerodynamics.pyw. Can ignore this:"""
            dfBooster=pd.read_excel(r'{}\Inputs\RasAeroIIBooster.xlsx'.format(Directory),header=0,engine='openpyxl')
            array_mach_booster=dfBooster.iloc[:,0].values
            array_mach_booster=np.append(array_mach_booster,array_mach_booster[-1])
            array_mach_booster_unique=np.unique(array_mach_booster)
            array_alpha_booster=dfBooster.iloc[:,1].values
            array_alpha_booster=np.append(array_alpha_booster,array_alpha_booster[-1])
            array_alpha_booster=np.array(array_alpha_booster)*np.pi/180
            array_alpha_booster_unique=np.unique(array_alpha_booster)
            array_CA_booster=dfBooster.iloc[:,5].values
            array_CN_booster=dfBooster.iloc[:,8].values
            array_cop_booster=dfBooster.iloc[:,12].values
            array_cop_booster=np.array(array_cop_booster)*0.0254
            CA_lookupmatrix_booster=np.reshape(array_CA_booster,(len(array_alpha_booster_unique),len(array_mach_booster_unique)))
            CN_lookupmatrix_booster=np.reshape(array_CN_booster,(len(array_alpha_booster_unique),len(array_mach_booster_unique)))
            cop_lookupmatrix_booster=np.reshape(array_cop_booster,(len(array_alpha_booster_unique),len(array_mach_booster_unique)))
            CA_lookupBooster=scipy.interpolate.interp2d(array_mach_booster_unique,array_alpha_booster_unique,CA_lookupmatrix_booster)
            CN_lookupBooster=scipy.interpolate.interp2d(array_mach_booster_unique,array_alpha_booster_unique,CN_lookupmatrix_booster)
            cop_lookupBooster=scipy.interpolate.interp2d(array_mach_booster_unique,array_alpha_booster_unique,cop_lookupmatrix_booster)
            dfBooster15=pd.read_excel(r'{}\Inputs\RasAeroIIBooster15.xlsx'.format(Directory),header=0,engine='openpyxl')
            array_mach_booster15=dfBooster15.iloc[:,0].values
            array_mach_booster15=np.append(array_mach_booster15,array_mach_booster15[-1])
            array_mach_booster15_unique=np.unique(array_mach_booster15)
            array_alpha_booster15=dfBooster15.iloc[:,1].values
            array_alpha_booster15=np.append(array_alpha_booster15,array_alpha_booster15[-1])
            array_alpha_booster15=np.array(array_alpha_booster15)*np.pi/180
            array_alpha_booster15_unique=np.unique(array_alpha_booster15)
            array_CA_booster15=dfBooster15.iloc[:,5].values
            array_CN_booster15=dfBooster15.iloc[:,8].values
            array_cop_booster15=dfBooster15.iloc[:,12].values
            array_cop_booster15=np.array(array_cop_booster15)*0.0254
            CA_lookupmatrix_booster15=np.reshape(array_CA_booster15,(len(array_alpha_booster15_unique),len(array_mach_booster15_unique)))
            CN_lookupmatrix_booster15=np.reshape(array_CN_booster15,(len(array_alpha_booster15_unique),len(array_mach_booster15_unique)))
            cop_lookupmatrix_booster15=np.reshape(array_cop_booster15,(len(array_alpha_booster15_unique),len(array_mach_booster15_unique)))
            CA_lookupBooster15=scipy.interpolate.interp2d(array_mach_booster15_unique,array_alpha_booster15_unique,CA_lookupmatrix_booster15)
            CN_lookupBooster15=scipy.interpolate.interp2d(array_mach_booster15_unique,array_alpha_booster15_unique,CN_lookupmatrix_booster15)
            cop_lookupBooster15=scipy.interpolate.interp2d(array_mach_booster15_unique,array_alpha_booster15_unique,cop_lookupmatrix_booster15)

            RASAero=np.array([0,0,0],dtype=float)
            RASAero15=np.array([0,0,0],dtype=float)
            RASAeroNose=np.array([0,0,0],dtype=float)
            RASAeroNose15=np.array([0,0,0],dtype=float)
            RASAeroBooster=np.array([0,0,0],dtype=float)
            RASAeroBooster15=np.array([0,0,0],dtype=float)

            """From side_damping.pyw. Can ignore this:"""
            dfSideDamping=pd.read_excel(r'{}\inputs\side_damping.xlsx'.format(Directory),header=0,engine='openpyxl')

            """Placeholder variable:"""
            TimeApogee=1e10

            RASAero=np.array([CA_lookup(mach,taoa),CN_lookup(mach,taoa),cop_lookup(mach,taoa)],dtype=float)
            cop=                                    float(RocketLength-RASAero[2])#float(RocketLength-self.aerodynamic_tables.lookup(mach,abs(taoa))[2])
            StabilityMargin=                        float((cog-cop)/(2*rbod))
            RelativeVelocityWind=                   np.array([0,0,0],dtype=float)
            ThrustMagnitude=                        float(df_thrust.iloc[0,1]+(df_thrust.iloc[0,2]-pres)*NozzleExitArea)
            thrust=                                 0
            gr=                                     (EarthGravitationalConstant*EarthMass)/((EarthRadius+alti)**2)
            vwnda=                                  np.array([0,0,0],dtype=float)

##                    """Default orientation quaternion:"""
##                    """qanga0=0.49240387650610407"""
##                    """qanga1=-0.49240387650610395"""
##                    """qanga2=-0.4131759111665348"""
##                    """qanga3=-0.5868240888334652"""
##                    qanga0=math.cos(panga[2]/2)*math.cos(panga[1]/2)*math.cos(panga[0]/2)+math.sin(panga[2]/2)*math.sin(panga[1]/2)*math.sin(panga[0]/2)
##                    qanga1=math.cos(panga[2]/2)*math.cos(panga[1]/2)*math.sin(panga[0]/2)-math.sin(panga[2]/2)*math.sin(panga[1]/2)*math.cos(panga[0]/2)
##                    qanga2=math.cos(panga[2]/2)*math.sin(panga[1]/2)*math.cos(panga[0]/2)+math.sin(panga[2]/2)*math.cos(panga[1]/2)*math.sin(panga[0]/2)
##                    qanga3=math.sin(panga[2]/2)*math.cos(panga[1]/2)*math.cos(panga[0]/2)-math.cos(panga[2]/2)*math.sin(panga[1]/2)*math.sin(panga[0]/2)
##                    """State initial conditions:"""
##                    y0 = [0,0,0,0,0,0,0,0,qanga0,qanga1,qanga2,qanga3]

            print("Monte Carlo Uncertainties:")
            if CheckMonteCarloUI==1:
                print("Thrust Misalignment (Yawing) (deg):",round(self.monte_carlo.outputs()[2],3))
                print("Thrust Misalignment (Pitching) (deg):",round(self.monte_carlo.outputs()[3],3))
                print("Thrust Magnitude Variation (%):",round(self.monte_carlo.outputs()[4],3))
                print("Wind Magnitude Variation (%):",round(self.monte_carlo.outputs()[6],3))
                print("Wind Direction Variation (deg):",round(self.monte_carlo.outputs()[7],3))
                print("Aerodynamic Drag Coefficient Variation (%):",round(self.monte_carlo.outputs()[8],3))
                print("Aerodynamic Lift Coefficient Variation (%):",round(self.monte_carlo.outputs()[9],3))
                print("Aerodynamic Moment Coefficient Variation (%):",round(self.monte_carlo.outputs()[10],3))
                print("Centre-Of-Pressure Variation (%):",round(self.monte_carlo.outputs()[11],3))
                print("Fin Cant Angle Variation (deg):",round(self.monte_carlo.outputs()[12],3))
                print("Launch Altitude (m):",round(LaunchAltitude+self.monte_carlo.outputs()[13],3))
                print("Launch Elevation (deg):",round(abs(-LaunchElevation-self.monte_carlo.outputs()[0]),3))
                print("Launch Azimuth (deg):",round(LaunchAzimuth+self.monte_carlo.outputs()[1],3))
                print("Burnout Time Variation (s):",round(TimeBurn+self.monte_carlo.outputs()[5],3))
            elif CheckMonteCarloUI==0:
                print("Thrust Misalignment (Yawing) (deg):",round(0,3))
                print("Thrust Misalignment (Pitching) (deg):",round(0,3))
                print("Thrust Magnitude Variation (%):",round(0,3))
                print("Wind Magnitude Variation (%):",round(0,3))
                print("Wind Direction Variation (deg):",round(0,3))
                print("Aerodynamic Drag Coefficient Variation (%):",round(0,3))
                print("Aerodynamic Lift Coefficient Variation (%):",round(0,3))
                print("Aerodynamic Moment Coefficient Variation (%):",round(0,3))
                print("Centre-Of-Pressure Variation (%):",round(0,3))
                print("Fin Cant Angle Variation (deg):",round(0,3))
                print("Launch Altitude (m):",round(LaunchAltitude,3))
                print("Launch Elevation (deg):",round(abs(-LaunchElevation),3))
                print("Launch Azimuth (deg):",round(LaunchAzimuth,3))
                print("Burnout Time Variation (s):",round(TimeBurn,3))

            """Previously appended rows to a Pandas dataframe for every iteration, but it slowed the program tremendously. REMOVED:"""
            #OutputColumns=["step_number","time (s)","altitude (m)","mass (kg)","MOI_xx","MOI_yy","MOI_zz","position_kinematic_North (m)","position_kinematic_East (m)","position_kinematic_Down (m)","velocity_kinematic_North (m)","velocity_kinematic_East (m)","velocity_kinematic_Down (m)","acceleration_kinematic_North (m)","acceleration_kinematic_East (m)","acceleration_kinematic_Down (m)","position_angular_roll (rad)","position_angular_pitch (rad)","position_angular_yaw (rad)","velocity_angular_roll (rad)","velocity_angular_pitch (rad)","velocity_angular_yaw (rad)","acceleration_angular_roll (rad)","acceleration_angular_pitch (rad)","acceleration_angular_yaw (rad)","air_speed (m/s)","speed_of_sound (m/s)","mach_number","centre-of-gravity","centre-of-pressure","stability_margin (calibres)","angle_of_attack (rad)","angle_of_sideslip (rad)","total_angle_of_attack (rad)","CD","CL","CC","air_density","dynamic_pressure","aerodynamic_forces_body","aerodynamic_moments_body","wind_magnitude (m/s)","wind_bearing (deg clockwise)","wind_relative_velocity (m/s)","thrust_magnitude (N)","acceleration_gravity (m/s2)","latitude (deg)","longitude (deg)","pitch_damping_coefficient","yaw_damping_coefficient","momentum_thrust (N)","aerodynamic roll angle (rad)","roll moment (Nm)"]
            #dfOutput=pd.DataFrame(columns=OutputColumns)
            #ListOutput=[[StepNumber,t,alti,mass,ixx,iyy,izz,pgeoa[0],pgeoa[1],pgeoa[2],0,0,0,0,0,0,panga[0],panga[1],panga[2],0,0,0,0,0,0,AirSpeed,SpeedOfSound,mach,cog,cop,StabilityMargin,alpa,beta,taoa,Ccof[0],Ccof[1],Ccof[2],rho,pres,np.array([Clinear[0],Clinear[1],Clinear[2]],dtype=float),np.array([Cmom[0],Cmom[1],Cmom[2]],dtype=float),WindMagnitude,WindBearing,np.array([RelativeVelocityWind[0],RelativeVelocityWind[1],RelativeVelocityWind[2]],dtype=float),ThrustMagnitude,gr,(ilatl[0]+0)*(180/math.pi),(ilatl[1]+0)*(180/math.pi),pitdamp,yawdamp,thrust,phip,Cmom[0]]]
            #dfOutputRow=pd.DataFrame(ListOutput,columns=OutputColumns)
            #dfOutput=dfOutput._append(dfOutputRow)
            Range=math.sqrt(pgeoa[0]*pgeoa[0]+pgeoa[1]*pgeoa[1])
            print('time {0:.3f} s.'.format(t),' altitude {0:.3f} m.'.format(-pgeoa[2]),' elevation {0:.3f}\xb0.'.format(panga[1]*(180/math.pi)),' latitude {0:.3f}\xb0 N.'.format((ilatl[0]+ltloa[0])*(180/math.pi)),' longitude {0:.3f}\xb0 E.'.format((ilatl[1]+ltloa[1])*(180/math.pi)),' range {0:.3f} m.'.format(Range) )#,round(Schar,3),round(CA,3))#,round(cop,3),round(cog,3),self.state,mass)

        while t<Variables["TimeMax"]:
##            """Read previous timestep:"""
##            file1 = open("TimestepFirst.txt","r")
##            TimePrevious=float(file1.readlines()[0])
##            file1.close()
            #dt=t-TimePrevious
            dt=Variables["TimeSize"]
            
##            vgeoa[0]=y[0]
##            vgeoa[1]=y[1]
##            vgeoa[2]=y[2]
##            omega[0]=y[3]
##            omega[1]=y[4]
##            omega[2]=y[5]
##            ltloa[0]=y[6]
##            ltloa[1]=y[7]
##            qanga[0]=y[8]
##            qanga[1]=y[9]
##            qanga[2]=y[10]
##            qanga[3]=y[11]

            """Array of state variables 1, 2 and 3:"""
            vgeoa=np.array([vgeoa[0],vgeoa[1],vgeoa[2]],dtype=float)

            Mach=0
            pres=0
            thrustv=np.array([0,0,0],dtype=float)
            Cl=0
            Clins=0
            Cdp=0
            Cltot=0
            Cdptot=0
            fincp=0
            cna=0
            pitdamp=0
            yawdamp=0
            cdef=np.array([0,0,0,0],dtype=float)
            Va2=vaera[0]*vaera[0]+vaera[1]*vaera[1]+vaera[2]*vaera[2]
            Vk2=vgeoa[0]*vgeoa[0]+vgeoa[1]*vgeoa[1]+vgeoa[2]*vgeoa[2]
            gr=0
            J=0
            K=0
            L=0
            D0=0
            D1=0
            D2=0
            D3=0
            gama=0
            xsia=0
            a11pa=0
            a12pa=0
            a13pa=0
            alpa=0
            beta=0
            a11a=0
            a21a=0
            a31a=0
            alpap=0
            betap=0
            phipp=0
            taoap=0
            muap=0
            taoa=0
            phip=0
            mua=0
            VaB=np.array([0,0,0],dtype=float)
            quant1=0
            quant2=0
            Ccof=np.array([0,0,0],dtype=float)
            Cmom=np.array([0,0,0],dtype=float)
            fgeoa=np.array([0,0,0],dtype=float)
            vbody=np.array([0,0,0],dtype=float)
            omegt=np.array([0,0,0],dtype=float)
            omgtt=np.array([0,0,0],dtype=float)
            raild=np.array([1,0,0],dtype=float)
            conmom=np.array([0,0,0],dtype=float)
            marm=0
            alpaeq=0
            betaeq=0 
            Macheq=0
            taoaeq=0
            Clin=np.array([0,0,0],dtype=float)
            peara=              np.array([EarthRadius-pgeoa[2]+LaunchAltitude,0,0],dtype=float)
            alti=               np.linalg.norm(peara)-EarthRadius
            toth=               EarthRadius+alti
            height=             alti
            ambient_pressure = np.interp(height,atmosphere_altitude_array,atmosphere_pressure_array)
            pres=ambient_pressure
            cog=np.interp(t, timem_array, cogm_array)
            mass=np.interp(t, timem_array, massm_array)
            ixx=np.interp(t, timem_array, ixxm_array)
            iyy=np.interp(t, timem_array, iyym_array)
            izz=np.interp(t, timem_array, izzm_array)

            """Update body mass properties:"""
            Check4in1=Variables["Check4in1"]
            if (ApogeeMessage==True) and (Check4in1==True):
                if Nose==True:
                    if (self.BodyState==2 and (t>TimeApogee+Variables["DrogueDelay"]+Variables["StageDelay"])) or (self.BodyState==3):       
                        if taoaeq<=15:
                            mass=Variables["NoseMass"]
                            cog=Variables["NoseCOMx"]
                            cogy=Variables["NoseCOMy"]
                            cogz=Variables["NoseCOMz"]
                            ixx=Variables["NoseMOIx"]
                            iyy=Variables["NoseMOIy"]
                            izz=Variables["NoseMOIz"]
                            if self.BodyState==2:
                                """2kg parachute mass:"""
                                mass+=2
                if Booster==True:
                    mass=BoosterMass=Variables["BoosterMass"]
                    cog=BoosterCOMx=Variables["BoosterCOMx"]
                    cogy=Variables["BoosterCOMy"]
                    cogz=Variables["BoosterCOMz"]
                    ixx=Variables["BoosterMOIx"]
                    iyy=Variables["BoosterMOIy"]
                    izz=Variables["BoosterMOIz"]
            CheckStaging=Variables["CheckStaging"]
            if CheckStaging==True:             
                StageTime=Variables["StageTime"]
                StageDelay=Variables["StageDelay"]
                if t>StageTime+StageDelay:
                    """Aerodynamic file input"""
                    NumberStages=Variables["NumberStages"]
                    StageMass=Variables["StageMass"]
                    StageCOMx=Variables["StageCOMx"]
                    StageCOMy=Variables["StageCOMy"]
                    StageCOMz=Variables["StageCOMz"]
                    StageMOIx=Variables["StageMOIx"]
                    StageMOIy=Variables["StageMOIy"]
                    StageMOIz=Variables["StageMOIz"]
                    self.BodyState=5

            inertia[0][0]=ixx
            inertia[1][1]=iyy
            inertia[2][2]=izz
            inertia[0][1]=ixy
            inertia[0][2]=ixz
            inertia[1][2]=iyz
            A=inertia[0][0]
            B=inertia[1][1]
            C=inertia[2][2]
            D=-inertia[1][2]
            E=-inertia[0][2]
            F=-inertia[0][1]
            """PyROPS gravity convention: downward is positive:"""
            AccelerationGravity=(EarthGravitationalConstant*EarthMass)/((EarthRadius+alti)**2)
            gr=AccelerationGravity

            """Program starts. Begins with computation of the aerodynamic angle of attack (see HYROPS key for variable names)""" 
            if(Va2>0):
                gama=math.asin(vaera[2]/math.sqrt(Va2))
                quant2=vaera[0]/(math.sqrt(Va2)*math.cos(gama))
                if quant2>=-1 and quant2<=1:
                    xsia=math.acos(quant2)
                if(vaera[1]>0):
                    xsia=(2*math.pi)-xsia

            a11pa=(math.cos(xsia)*math.cos(gama)*math.cos(panga[1])*math.cos(panga[2]))+(math.sin(xsia)*math.cos(gama)*math.sin(panga[2])*math.cos(panga[1]))+(math.sin(gama)*math.sin(panga[1]))
            a12pa=(math.cos(xsia)*math.cos(gama)*((math.sin(panga[1])*math.sin(panga[0])*math.cos(panga[2]))-(math.sin(panga[2])*math.cos(panga[0]))))+(math.sin(xsia)*math.cos(gama)*((math.sin(panga[1])*math.sin(panga[0])*math.sin(panga[2]))+(math.cos(panga[2])*math.cos(panga[0]))))-(math.sin(gama)*math.cos(panga[1])*math.sin(panga[0]))
            a13pa=(math.cos(xsia)*math.cos(gama)*((math.cos(panga[2])*math.sin(panga[1])*math.cos(panga[0]))+(math.sin(panga[0])*math.sin(panga[2]))))+(math.sin(xsia)*math.cos(gama)*((math.sin(panga[1])*math.cos(panga[0])*math.sin(panga[2]))-(math.sin(panga[0])*math.cos(panga[2]))))-(math.sin(gama)*math.cos(panga[1])*math.cos(panga[0]))
            
            beta=math.asin(a12pa)
            if(math.cos(beta)!=0):
                quant2=a13pa/math.cos(beta)
                if((quant2>=-1) and (quant2<=1)):
                    alpa=math.asin(quant2)
                    if(a11pa<0):
                        if(alpa>0):
                            alpa=math.pi-alpa
                        if(alpa<0):
                            alpa=-math.pi-alpa
                
            while(alpa>(2*math.pi)):
                alpa-=2*math.pi
            while(alpa<-(2*math.pi)):
                alpa+=2*math.pi
            while(beta>(2*math.pi)):
                beta-=2*math.pi
            while(beta<-(2*math.pi)):
                beta+=2*math.pi

            VaB=np.array([math.sqrt(Va2),0,0],dtype=float)
            VaB=Rotation.from_euler('xyz',[0,0,beta*(180/math.pi)],degrees=True).apply(VaB)
            VaB=Rotation.from_euler('xyz',[0,-alpa*(180/math.pi),0],degrees=True).apply(VaB)
            if Va2>0:
                taoa=math.asin(math.sqrt((VaB[1]*VaB[1])+(VaB[2]*VaB[2]))/math.sqrt(Va2))
            
            phip=0
            if abs(VaB[2])>0:
                phip=math.atan(abs(VaB[1])/abs(VaB[2]))
                if((VaB[1]>=0) and (VaB[2]<0)):
                    phip=math.pi-phip
                if((VaB[1]<=0) and (VaB[2]<0)):
                    phip=math.pi+phip
                if((VaB[1]<=0) and (VaB[2]>0)):
                    phip=(2*math.pi)-phip

            if(VaB[2]==0):
                if(VaB[1]>0):
                    phip=math.pi*0.5
                if(VaB[1]<0):
                    phip=math.pi*1.5

            if CheckMonteCarloUI==1:
                thrust_vector=Rotation.from_euler('xyz',[0,self.monte_carlo.outputs()[3],0],degrees=True).apply(thrust_vector_ideal)
                """thrustv RotateY(MC_TM) in HYROPS, MC_TM=MC_THRUST_MISALIGNMENT*NormalRandom()*rad, MC_TM=0 without uncertainty."""
                thrust_vector=Rotation.from_euler('xyz',[0,0,self.monte_carlo.outputs()[2]],degrees=True).apply(thrust_vector)
                """thrustv RotateX(MC_TRA) in HYROPS, MC_TRA=NormalRandom()*2.0*PI, MC_TRA=0 without uncertainty."""
            else:
                thrust_vector=Rotation.from_euler('xyz',[0,0,0],degrees=True).apply(thrust_vector_ideal)
                """thrustv RotateY(MC_TM) in HYROPS, MC_TM=MC_THRUST_MISALIGNMENT*NormalRandom()*rad, MC_TM=0 without uncertainty."""
                thrust_vector=Rotation.from_euler('xyz',[0,0,0],degrees=True).apply(thrust_vector)                
                """thrustv RotateX(MC_TRA) in HYROPS, MC_TRA=NormalRandom()*2.0*PI, MC_TRA=0 without uncertainty."""

            CheckThrust=Variables["CheckThrust"]

            BurnTimeActual=(TimeBurn+self.monte_carlo.outputs()[5])
            if CheckMonteCarloUI==0:
                BurnTimeActual=(TimeBurn)

            if t<BurnTimeActual:
                """time*=MC_THRFAC in HYROPS, MC_THRFAC=1.0+(MC_MOTOR_BURNTIME*NormalRandom()), MC_THRFAC=1.0 without uncertainty."""
                """Approximating the momentum thrust. Using n-th degree thrust curve polynomial specified by user:"""
                if CheckThrust==True:
                    MomentumThrustCurveFit=self.thrust_curve.fit(t)
                    """Removed '+self.monte_carlo.outputs()[4]':"""
                    thrust = (MomentumThrustCurveFit+(np.interp(t,timey_array,pressure_array)-ambient_pressure)*NozzleExitArea)
                    if CheckMonteCarloUI==1:
                        thrust+=thrust*((Variables["ThrustMagnitudeLower"]+stats.truncnorm.rvs((0)/1.1,(1)/1.1,loc=0,scale=1.1,size=1)[0]*(Variables["ThrustMagnitudeUpper"]-Variables["ThrustMagnitudeLower"])))/100
                        """thrust*MC_THRMAG in HYROPS, MC_THRMAG=1.0+(MC_THRUST_MAGNITUDE*NormalRandom()), MC_THRMAG=1.0 without uncertainty. # Nozzle exit area: 0.007056m2 for P1BIIr."""
                    thrustv=thrust * thrust_vector 
                    ThrustMagnitude= np.linalg.norm(thrustv)

                else:
                    """Else, reading momentum thrust directly from thrust curve input file:"""
                    """Removed '+self.monte_carlo.outputs()[4]':"""
                    thrust = (np.interp(t, timey_array, thrusty_array)+(np.interp(t,timey_array,pressure_array)-ambient_pressure)*NozzleExitArea)
                    if CheckMonteCarloUI==1:
                        thrust+=thrust*((Variables["ThrustMagnitudeLower"]+stats.truncnorm.rvs((0)/1.1,(1)/1.1,loc=0,scale=1.1,size=1)[0]*(Variables["ThrustMagnitudeUpper"]-Variables["ThrustMagnitudeLower"])))/100
                        """thrust*MC_THRMAG in HYROPS, MC_THRMAG=1.0+(MC_THRUST_MAGNITUDE*NormalRandom()), MC_THRMAG=1.0 without uncertainty. # Nozzle exit area: 0.007056m2 for P1BIIr."""    
                    thrustv=thrust * thrust_vector 
                    ThrustMagnitude= np.linalg.norm(thrustv)

            elif self.BodyState==5:
                """Approximating the momentum thrust. Using n-th degree thrust curve polynomial specified by user:"""
                if CheckThrust==True:
                    df_thrust_staging=pd.read_excel(r'{}\inputs\thrust_curve_staging.xlsx'.format(Directory),header=0)
                    timey_array=df_thrust_staging.iloc[:,0].values
                    thrusty_array=df_thrust_staging.iloc[:,1].values
                    pressure_array=df_thrust_staging.iloc[:,2].values
                    MomentumThrustCurveFit=self.thrust_curve.fit(t)
                    """Removed '+self.monte_carlo.outputs()[4]':"""
                    thrust = (MomentumThrustCurveFit+(np.interp(t,timey_array,pressure_array)-ambient_pressure)*NozzleExitArea)
                    if CheckMonteCarloUI==1:
                        thrust+=thrust*((Variables["ThrustMagnitudeLower"]+stats.truncnorm.rvs((0)/1.1,(1)/1.1,loc=0,scale=1.1,size=1)[0]*(Variables["ThrustMagnitudeUpper"]-Variables["ThrustMagnitudeLower"])))/100
                        """thrust*MC_THRMAG in HYROPS, MC_THRMAG=1.0+(MC_THRUST_MAGNITUDE*NormalRandom()), MC_THRMAG=1.0 without uncertainty. # Nozzle exit area: 0.007056m2 for P1BIIr."""
                    thrustv=thrust * thrust_vector 
                    ThrustMagnitude= np.linalg.norm(thrustv)

                else:
                    """Else, Reading momentum thrust directly from thrust curve input file:"""
                    df_thrust_staging=pd.read_excel(r'{}\inputs\thrust_curve_staging.xlsx'.format(Directory),header=0)
                    timey_array=df_thrust_staging.iloc[:,0].values
                    thrusty_array=df_thrust_staging.iloc[:,1].values
                    pressure_array=df_thrust_staging.iloc[:,2].values
                    """Removed '+self.monte_carlo.outputs()[4]':"""
                    thrust = (np.interp(t, timey_array, thrusty_array)+(np.interp(t,timey_array,pressure_array)-ambient_pressure)*NozzleExitArea)
                    if CheckMonteCarloUI==1:
                        thrust+=thrust*((Variables["ThrustMagnitudeLower"]+stats.truncnorm.rvs((0)/1.1,(1)/1.1,loc=0,scale=1.1,size=1)[0]*(Variables["ThrustMagnitudeUpper"]-Variables["ThrustMagnitudeLower"])))/100
                        """thrust*MC_THRMAG in HYROPS, MC_THRMAG=1.0+(MC_THRUST_MAGNITUDE*NormalRandom()), MC_THRMAG=1.0 without uncertainty. # Nozzle exit area: 0.007056m2 for P1BIIr."""    
                    thrustv=thrust * thrust_vector 
                    ThrustMagnitude= np.linalg.norm(thrustv)
            else:
                if BurnoutMessage==False:
                    #print("Burnout at T{0:.3f} seconds.".format(t))
                    BurnoutMessage=True
                thrust=0
                thrustv=np.array([0,0,0],dtype=float)
                forceThrustBody=np.array([0,0,0],dtype=float)

            AirSpeed=math.sqrt(Va2)
            ambient_density = np.interp(height,atmosphere_altitude_array,atmosphere_density_array)
            rho=ambient_density
            SpeedOfSound = math.sqrt(gamma*Rair*np.interp(height,atmosphere_altitude_array,atmosphere_temperature_array))    
            mach=AirSpeed/SpeedOfSound
            Macheq=mach
            if(Macheq>=5):
                Macheq=5
            if(Macheq<0.01):
                Macheq=0.01
            taoaeq=abs(taoa*(180/math.pi))

            if(taoaeq>4):
                if Lengthpgeoa>RailLength and AngleOfAttackMessage==False:
                    print("Angle of attack:",taoaeq,"degrees exceeds 4 degrees at T{0:.3f} seconds. Flight conditions are not favorable.".format(t))
                    AngleOfAttackMessage=True

            """The below commented-out code was removed. It was initially included in the simulator, line 4765 in simulatex.h in HYROPS code:"""
            #if(taoaeq>15):
            #    taoaeq=15
            #    taoa=taoaeq*math.pi/180

            ParachuteList.append(-pgeoa[2])
            if len(ParachuteList)>2:
                TrackAltitude1=ParachuteList[-1]-ParachuteList[-2]
#                TrackAltitude2=ParachuteList[-2]-ParachuteList[-3]
                
            if TrackAltitude1<0:
                if ApogeeMessage==False:
                    ApogeeMessage=True
                    TimeApogee=t
                    print("Apogee {0:.3f}m".format(ParachuteList[-3])," reached at T{0:.3f} seconds".format(t-dt))
                    
                    """Write Apogee Time:"""
                    file1 = open("Apogee.txt","w")
                    file1.write(str(t))
                    file1.close()

            """Uncertainties during separation at apogee:"""
            if Variables["Check4in1"]==True and t>TimeApogee+Variables["StageDelay"] and Variables["CheckTurbulence"]==True:
                if SeparationApogeeMessage==False:
                    SeparationApogeeMessage=True
                    print("Apogee Elevation Variation (deg):",round(abs(-self.monte_carlo.outputs()[17]),3))
                    print("Apogee Azimuth Variation (deg):",round(self.monte_carlo.outputs()[18],3))
                    print("Apogee Wind Direction Variation (deg):",round(self.monte_carlo.outputs()[19],3))
                    elevf=0
                    numer=0
                    denom=0
                    numer=2*((qanga[1]*qanga[2])+(qanga[0]*qanga[3]))
                    denom=(qanga[0]*qanga[0])+(qanga[1]*qanga[1])-(qanga[2]*qanga[2])-(qanga[3]*qanga[3])
                    finpa=np.array([0,0,0],dtype=float) 
                    if numer>=0 and denom>0:
                        finpa[2]=math.atan(numer/denom)
                    if numer<=0 and denom<0:
                        finpa[2]=math.atan(abs(numer)/abs(denom))+math.pi
                    if numer>=0 and denom<0:
                        finpa[2]=math.pi-math.atan(abs(numer)/abs(denom))
                    if numer<=0 and denom>0:
                        finpa[2]=2*math.pi-math.atan(abs(numer)/abs(denom))
                    numer=2*((qanga[2]*qanga[3])+(qanga[0]*qanga[1]))
                    denom=(qanga[0]*qanga[0])-(qanga[1]*qanga[1])-(qanga[2]*qanga[2])+(qanga[3]*qanga[3])
                    if numer>=0 and denom>0:
                        finpa[0]=math.atan(numer/denom)
                    if numer<=0 and denom<0:
                        finpa[0]=math.atan(abs(numer)/abs(denom))+math.pi
                    if numer>=0 and denom<0:
                        finpa[0]=math.pi-math.atan(abs(numer)/abs(denom))
                    if numer<=0 and denom>0:
                        finpa[0]=2*math.pi-math.atan(abs(numer)/abs(denom))
                    elevf=-2*((qanga[1]*qanga[3])-(qanga[0]*qanga[2]))
                    if elevf>=-1 and elevf<=1:
                        finpa[1]=math.asin(elevf)
                    while(finpa[0]>(2*math.pi)):
                        finpa[0]-=2*math.pi
                    while(finpa[0]<(-2*math.pi)):
                        finpa[0]+=2*math.pi
                    while(finpa[1]>(2*math.pi)):
                        finpa[1]-=2*math.pi
                    while(finpa[1]<(-2*math.pi)):
                        finpa[1]+=2*math.pi
                    while finpa[2]>2*math.pi:
                        finpa[2]-=2*math.pi
                    while finpa[2]<(-2*math.pi):
                        finpa[2]+=2*math.pi

                    panga=np.array([finpa[0],finpa[1]-self.monte_carlo.outputs()[17]*(math.pi/180),finpa[2]+self.monte_carlo.outputs()[18]*(math.pi/180)],dtype=float)
                    qanga[0]=math.cos(panga[2]/2)*math.cos(panga[1]/2)*math.cos(panga[0]/2)+math.sin(panga[2]/2)*math.sin(panga[1]/2)*math.sin(panga[0]/2)
                    qanga[1]=math.cos(panga[2]/2)*math.cos(panga[1]/2)*math.sin(panga[0]/2)-math.sin(panga[2]/2)*math.sin(panga[1]/2)*math.cos(panga[0]/2)
                    qanga[2]=math.cos(panga[2]/2)*math.sin(panga[1]/2)*math.cos(panga[0]/2)+math.sin(panga[2]/2)*math.cos(panga[1]/2)*math.sin(panga[0]/2)
                    qanga[3]=math.sin(panga[2]/2)*math.cos(panga[1]/2)*math.cos(panga[0]/2)-math.cos(panga[2]/2)*math.sin(panga[1]/2)*math.sin(panga[0]/2)
                    
            RASAero=np.array([CA_lookup(mach,taoa),CN_lookup(mach,taoa),cop_lookup(mach,taoa)],dtype=float)
            RASAero15=np.array([CA_lookup15(mach,taoa),CN_lookup15(mach,taoa),cop_lookup15(mach,taoa)],dtype=float)
            RASAeroNose=np.array([CA_lookupNose(mach,taoa),CN_lookupNose(mach,taoa),cop_lookupNose(mach,taoa)],dtype=float)
            RASAeroNose15=np.array([CA_lookupNose15(mach,taoa),CN_lookupNose15(mach,taoa),cop_lookupNose15(mach,taoa)],dtype=float)
            RASAeroBooster=np.array([CA_lookupBooster(mach,taoa),CN_lookupBooster(mach,taoa),cop_lookupBooster(mach,taoa)],dtype=float)
            RASAeroBooster15=np.array([CA_lookupBooster15(mach,taoa),CN_lookupBooster15(mach,taoa),cop_lookupBooster15(mach,taoa)],dtype=float)

            if taoaeq<=4:
                CA_non_mc = float(RASAero[0])#float(self.aerodynamic_tables.lookup(mach,abs(taoa))[0])
                CN_non_mc = float(RASAero[1])#float(self.aerodynamic_tables.lookup(mach,abs(taoa))[1])
                cop_non_mc = float(RocketLength-RASAero[2])#float(RocketLength-self.aerodynamic_tables.lookup(mach,abs(taoa))[2])

            elif 4<taoaeq<=15:
                CA_non_mc = float(RASAero15[0])#float(self.aerodynamic_tables.lookup15(mach,abs(taoa))[0])
                CN_non_mc = float(RASAero15[1])#float(self.aerodynamic_tables.lookup15(mach,abs(taoa))[1])
                cop_non_mc = float(RocketLength-RASAero15[2])#float(RocketLength-self.aerodynamic_tables.lookup15(mach,abs(taoa))[2])

            else:
                CA_non_mc = float(CA_lookup15(0,0*math.pi/180))#float(self.aerodynamic_tables.lookup15(mach,abs(taoa))[0])
                CN_non_mc = float(CN_lookup15(0,0*math.pi/180))#float(self.aerodynamic_tables.lookup15(mach,abs(taoa))[1])
                cop_non_mc = float(RocketLength-cop_lookup15(0,0*math.pi/180))#float(RocketLength-self.aerodynamic_tables.lookup15(mach,abs(0))[2])
                
            """In HYROPS the below commented-out code was enabled as the lookup tables looked at empty cells when total angle of attack exceeded 15 degrees, setting aerodynamic coefficients CA and CN (in the Body Frame) to zero whenever the condition is met, which is incorrect."""
            """The aerodynamic lookup tables return this in HYROPS. See lines 4146 to line 4149 in 'LookupParamsMSVM' in simulatex.h in the HYROPS code."""
            #else:
            #    CA_non_mc=0
            #    CN_non_mc=0

            """Modelling body state aerodynamics post-separation:"""
            if ApogeeMessage==True:
                """Modelling the aerodynamics of the assembly of the parachute and the nosecone containing the payload:"""
                if Variables["Check4in1"]==True and self.BodyState==2:
##                    """Read previous timestep:"""
##                    file1=open("Apogee.txt","r")
##                    TimeApogee=float(file1.readlines()[0])
##                    file1.close()
                    if t>TimeApogee+Variables["DrogueDelay"]+Variables["StageDelay"]:
                        if taoaeq<=4:
                            CA_non_mc=0
                            """Do a CFD to approximate CN for both the parachute and the nosecone/payload assembly at taoa=0"""
                            """CN=0.4*taoa*10, used in the below line, is what was used in HYROPS. 'CN=0.4*taoa*10.0;' at HYROPS simulatex.h line 5184:"""
                            CN_non_mc=4*taoa
                            """In HYRORPS, an aerodynamic moment arm is 1 metre is used once the parachute is deployed. 'double Xc=1.0;' in line 5103 in simulatex.h. 'Xc' is used in the computation of aerodynamic pitch and yaw moments in line 5202 and line 5203 in simulatex.h"""
                            cop_non_mc=cog-1
                        elif 4<taoaeq<=15:
                            CA_non_mc=0
                            """Do a CFD to approximate CN for both the parachute and the nosecone/payload assembly at taoa=0"""
                            """CN=0.4*taoa*10, used in the below line, is what was used in HYROPS. 'CN=0.4*taoa*10.0;' at HYROPS simulatex.h line 5184:"""
                            CN_non_mc=4*taoa
                            """In HYRORPS, an aerodynamic moment arm is 1 metre is used once the parachute is deployed. 'double Xc=1.0;' in line 5103 in simulatex.h. 'Xc' is used in the computation of aerodynamic pitch and yaw moments in line 5202 and line 5203 in simulatex.h"""
                            cop_non_mc=cog-1
                        elif taoaeq<=90:
                            CA_non_mc=0
                            """Do a CFD to approximate CN for both the parachute and the nosecone/payload assembly at taoa=0"""
                            """CN=0.4*taoa*10, used in the below line, is what was used in HYROPS. 'CN=0.4*taoa*10.0;' at HYROPS simulatex.h line 5184:"""
                            CN_non_mc=4*taoa
                            """In HYRORPS, an aerodynamic moment arm is 1 metre is used once the parachute is deployed. 'double Xc=1.0;' in line 5103 in simulatex.h. 'Xc' is used in the computation of aerodynamic pitch and yaw moments in line 5202 and line 5203 in simulatex.h"""
                            cop_non_mc=cog-1
                        else:
                            CA_non_mc=0
                            """Do a CFD to approximate CN for both the parachute and the nosecone/payload assembly at taoa=0"""
                            """CN=0.4*taoa*10, used in the below line, is what was used in HYROPS. 'CN=0.4*taoa*10.0;' at HYROPS simulatex.h line 5184:"""
                            CN_non_mc=4*taoa
                            """In HYRORPS, an aerodynamic moment arm is 1 metre is used once the parachute is deployed. 'double Xc=1.0;' in line 5103 in simulatex.h. 'Xc' is used in the computation of aerodynamic pitch and yaw moments in line 5202 and line 5203 in simulatex.h"""
                            cop_non_mc=cog-1                        
                        """In HYROPS this is enabled:"""
                        #else:
                        #    CA_non_mc=0
                        #    CN_non_mc=0
                        
                        """The rbod should be 'Variables["ParachuteDiameter"]/2' """
                        rbod=Variables["ParachuteDiameter"]/2
                        if (t>TimeApogee+Variables["DrogueDelay"]+Variables["StageDelay"]) and (pgeoa[2]<=-(Variables["ParachuteDelay"])):  
                            rbod=Variables["DrogueDiameter"]/2
                        Schar=math.pi*rbod*rbod
  
                """Modelling the aerodynamics of the ballistic nosecone, unfortunately containing the payload:"""
                if Variables["Check4in1"]==True and self.BodyState==3:
##                    """Read previous timestep:"""
##                    file1=open("Apogee.txt","r")
##                    TimeApogee=float(file1.readlines()[0])
##                    file1.close()
                    if t>TimeApogee+Variables["StageDelay"]:
                        if taoaeq<=4:
                            CA_non_mc=RASAeroNose[0]
                            CN_non_mc=RASAeroNose[1]
                            cop_non_mc=float(Variables["NoseLength"]-RASAeroNose[2])
                        elif 4<taoaeq<=15:
                            CA_non_mc=RASAeroNose15[0]
                            CN_non_mc=RASAeroNose15[1]
                            cop_non_mc=float(Variables["NoseLength"]-RASAeroNose15[2])
                        else:
                            CA_non_mc = CA_lookupNose15(mach,15*math.pi/180)
                            CN_non_mc = CN_lookupNose15(mach,15*math.pi/180)
                            cop_non_mc =float(Variables["NoseLength"]-cop_lookupNose15(mach,15*math.pi/180))
                        """In HYROPS this is enabled:"""
                        #else:
                        #    CA_non_mc=0
                        #    CN_non_mc=0
                        rbod=Variables["NoseRadius"]
                        Schar=math.pi*rbod*rbod

                """Modelling the aerodynamics of the ballistic body separated from the nosecone and payload:"""
                if Variables["Check4in1"]==True and self.BodyState==4:
##                    """Read previous timestep:"""
##                    file1=open("Apogee.txt","r")
##                    TimeApogee=float(file1.readlines()[0])
##                    file1.close()     
                    if t>TimeApogee+Variables["StageDelay"]:
                        if taoaeq<=4:
                            CA_non_mc=RASAeroBooster[0]
                            CN_non_mc=RASAeroBooster[1]
                            cop_non_mc=float((RocketLength-(Variables["NoseLength"]-2*Variables["NoseRadius"]))-RASAeroBooster[2])
                        elif 4<taoaeq<=15:
                            CA_non_mc=RASAeroBooster15[0]
                            CN_non_mc=RASAeroBooster15[1]
                            cop_non_mc=float((RocketLength-(Variables["NoseLength"]-2*Variables["NoseRadius"]))-RASAeroBooster15[2])
                        else:
                            CA_non_mc = float(CA_lookupBooster15(0,0*math.pi/180))
                            CN_non_mc = float(CN_lookupBooster15(0,0*math.pi/180))
                            cop_non_mc = float((RocketLength-(Variables["NoseLength"]-2*Variables["NoseRadius"]))-cop_lookupBooster15(0,0*math.pi/180))
                        """In HYROPS this is enabled:"""
                        #else:
                        #    CA_non_mc=0
                        #    CN_non_mc=0
                        rbod=Variables["RocketBodyRadius"]
                        Schar=math.pi*rbod*rbod

            Finless=False
            if self.BodyState==2 or self.BodyState==3:
                Finless=True
            
            if Variables["Check4in1"]==False:
                CA=CA_non_mc*(1+(self.monte_carlo.outputs()[8]/100)) #CA_non_mc+self.monte_carlo.outputs()[8]
                if CheckMonteCarloUI==0:
                    CA=CA_non_mc
                """ Clin[0]*=MC_DRAG in HYROPS, MC_DRAG=1.0+(MC_DRAG_COEFFICIENT*NormalRandom()), MC_DRAG=1.0 without uncertainty."""
                CN=CN_non_mc*(1+(self.monte_carlo.outputs()[9]/100)) #CN_non_mc+self.monte_carlo.outputs()[9]
                if CheckMonteCarloUI==0:
                    CN=CN_non_mc
                """Clin[2]*=MC_LIFT in HYROPS, MC_LIFT=1.0+(MC_LIFT_COEFFICIENT*NormalRandom()), MC_LIFT=1.0 without uncertainty."""
                cop=cop_non_mc*(1+(self.monte_carlo.outputs()[11]/100)) #cop_non_mc+self.monte_carlo.outputs()[11]
                """cop*=MC_CP in HYROPS, MC_CP=1.0+(MC_CENTER_OF_PRESSURE*NormalRandom()), MC_CP=1.0 without uncertainty."""
                if CheckMonteCarloUI==0:
                    cop=cop_non_mc
            else:
                if t<=TimeApogee+Variables["StageDelay"]:
                    CA=CA_non_mc*(1+(self.monte_carlo.outputs()[8]/100)) #CA_non_mc+self.monte_carlo.outputs()[8]
                    if CheckMonteCarloUI==0:
                        CA=CA_non_mc
                    """ Clin[0]*=MC_DRAG in HYROPS, MC_DRAG=1.0+(MC_DRAG_COEFFICIENT*NormalRandom()), MC_DRAG=1.0 without uncertainty."""
                    CN=CN_non_mc*(1+(self.monte_carlo.outputs()[9]/100)) #CN_non_mc+self.monte_carlo.outputs()[9]
                    if CheckMonteCarloUI==0:
                        CN=CN_non_mc
                    """Clin[2]*=MC_LIFT in HYROPS, MC_LIFT=1.0+(MC_LIFT_COEFFICIENT*NormalRandom()), MC_LIFT=1.0 without uncertainty."""
                    cop=cop_non_mc*(1+(self.monte_carlo.outputs()[11]/100)) #cop_non_mc+self.monte_carlo.outputs()[11]
                    if CheckMonteCarloUI==0:
                        cop=cop_non_mc
                    """cop*=MC_CP in HYROPS, MC_CP=1.0+(MC_CENTER_OF_PRESSURE*NormalRandom()), MC_CP=1.0 without uncertainty."""                 
                elif t>TimeApogee+Variables["StageDelay"] and Finless==False:
                    CA=CA_non_mc*(1+(self.monte_carlo.outputs()[8]/100)) #CA_non_mc+self.monte_carlo.outputs()[8]
                    if CheckMonteCarloUI==0:
                        CA=CA_non_mc
                    """ Clin[0]*=MC_DRAG in HYROPS, MC_DRAG=1.0+(MC_DRAG_COEFFICIENT*NormalRandom()), MC_DRAG=1.0 without uncertainty."""
                    CN=CN_non_mc*(1+(self.monte_carlo.outputs()[9]/100)) #CN_non_mc+self.monte_carlo.outputs()[9]
                    if CheckMonteCarloUI==0:
                        CN=CN_non_mc
                    """Clin[2]*=MC_LIFT in HYROPS, MC_LIFT=1.0+(MC_LIFT_COEFFICIENT*NormalRandom()), MC_LIFT=1.0 without uncertainty."""
                    cop=cop_non_mc*(1+(self.monte_carlo.outputs()[11]/100)) #cop_non_mc+self.monte_carlo.outputs()[11]
                    if CheckMonteCarloUI==0:
                        cop=cop_non_mc
                    """cop*=MC_CP in HYROPS, MC_CP=1.0+(MC_CENTER_OF_PRESSURE*NormalRandom()), MC_CP=1.0 without uncertainty."""
                else:
                    CA=CA_non_mc
                    """ Clin[0]*=MC_DRAG in HYROPS, MC_DRAG=1.0+(MC_DRAG_COEFFICIENT*NormalRandom()), MC_DRAG=1.0 without uncertainty."""
                    CN=CN_non_mc
                    """Clin[2]*=MC_LIFT in HYROPS, MC_LIFT=1.0+(MC_LIFT_COEFFICIENT*NormalRandom()), MC_LIFT=1.0 without uncertainty."""
                    cop=cop_non_mc
                    """cop*=MC_CP in HYROPS, MC_CP=1.0+(MC_CENTER_OF_PRESSURE*NormalRandom()), MC_CP=1.0 without uncertainty."""   

            StabilityMargin=(cog-cop)/(2*rbod)
            if StabilityMargin<0:
                if StabilityMarginMessage==False:
                    print("StabilityMargin: {0:.3f}".format(StabilityMargin)+" calibres less than zero at T{000:.3f} seconds. Flight is unstable.".format(t))
                    StabilityMarginMessage=True
                """Remove the below commented-out two lines of code. For investigating simulation convergence:"""
                #cop=cog
                #StabilityMargin=(cog-cop)/(2*rbod)
                #return

            Clin[0]=CA
            Clin[2]=CN
            Clins=Clin[2]

            """Linear aerodynamic coefficients:"""
            Ccof[0]=-Clin[0]
            Ccof[1]= Clins*math.sin(phip)
            Ccof[2]= Clins*math.cos(phip)

            """Side Damping:"""
            """Moving the call in the following line of the pandas .read_excel method to outside the loop increased runtime fivefold:"""
##                dfSideDamping=pd.read_excel(r'{}\inputs\side_damping.xlsx'.format(Directory),header=0,engine='openpyxl')
            r1=float(dfSideDamping.at[0,"Segment Radius (m)"])
            r2=float(dfSideDamping.at[1,"Segment Radius (m)"])
            r3=float(dfSideDamping.at[2,"Segment Radius (m)"])
            r4=float(dfSideDamping.at[3,"Segment Radius (m)"])
            r5=float(dfSideDamping.at[3,"Segment Radius (m)"])
            r6=float(dfSideDamping.at[3,"Segment Radius (m)"])
            r7=float(dfSideDamping.at[3,"Segment Radius (m)"])
            r8=float(dfSideDamping.at[3,"Segment Radius (m)"])
            s1=float(dfSideDamping.at[0,"Segment Length (m)"])
            s2=float(dfSideDamping.at[1,"Segment Length (m)"])
            s3=float(dfSideDamping.at[2,"Segment Length (m)"])
            s4=float(dfSideDamping.at[3,"Segment Length (m)"])
            s5=float(dfSideDamping.at[3,"Segment Length (m)"])
            s6=float(dfSideDamping.at[3,"Segment Length (m)"])
            s7=float(dfSideDamping.at[3,"Segment Length (m)"])
            s8=float(dfSideDamping.at[3,"Segment Length (m)"])
            if t>TimeApogee+Variables["StageDelay"] and Finless==True:
                r1=Variables["NoseRadius"]
                r2=0
                r3=0
                r4=0
                r5=0
                r6=0
                r7=0
                r8=0
                s1=Variables["NoseLength"]
                s2=0
                s3=0
                s4=0
                s5=0
                s6=0
                s7=0
                s8=0

            """Damping Variables:"""
            NoseRadius=                             Variables["NoseRadius"]
            NoseLength=                             Variables["NoseLength"]
            FinRootChord=                           Variables["FinRootChord"]
            FinTipChord=                            Variables["FinTipChord"]
            FinSweep=                               Variables["FinSweep"]
            FinSpan=                                Variables["FinSpan"]
            FinLocation=                            Variables["FinLocation"]
                
            RadiusOgive=((NoseRadius*NoseRadius)+(NoseLength*NoseLength))/(2*NoseRadius)
            AreaNose=(math.asin(NoseLength/RadiusOgive)*RadiusOgive*RadiusOgive)-(NoseLength*(RadiusOgive-NoseRadius))
            TotalLength=s1+s2+s3+s4
            Aeq=(NoseRadius+r1)*s1+(r1+r2)*s2+(r2+r3)*s3+(r3+r4)*s4+(r4+r5)*s5+(r5+r6)*s6+(r6+r7)*s7+(r7+r8)*s8+AreaNose
            Leq=NoseLength+TotalLength
            Deq=Aeq/Leq
            if Va2>0:
                m1=(0.275*Deq)/Schar
                m2=((cog**4)+(Leq-cog)**4)
                m3=(0.6*4*((FinRootChord+FinTipChord)*0.5*FinSpan)*(((FinLocation+0.5*FinSweep+0.25*(FinRootChord+FinTipChord))-cog)**3))/Schar
                multiplier=3*(m1*m2+m3)
                pitdamp=multiplier*(omega[1]/math.sqrt(Va2))**2
                yawdamp=multiplier*(omega[2]/math.sqrt(Va2))**2

            """Spin:"""
            Spin=False
            if CheckMonteCarloUI==0:
                Spin=True
            if t>TimeApogee+Variables["StageDelay"] and Finless==True:
                Spin=False
            if Va2>0 and Spin==True:
                FinSpanRoot=                            Variables["FinSpanRoot"]
                rootspan=                               FinSpanRoot
                FinCantAngle=                           Variables["FinCantAngle"]
                cant=                                   FinCantAngle+self.monte_carlo.outputs()[12]
                if CheckMonteCarloUI==0:
                   cant=                                   FinCantAngle              
                """cant*=MC_FINCANT in HYROPS, MC_FINCANT=1.0+(MC_FIN_CANT*NormalRandom()*rad), MC_FINCANT=1 without uncertainty."""
                """Cant units: degrees"""
                span=                                   FinSpan
                tipchord=                               FinTipChord
                rootchord=                              FinRootChord
                sweep=                                  FinSweep
                refarea=                                Schar
                numfins=4
                gamma=1.4
                s=span
                sg=(tipchord-rootchord)/s
                Smac=(((2*tipchord)+rootchord)*s)/(3*(rootchord+tipchord))
                Lmac=(sg*Smac)+rootchord
                r=rootspan
                tau=r/(s+r)
                DeltaX=abs(sweep+((tipchord-rootchord)*0.5))
                cosG=s/((DeltaX*DeltaX)+(s*s))
                Afin=(rootchord+tipchord)*span*0.5
                rollmoment=0
                rollmomentsp=0
                rollmomentsb=0
                rolldampingmoment=0
                rolldampingmomentsp=0
                rolldampingmomentsb=0
                betsp=1
                rollsum=((rootchord+tipchord)*r*r*s*0.5)+(((rootchord+(2*tipchord))*r*s*s)/3)+(((rootchord+(3*tipchord))*s*s*s)/12)
                Aref=refarea
                ARfin=(2*span**2)/(span*0.5*(rootchord+tipchord))
                eps1=0
                eps2=0
                aa=0
                bb=0
                cc=0
                dd=0
                ee=0
                ff=0
                rollsumsp=0
                cna1=0
                cnasub=0
                cnasup=0
                cna=0
                fincp=0
                Cl=0
                Cdp=0
                MTlower=0.8
                MTupper=1.5
                rad=math.pi/180
                e=2.718281828

                if mach>1:
                    betsp=math.sqrt((mach*mach)-1)
                else:
                    betsp=math.sqrt(1-(mach*mach))

                CheckRollControl=Variables["CheckRollControl"]
                if CheckRollControl==True:
                    if t>Variables["RollControlTimeInitial"] and t<Variables["RollControlTimeFinal"]:
                        RollControlFrequency=Variables["RollControlFrequency"]
                        if (round(t,3))%(1/RollControlFrequency)==0:
                            if omega[0]<Variables["RollControlLower"]:
                                Cmom[0]=Variables["RollControlForce"]*rbod
                            if omega[0]>Variables["RollControlUpper"]:
                                Cmom[0]=-Variables["RollControlForce"]*rbod

                else:   
                    if mach<MTlower:
                        Mach=0
                        cna1=(2*math.pi*s*s)/((1+math.sqrt(1+((betsp*s*s)/(Afin*cosG))**2))*Aref)
                        cna=(numfins/2)*cna1*(1-(0.06*(math.sin(phip*2))**2))*(1+tau)
                        rollmoment=(Smac+r)*cna*cant*rad
                        rolldampingmoment=-(numfins*2*math.pi*omega[0]*rollsum)/(Aref*math.sqrt(Va2)*betsp)
                        Cdp=((4*(taoa*cna1)**2)/(1.1*cna1))*(Afin/Aref)

                    if mach>MTupper:
                        Mach=1
                        k1=2/betsp
                        k2=(((gamma+1)*(mach)**4)-(4*betsp*betsp))/(4*(betsp)**4)
                        k3=(((gamma+1)*(mach)**8)+(((2*gamma*gamma)-(7*gamma)-5)*(mach)**6)+(10*(gamma+1)*(mach)**4)+8)/(6*(betsp)**7)
                        
                        cna1=(Afin/Aref)*(k1+(k2*taoa)+(k3*taoa*taoa))
                        cna=(numfins/2)*cna1*(1-(0.06*(math.sin(phip*2))**2))*(1+tau)
                        rollmoment=(Smac+r)*cna*cant*rad
                        rollsumsp=sg*(((k1*(omega[0]/math.sqrt(Va2))*(r+s)**4)/4) + (((k2*(omega[0]/math.sqrt(Va2))**2)*(r+s)**5)/5) + ((k3*((omega[0]/math.sqrt(Va2))**3)*(r+s)**6)/6))
                        rollsumsp-=sg*(((k1*(omega[0]/math.sqrt(Va2))*(r)**4)/4) + ((k2*((omega[0]/math.sqrt(Va2))**2)*(r)**5)/5) + ((k3*(omega[0]/math.sqrt(Va2))**3*(r)**6)/6))
                        rollsumsp+=(rootchord-(sg*r))*(((k1*(omega[0]/math.sqrt(Va2))*(r+s)**3)/3) + ((k2*((omega[0]/math.sqrt(Va2))**2)*(r+s)**4)/4) + ((k3*((omega[0]/math.sqrt(Va2))**3)*(r+s)**5)/5))
                        rollsumsp-=(rootchord-(sg*r))*(((k1*(omega[0]/math.sqrt(Va2))*(r)**3)/3) + ((k2*((omega[0]/math.sqrt(Va2))**2)*(r)**4)/4) + ((k3*((omega[0]/math.sqrt(Va2))**3)*(r)**5)/5))
                        rolldampingmoment=-(numfins*rollsumsp)/Aref
                        Cdp=betsp*((taoa*cna1)**2)*(Afin/Aref)

                    if mach>=MTlower and mach<=MTupper:
                        Mach=0
                        betsp=math.sqrt(1-(MTlower*MTlower))
                        cna1=(2*math.pi*s*s)/((1+math.sqrt(1+((betsp*s*s)/(Afin*cosG))**2))*Aref)
                        cnasub=(numfins/2)*cna1*(1-(0.06*(math.sin(phip*2))**2))*(1+tau)
                        rollmomentsb=(Smac+r)*cnasub*cant*rad
                        rolldampingmomentsb=-(numfins*2*math.pi*omega[0]*rollsum)/(Aref*math.sqrt(Va2)*betsp)
                        betsp=math.sqrt((MTupper*MTupper)-1)
                        k1=2/betsp
                        k2=(((gamma+1)*(MTupper)**4) - (4*betsp*betsp))/(4*(betsp)**4)
                        k3=(((gamma+1)*(MTupper)**8) + (((2*gamma*gamma)-(7*gamma)-5)*(MTupper)**6) + (10*(gamma+1)*(MTupper)**4)+8)/(6*(betsp)**7)
                        cna1=(Afin/Aref)*(k1+(k2*taoa)+(k3*taoa*taoa))
                        cnasup=(numfins/2)*cna1*(1-(0.06*(math.sin(phip*2))**2))*(1+tau)
                        rollmomentsp=(Smac+r)*cnasup*cant*rad
                        rollsumsp=sg*(((k1*(omega[0]/math.sqrt(Va2))*(r+s)**4)/4) + ((k2*((omega[0]/math.sqrt(Va2))**2)*(r+s)**5)/5) + ((k3*(omega[0]/math.sqrt(Va2))**3)*(r+s)**6)/6) 
                        rollsumsp-=sg*(((k1*(omega[0]/math.sqrt(Va2))*(r)**4)/4)+((k2*((omega[0]/math.sqrt(Va2))**2*(r)**5)/5) + ((k3*(omega[0]/math.sqrt(Va2))**3)*(r)**6)/6))
                        rollsumsp+=(rootchord-(sg*r))*(((k1*(omega[0]/math.sqrt(Va2))*(r+s)**3)/3) + ((k2*((omega[0]/math.sqrt(Va2))**2)*(r+s)**4)/4) + ((k3*((omega[0]/math.sqrt(Va2))**3)*(r+s)**5)/5))
                        rollsumsp-=(rootchord-(sg*r))*(((k1*(omega[0]/math.sqrt(Va2))*(r)**3)/3) + ((k2*((omega[0]/math.sqrt(Va2))**2)*(r)**4)/4) + ((k3*((omega[0]/math.sqrt(Va2))**3)*(r)**5)/5)) 

                        rolldampingmomentsp=-(numfins*rollsumsp)/Aref
                        rollmoment=(rollmomentsb*((MTupper-mach)/(MTupper-MTlower)))+(rollmomentsp*((mach-MTlower)/(MTupper-MTlower)))
                        rolldampingmoment=(rolldampingmomentsb*((MTupper-mach)/(MTupper-MTlower)))+(rolldampingmomentsp*((mach-MTlower)/(MTupper-MTlower)))      
                        cna=(cnasub*((MTupper-mach)/(MTupper-MTlower)))+(cnasup*((mach-MTlower)/(MTupper-MTlower)))

                    Cl=rollmoment+rolldampingmoment

                    if omega[0]>0:
                        Cdp=abs(Cl)/(Smac+r)

                    '''Fin centre-of-pressure:'''
                    if mach<=0.5:
                        fincp=(sweep/3)*((rootchord+(2*tipchord))/(rootchord+tipchord))
                        fincp+=(1/(6*(rootchord+tipchord)))*((rootchord)**2+(tipchord)**2+(rootchord*tipchord))

                    if mach>=2:
                        fincp=(((ARfin*betsp)-0.67)/((2*ARfin*betsp)-1))*Lmac
                        fincp+=(sweep/3)*((rootchord+(2*tipchord))/(rootchord+tipchord))

                    if mach>0.5 and mach<2:
                        eps1=((math.sqrt(3)*ARfin)-0.67)/((2*math.sqrt(3)*ARfin)-1)
                        eps2=(0.68*ARfin)/((12*math.sqrt(3)*ARfin*ARfin)-(12*ARfin)+math.sqrt(3))
                        aa=-( 0.5267*eps1)+( 0.5926*eps2)+(8*0.0165)
                        bb=+( 4.2798*eps1)-( 4.7407*eps2)-(8*0.1337)
                        cc=-(13.1687*eps1)+(14.2222*eps2)+(8*0.4115)
                        dd=+(18.4362*eps1)-(18.9630*eps2)-(8*0.5761)
                        ee=-(10.5350*eps1)+(10.4815*eps2)+(8*0.3292)
                        ff=+( 2.0535*eps1)-( 2.0000*eps2)-(8*0.0329)
                        fincp=((aa*(mach)**5)+(bb*(mach)**4)+(cc*(mach)**3)+(dd*(mach)**2)+(ee*mach)+ff)*Lmac
                        fincp+=(sweep/3)*((rootchord+(2*tipchord))/(rootchord+tipchord))
              
                    Cdptot=Cdp
                    Cltot=Cl
                    Cmom[0]=0.5*Cltot*Va2*0.5*rho*Schar
                    Cmom[0]+=(((Ccof[1]+conmom[1])*Variables["SolidWorksCOMz"])+((Ccof[2]+conmom[2])*Variables["SolidWorksCOMy"]))*Va2*0.5*rho*Schar
                    Cmom[0]+=(Variables["SolidWorksCOMy"]*thrustv[2])+(Variables["SolidWorksCOMz"]*thrustv[1])
                    Ccof[0]-=Cdptot
                    
            else:
                Cmom[0]=0

            """The constraint in the following lines was commented-out (line 4838 of HYROPS code). It should not have been there as it removes the aerodynamic moment attributed to the aerodynamic lift force at the c.o.p. (moment about the cog)."""
            """Although RASAero lookup tables only provide coefficients to model the aerodynamic drag and lift forces and the airframe c.o.p. to a maximum total angle of attack of 15 degrees, drag and lift forces still need to be modelled when this angle of attack is greater than 15 degrees"""
            """Alternatively, for recovery and ballistic nosecone and payload simulations, can investigate the use of a simpler geometry to model the nosecone and payload (else need to run CFDs at different flow Reynolds numbers and body angles of attack), so at least the aerodynamic drag and lift coefficient values are computed at all total angles of attack, but the model would also need to account for the mach number, and nature of the fluid stream (Reynolds number), so on...""" 
            """With no aerodynamic drag and lift forces (the lookup tables in HYROPS are looking at incorrect column numbers when total angle of attack is greater than 15 degrees), the lookup tables in HYROPS return CA=0 and CN=0 (in coefficients in the Body Frame) at large 'total angles of attack', in HYROPS the body was then free to pivot about a point which is both its c.o.g. and c.o.p. which is obviously incorrect."""
##            if(taoaeq<=15):
            marm=(cog-cop)*(1+(self.monte_carlo.outputs()[10]/100)) #(cog-cop)+self.monte_carlo.outputs()[10]
            if CheckMonteCarloUI==0:
                marm=(cog-cop)
            """marm*=MC_MOM in HYROPS, MC_MOM=1.0+(MC_MOMENT_COEFFICIENTS*NormalRandom()), MC_MOM=1.0 without uncertainty."""

            if(omega[1]<0):                                                       
                pitdamp*=-1
            if(omega[2]<0):                                                           
                yawdamp*=-1

            """In HYROPS, there are no pitching or yawing moments for the first 5 seconds once the parachute has been deployment. 'if(t<(ApogeeTime+5.0)){Cmom=zerovector;}' in line 5205 in simulatex.h"""
            """This is not implemented in the following lines of code:"""
            Cmom[1]=-0.5*rho*Schar*Va2*(((Ccof[2]+conmom[2])*marm)+(Ccof[0]*Variables["SolidWorksCOMz"])-pitdamp)
            Cmom[2]= 0.5*rho*Schar*Va2*(((Ccof[1]+conmom[1])*marm)+(Ccof[0]*Variables["SolidWorksCOMy"])-yawdamp)
            Cmom[1]+=(Variables["SolidWorksCOMz"]*thrustv[0])+(cog*thrustv[2])
            Cmom[2]+=(Variables["SolidWorksCOMy"]*thrustv[0])+(cog*thrustv[1])
            
            fgeoa[0]=((0.5*rho*Schar*Ccof[0]*Va2)+thrustv[0])/mass
            fgeoa[1]=((0.5*rho*Schar*Ccof[1]*Va2)+thrustv[1])/mass
            fgeoa[2]=((0.5*rho*Schar*Ccof[2]*Va2)+thrustv[2])/mass

            if Variables["Check4in1"]==True and self.BodyState==2 and (t>TimeApogee+Variables["DrogueDelay"]+Variables["StageDelay"]):     
                fgeoa[0]=((0.5*rho*math.pi*Variables["NoseRadius"]*Variables["NoseRadius"]*Ccof[0]*Va2)+thrustv[0])/(mass)
                fgeoa[1]=((0.5*rho*math.pi*Variables["NoseRadius"]*Variables["NoseRadius"]*Ccof[1]*Va2)+thrustv[1])/(mass)
                fgeoa[2]=((0.5*rho*math.pi*Variables["NoseRadius"]*Variables["NoseRadius"]*Ccof[2]*Va2)+thrustv[2])/(mass)
            
            vector=fgeoa
            offset=np.array([0,0,0],dtype=float)
            ageoa=np.array([0,0,0],dtype=float)
            ageoa[0]=(vector[0]*( math.cos(panga[2]+offset[2])*math.cos(panga[1]+offset[1]))+vector[1]*( math.sin(panga[2]+offset[2])*math.cos(panga[0]+offset[0])+math.cos(panga[2]+offset[2])*math.sin(panga[1]+offset[1])*math.sin(panga[0]+offset[0]))+vector[2]*( math.sin(panga[2]+offset[2])*math.sin(panga[0]+offset[0])-math.cos(panga[2]+offset[2])*math.sin(panga[1]+offset[1])*math.cos(panga[0]+offset[0]) ))
            ageoa[1]=(vector[0]*(-math.sin(panga[2]+offset[2])*math.cos(panga[1]+offset[1]))+vector[1]*( math.cos(panga[2]+offset[2])*math.cos(panga[0]+offset[0])-math.sin(panga[2]+offset[2])*math.sin(panga[1]+offset[1])*math.sin(panga[0]+offset[0]))+vector[2]*( math.cos(panga[2]+offset[2])*math.sin(panga[0]+offset[0])+math.sin(panga[2]+offset[2])*math.sin(panga[1]+offset[1])*math.cos(panga[0]+offset[0]) ))
            ageoa[2]=(vector[0]*( math.sin(panga[1]+offset[1])                                       )+vector[1]*(-math.cos(panga[1]+offset[1])*math.sin(panga[0]+offset[0])                                                                                                                     )+vector[2]*( math.cos(panga[1]+offset[1])*math.cos(panga[0]+offset[0])                                                                                                                      ))
            vector=np.array([0,0,0],dtype=float)
            #np.array([ageoa[0],ageoa[1],ageoa[2]],dtype=float)=self.transform_frame.transformOB(fgeoa,panga)

            ageoa[0]-=(1/toth)*(-(vgeoa[0]*vgeoa[2])+(vgeoa[1]*vgeoa[1]*math.tan(ltloa[0])))                                  
            ageoa[1]-=(1/toth)*(-(vgeoa[1]*vgeoa[2])-(vgeoa[0]*vgeoa[1]*math.tan(ltloa[0])))                    
            ageoa[2]-=(1/toth)*(vgeoa[0]*vgeoa[0]+vgeoa[1]*vgeoa[1])                                            
            ageoa[0]-=omgt*((2*vgeoa[1]*math.sin(ltloa[0]))+(omgt*toth*math.sin(ltloa[0])*math.cos(ltloa[0])))
            ageoa[1]-=omgt*2*(-(vgeoa[2]*math.cos(ltloa[0]))-(vgeoa[0]*math.sin(ltloa[0])))                   
            ageoa[2]-=omgt*((2*vgeoa[1]*math.cos(ltloa[0]))+(omgt*toth*math.cos(ltloa[0])*math.cos(ltloa[0])))  
            ageoa[2]+=gr

            if self.BodyState==2 and (t>TimeApogee+Variables["DrogueDelay"]+Variables["StageDelay"]):
                KinematicVelocityMagnitude=np.linalg.norm(np.array([vgeoa[0],vgeoa[1],vgeoa[2]],dtype=float))
                """Accelerations due to aerodynamic drag"""
                DragAccelerationMagnitudeX=0.5*rho*Schar*Va2*Variables["ParachuteCD"]*(abs(vgeoa[0])/KinematicVelocityMagnitude)/mass
                DragAccelerationMagnitudeY=0.5*rho*Schar*Va2*Variables["ParachuteCD"]*(abs(vgeoa[1])/KinematicVelocityMagnitude)/mass
                DragAccelerationMagnitudeZ=0.5*rho*Schar*Va2*Variables["ParachuteCD"]*(abs(vgeoa[2])/KinematicVelocityMagnitude)/mass
                
                if (t>TimeApogee+Variables["DrogueDelay"]+Variables["StageDelay"]) and (pgeoa[2]<=-(Variables["ParachuteDelay"])):
                    if DrogueDeploymentMessage==False:
                        print('Drogue parachute deployment at T{0:.3f} seconds.'.format(t))
                        DrogueDeploymentMessage=True
                    DragAccelerationMagnitudeX=0.5*rho*Schar*Va2*Variables["DrogueCD"]*(abs(vgeoa[0])/KinematicVelocityMagnitude)/mass
                    DragAccelerationMagnitudeY=0.5*rho*Schar*Va2*Variables["DrogueCD"]*(abs(vgeoa[1])/KinematicVelocityMagnitude)/mass
                    DragAccelerationMagnitudeZ=0.5*rho*Schar*Va2*Variables["DrogueCD"]*(abs(vgeoa[2])/KinematicVelocityMagnitude)/mass
                    
                if (t>TimeApogee+Variables["DrogueDelay"]+Variables["StageDelay"]) and (pgeoa[2]>-(Variables["ParachuteDelay"])):
                    if ParachuteDeploymentMessage==False:
                        print('Main parachute deployment at T{0:.3f} seconds.'.format(t))
                        ParachuteDeploymentMessage=True
                Drag=np.linalg.norm(np.array([DragAccelerationMagnitudeX,DragAccelerationMagnitudeY,DragAccelerationMagnitudeZ],dtype=float))
                DragAccelerationX=-np.sign(vgeoa[0])*DragAccelerationMagnitudeX
                DragAccelerationY=-np.sign(vgeoa[1])*DragAccelerationMagnitudeY
                DragAccelerationZ=-np.sign(gr)*DragAccelerationMagnitudeZ
                ageoa[0]+=DragAccelerationX
                ageoa[1]+=DragAccelerationY
                ageoa[2]+=DragAccelerationZ#*0.3 ###
            
            Lengthpgeoa=math.sqrt(pgeoa[0]*pgeoa[0]+pgeoa[1]*pgeoa[1]+pgeoa[2]*pgeoa[2])
            RailLength=raillength-cog         
            raild=Rotation.from_euler('xyz',[0,-inpa[1]*(180/math.pi),0],degrees=True).apply(raild)
            if CheckMonteCarloUI==0:            
                raild=Rotation.from_euler('xyz',[0,0,-inpa[2]*(180/math.pi)],degrees=True).apply(raild)  
            else:
                raild=Rotation.from_euler('xyz',[0,0,-inpa[2]*(180/math.pi)+1],degrees=True).apply(raild)

            if Lengthpgeoa<RailLength:
                ageoa=raild*(ageoa[0]*raild[0]+ageoa[1]*raild[1]+ageoa[2]*raild[2])
                aanga[0]=0
                aanga[1]=0
                aanga[2]=0
##                omega[0]=0
##                omega[1]=0
##                omega[2]=0

            else:
                if RailMessage==False:
                    print('Rocket left the launch gantry at T{0:.3f} seconds.'.format(t))
                    RailMessage=True
                omgtt[0]= omgt*math.cos(ltloa[0])
                omgtt[1]= 0
                omgtt[2]=-omgt*math.sin(ltloa[0])
                vector=omgtt
                offset=np.array([0,0,0],dtype=float)
                omegt=np.array([0,0,0],dtype=float)
                omegt[0]=(vector[0]*( (math.cos(panga[1]+offset[1])*math.cos(panga[2]+offset[2])                                                                                                                       )))-(vector[1]*( (math.cos(panga[1]+offset[1])*math.sin(panga[2]+offset[2])                                       )                                                                                ))+(vector[2]*math.sin(panga[1]+offset[1]                                       ))
                omegt[1]=(vector[0]*( (math.sin(panga[0]+offset[0])*math.sin(panga[1]+offset[1])*math.cos(panga[2]+offset[2]))+(math.cos(panga[0]+offset[0])*math.sin(panga[2]+offset[2]))))+(vector[1]*(-(math.sin(panga[0]+offset[0])*math.sin(panga[1]+offset[1])*math.sin(panga[2]+offset[2]))+(math.cos(panga[0]+offset[0])*math.cos(panga[2]+offset[2]))))-(vector[2]*math.sin(panga[0]+offset[0])*math.cos(panga[1]+offset[1]))
                omegt[2]=(vector[0]*(-(math.cos(panga[0]+offset[0])*math.sin(panga[1]+offset[1])*math.cos(panga[2]+offset[2]))+(math.sin(panga[0]+offset[0])*math.sin(panga[2]+offset[2]))))+(vector[1]*( (math.cos(panga[0]+offset[0])*math.sin(panga[1]+offset[1])*math.sin(panga[2]+offset[2]))+(math.sin(panga[0]+offset[0])*math.cos(panga[2]+offset[2]))))+(vector[2]*math.cos(panga[0]+offset[0])*math.cos(panga[1]+offset[1]))
                vector=np.array([0,0,0],dtype=float)
                #omegt=self.transform_frame.transformBO(omgtt,panga)

                J=((omega[2]*omega[1])*(B-C))+(E*omega[0]*omega[1])-(F*omega[2]*omega[0])+(D*((omega[1]*omega[1])-(omega[2]*omega[2])))+Cmom[0]         
                K=((omega[2]*omega[0])*(C-A))+(F*omega[2]*omega[1])-(D*omega[0]*omega[1])+(E*((omega[2]*omega[2])-(omega[0]*omega[0])))+Cmom[1]                
                L=((omega[0]*omega[1])*(A-B))+(D*omega[0]*omega[2])-(E*omega[2]*omega[1])+(F*((omega[0]*omega[0])-(omega[1]*omega[1])))+Cmom[2]         
                J-=(A*((omega[2]*omegt[1])-(omega[1]*omegt[2])))+((C-B)*((omega[1]*omegt[2])+(omega[2]*omegt[1])+(omegt[2]*omegt[1])))                  
                K-=(B*((omega[0]*omegt[2])-(omega[2]*omegt[0])))+((A-C)*((omega[0]*omegt[2])+(omega[2]*omegt[0])+(omegt[2]*omegt[0])))                  
                L-=(C*((omega[1]*omegt[0])-(omega[0]*omegt[1])))+((B-A)*((omega[1]*omegt[0])+(omega[0]*omegt[1])+(omegt[0]*omegt[1])))                  
                J-=2*((omegt[0]*((F*omega[2])-(E*omega[1])))+(D*((omega[2]*omegt[2])-(omega[1]*omegt[1]))))                                             
                K-=2*((omegt[1]*((D*omega[0])-(F*omega[2])))+(E*((omega[2]*omegt[2])-(omega[1]*omegt[1]))))                                             
                L-=2*((omegt[2]*((E*omega[1])-(D*omega[0])))+(F*((omega[2]*omegt[2])-(omega[1]*omegt[1]))))
                J-=(D*((omegt[2]*omegt[2])-(omegt[1]*omegt[1])))+(F*omegt[2]*omegt[0])-(E*omegt[1]*omegt[0])                                            
                K-=(E*((omegt[0]*omegt[0])-(omegt[2]*omegt[2])))+(D*omegt[0]*omegt[1])-(F*omegt[2]*omegt[1])                                            
                L-=(F*((omegt[1]*omegt[1])-(omegt[0]*omegt[0])))+(E*omegt[1]*omegt[2])-(D*omegt[0]*omegt[2])                                            
                
                D0=(A*B*C)-(2*D*E*F)-((A*D*D)+(B*E*E)+(C*F*F))                                                                                          
                D1=(J*((B*C)-(D*D)))+(K*((F*C)+(E*D)))+(L*((F*D)+(B*E)))                                                                                
                D2=(J*((F*C)+(D*E)))+(K*((A*C)-(E*E)))+(L*((A*D)+(E*F)))                                                                                
                D3=(J*((F*D)+(B*E)))+(K*((A*D)+(E*F)))+(L*((A*B)-(F*F)))                                                                                
                aanga[0]=D1/D0                                                                             
                aanga[1]=D2/D0                                                                           
                aanga[2]=D3/D0

            height=alti
            ortherr=0
            alti=np.linalg.norm(np.array([EarthRadius-pgeoa[2]+LaunchAltitude,0,0],dtype=float))-EarthRadius
            toth=EarthRadius+alti
            omegeart=np.array([0,0,0],dtype=float)
            veara=np.array([0,0,0],dtype=float)
          
            dcora[0]=vgeoa[0]/toth                          
            dcora[1]=vgeoa[1]/(toth*math.cos(ltloa[0]))

            veara=vgeoa
            veara=Rotation.from_euler('xyz',[0,(-ltloa[0]-(math.pi*0.5))*(180/math.pi),0],degrees=True).apply(veara)
            veara=Rotation.from_euler('xyz',[0,0,(ltloa[1])*(180/math.pi)],degrees=True).apply(veara)

            vgeoa=vgeoa+ageoa*dt
            omega=omega+aanga*dt     
            peara=peara+veara*dt
            ltloa[0]+=dcora[0]*dt
            ltloa[1]+=dcora[1]*dt

            omegeart[0]=-dt*dcora[1]*math.cos(ltloa[0])
            omegeart[1]=-dt*dcora[0]                   
            omegeart[2]= dt*dcora[1]*math.sin(ltloa[0])

            vector=omegeart
            offset=np.array([0,0,0],dtype=float)
            omega[0]-=(vector[0]*( (math.cos(panga[1]+offset[1])*math.cos(panga[2]+offset[2])                                                                                                                       )))-(vector[1]*( (math.cos(panga[1]+offset[1])*math.sin(panga[2]+offset[2])                                       )                                                                                ))+(vector[2]*math.sin(panga[1]+offset[1]                                       ))
            omega[1]-=(vector[0]*( (math.sin(panga[0]+offset[0])*math.sin(panga[1]+offset[1])*math.cos(panga[2]+offset[2]))+(math.cos(panga[0]+offset[0])*math.sin(panga[2]+offset[2]))))+(vector[1]*(-(math.sin(panga[0]+offset[0])*math.sin(panga[1]+offset[1])*math.sin(panga[2]+offset[2]))+(math.cos(panga[0]+offset[0])*math.cos(panga[2]+offset[2]))))-(vector[2]*math.sin(panga[0]+offset[0])*math.cos(panga[1]+offset[1]))
            omega[2]-=(vector[0]*(-(math.cos(panga[0]+offset[0])*math.sin(panga[1]+offset[1])*math.cos(panga[2]+offset[2]))+(math.sin(panga[0]+offset[0])*math.sin(panga[2]+offset[2]))))+(vector[1]*( (math.cos(panga[0]+offset[0])*math.sin(panga[1]+offset[1])*math.sin(panga[2]+offset[2]))+(math.sin(panga[0]+offset[0])*math.cos(panga[2]+offset[2]))))+(vector[2]*math.cos(panga[0]+offset[0])*math.cos(panga[1]+offset[1]))
            vector=np.array([0,0,0],dtype=float)
            #omega=omega+(-1)*self.transform_frame.transformBO(omegeart,panga)

            #if DrogueDeploymentMessage==True:
            #    aanga[0]=0
            #    omega[0]=0
                
            ortherr=1-((qanga[0]*qanga[0])+(qanga[1]*qanga[1])+(qanga[2]*qanga[2])+(qanga[3]*qanga[3]))
            dqana[0]=0.5*(-(omega[0]*qanga[1])-(omega[1]*qanga[2])-(omega[2]*qanga[3]))
            dqana[1]=0.5*( (omega[0]*qanga[0])+(omega[2]*qanga[2])-(omega[1]*qanga[3]))
            dqana[2]=0.5*( (omega[1]*qanga[0])-(omega[2]*qanga[1])+(omega[0]*qanga[3]))
            dqana[3]=0.5*( (omega[2]*qanga[0])+(omega[1]*qanga[1])-(omega[0]*qanga[2]))
            dqana[0]+=qanga[0]*0.5*ortherr
            dqana[1]+=qanga[1]*0.5*ortherr
            dqana[2]+=qanga[2]*0.5*ortherr
            dqana[3]+=qanga[3]*0.5*ortherr
            qanga[0]+=dt*dqana[0]
            qanga[1]+=dt*dqana[1]
            qanga[2]+=dt*dqana[2]
            qanga[3]+=dt*dqana[3]

            wind_magnitude_1=(np.interp(height, altitude_array, magnitude1_array))#wind_magnitude_1=self.wind_vector.interpolate(height)[0]
            wind_magnitude_2=(np.interp(height, altitude_array, magnitude2_array))#wind_magnitude_2=self.wind_vector.interpolate(height)[1]
            WindAngle=math.atan2(wind_magnitude_2,wind_magnitude_1)
            
            wind_magnitude_1=wind_magnitude_1*(1+(self.monte_carlo.outputs()[6]/100)) #wind_magnitude_1+self.monte_carlo.outputs()[6]*math.cos(WindAngle)
            if CheckMonteCarloUI==0:
                wind_magnitude_1=wind_magnitude_1
            """vwnda*=MC_WINDM in HYROPS, MC_WINDM=1.0+(MC_WIND_MAGNITUDE*NormalRandom()), MC_WINDM=1 without uncertainty."""
            wind_magnitude_2=wind_magnitude_2*(1+(self.monte_carlo.outputs()[6]/100)) #wind_magnitude_2+self.monte_carlo.outputs()[6]*math.sin(WindAngle)
            if CheckMonteCarloUI==0:
                wind_magnitude_2=wind_magnitude_2
            wind_force=np.array([wind_magnitude_1,wind_magnitude_2,0])

            if Variables["Check4in1"]==True and t>TimeApogee+Variables["StageDelay"]:
                rotation_wind=Rotation.from_euler('xyz',[0,0,self.monte_carlo.outputs()[7]+self.monte_carlo.outputs()[19]],degrees=True)
                if CheckMonteCarloUI==0:
                    rotation_wind=Rotation.from_euler('xyz',[0,0,0],degrees=True)
            else:
                rotation_wind=Rotation.from_euler('xyz',[0,0,self.monte_carlo.outputs()[7]],degrees=True)
                """vwnda RotateZ(MC_WINDD) in HYROPS, MC_WINDD=MC_WIND_DIRECTION*NormalRandom()*rad, MC_WINDD=0 without uncertainty."""
                if CheckMonteCarloUI==0:
                    rotation_wind=Rotation.from_euler('xyz',[0,0,0],degrees=True)

            wind_force_redirected=rotation_wind.apply(wind_force)
            WindMagnitude=np.linalg.norm(wind_force)
            WindBearing=WindAngle
            vwnda=wind_force_redirected

            CheckTurbulence=Variables["CheckTurbulence"]
            if CheckTurbulence==True:
                Turbulence=self.wind_vector.parameters(dt,np.linalg.norm(vaera),-pgeoa[2])
                vwnda+=Turbulence

            """The below line has been adjusted to use the equation contained in Boiffier, "The Dynamics of Flight". Boiffier: "Vk=Vw+Va". "Vk=Va-Vw" in line 5044 in simulatex.h in the HYROPS code."""
            vaera=vgeoa-vwnda
            RelativeVelocityWind=vgeoa-vaera

            elevf=0
            numer=0
            denom=0

            pgeoa=pgeoa+(vgeoa*dt)
            if self.BodyState==2 and (t>TimeApogee+Variables["DrogueDelay"]+Variables["StageDelay"]):
                pgeoa+=vwnda*dt

            numer=2*((qanga[1]*qanga[2])+(qanga[0]*qanga[3]))
            denom=(qanga[0]*qanga[0])+(qanga[1]*qanga[1])-(qanga[2]*qanga[2])-(qanga[3]*qanga[3])
            if numer>=0 and denom>0:
                panga[2]=math.atan(numer/denom)
            if numer<=0 and denom<0:
                panga[2]=math.atan(abs(numer)/abs(denom))+math.pi
            if numer>=0 and denom<0:
                panga[2]=math.pi-math.atan(abs(numer)/abs(denom))
            if numer<=0 and denom>0:
                panga[2]=2*math.pi-math.atan(abs(numer)/abs(denom))

            numer=2*((qanga[2]*qanga[3])+(qanga[0]*qanga[1]))
            denom=(qanga[0]*qanga[0])-(qanga[1]*qanga[1])-(qanga[2]*qanga[2])+(qanga[3]*qanga[3])
            if numer>=0 and denom>0:
                panga[0]=math.atan(numer/denom)
            if numer<=0 and denom<0:
                panga[0]=math.atan(abs(numer)/abs(denom))+math.pi
            if numer>=0 and denom<0:
                panga[0]=math.pi-math.atan(abs(numer)/abs(denom))
            if numer<=0 and denom>0:
                panga[0]=2*math.pi-math.atan(abs(numer)/abs(denom))

            elevf=-2*((qanga[1]*qanga[3])-(qanga[0]*qanga[2]))
            if elevf>=-1 and elevf<=1:
                panga[1]=math.asin(elevf)
            
            while(panga[0]>(2*math.pi)):
                panga[0]-=2*math.pi
            while(panga[0]<(-2*math.pi)):
                panga[0]+=2*math.pi
            while(panga[1]>(2*math.pi)):
                panga[1]-=2*math.pi
            while(panga[1]<(-2*math.pi)):
                panga[1]+=2*math.pi
            while panga[2]>2*math.pi:
                panga[2]-=2*math.pi
            while panga[2]<(-2*math.pi):
                panga[2]+=2*math.pi

            Range=math.sqrt(pgeoa[0]*pgeoa[0]+pgeoa[1]*pgeoa[1])
            
            if (np.linalg.norm(peara)-EarthRadius)<-1.0:
                print("Message: Inertial Position < Earth Radius")
                break

            StepNumber+=1
            #ListOutput1=[[StepNumber,t,alti,mass,ixx,iyy,izz,pgeoa[0],pgeoa[1],pgeoa[2],vgeoa[0],vgeoa[1],vgeoa[2],ageoa[0],ageoa[1],ageoa[2],panga[0],panga[1],panga[2],omega[0],omega[1],omega[2],qanga[3],aanga[1],aanga[2],AirSpeed,SpeedOfSound,mach,cog,cop,StabilityMargin,alpa,beta,taoa,Ccof[0],Ccof[1],Ccof[2],rho,pres,np.array([Clinear[0],Clinear[1],Clinear[2]],dtype=float),np.array([Cmom[0],Cmom[1],Cmom[2]],dtype=float),WindMagnitude,WindBearing,np.array([RelativeVelocityWind[0],RelativeVelocityWind[1],RelativeVelocityWind[2]],dtype=float),ThrustMagnitude,gr,(ilatl[0]+ltloa[0])*(180/math.pi),(ilatl[1]+ltloa[1])*(180/math.pi),pitdamp,yawdamp,thrust,phip,Cmom[0]]]

            """If '4-in-1 Simulation' checkbox is enabled, write to the output file at one second periods:"""
            if Variables["Check4in1"]==True:
                if (round(2*t,3))%2==0:
                    list0.append(StepNumber)
                    list1.append(t)
                    list2.append(alti)
                    list3.append(mass)
                    list4.append(ixx)
                    list5.append(iyy)
                    list6.append(izz)

                    """FOR OPENGL TRAJECTORY GRAPHICS:"""
                    #list7.append(pgeoa[0])
                    if t==0:
                        list7.append(0)
                    else:
                        list7.append(pgeoa[0])
                        
                    """FOR OPENGL TRAJECTORY GRAPHICS:"""
                    #list8.append(pgeoa[1])
                    if t==0:
                        list8.append(0)
                    else:
                        list8.append(pgeoa[1])

                    list9.append(pgeoa[2])
                    list10.append(vgeoa[0])
                    list11.append(vgeoa[1])
                    list12.append(vgeoa[2])
                    list13.append(ageoa[0])
                    list14.append(ageoa[1])
                    list15.append(ageoa[2])
                    list16.append(panga[0])
                    list17.append(panga[1])
                    list18.append(panga[2])
                    list19.append(omega[0])
                    list20.append(omega[1])
                    list21.append(omega[2])
                    list22.append(aanga[0])
                    list23.append(aanga[1])
                    list24.append(aanga[2])
                    list25.append(AirSpeed)
                    list26.append(SpeedOfSound)
                    list27.append(mach)
                    list28.append(cog)
                    list29.append(cop)
                    list30.append(StabilityMargin)
                    list31.append(alpa)
                    list32.append(beta)
                    list33.append(taoa)
                    if self.BodyState!=2 or (t<=TimeApogee+Variables["DrogueDelay"]+Variables["StageDelay"]):
                        list34.append(Ccof[0])
                    elif self.BodyState==2 and (t>TimeApogee+Variables["DrogueDelay"]+Variables["StageDelay"]) and (pgeoa[2]>-(Variables["ParachuteDelay"])):
                        list34.append(-Variables["ParachuteCD"])
                    elif self.BodyState==2 and (t>TimeApogee+Variables["DrogueDelay"]+Variables["StageDelay"]) and (pgeoa[2]<=-(Variables["ParachuteDelay"])):
                        list34.append(-Variables["DrogueCD"])
                    list35.append(Ccof[1])
                    list36.append(Ccof[2])
                    list37.append(rho)
                    list38.append(pres)
                    if self.BodyState!=2 or (t<=TimeApogee+Variables["DrogueDelay"]+Variables["StageDelay"]):
                        list39.append(Ccof[0]*0.5*rho*Schar*Va2)
                    elif self.BodyState==2 and (t>TimeApogee+Variables["DrogueDelay"]+Variables["StageDelay"]) and (pgeoa[2]>-(Variables["ParachuteDelay"])):
                        list39.append(-Variables["ParachuteCD"]*0.5*rho*Schar*Va2)  
                    elif self.BodyState==2 and (t>TimeApogee+Variables["DrogueDelay"]+Variables["StageDelay"]) and (pgeoa[2]<=-(Variables["ParachuteDelay"])):
                        list39.append(-Variables["DrogueCD"]*0.5*rho*Schar*Va2)  
                    list40.append(Ccof[1]*0.5*rho*Schar*Va2)
                    list41.append(Ccof[2]*0.5*rho*Schar*Va2)
                    list42.append(Cmom[0])
                    list43.append(Cmom[1])
                    list44.append(Cmom[2])            
                    list45.append(WindMagnitude)
                    list46.append(WindBearing)
                    list47.append(np.array([RelativeVelocityWind[0],RelativeVelocityWind[1],RelativeVelocityWind[2]],dtype=float))
                    list48.append(ThrustMagnitude)
                    list49.append(gr)
                    list50.append((ilatl[0]+ltloa[0])*(180/math.pi))
                    list51.append((ilatl[1]+ltloa[1])*(180/math.pi))
                    list52.append(pitdamp)
                    list53.append(yawdamp)
                    list54.append(thrust)
                    list55.append(phip)
                    list56.append(Cmom[0])
                        
                    """Additional output variables:"""
                    list57.append(DragAccelerationX)
                    list58.append(DragAccelerationY)
                    list59.append(DragAccelerationZ)
                    list60.append(Schar)
                    list61.append(0)
                    list62.append(0)
                    list63.append(0)
                    list64.append(0)
                    list65.append(0)
                    list66.append(0)

            else:
                """Else write to the output file using the solver timestep size:"""
                list0.append(StepNumber)
                list1.append(t)
                list2.append(alti)
                list3.append(mass)
                list4.append(ixx)
                list5.append(iyy)
                list6.append(izz)
                list7.append(pgeoa[0])
                list8.append(pgeoa[1])
                list9.append(pgeoa[2])
                list10.append(vgeoa[0])
                list11.append(vgeoa[1])
                list12.append(vgeoa[2])
                list13.append(ageoa[0])
                list14.append(ageoa[1])
                list15.append(ageoa[2])
                list16.append(panga[0])
                list17.append(panga[1])
                list18.append(panga[2])
                list19.append(omega[0])
                list20.append(omega[1])
                list21.append(omega[2])
                list22.append(aanga[0])
                list23.append(aanga[1])
                list24.append(aanga[2])
                list25.append(AirSpeed)
                list26.append(SpeedOfSound)
                list27.append(mach)
                list28.append(cog)
                list29.append(cop)
                list30.append(StabilityMargin)
                list31.append(alpa)
                list32.append(beta)
                list33.append(taoa)
                if self.BodyState!=2 or (t<=TimeApogee+Variables["DrogueDelay"]+Variables["StageDelay"]):
                    list34.append(Ccof[0])
                elif self.BodyState==2 and (t>TimeApogee+Variables["DrogueDelay"]+Variables["StageDelay"]) and (pgeoa[2]>-(Variables["ParachuteDelay"])):
                    list34.append(-Variables["ParachuteCD"])
                elif self.BodyState==2 and (t>TimeApogee+Variables["DrogueDelay"]+Variables["StageDelay"]) and (pgeoa[2]<=-(Variables["ParachuteDelay"])):
                    list34.append(-Variables["DrogueCD"])
                list35.append(Ccof[1])
                list36.append(Ccof[2])
                list37.append(rho)
                list38.append(pres)
                if self.BodyState!=2 or (t<=TimeApogee+Variables["DrogueDelay"]+Variables["StageDelay"]):
                    list39.append(Ccof[0]*0.5*rho*Schar*Va2)
                elif self.BodyState==2 and (t>TimeApogee+Variables["DrogueDelay"]+Variables["StageDelay"]) and (pgeoa[2]>-(Variables["ParachuteDelay"])):
                    list39.append(-Variables["ParachuteCD"]*0.5*rho*Schar*Va2)  
                elif self.BodyState==2 and (t>TimeApogee+Variables["DrogueDelay"]+Variables["StageDelay"]) and (pgeoa[2]<=-(Variables["ParachuteDelay"])):
                    list39.append(-Variables["DrogueCD"]*0.5*rho*Schar*Va2)     
                list40.append(Ccof[1]*0.5*rho*Schar*Va2)
                list41.append(Ccof[2]*0.5*rho*Schar*Va2)
                list42.append(Cmom[0])
                list43.append(Cmom[1])
                list44.append(Cmom[2])            
                list45.append(WindMagnitude)
                list46.append(WindBearing)
                list47.append(np.array([RelativeVelocityWind[0],RelativeVelocityWind[1],RelativeVelocityWind[2]],dtype=float))
                list48.append(ThrustMagnitude)
                list49.append(gr)
                list50.append((ilatl[0]+ltloa[0])*(180/math.pi))
                list51.append((ilatl[1]+ltloa[1])*(180/math.pi))
                list52.append(pitdamp)
                list53.append(yawdamp)
                list54.append(thrust)
                list55.append(phip)
                list56.append(Cmom[0])
                    
                """Additional output variables:"""
                list57.append(DragAccelerationX)
                list58.append(DragAccelerationY)
                list59.append(DragAccelerationZ)
                list60.append(Schar)
                list61.append(0)
                list62.append(0)
                list63.append(0)
                list64.append(0)
                list65.append(0)
                list66.append(0)

            #dfOutputRow1=pd.DataFrame(ListOutput1,columns=OutputColumns)
            #dfOutput=dfOutput._append(dfOutputRow1)

            CheckStream=Variables["CheckStream"]        
            if (round(2*t,3))%2==0 and t>0:
                print('time {0:.3f} s.'.format(t),' altitude {0:.3f} m.'.format(-pgeoa[2]),' elevation {0:.3f}\xb0.'.format(panga[1]*(180/math.pi)),' latitude {0:.3f}\xb0 N.'.format((ilatl[0]+ltloa[0])*(180/math.pi)),' longitude {0:.3f}\xb0 E.'.format((ilatl[1]+ltloa[1])*(180/math.pi)),' range {0:.3f} m.'.format(Range),' Wind:','[','{0:.1f} '.format(vwnda[0]),'{0:.1f} '.format(vwnda[1]),'{0:.1f} '.format(vwnda[2]),']','m/s', " M: {0:.3f}".format(mach))

##            """Write Timestep:"""
##            TimePrevious=t # Update TimePrevious
##            file1 = open("TimestepFirst.txt","w")
##            file1.write(str(TimePrevious))
##            file1.close()

            t +=Variables["TimeSize"]
            #dt+=Variables["TimeSize"]
                
        """Output file:"""
        dfOutput=pd.DataFrame()
        dfOutput["step_number"]=list0
        dfOutput["time (s)"]=list1
        dfOutput["altitude (m)"]=list2
        dfOutput["mass (kg)"]=list3
        dfOutput["MOI_xx"]=list4
        dfOutput["MOI_yy"]=list5
        dfOutput["MOI_zz"]=list6
        dfOutput["position_kinematic_North (m)"]=list7
        dfOutput["position_kinematic_East (m)"]=list8
        dfOutput["position_kinematic_Down (m)"]=list9
        dfOutput["velocity_kinematic_North (m/s)"]=list10
        dfOutput["velocity_kinematic_East (m/s)"]=list11
        dfOutput["velocity_kinematic_Down (m/s)"]=list12
        dfOutput["acceleration_kinematic_North (m/s2)"]=list13
        dfOutput["acceleration_kinematic_East (m/s2)"]=list14
        dfOutput["acceleration_kinematic_Down (m/s2)"]=list15
        dfOutput["position_angular_roll (rad)"]=list16
        dfOutput["position_angular_pitch (rad)"]=list17
        dfOutput["position_angular_yaw (rad)"]=list18
        dfOutput["velocity_angular_roll (rad/s)"]=list19
        dfOutput["velocity_angular_pitch (rad/s)"]=list20
        dfOutput["velocity_angular_yaw (rad/s)"]=list21
        dfOutput["acceleration_angular_roll (rad/s2)"]=list22
        dfOutput["acceleration_angular_pitch (rad/s2)"]=list23
        dfOutput["acceleration_angular_yaw (rad/s2)"]=list24
        dfOutput["air_speed (m/s)"]=list25
        dfOutput["speed_of_sound (m/s)"]=list26
        dfOutput["mach_number"]=list27
        dfOutput["centre-of-gravity (m)"]=list28
        dfOutput["centre-of-pressure (m)"]=list29
        dfOutput["stability_margin (calibres)"]=list30
        dfOutput["angle_of_attack (rad)"]=list31
        dfOutput["angle_of_sideslip (rad)"]=list32
        dfOutput["total_angle_of_attack (rad)"]=list33
        dfOutput["CD"]=list34
        dfOutput["CL"]=list35
        dfOutput["CC"]=list36
        dfOutput["air_density (kg/m3)"]=list37
        dfOutput["dynamic_pressure (Pa)"]=list38
        dfOutput["aerodynamic_forces_body[0] (N)"]=list39
        dfOutput["aerodynamic_forces_body[1] (N)"]=list40
        dfOutput["aerodynamic_forces_body[2] (N)"]=list41        
        dfOutput["aerodynamic_moments_body[0] (Nm)"]=list42
        dfOutput["aerodynamic_moments_body[1] (Nm)"]=list43
        dfOutput["aerodynamic_moments_body[2] (Nm)"]=list44      
        dfOutput["wind_magnitude (m/s)"]=list45
        dfOutput["wind_bearing (deg clockwise)"]=list46
        dfOutput["wind_relative_velocity (m/s)"]=list47
        dfOutput["thrust_magnitude (N)"]=list48
        dfOutput["acceleration_gravity (m/s2)"]=list49
        dfOutput["latitude (deg)"]=list50
        dfOutput["longitude (deg)"]=list51
        dfOutput["pitch_damping_coefficient"]=list52
        dfOutput["yaw_damping_coefficient"]=list53
        dfOutput["momentum_thrust (N)"]=list54
        dfOutput["aerodynamic roll angle (rad)"]=list55
        dfOutput["roll moment (Nm)"]=list56
        dfOutput["Parachute Drag Acceleration, North (m/s2)"]=list57
        dfOutput["Parachute Drag Acceleration, East (m/s2)"]=list58
        dfOutput["Parachute Drag Acceleration, Down (m/s2)"]=list59
        dfOutput["Reference Area (m2)"]=list60
        dfOutput["list1"]=list61
        dfOutput["list2"]=list62
        dfOutput["list3"]=list63
        dfOutput["list4"]=list64
        dfOutput["list5"]=list65
        dfOutput["list6"]=list66    

        if self.input_values["NumberRuns"]==1 and CheckMonteCarloUI==0:
            dfOutput.to_excel(r'{}\Outputs\Simulation.xlsx'.format(Directory),index=False)

##        path=r"TimestepFirst.txt"
##        exist = os.path.isfile(path)
##        if exist==True:
##            os.remove("TimestepFirst.txt")
        path=r"VariablesFirst.txt"
        exist = os.path.isfile(path)
        if exist==True:
            os.remove("VariablesFirst.txt")
        path=r"body\TurbulenceFirst.txt"
        exist = os.path.isfile(path)
        if exist==True:
            os.remove("body\TurbulenceFirst.txt")



        """Monte Carlo summarised output file:"""
        if self.input_values["NumberRuns"]>1 and self.BodyState==1:
            MonteCarloColumns=["East (metres)","North (metres)","Apogee (metres)"]
            MonteCarloFile=os.path.isfile(r'{}\Outputs\Monte Carlo Rocket Ballistic000.xlsx'.format(Directory))
            if MonteCarloFile==True:
                MonteCarlo=pd.read_excel(r'{}\Outputs\Monte Carlo Rocket Ballistic000.xlsx'.format(Directory),header=0)
            else:
                MonteCarlo=pd.DataFrame(columns=MonteCarloColumns)
            """For southern hemisphere, South-East launch direction, min(pgeoa[0]) and max(pgeoa[1]):"""
            ListMonteCarlo=[[max(list8),min(list7),-min(list9)]]
            MonteCarloRow=pd.DataFrame(ListMonteCarlo,columns=MonteCarloColumns)
            MonteCarlo=MonteCarlo._append(MonteCarloRow)
            MonteCarlo.to_excel(r'{}\Outputs\Monte Carlo Rocket Ballistic000.xlsx'.format(Directory),columns=MonteCarloColumns,index=False)
        elif self.input_values["NumberRuns"]>1 and self.BodyState==2:
            MonteCarloColumns=["East (metres)","North (metres)","Apogee (metres)"]
            MonteCarloFile=os.path.isfile(r'{}\Outputs\Monte Carlo Nosecone Payload Parachute000.xlsx'.format(Directory))
            if MonteCarloFile==True:
                MonteCarlo=pd.read_excel(r'{}\Outputs\Monte Carlo Nosecone Payload Parachute000.xlsx'.format(Directory),header=0)
            else:
                MonteCarlo=pd.DataFrame(columns=MonteCarloColumns)
            """For southern hemisphere, South-East launch direction, min(pgeoa[0]) and max(pgeoa[1]):"""
            ListMonteCarlo=[[max(list8),min(list7),-min(list9)]]
            MonteCarloRow=pd.DataFrame(ListMonteCarlo,columns=MonteCarloColumns)
            MonteCarlo=MonteCarlo._append(MonteCarloRow)
            MonteCarlo.to_excel(r'{}\Outputs\Monte Carlo Nosecone Payload Parachute000.xlsx'.format(Directory),columns=MonteCarloColumns,index=False)
        elif self.input_values["NumberRuns"]>1 and self.BodyState==3:
            MonteCarloColumns=["East (metres)","North (metres)","Apogee (metres)"]
            MonteCarloFile=os.path.isfile(r'{}\Outputs\Monte Carlo Nosecone Payload Ballistic000.xlsx'.format(Directory))
            if MonteCarloFile==True:
                MonteCarlo=pd.read_excel(r'{}\Outputs\Monte Carlo Nosecone Payload Ballistic000.xlsx'.format(Directory),header=0)
            else:
                MonteCarlo=pd.DataFrame(columns=MonteCarloColumns)
            """For southern hemisphere, South-East launch direction, min(pgeoa[0]) and max(pgeoa[1]):"""
            ListMonteCarlo=[[max(list8),min(list7),-min(list9)]]
            MonteCarloRow=pd.DataFrame(ListMonteCarlo,columns=MonteCarloColumns)
            MonteCarlo=MonteCarlo._append(MonteCarloRow)
            MonteCarlo.to_excel(r'{}\Outputs\Monte Carlo Nosecone Payload Ballistic000.xlsx'.format(Directory),columns=MonteCarloColumns,index=False)
        elif self.input_values["NumberRuns"]>1 and self.BodyState==4:
            MonteCarloColumns=["East (metres)","North (metres)","Apogee (metres)"]
            MonteCarloFile=os.path.isfile(r'{}\Outputs\Monte Carlo Booster Ballistic000.xlsx'.format(Directory))
            if MonteCarloFile==True:
                MonteCarlo=pd.read_excel(r'{}\Outputs\Monte Carlo Booster Ballistic000.xlsx'.format(Directory),header=0)
            else:
                MonteCarlo=pd.DataFrame(columns=MonteCarloColumns)
            """For southern hemisphere, South-East launch direction, min(pgeoa[0]) and max(pgeoa[1]):"""
            ListMonteCarlo=[[max(list8),min(list7),-min(list9)]]
            MonteCarloRow=pd.DataFrame(ListMonteCarlo,columns=MonteCarloColumns)
            MonteCarlo=MonteCarlo._append(MonteCarloRow)
            MonteCarlo.to_excel(r'{}\Outputs\Monte Carlo Booster Ballistic000.xlsx'.format(Directory),columns=MonteCarloColumns,index=False)

        """Additional output files:"""
        if min(dfOutput["stability_margin (calibres)"])>0: #self.input_values["NumberRuns"]==1 and
            """OpenGL Trajectory Kinematics:"""
            if self.BodyState==1:
                OpenGLColumns=["time","North","East","altitude","roll","pitch","yaw","range","velocity","acceleration","alpha","beta","aoa","roll rate","pitch rate","yaw rate"]
                OpenGL=pd.DataFrame(index=None,columns=OpenGLColumns)
                OpenGL["time"]=dfOutput["time (s)"]
                OpenGL["North"]=dfOutput["position_kinematic_North (m)"]
                OpenGL["East"]=dfOutput["position_kinematic_East (m)"]
                OpenGL["altitude"]=dfOutput["altitude (m)"]*(-1)
                OpenGL["roll"]=dfOutput["position_angular_roll (rad)"]
                OpenGL["pitch"]=dfOutput["position_angular_pitch (rad)"]
                OpenGL["yaw"]=dfOutput["position_angular_yaw (rad)"]
                OpenGL["range"]=( dfOutput["position_kinematic_North (m)"]**2 + dfOutput["position_kinematic_East (m)"]**2 )**(1/2)
                OpenGL["velocity"]=( dfOutput["velocity_kinematic_North (m/s)"]**2 + dfOutput["velocity_kinematic_East (m/s)"]**2 + dfOutput["velocity_kinematic_Down (m/s)"]**2 )**(1/2)
                OpenGL["acceleration"]=( dfOutput["acceleration_kinematic_North (m/s2)"]**2 + dfOutput["acceleration_kinematic_East (m/s2)"]**2 + dfOutput["acceleration_kinematic_Down (m/s2)"]**2 )**(1/2)
                OpenGL["alpha"]=dfOutput["angle_of_attack (rad)"]
                OpenGL["beta"]=dfOutput["angle_of_sideslip (rad)"]
                OpenGL["aoa"]=dfOutput["total_angle_of_attack (rad)"]
                OpenGL["roll rate"]=dfOutput["velocity_angular_roll (rad/s)"]
                OpenGL["pitch rate"]=dfOutput["velocity_angular_pitch (rad/s)"]
                OpenGL["yaw rate"]=dfOutput["velocity_angular_yaw (rad/s)"]
                OpenGL.to_excel(r'C:\ASRI_Simulator\body\OpenGL1.xlsx',columns=OpenGLColumns,index=False)
            elif self.BodyState==2:
                OpenGLColumns=["time","North","East","altitude","roll","pitch","yaw","range","velocity","acceleration","alpha","beta","aoa","roll rate","pitch rate","yaw rate"]
                OpenGL=pd.DataFrame(index=None,columns=OpenGLColumns)
                OpenGL["time"]=dfOutput["time (s)"]
                OpenGL["North"]=dfOutput["position_kinematic_North (m)"]
                OpenGL["East"]=dfOutput["position_kinematic_East (m)"]
                OpenGL["altitude"]=dfOutput["altitude (m)"]*(-1)
                OpenGL["roll"]=dfOutput["position_angular_roll (rad)"]
                OpenGL["pitch"]=dfOutput["position_angular_pitch (rad)"]
                OpenGL["yaw"]=dfOutput["position_angular_yaw (rad)"]
                OpenGL["range"]=( dfOutput["position_kinematic_North (m)"]**2 + dfOutput["position_kinematic_East (m)"]**2 )**(1/2)
                OpenGL["velocity"]=( dfOutput["velocity_kinematic_North (m/s)"]**2 + dfOutput["velocity_kinematic_East (m/s)"]**2 + dfOutput["velocity_kinematic_Down (m/s)"]**2 )**(1/2)
                OpenGL["acceleration"]=( dfOutput["acceleration_kinematic_North (m/s2)"]**2 + dfOutput["acceleration_kinematic_East (m/s2)"]**2 + dfOutput["acceleration_kinematic_Down (m/s2)"]**2 )**(1/2)
                OpenGL["alpha"]=dfOutput["angle_of_attack (rad)"]
                OpenGL["beta"]=dfOutput["angle_of_sideslip (rad)"]
                OpenGL["aoa"]=dfOutput["total_angle_of_attack (rad)"]
                OpenGL["roll rate"]=dfOutput["velocity_angular_roll (rad/s)"]
                OpenGL["pitch rate"]=dfOutput["velocity_angular_pitch (rad/s)"]
                OpenGL["yaw rate"]=dfOutput["velocity_angular_yaw (rad/s)"]
                OpenGL.to_excel(r'C:\ASRI_Simulator\body\OpenGL2.xlsx',columns=OpenGLColumns,index=False)
            elif self.BodyState==3:
                OpenGLColumns=["time","North","East","altitude","roll","pitch","yaw","range","velocity","acceleration","alpha","beta","aoa","roll rate","pitch rate","yaw rate"]
                OpenGL=pd.DataFrame(index=None,columns=OpenGLColumns)
                OpenGL["time"]=dfOutput["time (s)"]
                OpenGL["North"]=dfOutput["position_kinematic_North (m)"]
                OpenGL["East"]=dfOutput["position_kinematic_East (m)"]
                OpenGL["altitude"]=dfOutput["altitude (m)"]*(-1)
                OpenGL["roll"]=dfOutput["position_angular_roll (rad)"]
                OpenGL["pitch"]=dfOutput["position_angular_pitch (rad)"]
                OpenGL["yaw"]=dfOutput["position_angular_yaw (rad)"]
                OpenGL["range"]=( dfOutput["position_kinematic_North (m)"]**2 + dfOutput["position_kinematic_East (m)"]**2 )**(1/2)
                OpenGL["velocity"]=( dfOutput["velocity_kinematic_North (m/s)"]**2 + dfOutput["velocity_kinematic_East (m/s)"]**2 + dfOutput["velocity_kinematic_Down (m/s)"]**2 )**(1/2)
                OpenGL["acceleration"]=( dfOutput["acceleration_kinematic_North (m/s2)"]**2 + dfOutput["acceleration_kinematic_East (m/s2)"]**2 + dfOutput["acceleration_kinematic_Down (m/s2)"]**2 )**(1/2)
                OpenGL["alpha"]=dfOutput["angle_of_attack (rad)"]
                OpenGL["beta"]=dfOutput["angle_of_sideslip (rad)"]
                OpenGL["aoa"]=dfOutput["total_angle_of_attack (rad)"]
                OpenGL["roll rate"]=dfOutput["velocity_angular_roll (rad/s)"]
                OpenGL["pitch rate"]=dfOutput["velocity_angular_pitch (rad/s)"]
                OpenGL["yaw rate"]=dfOutput["velocity_angular_yaw (rad/s)"]
                OpenGL.to_excel(r'C:\ASRI_Simulator\body\OpenGL3.xlsx',columns=OpenGLColumns,index=False)
            else:
                OpenGLColumns=["time","North","East","altitude","roll","pitch","yaw","range","velocity","acceleration","alpha","beta","aoa","roll rate","pitch rate","yaw rate"]
                OpenGL=pd.DataFrame(index=None,columns=OpenGLColumns)
                OpenGL["time"]=dfOutput["time (s)"]
                OpenGL["North"]=dfOutput["position_kinematic_North (m)"]
                OpenGL["East"]=dfOutput["position_kinematic_East (m)"]
                OpenGL["altitude"]=dfOutput["altitude (m)"]*(-1)
                OpenGL["roll"]=dfOutput["position_angular_roll (rad)"]
                OpenGL["pitch"]=dfOutput["position_angular_pitch (rad)"]
                OpenGL["yaw"]=dfOutput["position_angular_yaw (rad)"]
                OpenGL["range"]=( dfOutput["position_kinematic_North (m)"]**2 + dfOutput["position_kinematic_East (m)"]**2 )**(1/2)
                OpenGL["velocity"]=( dfOutput["velocity_kinematic_North (m/s)"]**2 + dfOutput["velocity_kinematic_East (m/s)"]**2 + dfOutput["velocity_kinematic_Down (m/s)"]**2 )**(1/2)
                OpenGL["acceleration"]=( dfOutput["acceleration_kinematic_North (m/s2)"]**2 + dfOutput["acceleration_kinematic_East (m/s2)"]**2 + dfOutput["acceleration_kinematic_Down (m/s2)"]**2 )**(1/2)
                OpenGL["alpha"]=dfOutput["angle_of_attack (rad)"]
                OpenGL["beta"]=dfOutput["angle_of_sideslip (rad)"]
                OpenGL["aoa"]=dfOutput["total_angle_of_attack (rad)"]
                OpenGL["roll rate"]=dfOutput["velocity_angular_roll (rad/s)"]
                OpenGL["pitch rate"]=dfOutput["velocity_angular_pitch (rad/s)"]
                OpenGL["yaw rate"]=dfOutput["velocity_angular_yaw (rad/s)"]
                OpenGL.to_excel(r'C:\ASRI_Simulator\body\OpenGL4.xlsx',columns=OpenGLColumns,index=False)
##
##
##
##            """OTR output files:"""
##            OTR=False
##            if OTR==True:   
##                """OTR output file 1:"""
##                OTR1Columns=["North","East"]
##                ListOTR1=[[dfOutput.iloc[-1,7],dfOutput.iloc[-1,8]]]
##                OTR1=pd.DataFrame(ListOTR1,columns=OTR1Columns)
##                OTR1.to_excel(r'C:\ASRI_Simulator\body\OTR\OTR_file_1.xlsx',columns=OTR1Columns,index=False)
##
##                """OTR output file 2:"""
##                OTR2Columns=["Time Step #","Time(s)","Position (North, metres)","Position (East, metres)","Altitude (metres)","Velocity (North, m/s)","Velocity (East, m/s)","Velocity (Down, m/s)","Acceleration (North, m/s2)","Acceleration (East, m/s2)","Acceleration (Down, m/s2)"]
##                OTR2=pd.DataFrame(index=None,columns=OTR2Columns)
##                OTR2["Time Step #"]=dfOutput["step_number"]
##                OTR2["Time(s)"]=dfOutput["time (s)"]
##                OTR2["Position (North, metres)"]=dfOutput["position_kinematic_North (m)"]
##                OTR2["Position (East, metres)"]=dfOutput["position_kinematic_East (m)"]
##                OTR2["Altitude (metres)"]=dfOutput["altitude (m)"]
##                OTR2["Velocity (North, m/s)"]=dfOutput["velocity_kinematic_North (m/s)"]
##                OTR2["Velocity (East, m/s)"]=dfOutput["velocity_kinematic_East (m/s)"]
##                OTR2["Velocity (Down, m/s)"]=dfOutput["velocity_kinematic_Down (m/s)"]
##                OTR2["Acceleration (North, m/s2)"]=dfOutput["acceleration_kinematic_North (m/s2)"]
##                OTR2["Acceleration (East, m/s2)"]=dfOutput["acceleration_kinematic_East (m/s2)"]
##                OTR2["Acceleration (Down, m/s2)"]=dfOutput["acceleration_kinematic_Down (m/s2)"]
##                OTR2.to_excel(r'C:\ASRI_Simulator\body\OTR\OTR_file_2.xlsx',columns=OTR2Columns,index=False)
