import math
import numpy as np
import pandas as pd

"""Phoenix 1BIIr (as per HYROPS model):"""
"""endMass=                49.35224149"""
"""endCOM=                 1.386632"""
"""endMOIx=                0.04023116"""
"""endMOIy=                180.8297"""
"""endMOIz=                180.8297"""
"""calculator_t=           0"""
"""calculator_t_burn=      17.1"""
"""calculator_timestep=    0.005"""
"""rhoFuel=                *1065 (as per Nino, not 924 kg/m3)"""
"""RFuel=                  0.0735"""
"""TFuelInitial=           0.0375"""
"""TFuel=                  0.0375"""
"""LFuel=                  0.51"""
"""COMFuel=                0.5395"""
"""rhoOxid=                *880 (as per Nino, not 755 kg/m3)"""
"""ROxid=                  0.07633"""
"""LOxid=                  1.9869"""
"""LOxidInitial=           1.9869"""
"""COMOxid=                2.20745"""
"""timestepsFinal=         30000"""

def MassPropertiesCalculation(Directory,endMass,endCOM,endMOIx,endMOIy,endMOIz,calculator_t,calculator_t_burn,calculator_t_burn_fuel,calculator_timestep,rhoFuel,RFuel,TFuelInitial,TFuel,LFuel,COMFuel,rhoOxid,ROxid,LOxid,LOxidInitial,COMOxid,timestepsFinal):
    """Example:"""
    MCyl=1
    LCyl=0.5
    RCyl=0.01

    MoixCyl=0.5*MCyl*(RCyl*RCyl)
    MoiyCyl=(MCyl/12)*(3*RCyl*RCyl+LCyl*LCyl)
    MoizCyl=MoiyCyl

    MCylH=1
    LCylH=0.5
    RCylH=0.01
    TCylH=0.001

    """HYROPS incorrectly has a negative separator in below equation:"""
    MoixCylH=0.5*MCylH*((RCylH*RCylH)+(RCylH-TCylH)**2 )
    MoiyCylH=(1/12)*MCylH*((3*(RCylH**2)+3*((RCylH-TCylH)**2))+LCylH**2)
    MoizCylH=MoiyCylH

    """Example: Checked and happy with the following calculations using the above quantities:"""
    ##print(MoixCyl,MoiyCyl,MoizCyl)
    ##print(MoixCylH,MoiyCylH,MoizCylH)
    ##
    ##V=math.pi*RCyl*RCyl*LCyl
    ##rho=1/V
    ##print(rho) #rho for manually cross-checking mass parameters in external program (Excel or SolidWorks)
    ##
    ##VH=math.pi*(RCylH*RCylH-(RCylH-TCylH)*(RCylH-TCylH))*LCylH
    ##rhoH=1/VH
    ##print(rhoH) #rho for manually cross-checking mass parameters in external program (Excel or SolidWorks)

    """Dataframes:"""
    dfFuel=pd.DataFrame(index=None,columns=["thick","time","mass","centre-of-mass","MOIx","MOIy","MOIz"])
    counterFuel=0
    dfOxid=pd.DataFrame(index=None,columns=["length","time","mass","centre-of-mass","MOIx","MOIy","MOIz"])
    counterOxid=0

    if calculator_t_burn==calculator_t_burn_fuel:
        CompleteGrainBurn=True
    else:
        CompleteGrainBurn=False

    """FUEL:"""
    while calculator_t<=calculator_t_burn:

        VFuel=math.pi*(RFuel*RFuel-(RFuel-TFuel)*(RFuel-TFuel))*LFuel
        MFuel=rhoFuel*VFuel
        MoixFuel=0.5*MFuel*((RFuel*RFuel)+(RFuel-TFuel)**2 ) #HYROPS incorrectly has a negative separator
        MoiyFuel=(1/12)*MFuel*((3*(RFuel**2)+3*((RFuel-TFuel)**2))+LFuel**2) +MFuel*COMFuel*COMFuel
        MoizFuel=MoiyFuel

        calculator_mass_fuel=MFuel
        calculator_COM_fuel=COMFuel
        calculator_MOIx_fuel=MoixFuel
        calculator_MOIy_fuel=MoiyFuel
        calculator_MOIz_fuel=MoizFuel
        
        """F: Fuel, O: Oxidiser"""
        #print("time={:.7f}.   Mass_F={:.3f}.   COM_F={:.3f}.   MOI_F=  {:.8f}.  {:.8f}.".format(calculator_t,calculator_mass_fuel,calculator_COM_fuel,calculator_MOIx_fuel,calculator_MOIy_fuel))#,TFuel,InnerRadius))     #InnerRadius=RFuel-TFuel, and "... Thickness={:.6f}.   InnerRadius={:.6f}"
        dfFuel.loc[counterFuel]=[TFuel,counterFuel*calculator_timestep,calculator_mass_fuel,calculator_COM_fuel,calculator_MOIx_fuel,calculator_MOIy_fuel,calculator_MOIz_fuel]
        counterFuel+=1

        if TFuel>0 and calculator_t_burn_fuel>=0:
            TFuel-=(TFuelInitial/calculator_t_burn)*calculator_timestep
        elif TFuel<0:
            TFuel=0
        else:
            pass
        calculator_t+=calculator_timestep
        calculator_t=round(calculator_t,3)
        calculator_t_burn_fuel-=calculator_timestep
    if CompleteGrainBurn==True:
        dfFuel=dfFuel._append(pd.DataFrame([[0,calculator_t_burn,0,0,0,0,0]],columns=dfFuel.columns.values),ignore_index=True)

    """NOTE: Fuel MOIx is correct. MOIx of fuelgrain in HYROPS is incorrect."""

    calculator_t=0 #reset time

    """OXIDISER:"""
    while LOxid>0:

        LOxidIncrement=(calculator_timestep/calculator_t_burn)*(LOxidInitial/2) #subtract fixed increments
        calculator_COM_oxid=COMOxid-(LOxidInitial-LOxid)/2 #alternatively: "calculator_COM_oxid=COMOxid-counterOxid*LOxidIncrement"

        VOxid=math.pi*(ROxid**2)*LOxid
        MOxid=rhoOxid*VOxid
        MoixOxid=0.5*MOxid*(ROxid**2)
        MoiyOxid=(MOxid/12)*(3*ROxid**2+LOxid*LOxid) +MOxid*calculator_COM_oxid*calculator_COM_oxid #no parallel-axis theorem yet
        MoizOxid=MoiyOxid

        calculator_mass_oxid=MOxid
        calculator_MOIx_oxid=MoixOxid
        calculator_MOIy_oxid=MoiyOxid
        calculator_MOIz_oxid=MoizOxid

        #print("time={:.3f}.   L_O={:.6f}.   Mass_O={:.6f}.   COM_O={:.6f}.   MOI_O=  {:.8f}.  {:.8f}.".format(calculator_t,LOxid,calculator_mass_oxid,calculator_COM_oxid,calculator_MOIx_oxid,calculator_MOIy_oxid))#,TFuel,InnerRadius))     #InnerRadius=RFuel-TFuel, and "... Thickness={:.6f}.   InnerRadius={:.6f}"
        dfOxid.loc[counterOxid]=[LOxid,counterOxid*calculator_timestep,calculator_mass_oxid,calculator_COM_oxid,calculator_MOIx_oxid,calculator_MOIy_oxid,calculator_MOIz_oxid]
        counterOxid+=1
        LOxid-=2*LOxidIncrement
        calculator_t+=calculator_timestep
    if CompleteGrainBurn==True:
        dfOxid=dfOxid._append(pd.DataFrame([[0,calculator_t_burn,0,0,0,0,0]],columns=dfOxid.columns.values),ignore_index=True)

    dfTotal=pd.DataFrame(index=None,columns=["time","mass","centre-of-mass","MOIx","MOIy","MOIz"])
    dfTotal['time']=dfOxid['time']
    dfTotal['mass']=dfFuel['mass']+dfOxid['mass']+endMass
    dfTotal['centre-of-mass']=(dfFuel['mass']*dfFuel['centre-of-mass']+dfOxid['mass']*dfOxid['centre-of-mass']+endMass*endCOM)/(endMass+dfFuel['mass']+dfOxid['mass'])
    dfTotal['MOIx']=dfFuel['MOIx']+dfOxid['MOIx']+endMOIx
    dfTotal['MOIy']=dfFuel['MOIy']+dfOxid['MOIy']+endMOIy
    dfTotal['MOIz']=dfFuel['MOIz']+dfOxid['MOIz']+endMOIz
    dfFinal=pd.DataFrame(index=None,columns=["time","mass","centre-of-mass","MOIx","MOIy","MOIz"])
    dfFinal1=pd.DataFrame(index=None,columns=["time","mass","centre-of-mass","MOIx","MOIy","MOIz"])
    dfFinal2=pd.DataFrame(index=None,columns=["time","mass","centre-of-mass","MOIx","MOIy","MOIz"])
    counterFinal1=0
    counterFinal2=1

    #print(dfTotal.at[len(dfTotal)-1,'time'])
    while counterFinal1<len(dfTotal):
        dfFinal1.at[counterFinal1,'time']=dfTotal.at[counterFinal1,'time']
        dfFinal1.at[counterFinal1,'mass']=dfTotal.at[counterFinal1,'mass']
        dfFinal1.at[counterFinal1,'centre-of-mass']=dfTotal.at[counterFinal1,'centre-of-mass']
        dfFinal1.at[counterFinal1,'MOIx']=dfTotal.at[counterFinal1,'MOIx']
        dfFinal1.at[counterFinal1,'MOIy']=dfTotal.at[counterFinal1,'MOIy']
        dfFinal1.at[counterFinal1,'MOIz']=dfTotal.at[counterFinal1,'MOIz']
        counterFinal1+=1

    while (timestepsFinal)-counterFinal2>0:
        dfFinal2.at[counterFinal2,'time']=dfTotal.at[len(dfTotal)-1,'time']+calculator_timestep*counterFinal2
        dfFinal2.at[counterFinal2,'mass']=dfTotal.at[len(dfTotal)-1,'mass']
        dfFinal2.at[counterFinal2,'centre-of-mass']=dfTotal.at[len(dfTotal)-1,'centre-of-mass']
        dfFinal2.at[counterFinal2,'MOIx']=dfTotal.at[len(dfTotal)-1,'MOIx']
        dfFinal2.at[counterFinal2,'MOIy']=dfTotal.at[len(dfTotal)-1,'MOIy']
        dfFinal2.at[counterFinal2,'MOIz']=dfTotal.at[len(dfTotal)-1,'MOIz']
        counterFinal2+=1

    dfFinal=dfFinal1._append(dfFinal2,ignore_index=True)
    #dfFuel.to_csv(r'{}\inputs\calculator_fuel.csv'.format(Directory),columns=["time","mass","centre-of-mass","MOIx","MOIy","MOIz"],index=False)
    #dfOxid.to_csv(r'{}\inputs\calculator_oxid.csv'.format(Directory),columns=["time","mass","centre-of-mass","MOIx","MOIy","MOIz"],index=False)
    dfFinal.to_excel(r'{}\Inputs\mass_properties.xlsx'.format(Directory),columns=["time","mass","MOIx","MOIy","MOIz","centre-of-mass"],index=False)
    pd.set_option('display.max_columns', None)

