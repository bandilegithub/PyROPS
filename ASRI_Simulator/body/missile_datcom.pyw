import os
import pandas as pd

def MissileDATCOM(MissileDATCOMCardRows,MissileDATCOMCards,Directory):
    dfMD=pd.DataFrame(index=None,columns=["Mach","Alpha","-","-","-","CD","-","-","CN","-","-","-","CP"])
    MDIndex=0
    with open(r'{}\Inputs\MissileDATCOM.txt'.format(Directory)) as f:
        for index, line in enumerate(f):
            if '                   ----- LONGITUDINAL -----     -- LATERAL DIRECTIONAL --' in line:
                MDIndex=index
                break
    with open(r'{}\Inputs\MissileDATCOM.txt'.format(Directory)) as f:
        lines = f.readlines()
        
    """'Card' row number:"""
    MDcounter=0
    """'Card' number:"""
    MDcounter2=0
    
    skip=622
    while MDcounter2<MissileDATCOMCards:
        autoskip=MDcounter2*skip
        while MDcounter<=MissileDATCOMCardRows:
            dfMD.at[MDcounter+MDcounter2*19,'Mach']="{}".format(lines[MDIndex-6+autoskip:MDIndex-5+autoskip])[24:28] # or 0.1*(counter2+1)
            dfMD.at[MDcounter+MDcounter2*19,'Alpha']="{}".format(lines[MDIndex+25+MDcounter+autoskip:MDIndex+26+MDcounter+autoskip])[12:16]
            dfMD.at[MDcounter+MDcounter2*19,'CN']="{}".format(lines[MDIndex+25+MDcounter+autoskip:MDIndex+26+MDcounter+autoskip])[21:26]
            dfMD.at[MDcounter+MDcounter2*19,'CD']="{}".format(lines[MDIndex+25+MDcounter+autoskip:MDIndex+26+MDcounter+autoskip])[31:36]
            dfMD.at[MDcounter+MDcounter2*19,'CP']="{}".format(lines[MDIndex+25+MDcounter+autoskip:MDIndex+26+MDcounter+autoskip])[50:56]
            MDcounter+=1
        MDcounter=0
        MDcounter2+=1
    print(dfMD.to_string())
    dfMD.to_excel(r'{}\Inputs\MissileDATCOM.xlsx'.format(Directory),index=False,engine='openpyxl')
