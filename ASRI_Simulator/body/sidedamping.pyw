import math

def SideDamping(EEE,FFF,cog,Parameters,Parameters2,Schar,Va2):
    r1=                                     Parameters[0]
    r2=                                     Parameters[1]
    r3=                                     Parameters[2]
    r4=                                     Parameters[3]
    r5=                                     Parameters[4]
    r6=                                     Parameters[5]
    r7=                                     Parameters[6]
    r8=                                     Parameters[7]
    s1=                                     Parameters[8]
    s2=                                     Parameters[9]
    s3=                                     Parameters[10]
    s4=                                     Parameters[11]
    s5=                                     Parameters[12]
    s6=                                     Parameters[13]
    s7=                                     Parameters[14]
    s8=                                     Parameters[15]
    NoseRadius=                             Parameters2[0]
    NoseLength=                             Parameters2[1]
    FinRootChord=                           Parameters2[2]
    FinTipChord=                            Parameters2[3]
    FinSweep=                               Parameters2[4]
    FinSpan=                                Parameters2[5]
    FinLocation=                            Parameters2[6]
        
    RadiusOgive=((NoseRadius*NoseRadius)+(NoseLength*NoseLength))/(2*NoseRadius)
    AreaNose=(math.asin(NoseLength/RadiusOgive)*RadiusOgive*RadiusOgive)-(NoseLength*(RadiusOgive-NoseRadius))
    TotalLength=s1+s2+s3+s4
    Aeq=(NoseRadius+r1)*s1+(r1+r2)*s2+(r2+r3)*s3+(r3+r4)*s4+(r4+r5)*s5+(r5+r6)*s6+(r6+r7)*s7+(r7+r8)*s8+AreaNose
    Leq=NoseLength+TotalLength
    Deq=Aeq/Leq
    pitdamp=0
    yawdamp=0
    
    if Va2>0:
        m1=(0.275*Deq)/Schar
        m2=((cog**4)+(Leq-cog)**4)
        m3=(0.6*4*((FinRootChord+FinTipChord)*0.5*FinSpan)*(((FinLocation+0.5*FinSweep+0.25*(FinRootChord+FinTipChord))-cog)**3))/Schar
        multiplier=3*(m1*m2+m3)
        pitdamp=multiplier*(EEE/math.sqrt(Va2))**2
        yawdamp=multiplier*(FFF/math.sqrt(Va2))**2
        
    return pitdamp,yawdamp
