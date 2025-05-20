import math

def fins(mach,taoa,phip,Parameters3,Va2,DDD,Schar,Variation,TimeStep,rbod,Thruster,t):
    FinRootChord=Parameters3[0]
    FinTipChord=Parameters3[1]
    FinSweep=Parameters3[2]
    FinSpan=Parameters3[3]
    FinLocation=Parameters3[4]
    FinSpanRoot=Parameters3[5]
    FinCantAngle=Parameters3[6]
    CheckRollControl=Parameters3[7]
    RollControlTimeInitial=Parameters3[8]
    RollControlTimeFinal=Parameters3[9]
    RollControlFrequency=Parameters3[10]
    RollControlLower=Parameters3[11]
    RollControlUpper=Parameters3[12]
    RollControlForce=Parameters3[13]
    
    rootspan=                               FinSpanRoot
    cant=                                   FinCantAngle+Variation
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



    """CheckRollControl=Parameters3[7]
    if CheckRollControl==True:
        if t>Parameters3[8] and t<Parameters3[9]:
            RollControlFrequency=Parameters3[10]
            if (round(t,3))%(1/RollControlFrequency)==0:
                if DDD<Parameters3[11]:
                    Cmom[0]=Parameters3[13]*rbod
                if DDD>Parameters3[12]:
                    Cmom[0]=-Parameters3[13]*rbod
    """

    RCSmom=0
    ControlTime=0
    if CheckRollControl==True:
        if t>Parameters3[8] and t<Parameters3[9]:
            Period=(1/Parameters3[10])
            PulseWidth=Period/2
            if DDD<Parameters3[11]:
                if ControlTime<=PulseWidth:
                    Thruster=True
                    RCSmom=Parameters3[13]*rbod
                elif ControlTime>PulseWidth and ControlTime<=(PulseWidth*2):
                    Thruster=False
                if (ControlTime+TimeStep)<Period:
                    ControlTime=round(ControlTime+TimeStep,3)
                elif (ControlTime+TimeStep)>=Period:
                    ControlTime=0
            if DDD>Parameters3[12]:
                if ControlTime<=PulseWidth:
                    Thruster=True
                    RCSmom=-Parameters3[13]*rbod
                elif ControlTime>PulseWidth and ControlTime<=(PulseWidth*2):
                    Thruster=False
                if (ControlTime+TimeStep)<Period:
                    ControlTime=round(ControlTime+TimeStep,3)
                elif (ControlTime+TimeStep)>=Period:
                    ControlTime=0
            if DDD>=Parameters3[11] and DDD<=Parameters3[12]:
                ControlTime=0 

    else:
        if mach<MTlower:
            Mach=0
            cna1=(2*math.pi*s*s)/((1+math.sqrt(1+((betsp*s*s)/(Afin*cosG))**2))*Aref)
            cna=(numfins/2)*cna1*(1-(0.06*(math.sin(phip*2))**2))*(1+tau)
            rollmoment=(Smac+r)*cna*cant*rad
            rolldampingmoment=-(numfins*2*math.pi*DDD*rollsum)/(Aref*math.sqrt(Va2)*betsp)
            Cdp=((4*(taoa*cna1)**2)/(1.1*cna1))*(Afin/Aref)

        if mach>MTupper:
            Mach=1
            k1=2/betsp
            k2=(((gamma+1)*(mach)**4)-(4*betsp*betsp))/(4*(betsp)**4)
            k3=(((gamma+1)*(mach)**8)+(((2*gamma*gamma)-(7*gamma)-5)*(mach)**6)+(10*(gamma+1)*(mach)**4)+8)/(6*(betsp)**7)
            
            cna1=(Afin/Aref)*(k1+(k2*taoa)+(k3*taoa*taoa))
            cna=(numfins/2)*cna1*(1-(0.06*(math.sin(phip*2))**2))*(1+tau)
            rollmoment=(Smac+r)*cna*cant*rad
            rollsumsp=sg*(((k1*(DDD/math.sqrt(Va2))*(r+s)**4)/4) + (((k2*(DDD/math.sqrt(Va2))**2)*(r+s)**5)/5) + ((k3*((DDD/math.sqrt(Va2))**3)*(r+s)**6)/6))
            rollsumsp-=sg*(((k1*(DDD/math.sqrt(Va2))*(r)**4)/4) + ((k2*((DDD/math.sqrt(Va2))**2)*(r)**5)/5) + ((k3*(DDD/math.sqrt(Va2))**3*(r)**6)/6))
            rollsumsp+=(rootchord-(sg*r))*(((k1*(DDD/math.sqrt(Va2))*(r+s)**3)/3) + ((k2*((DDD/math.sqrt(Va2))**2)*(r+s)**4)/4) + ((k3*((DDD/math.sqrt(Va2))**3)*(r+s)**5)/5))
            rollsumsp-=(rootchord-(sg*r))*(((k1*(DDD/math.sqrt(Va2))*(r)**3)/3) + ((k2*((DDD/math.sqrt(Va2))**2)*(r)**4)/4) + ((k3*((DDD/math.sqrt(Va2))**3)*(r)**5)/5))
            rolldampingmoment=-(numfins*rollsumsp)/Aref
            Cdp=betsp*((taoa*cna1)**2)*(Afin/Aref)

        if mach>=MTlower and mach<=MTupper:
            Mach=0
            betsp=math.sqrt(1-(MTlower*MTlower))
            cna1=(2*math.pi*s*s)/((1+math.sqrt(1+((betsp*s*s)/(Afin*cosG))**2))*Aref)
            cnasub=(numfins/2)*cna1*(1-(0.06*(math.sin(phip*2))**2))*(1+tau)
            rollmomentsb=(Smac+r)*cnasub*cant*rad
            rolldampingmomentsb=-(numfins*2*math.pi*DDD*rollsum)/(Aref*math.sqrt(Va2)*betsp)
            betsp=math.sqrt((MTupper*MTupper)-1)
            k1=2/betsp
            k2=(((gamma+1)*(MTupper)**4) - (4*betsp*betsp))/(4*(betsp)**4)
            k3=(((gamma+1)*(MTupper)**8) + (((2*gamma*gamma)-(7*gamma)-5)*(MTupper)**6) + (10*(gamma+1)*(MTupper)**4)+8)/(6*(betsp)**7)
            cna1=(Afin/Aref)*(k1+(k2*taoa)+(k3*taoa*taoa))
            cnasup=(numfins/2)*cna1*(1-(0.06*(math.sin(phip*2))**2))*(1+tau)
            rollmomentsp=(Smac+r)*cnasup*cant*rad
            rollsumsp=sg*(((k1*(DDD/math.sqrt(Va2))*(r+s)**4)/4) + ((k2*((DDD/math.sqrt(Va2))**2)*(r+s)**5)/5) + ((k3*(DDD/math.sqrt(Va2))**3)*(r+s)**6)/6) 
            rollsumsp-=sg*(((k1*(DDD/math.sqrt(Va2))*(r)**4)/4)+((k2*((DDD/math.sqrt(Va2))**2*(r)**5)/5) + ((k3*(DDD/math.sqrt(Va2))**3)*(r)**6)/6))
            rollsumsp+=(rootchord-(sg*r))*(((k1*(DDD/math.sqrt(Va2))*(r+s)**3)/3) + ((k2*((DDD/math.sqrt(Va2))**2)*(r+s)**4)/4) + ((k3*((DDD/math.sqrt(Va2))**3)*(r+s)**5)/5))
            rollsumsp-=(rootchord-(sg*r))*(((k1*(DDD/math.sqrt(Va2))*(r)**3)/3) + ((k2*((DDD/math.sqrt(Va2))**2)*(r)**4)/4) + ((k3*((DDD/math.sqrt(Va2))**3)*(r)**5)/5)) 

            rolldampingmomentsp=-(numfins*rollsumsp)/Aref
            rollmoment=(rollmomentsb*((MTupper-mach)/(MTupper-MTlower)))+(rollmomentsp*((mach-MTlower)/(MTupper-MTlower)))
            rolldampingmoment=(rolldampingmomentsb*((MTupper-mach)/(MTupper-MTlower)))+(rolldampingmomentsp*((mach-MTlower)/(MTupper-MTlower)))      
            cna=(cnasub*((MTupper-mach)/(MTupper-MTlower)))+(cnasup*((mach-MTlower)/(MTupper-MTlower)))

        Cl=rollmoment+rolldampingmoment

        if DDD>0:
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

    return Cdp,Cl,RCSmom,Thruster
