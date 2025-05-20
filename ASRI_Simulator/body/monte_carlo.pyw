from scipy import stats
import statistics
import pandas as pd
import numpy as np
import math

class monte_carlo:
    def __init__(self,
                 mclower_elevation,
                 mcupper_elevation,
                 mclower_azimuth,
                 mcupper_azimuth,
                 mclower_thrustmx,
                 mcupper_thrustmx,
                 mclower_thrustmy,
                 mcupper_thrustmy,
                 mclower_thrustmag,
                 mcupper_thrustmag,
                 mclower_burntime,
                 mcupper_burntime,
                 mclower_windmag,
                 mcupper_windmag,
                 mclower_winddir,
                 mcupper_winddir,
                 mclower_aerodragcoeff,
                 mcupper_aerodragcoeff,
                 mclower_aeroliftcoeff,
                 mcupper_aeroliftcoeff,
                 mclower_aeromomentcoeff,
                 mcupper_aeromomentcoeff,
                 mclower_centrepressure,
                 mcupper_centrepressure,
                 mclower_fincant,
                 mcupper_fincant,
                 mclower_altitude,
                 mcupper_altitude,
                 normalised_main
                 ):

        self.mclower_thrustmx=mclower_thrustmx
        self.mcupper_thrustmx=mcupper_thrustmx
        self.mclower_thrustmy=mclower_thrustmy
        self.mcupper_thrustmy=mcupper_thrustmy
        self.mclower_thrustmag=mclower_thrustmag
        self.mcupper_thrustmag=mcupper_thrustmag
        self.mclower_windmag=mclower_windmag
        self.mcupper_windmag=mcupper_windmag
        self.mclower_winddir=mclower_winddir
        self.mcupper_winddir=mcupper_winddir
        self.mclower_aerodragcoeff=mclower_aerodragcoeff
        self.mcupper_aerodragcoeff=mcupper_aerodragcoeff
        self.mclower_aeroliftcoeff=mclower_aeroliftcoeff
        self.mcupper_aeroliftcoeff=mcupper_aeroliftcoeff
        self.mclower_aeromomentcoeff=mclower_aeromomentcoeff
        self.mcupper_aeromomentcoeff=mcupper_aeromomentcoeff
        self.mclower_centrepressure=mclower_centrepressure
        self.mcupper_centrepressure=mcupper_centrepressure
        self.mclower_fincant=mclower_fincant
        self.mcupper_fincant=mcupper_fincant
        self.mclower_altitude=mclower_altitude
        self.mcupper_altitude=mcupper_altitude
        self.mclower_elevation=mclower_elevation
        self.mcupper_elevation=mcupper_elevation
        self.mclower_azimuth=mclower_azimuth
        self.mcupper_azimuth=mcupper_azimuth
        self.mclower_burntime=mclower_burntime
        self.mcupper_burntime=mcupper_burntime
        self.normalised_main=normalised_main

    def outputs(self):

        MonteCarloElevationVariation=                ( (self.mclower_elevation)+self.normalised_main[11]*(self.mcupper_elevation-self.mclower_elevation) )
        MonteCarloAzimuthVariation=                  ( (self.mclower_azimuth)+self.normalised_main[12]*(self.mcupper_azimuth-self.mclower_azimuth) )
        MonteCarloThrustMisalignmentYawing=          ( (self.mclower_thrustmx)+self.normalised_main[0]*(self.mcupper_thrustmx-self.mclower_thrustmx) )
        MonteCarloThrustMisalignmentPitching=        ( (self.mclower_thrustmy)+self.normalised_main[1]*(self.mcupper_thrustmy-self.mclower_thrustmy) )
        MonteCarloThrustMagnitudeVariation=          ( (self.mclower_thrustmag)+self.normalised_main[2]*(self.mcupper_thrustmag-self.mclower_thrustmag) )
        MonteCarloTimeBurnout=                       ( (self.mclower_burntime)+self.normalised_main[19]*(self.mcupper_burntime-self.mclower_burntime) )
        MonteCarloWindMagnitudeVariation=            ( (self.mclower_windmag)+self.normalised_main[3]*(self.mcupper_windmag-self.mclower_windmag) )
        MonteCarloWindDirectionVariation=            ( (self.mclower_winddir)+self.normalised_main[4]*(self.mcupper_winddir-self.mclower_winddir) )
        MonteCarloDragCoefficientVariation=          ( (self.mclower_aerodragcoeff)+self.normalised_main[5]*(self.mcupper_aerodragcoeff-self.mclower_aerodragcoeff) )
        MonteCarloLiftCoefficientVariation=          ( (self.mclower_aeroliftcoeff)+self.normalised_main[6]*(self.mcupper_aeroliftcoeff-self.mclower_aeroliftcoeff) )
        MonteCarloMomentCoefficientVariation=        ( (self.mclower_aeromomentcoeff)+self.normalised_main[7]*(self.mcupper_aeromomentcoeff-self.mclower_aeromomentcoeff) )
        MonteCarloCentrePressureVariation=           ( (self.mclower_centrepressure)+self.normalised_main[8]*(self.mcupper_centrepressure-self.mclower_centrepressure) )
        MonteCarloFinCantVariation=                  ( (self.mclower_fincant)+self.normalised_main[9]*(self.mcupper_fincant-self.mclower_fincant) )
        MonteCarloAltitudeVariation=                 ( (self.mclower_altitude)+self.normalised_main[10]*(self.mcupper_altitude-self.mclower_altitude) )
        MonteCarloElevationVariationStaged=          ( (self.mclower_elevation)+self.normalised_main[13]*(self.mcupper_elevation-self.mclower_elevation) )
        MonteCarloAzimuthVariationStaged=            ( (self.mclower_azimuth)+self.normalised_main[14]*(self.mcupper_azimuth-self.mclower_azimuth) )
        MonteCarloWindDirectionVariationStaged=      ( (self.mclower_winddir)+self.normalised_main[15]*(self.mcupper_winddir-self.mclower_winddir) ) 
        MonteCarloElevationVariationApogee=          ( (self.mclower_elevation)+self.normalised_main[16]*(self.mcupper_elevation-self.mclower_elevation) )
        MonteCarloAzimuthVariationApogee=            ( (self.mclower_azimuth)+self.normalised_main[17]*(self.mcupper_azimuth-self.mclower_azimuth) )
        MonteCarloWindDirectionVariationApogee=      ( (self.mclower_winddir)+self.normalised_main[18]*(self.mcupper_winddir-self.mclower_winddir) )

        return np.array([MonteCarloElevationVariation,
                         MonteCarloAzimuthVariation,
                         MonteCarloThrustMisalignmentYawing,
                         MonteCarloThrustMisalignmentPitching,
                         MonteCarloThrustMagnitudeVariation,
                         MonteCarloTimeBurnout,
                         MonteCarloWindMagnitudeVariation,
                         MonteCarloWindDirectionVariation,
                         MonteCarloDragCoefficientVariation,
                         MonteCarloLiftCoefficientVariation,
                         MonteCarloMomentCoefficientVariation,
                         MonteCarloCentrePressureVariation,
                         MonteCarloFinCantVariation,
                         MonteCarloAltitudeVariation,
                         MonteCarloElevationVariationStaged,
                         MonteCarloAzimuthVariationStaged,
                         MonteCarloWindDirectionVariationStaged,
                         MonteCarloElevationVariationApogee,
                         MonteCarloAzimuthVariationApogee,
                         MonteCarloWindDirectionVariationApogee],dtype=float)

    @staticmethod
    def interface(MonteCarloInputs,normalised_main):
        elevation_lower=                      float(MonteCarloInputs["LaunchElevationLower"])
        elevation_upper=                      float(MonteCarloInputs["LaunchElevationUpper"])
        azimuth_lower=                        float(MonteCarloInputs["LaunchAzimuthLower"])
        azimuth_upper=                        float(MonteCarloInputs["LaunchAzimuthUpper"])
        thrustmx_lower=                       float(MonteCarloInputs["ThrustMisalignmentYawLower"])
        thrustmx_upper=                       float(MonteCarloInputs["ThrustMisalignmentYawUpper"])
        thrustmy_lower=                       float(MonteCarloInputs["ThrustMisalignmentPitchLower"])
        thrustmy_upper=                       float(MonteCarloInputs["ThrustMisalignmentPitchUpper"])
        thrustmag_lower=                      float(MonteCarloInputs["ThrustMagnitudeLower"])
        thrustmag_upper=                      float(MonteCarloInputs["ThrustMagnitudeUpper"])
        burntime_lower=                       float(MonteCarloInputs["TimeBurnLower"])
        burntime_upper=                       float(MonteCarloInputs["TimeBurnUpper"])
        windmag_lower=                        float(MonteCarloInputs["WindMagnitudeLower"])
        windmag_upper=                        float(MonteCarloInputs["WindMagnitudeUpper"])
        winddir_lower=                        float(MonteCarloInputs["WindDirectionLower"])
        winddir_upper=                        float(MonteCarloInputs["WindDirectionUpper"])
        aerodragcoeff_lower=                  float(MonteCarloInputs["AerodynamicDragLower"])
        aerodragcoeff_upper=                  float(MonteCarloInputs["AerodynamicDragUpper"])
        aeroliftcoeff_lower=                  float(MonteCarloInputs["AerodynamicLiftLower"])
        aeroliftcoeff_upper=                  float(MonteCarloInputs["AerodynamicLiftUpper"])
        aeromomentcoeff_lower=                float(MonteCarloInputs["AerodynamicMomentLower"])
        aeromomentcoeff_upper=                float(MonteCarloInputs["AerodynamicMomentUpper"])
        centrepressure_lower=                 float(MonteCarloInputs["CentreOfPressureLower"])
        centrepressure_upper=                 float(MonteCarloInputs["CentreOfPressureUpper"])
        fincant_lower=                        float(MonteCarloInputs["FinCantAngleLower"])
        fincant_upper=                        float(MonteCarloInputs["FinCantAngleUpper"])
        altitude_lower=                       float(MonteCarloInputs["LaunchAltitudeLower"])
        altitude_upper=                       float(MonteCarloInputs["LaunchAltitudeUpper"])

        return monte_carlo(elevation_lower,elevation_upper,azimuth_lower,azimuth_upper,thrustmx_lower,thrustmx_upper,thrustmy_lower,thrustmy_upper,thrustmag_lower,thrustmag_upper,burntime_lower,burntime_upper,windmag_lower,windmag_upper,winddir_lower,winddir_upper,aerodragcoeff_lower,aerodragcoeff_upper,aeroliftcoeff_lower,aeroliftcoeff_upper,aeromomentcoeff_lower,aeromomentcoeff_upper,centrepressure_lower,centrepressure_upper,fincant_lower,fincant_upper,altitude_lower,altitude_upper,normalised_main)

    @staticmethod
    def bounds(table,normalised_main):
        elevation_lower=                      float(table.at[0,"lower_limit"])
        elevation_upper=                      float(table.at[0,"upper_limit"])
        azimuth_lower=                        float(table.at[1,"lower_limit"])
        azimuth_upper=                        float(table.at[1,"upper_limit"])
        thrustmx_lower=                       float(table.at[2,"lower_limit"])
        thrustmx_upper=                       float(table.at[2,"upper_limit"])
        thrustmy_lower=                       float(table.at[3,"lower_limit"])
        thrustmy_upper=                       float(table.at[3,"upper_limit"])
        thrustmag_lower=                      float(table.at[4,"lower_limit"])
        thrustmag_upper=                      float(table.at[4,"upper_limit"])
        burntime_lower=                       float(table.at[5,"lower_limit"])
        burntime_upper=                       float(table.at[5,"upper_limit"])
        windmag_lower=                        float(table.at[6,"lower_limit"])
        windmag_upper=                        float(table.at[6,"upper_limit"])
        winddir_lower=                        float(table.at[7,"lower_limit"])
        winddir_upper=                        float(table.at[7,"upper_limit"])
        aerodragcoeff_lower=                  float(table.at[8,"lower_limit"])
        aerodragcoeff_upper=                  float(table.at[8,"upper_limit"])
        aeroliftcoeff_lower=                  float(table.at[9,"lower_limit"])
        aeroliftcoeff_upper=                  float(table.at[9,"upper_limit"])
        aeromomentcoeff_lower=                float(table.at[10,"lower_limit"])
        aeromomentcoeff_upper=                float(table.at[10,"upper_limit"])
        centrepressure_lower=                 float(table.at[11,"lower_limit"])
        centrepressure_upper=                 float(table.at[11,"upper_limit"])
        fincant_lower=                        float(table.at[12,"lower_limit"])
        fincant_upper=                        float(table.at[12,"upper_limit"])
        altitude_lower=                       float(table.at[13,"lower_limit"])
        altitude_upper=                       float(table.at[13,"upper_limit"])
        return monte_carlo(elevation_lower,elevation_upper,azimuth_lower,azimuth_upper,thrustmx_lower,thrustmx_upper,thrustmy_lower,thrustmy_upper,thrustmag_lower,thrustmag_upper,burntime_lower,burntime_upper,windmag_lower,windmag_upper,winddir_lower,winddir_upper,aerodragcoeff_lower,aerodragcoeff_upper,aeroliftcoeff_lower,aeroliftcoeff_upper,aeromomentcoeff_lower,aeromomentcoeff_upper,centrepressure_lower,centrepressure_upper,fincant_lower,fincant_upper,altitude_lower,altitude_upper,normalised_main)

    @staticmethod
    def user_input(user,normalised_main):
        return monte_carlo.interface(user,normalised_main)

    @staticmethod
    def read_excel(directory,normalised_main):
        dataframe=pd.read_excel(directory,header=0)
        dataframe.columns=["uncertainty", "units", "lower_limit", "upper_limit"]             
        return monte_carlo.bounds(dataframe,normalised_main)
    
    @staticmethod
    def random(user,directory,selection,population=20):
        """Truncate abscissae:"""
        a=(0-0)/1.1
        b=(1-0)/1.1
        low,high,mean,stddev,num_pop=                a,b,0,1.1,population
        normalised_main=                             stats.truncnorm.rvs(low,high,loc=mean,scale=stddev,size=num_pop)
        if selection == True:
            return monte_carlo.read_excel(directory,normalised_main)
        else:
            return monte_carlo.user_input(user,normalised_main)
