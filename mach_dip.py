"""
mach_dip.py

emulates a mach dip in the barometric pressure based on the vertical velocity of the rocket.
*** this mach dip has no basis in transonic shockwave physics ***

the mach dip will be emulated using a triangular piecewise linear curve for *altitude* with
upward/downward slopes specified. see parameters for descriptions. 

parameters:
    - MACH_DIP_UPWARD_SLOPE_HPA_PER_MPS         slope of the upward triangle segment in units of hPa per m/s
    - MACH_DIP_DOWNARD_SLOPE_HPA_PER_MPS        slope of the downward triangle segment in units of hPa per m/s
    - MACH_DIP_MIDPOINT_FACTOR                  middle of the triangle occurs at speed of sound * MIDPOINT_FACTOR
    - MACH_DIP_WIDTH_FACTOR                     mach dip happens only while the rocket vertical velocity is between
                                                (MIDPOINT - WIDTH/2, MIDPOINT + WIDTH/2)

author: jasper yun
"""

import ambiance
import numpy as np
import utils

MACH_DIP_UPWARD_SLOPE_HPA_PER_MPS = 2
MACH_DIP_DOWNWARD_SLOPE_HPA_PER_MPS = -2
MACH_DIP_MIDPOINT_FACTOR = 1.2
MACH_DIP_WIDTH_FACTOR = 0.2

# track altitudes of points inside the mach dip
triangle_start_datapoint = None

def get_mach_dip_altitude(data : utils.Sim_DataPoint):
    """
    returns mach dip-adjusted altitude_m
    """
    # check speed of sound based on true altitude
    global triangle_start_datapoint
    if utils.Settings.USE_NOISY_ALTITUDE == True:
        speed_of_sound_mps = ambiance.Atmosphere(data.rkt_pos_z_noisy).speed_of_sound[0]
    else:
        speed_of_sound_mps = ambiance.Atmosphere(data.rkt_pos_z).speed_of_sound[0]

    if (data.rkt_vel_z > speed_of_sound_mps * (MACH_DIP_MIDPOINT_FACTOR - MACH_DIP_WIDTH_FACTOR / 2) 
        and data.rkt_vel_z < speed_of_sound_mps * (MACH_DIP_MIDPOINT_FACTOR + MACH_DIP_WIDTH_FACTOR / 2)):
        
        if triangle_start_datapoint is None:
            triangle_start_datapoint = data

        base_pressure = ambiance.Atmosphere(triangle_start_datapoint.rkt_pos_z).pressure[0]
        # base_pressure = ambiance.Atmosphere(triangle_start_datapoint.rkt_pos_z).pressure[0]
        if data.rkt_acc_z > 0:
            # breaking sound barrier for first time, triangle shape is \/
            # pressure INCREASE corresponds to altitude DECREASE
            if data.rkt_vel_z < speed_of_sound_mps * MACH_DIP_MIDPOINT_FACTOR:
                # \ part of the triangle
                # adjustment is relative to first point of the triangle
                pressure_adjustment = MACH_DIP_UPWARD_SLOPE_HPA_PER_MPS * np.abs(data.rkt_vel_z - triangle_start_datapoint.rkt_vel_z)
            else:
                # / part of triangle
                pressure_adjustment = MACH_DIP_DOWNWARD_SLOPE_HPA_PER_MPS * np.abs(data.rkt_vel_z - triangle_start_datapoint.rkt_vel_z)
            
        else:
            # triangle shape is /\
            if data.rkt_vel_z > speed_of_sound_mps * MACH_DIP_MIDPOINT_FACTOR:
                # / part of triangle
                pressure_adjustment = MACH_DIP_DOWNWARD_SLOPE_HPA_PER_MPS * np.abs(data.rkt_vel_z - triangle_start_datapoint.rkt_vel_z)
            else:
                # \ part of triangle
                pressure_adjustment = MACH_DIP_UPWARD_SLOPE_HPA_PER_MPS * np.abs(data.rkt_vel_z - triangle_start_datapoint.rkt_vel_z)

        pressure = (base_pressure + pressure_adjustment)
        altitude = ambiance.Atmosphere.from_pressure(pressure).h[0]
        pressure /= 100 # hPa to be returned instead of Pa
            
    else:
        # do nothing
        triangle_start_datapoint = None
        altitude = data.rkt_pos_z
        pressure = ambiance.Atmosphere(data.rkt_pos_z).pressure[0] / 100 # hPa instead of Pa
    
    # return (pressure, altitude)
    return altitude

def test():
    # generate array of test datapoints
    array_length = 100
    acc_z = ([1] * int(array_length / 2)) + ([-1] * int(array_length / 2))
    vel_z_up = np.linspace(0, 1000, num=int(array_length/2))
    vel_z_down = np.linspace(1000, 0, num=int(array_length/2))
    vel_z = np.concatenate((vel_z_up, vel_z_down), axis=None)
    pos_z = np.linspace(0, 10000, num=array_length)

    results_altitude = []
    results_pressure = []

    for i in range(array_length):
        # create datapoint -- only pos_z, vel_z, acc_z are needed to test the function
        dp = utils.Sim_DataPoint(i, 0, pos_z[i], 0, 0, vel_z[i], 0, acc_z[i], 0)
        # pass to function and record results
        altitude = get_mach_dip_altitude(dp)
        pressure = ambiance.Atmosphere(altitude).pressure[0]
        results_altitude.append(altitude)
        results_pressure.append(pressure)

    # plot
    fig, ax = plt.subplots(2, 1)
    ax[0].plot(np.arange(array_length, step=1), results_altitude)
    ax[0].set_ylabel('altitude')
    ax[1].plot(np.arange(array_length, step=1), results_pressure)
    ax[1].set_ylabel('pressure')
    ax[1].set_xlabel('sample')
    plt.show()


if __name__ == '__main__':
    from matplotlib import pyplot as plt
    test()    
