"""
utils.py

provides constants and configurable parameters for the rocket

author: jasper yun
"""

from enum import Enum

# might be better to *not* use a class but this is quick and dirty to get it running
class Settings:

    # nature's constants
    GRAVITY = 9.80665 # m/s^2
    AIR_MASS_DENSITY = 1.204 # kg/m^3 -- for more accuracy, should be a function of altitude and temperature

    # environment settings
    GROUND_ALTITUDE_M = 0
    LOCAL_PRESSURE_hPa = 1018 # hPa

    # simulation parameters
    SIMULATION_TIMESTEP_MS = 1
    HARDWARE_UPDATE_TIMESTEP_MS = 10 * SIMULATION_TIMESTEP_MS
    PRINT_UPDATE_TIMESTEP_MS = 10 * SIMULATION_TIMESTEP_MS # frequency of printing to console
    USE_NOISY_ALTITUDE = False
    ALTITUDE_NOISE_MAX_AMPLITUDE_M = 10
    USE_HARDWARE_TARGET = True
    SIMULATION_SW_TARGET_LAUNCH_TIME_MS = 1000

    # communication over serial with FC
    PACKET_HEADER = 0xFFFF0000  # 32-bit value
    PACKET_TRAILER = 0x0000FFFF # 32-bit value

    # rocket parameters for simulation
    RKT_MASS_KG = 48
    RKT_DROGUE_DRAG_COEFF = 1.5
    RKT_DROGUE_AREA = 3.14 * (1.21 ** 2)
    RKT_MAIN_DRAG_COEFF = 2.2
    RKT_MAIN_AREA = 3.14 * (3.66 ** 2)
    RKT_LAUNCH_ANGLE = 90
    RKT_THRUST_CURVE_FILE = 'Cesaroni_21062O3400-P.rse' # rse file -- filename relative to this file

# used to track states of the simulation
class FLIGHT_STATES(Enum):
    PAD = 0
    BOOST = 1
    COAST = 2
    DROGUE_DESCENT = 3
    MAIN_DESCENT = 4
    LANDED = 5
    LAUNCH_COMMAND_START_SIM = 8
    ERROR = 9

class Sim_DataPoint:
    def __init__(self, time, pos_x, pos_z, pos_z_noisy, vel_x, vel_z, acc_x, acc_z, flight_state):
        self.time = time
        self.rkt_pos_x = pos_x
        self.rkt_pos_z = pos_z
        self.rkt_pos_z_noisy = pos_z_noisy
        self.rkt_vel_x = vel_x
        self.rkt_vel_z = vel_z
        self.rkt_acc_x = acc_x
        self.rkt_acc_z = acc_z
        self.rkt_flight_state = flight_state
    
    def unwrap(self):
        return [self.time, self.rkt_pos_x, self.rkt_pos_z, self.rkt_pos_z_noisy, self.rkt_vel_x, self.rkt_vel_z, self.rkt_acc_x, self.rkt_acc_z, self.rkt_flight_state]
        
