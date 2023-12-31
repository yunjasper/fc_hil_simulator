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

    # communication over serial with FC
    PACKET_HEADER = 0xFFFF0000  # 32-bit value
    PACKET_TRAILER = 0x0000FFFF # 32-bit value

    # rocket parameters for simulation
    RKT_MASS_KG = 40 # porthos is 56.3 kg
    RKT_DROGUE_DRAG_COEFF = 1.5 # porthos drogue
    RKT_DROGUE_AREA = 3.14 * (1.21 ** 2)
    RKT_MAIN_DRAG_COEFF = 2.2 # porthos main
    RKT_MAIN_AREA = 3.14 * (3.66 ** 2)
    RKT_LAUNCH_ANGLE = 90
    RKT_THRUST_CURVE_FILE = 'Cesaroni_21062O3400-P.rse' # rse file -- filename relative to this file
    # RKT_THRUST_CURVE_FILE = 'Cesaroni_14263N3400-P.rse'

    # uncomment for test case with no parachutes. trajectory should be a perfect parabola
    # RKT_DROGUE_DRAG_COEFF = 0
    # RKT_DROGUE_AREA = 0
    # RKT_MAIN_DRAG_COEFF = 0
    # RKT_MAIN_AREA = 0
    TEST_EARLY_EJECTION_CATO = False # as of Aug 7, this feature does not fully work -- I think ejection occurs twice
    EJECTION_CATO_ALTITUDE_M = 1000 / 2
    
    # Altus Metrum (AltOS) flight data usage configuration
    USE_ALTOS_FLIGHT_DATA = False
    ALTOS_FLIGHT_DATA_FILENAME = 'flight_data/2020-10-10-serial-2378-flight-0021.csv'
    ALTOS_FLIGHT_DATA_TYPES = ['time', 'accel_x', 'accel_y', 'accel_z', 'pressure']

    # simulation parameters
    USE_HARDWARE_TARGET = False
    SEND_ALTITUDE_INSTEAD_OF_PRESSURE = True
    SIMULATE_TRANSONIC_MACH_DIP = True
    USE_NOISY_ALTITUDE = False
    USE_GUI = True

    ALTITUDE_NOISE_MAX_AMPLITUDE_M = 3
    
    SIMULATION_TIMESTEP_MS = 10
    HARDWARE_UPDATE_TIMESTEP_MS = 1 * SIMULATION_TIMESTEP_MS
    PRINT_UPDATE_TIMESTEP_MS = 1 * SIMULATION_TIMESTEP_MS # frequency of printing to console
    SIMULATION_SW_TARGET_LAUNCH_TIME_MS = 100
    SIMULATION_TIME_ALLOWED_FOR_HW_TO_DETECT_LANDING_MS = 1000 * HARDWARE_UPDATE_TIMESTEP_MS

    # sim internal state variables -- update DATA_INDEX if you change anything in this list!
    data_column_names = ['time (ms)', 'position x (m)', 'position z (m)', 'position z (noisy) (m)', 'velocity x (m/s)', 
                'velocity z (m/s)', 'acceleration x (m/s2)', 'acceleration z (m/s2)', 'flight state']

    # data saving parameters
    DATA_SAVE_STRIDE_LOG = 10 # stride of data saved to csv file


    @classmethod
    def get_log_filename_format(cls) -> str:
        if cls.USE_HARDWARE_TARGET is True:
            log_filename = 'sim_log_HW_TARGET_'
        else:
            log_filename = 'sim_log_SW_TARGET_'
        return log_filename

    def format_rocket(self) -> str:
        """returns string in csv (comma-separated) format of rocket parameters"""
        return ('rocket parameters\nmass_kg, %.3f\ndrogue cd, %.3f\ndrogue area_m2, %.3f\nmain cd, \
                %.3f\nmain area_m2, %.3f\nlaunch angle_deg, %.3f\nthrust curve filename, %s\n\n' 
                % (self.RKT_MASS_KG, self.RKT_DROGUE_DRAG_COEFF, self.RKT_DROGUE_AREA, self.RKT_MAIN_DRAG_COEFF, 
                   self.RKT_MAIN_AREA, self.RKT_LAUNCH_ANGLE, self.RKT_THRUST_CURVE_FILE))
    
    def format_sim_settings(self) -> str:
        """returns string in csv (comma-separated) format of simulation parameters"""
        return ('simulation settings\ntimestep (ms), %.3f\nhardware update timestep (ms), %.3f\nprint update timestep (ms), \
                %.3f\nuse noisy altitude (boolean), %s\naltitude noise max amplitude (m), \
                %.3f\nuse hardware target (boolean), %s\nsim sw target launch time (ms), %.3f\nstride data save, %d\n\n' 
                % (self.SIMULATION_TIMESTEP_MS, self.HARDWARE_UPDATE_TIMESTEP_MS, self.PRINT_UPDATE_TIMESTEP_MS, 
                   self.USE_NOISY_ALTITUDE, self.ALTITUDE_NOISE_MAX_AMPLITUDE_M, self.USE_HARDWARE_TARGET, 
                   self.SIMULATION_SW_TARGET_LAUNCH_TIME_MS, self.DATA_SAVE_STRIDE_LOG))

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

class DATA_INDEX(Enum):
    # these must be in consecutive ascending order from 0, following data_column_names!
    TIME = 0
    POS_X = 1
    POS_Z = 2
    POS_Z_NOISY = 3
    VEL_X = 4
    VEL_Z = 5
    ACC_X = 6
    ACC_Z = 7
    FLIGHT_STATE = 8

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
        return [self.time, self.rkt_pos_x, self.rkt_pos_z, self.rkt_pos_z_noisy, self.rkt_vel_x, self.rkt_vel_z, self.rkt_acc_x, self.rkt_acc_z, self.rkt_flight_state.name]
        
    def format_all(self) -> str:
        return ('t = %d ms, posX = %.3f m, posZ = %.3f m, velX = %.3f m/s, velZ = %.3f m/s, accX = %.3f m/s2, accZ = %.3f m/s2, rktState = %d' %
                (self.time, self.rkt_pos_x, self.rkt_pos_z, self.rkt_vel_x, self.rkt_vel_z, self.rkt_acc_x, self.rkt_acc_z, self.rkt_flight_state.value))
    
    def format_z(self):
        return ('t = %d ms, posZ = %.3f m, velZ = %.3f m/s, accZ = %.3f m/s2, rktState = %d' % 
                (self.time, self.rkt_pos_z, self.rkt_vel_z, self.rkt_acc_z, self.rkt_flight_state.value))
    
    def format_x(self):
        return ('t = %d ms, posX = %.3f m, velX = %.3f m/s, accX = %.3f m/s2, rktState = %d' %
                (self.time, self.rkt_pos_x, self.rkt_vel_x, self.rkt_acc_x, self.rkt_flight_state.value))
