"""
flight_computer_mock.py

runs the flight computer ejection algorithm in Python while debugging the simulator.

author: jasper yun
"""

import ambiance
import numpy as np
from scipy import stats # for linear regression
import struct
import utils


class Flight_Computer:
    def __init__(self, send_altitude_instead_of_pressure=utils.Settings.SEND_ALTITUDE_INSTEAD_OF_PRESSURE) -> None:
        self.accX = 0
        self.accY = 0
        self.accZ = 0
        self.altitude = 0
        self.flight_state = utils.FLIGHT_STATES.PAD # TODO: fix sim setup so that we can start from PAD
        self.ground_altitude = utils.Settings.GROUND_ALTITUDE_M # cheating but whatever
        self.send_altitude_instead_of_pressure = send_altitude_instead_of_pressure

        # variables for the ejection algorithm
        self.EJ_MAIN_DEPLOYMENT = 1500 * 0.3048 # ft to m conversion
        self.EJ_LANDING_SAMPLES = 50
        self.EJ_LANDING_THRESHOLD = 3
        self.EJ_LAUNCH_THRESHOLD = 50 
        self.EJ_NUM_MEAS_REG = 50
        self.accZ_epsilon = 0.2

        # ring buffer
        self.alt_previous = self.EJ_NUM_MEAS_REG * [utils.Settings.GROUND_ALTITUDE_M]
        self.alt_previous_idx = 0

        self.update_iterations = 0 
        self.landing_samples_count = 0


    def parse_telemetry(self, telemetry):
        # expects telemetry in form of 'S,AccX,AccY,AccZ,Altitude,E\r\n'
        # if telemetry[0] != 'S' or telemetry[-3:] != 'E\r\n':
        #     print('Error: FC Mock - could not parse telemetry')
        # else:
        #     # try to split the string
        #     splitted = telemetry[2:-3].split(',')
        #     self.accX = float(splitted[0])
        #     self.accY = float(splitted[1])
        #     self.accZ = float(splitted[2])
        #     self.altitude = float(splitted[3])

        #     self.update_altitude_array(self.altitude)

        # for struct format:
        header, time, accX, accZ, altitude, trailer = struct.unpack('LffffL', telemetry)
        if header != utils.Settings.PACKET_HEADER or trailer != utils.Settings.PACKET_TRAILER:
            print('Error: FC Mock - could not parse telemetry')
        else:
            if self.send_altitude_instead_of_pressure == False:
                # 'altitude' contains pressure (hPa) data, convert to altitude (m)
                altitude = ambiance.Atmosphere.from_pressure(altitude * 100).h[0]
            
            self.accX = accX
            self.accY = 0
            self.accZ = accZ
            self.altitude = altitude
            self.update_altitude_array(self.altitude)
    
    def get_control_response(self):
        response = 'C,%d,E\r\n' % (self.flight_state.value)
        return response
    
    def pressure2altitude(self, pressure):
        return 145442.1609 * (1.0 - pow(pressure/utils.Settings.LOCAL_PRESSURE_hPa, 0.190266436))
    
    def update_altitude_array(self, altitude):
        # altitude = self.pressure2altitude(altitude)
        self.alt_previous[self.alt_previous_idx] = altitude
        self.alt_previous_idx = (self.alt_previous_idx + 1) % self.EJ_NUM_MEAS_REG

    def update_flight_status(self):
        # your ejection algo here
        
        if self.flight_state == utils.FLIGHT_STATES.PAD:
            # detect launch using altitude change
            current_altitude = self.alt_previous[(self.alt_previous_idx - 1) % self.EJ_NUM_MEAS_REG]
            if current_altitude - self.ground_altitude > self.EJ_LAUNCH_THRESHOLD:
                self.flight_state = utils.FLIGHT_STATES.BOOST
        
        elif self.flight_state == utils.FLIGHT_STATES.BOOST:
            # detect apogee -- since this is a mock, I can cheat on the algorithm by using python functions
            x_values = np.arange(self.EJ_NUM_MEAS_REG)
            # convert circular buffer to linear buffer
            y_values = []
            for i in range(self.EJ_NUM_MEAS_REG):
                y_values.append(self.alt_previous[(self.alt_previous_idx + i) % self.EJ_NUM_MEAS_REG])
            result = stats.linregress(x_values, y_values)
            if result.slope < -0:
                # check acceleration
                if  self.accZ > -1 * utils.Settings.GRAVITY - self.accZ_epsilon and self.accZ < -1 * utils.Settings.GRAVITY + self.accZ_epsilon:
                    self.flight_state = utils.FLIGHT_STATES.DROGUE_DESCENT
        
        elif self.flight_state == utils.FLIGHT_STATES.DROGUE_DESCENT:
            current_altitude = self.alt_previous[(self.alt_previous_idx - 1) % self.EJ_NUM_MEAS_REG]
            if current_altitude < self.EJ_MAIN_DEPLOYMENT:
                self.flight_state = utils.FLIGHT_STATES.MAIN_DESCENT

        elif self.flight_state == utils.FLIGHT_STATES.MAIN_DESCENT:
            min_alt = np.min(self.alt_previous)
            max_alt = np.max(self.alt_previous)
            if max_alt - min_alt < self.EJ_LANDING_THRESHOLD and np.abs(self.accZ) < self.accZ_epsilon and np.abs(self.altitude) <= utils.Settings.GROUND_ALTITUDE_M:
                self.landing_samples_count += 1
            if self.landing_samples_count > self.EJ_LANDING_SAMPLES:
                self.flight_state = utils.FLIGHT_STATES.LANDED

    def update_sim_step(self, telemetry):
        self.parse_telemetry(telemetry)
        self.update_flight_status()
        self.update_iterations += 1

