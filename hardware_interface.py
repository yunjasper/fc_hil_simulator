"""
hardware_interface.py

this code glues the sim and the hardware controller using serial communication.
note: this code accepts a handle to a serial.Serial object for the COM port. 
the opening and closing of the COM port should be handled elsewhere

the hardware controller (FC) should print its telemetry over the serial port
so that the sim engine can read its state. the telemetry must include a field
that indicates the flight state.

author: jasper yun
"""

import flight_computer_mock as fc
import serial
import utils

def altitude2pressure(altitude_m):
        # pressure to altitude (ft): 145442.1609 * (1.0 - pow(pressure/LOCAL_PRESSURE, 0.190266436))
        altitude_ft = altitude_m * 3.280839895
        return utils.Settings.LOCAL_PRESSURE_hPa * pow(1 - altitude_ft / 145442.1609, 0.190266436)

class Hardware_Interface:

    CONTROL_RESPONSE_PACKET_LENGTH = 7 # 'C,x,E\r\n'

    def __init__(self, com_port : serial.Serial, flight_state : utils.FLIGHT_STATES, use_hw_target) -> None:
        self.com_port = com_port
        self.flight_state = flight_state
        self.use_target_hw = use_hw_target
        self.mock_fc = None

        if use_hw_target == False:
            self.mock_fc = fc.Flight_Computer()

    def send(self, data : utils.Sim_DataPoint):
        # convert altitude (position in z axis) to barometric pressure
        altitude = data.rkt_pos_z
        pressure = altitude2pressure(altitude)
        # print data into string: S,AccX,0,AccZ,Altitude,E\r\n
        data_string = 'S,%.8f,0.000,%.8f,%.8f,E\r\n' % (data.rkt_acc_x, data.rkt_acc_z, altitude)
        if self.use_target_hw:
            self.com_port.write(data_string)
        else:
            self.mock_fc.update_sim_step(data_string)
            
    def parse_control_response(self, response):
        # verify that packet follows expected structure
        flight_state = utils.FLIGHT_STATES.ERROR
        if len(response) != Hardware_Interface.CONTROL_RESPONSE_PACKET_LENGTH:
                print('Error parsing control response: wrong length')
        elif response[0] != 'C' or response[-3:] != 'E\r\n':
            print('Error parsing contorl response: start or end characters are wrong')
        else:
            flight_state = utils.FLIGHT_STATES(int(response[2]))
        
        return flight_state
    
    def read_hw_state(self) -> utils.FLIGHT_STATES:
        if self.use_target_hw:
            # check for available data
            in_bytes = self.com_port.in_waiting()
            if in_bytes != 0:
                control_response = self.com_port.read(in_bytes)
                flight_state = self.parse_control_response(control_response)
                self.flight_state = flight_state
                return flight_state
            else:
                return self.flight_state # no change in state
        else:

            control_response = self.mock_fc.get_control_response()
            flight_state = self.parse_control_response(control_response)
            self.flight_state = flight_state
            return flight_state

