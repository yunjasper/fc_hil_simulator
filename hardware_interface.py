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

import ambiance # barometric pressure from international standard atmosphere model
import flight_computer_mock as fc
import serial
import struct
import utils

def altitude2pressure_hPa(altitude_m):
        """returns barometric pressure (hPa) corresponding to the given altitude (m)"""
        return ambiance.Atmosphere(altitude_m).pressure[0] / 100

class Hardware_Interface:

    CONTROL_RESPONSE_PACKET_LENGTH = 7 # 'C,x,E\r\n'
    LAUNCH_COMMAND_PACKET = 'launch\n' # hw should send this to the sim to indicate launched

    def __init__(self, com_port : serial.Serial, flight_state : utils.FLIGHT_STATES, use_hw_target) -> None:
        self.com_port = com_port
        self.flight_state = flight_state
        self.use_target_hw = use_hw_target
        self.mock_fc = None

        if use_hw_target == False:
            self.mock_fc = fc.Flight_Computer()

    def send(self, data : utils.Sim_DataPoint):

        


        if utils.Settings.USE_NOISY_ALTITUDE:
            altitude = data.rkt_pos_z_noisy
        else:
            altitude = data.rkt_pos_z
        pressure = altitude2pressure_hPa(altitude)
        
        if utils.Settings.SEND_ALTITUDE_INSTEAD_OF_PRESSURE == True:
            baro_data = altitude
        else:
            baro_data = pressure
        
        if utils.Settings.SIMULATE_TRANSONIC_MACH_DIP == True:
            

        # send data as string
        # data_packet = 'S,%.8f,0.000,%.8f,%.8f,E\r\n' % (data.rkt_acc_x, data.rkt_acc_z, altitude)
        # send data as struct (more efficient use of bytes on the line)
        data_packet = struct.pack('LLfffL', utils.Settings.PACKET_HEADER, int(data.time), data.rkt_acc_x, data.rkt_acc_z, baro_data, utils.Settings.PACKET_TRAILER)
        if self.use_target_hw:
            self.com_port.write(data_packet)
        else:
            self.mock_fc.update_sim_step(data_packet)
            
    def parse_control_response(self, response):
        # verify that packet follows expected structure
        flight_state = utils.FLIGHT_STATES.ERROR
        if len(response) != Hardware_Interface.CONTROL_RESPONSE_PACKET_LENGTH:
                print('Error parsing control response: wrong length -- message = ' + response)
        elif response == Hardware_Interface.LAUNCH_COMMAND_PACKET:
             flight_state = utils.FLIGHT_STATES.LAUNCH_COMMAND_START_SIM
        elif response[0] != 'C' or response[-3:] != 'E\r\n':
            print('Error parsing control response: start or end characters are wrong')
        else:
            flight_state = utils.FLIGHT_STATES(int(response[2]))
        
        return flight_state
    
    def read_hw_state(self) -> utils.FLIGHT_STATES:
        if self.use_target_hw:
            # wait until data is available. assumption: computer is *much* faster than target
            while self.com_port.in_waiting == 0:
                pass
            while self.com_port.in_waiting >= Hardware_Interface.CONTROL_RESPONSE_PACKET_LENGTH:
                control_response = self.com_port.read(Hardware_Interface.CONTROL_RESPONSE_PACKET_LENGTH).decode('utf-8')
                print('Control response: ' + control_response)
                flight_state = self.parse_control_response(control_response)
                if flight_state == utils.FLIGHT_STATES.LAUNCH_COMMAND_START_SIM: 
                    return flight_state
                elif flight_state == utils.FLIGHT_STATES.ERROR:
                     # clear the serial buffer
                    self.com_port.reset_input_buffer()
                
                self.flight_state = flight_state
            return self.flight_state
            
        else:
            control_response = self.mock_fc.get_control_response()
            flight_state = self.parse_control_response(control_response)
            self.flight_state = flight_state
            return flight_state

