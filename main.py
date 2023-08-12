"""
main.py

uses the other files to run the HIL sim

author: jasper yun
"""

import os
import serial
import time

# 'local' libraries
import altos_flight_data as aosfd
import hardware_interface
import rocket
import sim
import utils


# for now, hardcode the port. todo: make flexible 
SERIAL_PORT = 'COM7'
BAUD_RATE = 1152000
SERIAL_PORT_TIMEOUT = 5 # seconds

# todo: move this (and the SERIAL_PORT config settings) into hardware_interface
def open_COM_port() -> serial.Serial:
    try: 
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=SERIAL_PORT_TIMEOUT)
    except serial.SerialException:
        print('Cannot open requested port. Quitting program.')
        ser = None
    return ser


def main():
    # change folder to the folder where this file is saved
    abspath = os.path.abspath(__file__)
    dname = os.path.dirname(abspath)
    os.chdir(dname)

    # simulation setup
    rkt = rocket.Rocket(
        utils.Settings.RKT_MASS_KG,
        utils.Settings.RKT_DROGUE_DRAG_COEFF,
        utils.Settings.RKT_DROGUE_AREA,
        utils.Settings.RKT_MAIN_DRAG_COEFF,
        utils.Settings.RKT_MAIN_AREA,
        utils.Settings.RKT_LAUNCH_ANGLE,
    )
    rkt.assign_thrust_curve(utils.Settings.RKT_THRUST_CURVE_FILE)

    s = sim.Simulation()
    s.set_timestep_ms(utils.Settings.SIMULATION_TIMESTEP_MS)

    if utils.Settings.USE_HARDWARE_TARGET == True:
        ser = open_COM_port()
        ser.reset_input_buffer()
    else:
        ser = None

    hw = hardware_interface.Hardware_Interface(ser, s.sim_flight_state, use_hw_target=utils.Settings.USE_HARDWARE_TARGET)

    if utils.Settings.USE_ALTOS_FLIGHT_DATA == True:
        afd = aosfd.AltOS_Flight_Data(filename=utils.Settings.ALTOS_FLIGHT_DATA_FILENAME, 
                                      requested_column_names=utils.Settings.ALTOS_FLIGHT_DATA_TYPES)


    # start simulation
    # start simulation
    # log initial conditions as a datapoint
    s.log_datapoint()
    while s.rkt_flight_state != utils.FLIGHT_STATES.LANDED:
        start_time_ns = time.time_ns()
        
        # select which data source to use: sim engine or altus metrum flight data
        if utils.Settings.USE_ALTOS_FLIGHT_DATA == True and afd.data_index_last_accessed <= afd.apogee_index:
            if s.launched == False:
                s.launch_time = s.time
            data = afd.get_datapoint(s.time - s.launch_time)
            # TODO: determine how to automatically update simulation variables
            s.rkt_acc_x = data.get('accel_x')
            s.rkt_acc_z = data.get('accel_z')
            s.rkt_pos_z = data.get('altitude')
            s.update_kinematics(rkt, s.rkt_acc_x * rkt.dry_mass_kg, s.rkt_acc_z * rkt.dry_mass_kg)
        else:
            s.simulate_tick(rkt)
        
        if (s.time % utils.Settings.HARDWARE_UPDATE_TIMESTEP_MS == 0):
            # send data to controller
            # hw.wait_for_poll()
            hw.send(s.datalog[-1])
            hw_flight_state = hw.read_hw_state()
            if utils.Settings.USE_HARDWARE_TARGET == False:
                if s.time > utils.Settings.SIMULATION_SW_TARGET_LAUNCH_TIME_MS and s.launched == False:
                    s.sim_flight_state = utils.FLIGHT_STATES.BOOST
                    s.launch_time = s.time
                    s.launched = True
            elif hw_flight_state == utils.FLIGHT_STATES.LAUNCH_COMMAND_START_SIM and s.launched == False:
                s.sim_flight_state = utils.FLIGHT_STATES.BOOST
                s.launch_time = s.time
                s.launched = True
            
            s.rkt_flight_state = hw_flight_state
            if (hw_flight_state == utils.FLIGHT_STATES.DROGUE_DESCENT 
                    or hw_flight_state == utils.FLIGHT_STATES.MAIN_DESCENT):
                # simulation in these states depends on the FC's control, so must match sim state to rocket state
                if s.rkt_acc_z != 0 and s.rkt_vel_z != 0: # simulated rocket is still moving
                    s.sim_flight_state = hw_flight_state
                else:
                    # simulated rocket has landed, start countdown window for hardware to detect landing
                    s.s_flight_state = utils.FLIGHT_STATES.LANDED
                    if s.sim_landed == False:
                        s.sim_landed = True
                        s.sim_landed_time = s.time
                    if s.time - s.sim_landed_time > utils.Settings.SIMULATION_TIME_ALLOWED_FOR_HW_TO_DETECT_LANDING_MS:
                        print('Target device under test failed to detect landing within allowed simulation time. Terminating simulation.')
                        break
            
            # test simulation behavior with CATO due to early ejection (only testable with software mock FC)
            if utils.Settings.TEST_EARLY_EJECTION_CATO and utils.Settings.USE_HARDWARE_TARGET == False:
                if s.rkt_pos_z > utils.Settings.EJECTION_CATO_ALTITUDE_M:
                    s.sim_flight_state = utils.FLIGHT_STATES.DROGUE_DESCENT
                    # override mock FC's state
                    s.rkt_flight_state = utils.FLIGHT_STATES.DROGUE_DESCENT
                    hw.mock_fc.flight_state = utils.FLIGHT_STATES.DROGUE_DESCENT
                    hw.flight_state = utils.FLIGHT_STATES.DROGUE_DESCENT

        if (s.time % utils.Settings.PRINT_UPDATE_TIMESTEP_MS == 0):
            # print('t = %d ms\tpos z = %.3f m\tsim state = %d\trkt state = %d' % (s.time, s.rkt_pos_z, s.sim_flight_state.value, s.rkt_flight_state.value))
            print(s.datalog[-1].format_z() + ', simState = ' + str(s.sim_flight_state.value))

        s.log_datapoint()
        s.increment_tick()
        
        # sleep time :)
        end_time_ns = time.time_ns()
        sleep_time_ns = s.timestep_ms * 1000000 - (end_time_ns - start_time_ns)
        # if utils.Settings.USE_HARDWARE_TARGET:
            # if sleep_time_ns > 0:
            #     time.sleep(sleep_time_ns / 100000000000) # sleep argument is in seconds
    
        # for debugging only
        # if s.time > 25000:
        #     break

    if utils.Settings.USE_HARDWARE_TARGET == True:
        hw.com_port.close()
    
    # final datapoint after exiting simulation
    s.log_datapoint()
    
    # must call convert_log_to_df() before calling analyze, save, and plot!
    s.convert_log_to_df(stride=utils.Settings.DATA_SAVE_STRIDE_LOG)
    s.analyze_sim_results()
    s.save_sim_log_to_file()
    s.plot_simulation_results()

if __name__ == '__main__':
    main()