"""
main.py

uses the other files to run the HIL sim

author: jasper yun
"""

import os
import PySimpleGUI as sg
import serial
import time

# 'local' libraries
import altos_flight_data as aosfd
import gui
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


def setup_sim(values, window):
    """sets up simulation objects from GUI"""
    rkt = gui.get_rocket_options(values, window)
    s = sim.Simulation()
    

def save_sim_results(sim):
    # must call convert_log_to_df() before calling analyze, save, and plot!
    sim.convert_log_to_df(stride=utils.Settings.DATA_SAVE_STRIDE_LOG)
    sim.analyze_sim_results()
    sim.save_sim_log_to_file()
    sim.plot_simulation_results()


def main():
    # change folder to the folder where this file is saved
    abspath = os.path.abspath(__file__)
    dname = os.path.dirname(abspath)
    os.chdir(dname)

    # default options. may be updated through gui
    target_is_hw = utils.Settings.USE_HARDWARE_TARGET
    use_altos_flight_data = utils.Settings.USE_ALTOS_FLIGHT_DATA
    hw_timestep = utils.Settings.HARDWARE_UPDATE_TIMESTEP_MS
    print_timestep = utils.Settings.PRINT_UPDATE_TIMESTEP_MS
    sw_sim_launch_time = utils.Settings.SIMULATION_SW_TARGET_LAUNCH_TIME_MS
    hw_landing_detect_time = utils.Settings.SIMULATION_TIME_ALLOWED_FOR_HW_TO_DETECT_LANDING_MS

    # open gui and wait for start button to be pressed
    if utils.Settings.USE_GUI:
        while True:
            event, values = gui.window.read() 
            gui.update_gui(event, values, gui.window)
            if event == 'button_start_sim':
                # need to setup simulation to start running
                rkt = gui.get_rocket_options(values, gui.window)
                rkt.assign_thrust_curve(utils.Settings.RKT_THRUST_CURVE_FILE)

                s = sim.Simulation()
                s.set_timestep_ms(gui.get_sim_timestep(values, gui.window))
                s.ground_altitude = gui.get_ground_altitude(values, gui.window)
                s.simulate_mach_dip = gui.get_mach_dip_emulation(values, gui.window)
                s.altitude_noise_amplitude = gui.get_altitude_noise_amplitude(values, gui.window)
                send_alt_pressure = gui.get_send_alt_pressure_option(values, gui.window)
                
                sw_sim_launch_time = gui.get_sw_sim_launch_time(values, gui.window)
                hw_timestep = gui.get_hw_timestep(values, gui.window)
                print_timestep = gui.get_print_timestep(values, gui.window)
                hw_landing_detect_time = gui.get_hw_landing_detect_time(values, gui.window)

                target_is_hw = gui.get_sim_target(values, gui.window)
                if target_is_hw:
                    ser = open_COM_port()
                    ser.reset_input_buffer()
                else:
                    ser = None

                use_altos_flight_data = gui.get_altos_fd_option(values, gui.window)

                use_noisy_altitude = gui.get_noisy_altitude_option(values, gui.window)
                hw = hardware_interface.Hardware_Interface(ser, s.sim_flight_state, use_hw_target=target_is_hw, 
                                                           use_noisy_altitude=use_noisy_altitude, 
                                                           send_alt_instead_of_pressure=send_alt_pressure)
                break
            elif event == sg.WIN_CLOSED or event == 'Exit':
                exit()
    
    else: # do not use GUI
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
        # check GUI for events
        event, values = gui.window.read(timeout=1)
        if event == sg.WIN_CLOSED or event == 'Exit':
            save_sim_results(s)
            exit() # kill app
        gui.update_gui(event, values, gui.window)
        if gui.sim_is_running is False:
            break
        
        start_time_ns = time.time_ns()
        
        # select which data source to use: sim engine or altus metrum flight data
        if use_altos_flight_data == True and afd.data_index_last_accessed <= afd.apogee_index:
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
        
        if (s.time % hw_timestep == 0):
            # send data to controller
            # hw.wait_for_poll()
            hw.send(s.datalog[-1])
            hw_flight_state = hw.read_hw_state()
            if target_is_hw == False:
                if s.time > sw_sim_launch_time and s.launched == False:
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
                    s.sim_flight_state = utils.FLIGHT_STATES.LANDED
                    if s.sim_landed == False:
                        s.sim_landed = True
                        s.sim_landed_time = s.time
                    if s.time - s.sim_landed_time > hw_landing_detect_time:
                        print('Target device under test failed to detect landing within allowed simulation time. Terminating simulation.')
                        break
            
            # test simulation behavior with CATO due to early ejection (only testable with software mock FC)
            if utils.Settings.TEST_EARLY_EJECTION_CATO and target_is_hw == False:
                if s.rkt_pos_z > utils.Settings.EJECTION_CATO_ALTITUDE_M:
                    s.sim_flight_state = utils.FLIGHT_STATES.DROGUE_DESCENT
                    # override mock FC's state
                    s.rkt_flight_state = utils.FLIGHT_STATES.DROGUE_DESCENT
                    hw.mock_fc.flight_state = utils.FLIGHT_STATES.DROGUE_DESCENT
                    hw.flight_state = utils.FLIGHT_STATES.DROGUE_DESCENT

        if (s.time % print_timestep == 0):
            # print('t = %d ms\tpos z = %.3f m\tsim state = %d\trkt state = %d' % (s.time, s.rkt_pos_z, s.sim_flight_state.value, s.rkt_flight_state.value))
            print(s.datalog[-1].format_z() + ', simState = ' + str(s.sim_flight_state.value))

        s.log_datapoint()
        s.increment_tick()
        
        # sleep time :)
        # end_time_ns = time.time_ns()
        # sleep_time_ns = s.timestep_ms * 1000000 - (end_time_ns - start_time_ns)
        # if utils.Settings.USE_HARDWARE_TARGET:
            # if sleep_time_ns > 0:
            #     time.sleep(sleep_time_ns / 100000000000) # sleep argument is in seconds
    
        # for debugging only
        # if s.time > 25000:
        #     break

    if target_is_hw == True:
        hw.com_port.close()
    
    # final datapoint after exiting simulation
    s.log_datapoint()
    save_sim_results(s)
    

if __name__ == '__main__':
    while True:
        main()
    