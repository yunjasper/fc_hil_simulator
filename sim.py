"""
sim.py

calculates the trajectory of the rocket.

author: jasper yun
"""

# "big" libraries
import ambiance # atmospheric model
from matplotlib import pyplot as plt
import numpy as np
import os
import pandas as pd
import serial
import time

# "local" project libraries
import altos_flight_data as aosfd
import hardware_interface
import rocket
import utils

# to simulate the flight, let's do 2D (x and z) for now.
# assume that the rocket is a point particle
# thrust (function of time) operates directly on the point
# drag components impact trajectory through acceleration in x and z, potentially different drag coefficients
# no wind to start

# for now, hardcode the port. todo: make flexible 
SERIAL_PORT = 'COM7'
BAUD_RATE = 115200
SERIAL_PORT_TIMEOUT = 5 # seconds

class Simulation:

    data_column_names = ['time (ms)', 'position x (m)', 'position z (m)', 'position z (noisy) (m)', 'velocity x (m/s)', 
                'velocity z (m/s)', 'acceleration x (m/s2)', 'acceleration z (m/s2)', 'flight state']

    def __init__(self) -> None:
        self.time = 0
        self.launch_time = 0
        self.launched = False
        self.sim_landed = False # sim may land before hardware detects it
        self.sim_landed_time = 0
        self.sim_flight_state = utils.FLIGHT_STATES.PAD
        
        # rocket states
        self.rkt_pos_x = 0
        self.rkt_pos_z = utils.Settings.GROUND_ALTITUDE_M
        self.rkt_pos_z_noisy = utils.Settings.GROUND_ALTITUDE_M
        self.rkt_vel_x = 0
        self.rkt_vel_z = 0
        self.rkt_acc_x = 0
        self.rkt_acc_z = 0
        self.rkt_flight_state = utils.FLIGHT_STATES.PAD

        self.datalog = [] # list of utils.Sim_DataPoint objects
        self.datatable = pd.DataFrame(columns=self.data_column_names) # empty, to be filled in after sim ends
        
        base_filename = utils.Settings.SIMULATION_LOG_FILENAME_FORMAT + time.strftime('%Y-%m-%d_%H-%M-%S')
        self.session_folder = base_filename # folder name where outputs will be saved
        os.mkdir(self.session_folder)
        
    def set_timestep_ms(self, timestep_ms):
        self.timestep_ms = timestep_ms
    
    def increment_tick(self):
        self.time += self.timestep_ms

    def simulate_tick(self, rkt : rocket):

        def get_thrust(rkt : rocket, time_ms):
            if (self.time - self.launch_time) / 1000 < rkt.tc_burn_time:
                thrust = rkt.get_thrust_N((self.time - self.launch_time) / 1000)
            else:
                thrust = 0
            return thrust

        # variables for forces
        force_x = 0
        force_z = 0

        # calculate forces in x and z directions based on the flight phase
        rocket_weight = utils.Settings.GRAVITY * (rkt.dry_mass_kg + rkt.get_engine_propellant_mass_kg((self.time - self.launch_time) / 1000))

        if self.sim_flight_state == utils.FLIGHT_STATES.PAD:
            # rocket is stationary on the launch rail, all forces are balanced --> zero acceleration
            force_x = 0
            force_z = 0
        
        elif self.sim_flight_state == utils.FLIGHT_STATES.BOOST:
            thrust = get_thrust(rkt, self.time)
            # calculate forces
            la_rad = rkt.launch_angle_deg * np.pi / 180
            force_x = np.cos(la_rad) * thrust
            force_z = np.sin(la_rad) * thrust - rocket_weight
        
        elif self.sim_flight_state == utils.FLIGHT_STATES.COAST:
            # in off-nominal cases, motor may still be burning while hardware detected a different flight state
            thrust = get_thrust(rkt, self.time)
            force_x = 0
            force_z = thrust - rocket_weight

        elif self.sim_flight_state == utils.FLIGHT_STATES.DROGUE_DESCENT:
            force_x = 0
            # in off-nominal cases, motor may still be burning while hardware detected a different flight state
            thrust = get_thrust(rkt, self.time)
            # account for parachute drag, assume only in z direction for now. parachute drag force acts opposite to direction of motion
            air_mass_density = ambiance.Atmosphere(self.rkt_pos_z).density[0]
            force_drogue = rkt.drogue_drag_coeff * air_mass_density * (self.rkt_vel_z ** 2) * rkt.drogue_area_m2 / 2
            force_z = thrust + (-1 * np.sign(self.rkt_vel_z) * force_drogue) - rocket_weight
        
        elif self.sim_flight_state == utils.FLIGHT_STATES.MAIN_DESCENT:
            force_x = 0
            # in off-nominal cases, motor may still be burning while hardware detected a different flight state
            thrust = get_thrust(rkt, self.time)
            # account for parachute drag, assume only in z direction for now. parachute drag force acts opposite to direction of motion
            air_mass_density = ambiance.Atmosphere(self.rkt_pos_z).density[0]
            force_main = rkt.main_drag_coeff * air_mass_density * (self.rkt_vel_z ** 2) * rkt.main_area_m2 / 2
            force_z = thrust + (-1 * np.sign(self.rkt_vel_z) * force_main) - rocket_weight
        
            # rocket hits the ground
            if self.rkt_pos_z <= utils.Settings.GROUND_ALTITUDE_M:
                force_z = 0
        
        elif self.sim_flight_state == utils.FLIGHT_STATES.LANDED:
            # nothing to do
            force_x = 0
            force_z = 0
        
        self.update_kinematics(rkt, force_x, force_z)

    def update_kinematics(self, rocket : rocket.Rocket, force_x, force_z):
        if force_z <= 0 and self.rkt_pos_z <= utils.Settings.GROUND_ALTITUDE_M:
            # cannot go below ground so rocket is not moving
            self.rkt_acc_z = 0
            self.rkt_acc_x = 0
            self.rkt_vel_z = 0
            self.rkt_vel_x = 0
        else:
            # calculate resultant accelerations (F = m*a)
            self.rkt_acc_x = force_x / rocket.dry_mass_kg
            self.rkt_acc_z = force_z / rocket.dry_mass_kg
        
        # integrate to get velocities and positions
        self.rkt_vel_x += self.rkt_acc_x * self.timestep_ms / 1000
        self.rkt_vel_z += self.rkt_acc_z * self.timestep_ms / 1000
        self.rkt_pos_x += self.rkt_vel_x * self.timestep_ms / 1000
        self.rkt_pos_z += self.rkt_vel_z * self.timestep_ms / 1000
        self.rkt_pos_z_noisy = self.rkt_pos_z + (np.random.rand(1, 1)[0][0] - 0.5) * utils.Settings.ALTITUDE_NOISE_MAX_AMPLITUDE_M

    def log_datapoint(self):
        dp = utils.Sim_DataPoint(self.time, self.rkt_pos_x, self.rkt_pos_z, self.rkt_pos_z_noisy, 
                                 self.rkt_vel_x, self.rkt_vel_z, self.rkt_acc_x, self.rkt_acc_z, self.rkt_flight_state)
        self.datalog.append(dp)

    def convert_log_to_df(self, stride):
        print('Converting simulation session data...')
        conversion_progress_percent = 0

        datatable = pd.DataFrame(columns=self.data_column_names) # empty, to be filled in
        indices = np.arange(len(self.datalog), step=stride)
        for i in range(len(indices)):
            idx = indices[i]
            point = self.datalog[idx]
            datatable.loc[len(datatable)] = point.unwrap()
            
            # print conversion progress indicator
            if int(i / len(indices) * 100) >= conversion_progress_percent + 10:
                conversion_progress_percent += 10
                print('{}% done...'.format(conversion_progress_percent))
        
        print('100% done\n') # newline
        self.datatable = datatable
    
    def save_sim_log_to_file(self):
        settings = utils.Settings() # need to create the object to use the method
        os.chdir(self.session_folder)
        filename =  self.session_folder + '_SETTINGS.csv'
        f = open(filename, 'w')
        f.write(settings.format_rocket())
        f.write(settings.format_sim_settings())
        f.close()
        filename = self.session_folder + '_DATA.csv'
        f = open(filename, 'w')
        self.convert_log_to_df(stride=utils.Settings.DATA_SAVE_STRIDE_LOG)
        self.datatable.to_csv(filename, sep=',', columns=self.data_column_names, mode='a', index=False)
        f.close()
        os.chdir(os.pardir)

    def get_state_changes(self):
        # find state change time indices
        states = self.datatable['flight state']
        state_change_indices = []
        previous_state = utils.FLIGHT_STATES.PAD.name
        for i in range(len(states)):
            if states[i] != previous_state:
                state_change_indices.append(i)
                previous_state = states[i]
        if self.datalog[-1].rkt_flight_state == utils.FLIGHT_STATES.LANDED:
            # sim ends immediately upon landing detection, so landing detection must
            # be the last data point in the table
            state_change_indices.append(self.datatable.shape[0] - 1)

        return state_change_indices

    def plot_simulation_results(self):
        self.convert_log_to_df(stride=utils.Settings.DATA_SAVE_STRIDE_PLOT)
        state_change_indices = self.get_state_changes()
        
        plt.rc('lines', linewidth=2)
        plt.rc('axes', grid=True)
        plt.rc('grid', linestyle='--')

        figX, axX = plt.subplots(3, 1)
        time_values_s = self.datatable['time (ms)'] / 1000
        axX[0].plot(time_values_s, self.datatable['position x (m)'])
        for idx in state_change_indices:
            axX[0].axvline(time_values_s[idx], color='red')
        # if self.datalog[-1].rkt_flight_state == utils.FLIGHT_STATES.LANDED:
        #     axX[0].axvline(time_values_s[len(time_values_s) - 1], color='red')
        axX[0].set_ylabel('position x (m)')
        axX[1].plot(time_values_s, self.datatable['velocity x (m/s)'])
        axX[1].set_ylabel('velocity x (m/s)')
        axX[2].plot(time_values_s, self.datatable['acceleration x (m/s2)'])
        axX[2].set_ylabel('acceleration x (m/s2)')
        axX[2].set_xlabel('time (s)')

        figZ, axZ = plt.subplots(3, 1)
        axZ[0].plot(time_values_s, self.datatable['position z (noisy) (m)'])
        axZ[0].plot(time_values_s, self.datatable['position z (m)'])
        axZ[0].legend(['noisy', 'true'])
        for idx in state_change_indices:
            axZ[0].axvline(time_values_s[idx], color='red')
        # if self.datalog[-1].rkt_flight_state == utils.FLIGHT_STATES.LANDED:
        #     axZ[0].axvline(time_values_s[len(time_values_s) - 1], color='red')
        axZ[0].set_ylabel('position z (m)')
        axZ[1].plot(time_values_s, self.datatable['velocity z (m/s)'])
        axZ[1].set_ylabel('velocity z (m/s)')
        axZ[2].plot(time_values_s, self.datatable['acceleration z (m/s2)'])
        axZ[2].set_ylabel('acceleration z (m/s2)')
        axZ[2].set_xlabel('time (s)')

        # save plots
        if os.path.basename(os.getcwd()) != self.session_folder:
            os.chdir(self.session_folder)
        figX.savefig(self.session_folder + '_x_axis_plots.png', dpi=300)
        figZ.savefig(self.session_folder + '_z_axis_plots.png', dpi=300)
        os.chdir(os.pardir)
        plt.show()

    def analyze_sim_results(self):
        # want to know:
        #   - altitude of all state changes
        #   - time + altitude at ejection vs true apogee (shows lead/lag of algorithm)
        if self.datatable.empty:
            self.convert_log_to_df(stride=utils.Settings.DATA_SAVE_STRIDE_LOG)
        state_change_indices = self.get_state_changes()
        # get true apogee
        apogee_index = np.argmax(self.datatable['position z (m)'], axis=0)
        altitude_apogee = self.datatable['position z (m)'][apogee_index]
        time_apogee = self.datatable['time (ms)'][apogee_index]

        # get ejection stats
        ej_idx = 0
        for idx in state_change_indices:
            if self.datatable['flight state'][idx] == utils.FLIGHT_STATES.DROGUE_DESCENT.name:
                ej_idx = idx
                break
        altitude_ej = self.datatable['position z (m)'][ej_idx]
        time_ej = self.datatable['time (ms)'][ej_idx]
        
        os.chdir(self.session_folder)
        with open(self.session_folder + '_analysis.txt', 'w') as f:    
            f.write('Analysis of session:\n')
            print('Analysis of session:')
            for i in range(len(state_change_indices)):
                idx = state_change_indices[i]
                data_str = self.datalog[idx].format_all()
                print(f'State change {i}: ' + data_str)
                f.write(f'State change {i}: ' + data_str + '\n')
            f.write('\n')

            # print stats
            apogee_str = 'True apogee:\ttime (ms) = %.3f\taltitude (m) = %.3f' % (time_apogee, altitude_apogee)
            ej_str =     'Ejection:   \ttime (ms) = %.3f\taltitude (m) = %.3f' % (time_ej, altitude_ej)
            diff_str =   'Difference: \ttime (ms) = %.3f\taltitude (m) = %.3f' % (time_apogee - time_ej, altitude_apogee - altitude_ej)
            if time_apogee > time_ej:
                info_str = 'PREMATURE EJECTION by %.3f seconds' % ((time_apogee - time_ej) / 1000)
            elif time_apogee == time_ej:
                info_str = 'Ejection exactly on time, congratulations!'
            else:
                info_str = 'LATE EJECTION by %.3f seconds' % ((time_ej - time_apogee) / 1000)
            
            print(apogee_str)
            print(ej_str)
            print(diff_str)
            print(info_str)
            f.write(apogee_str + '\n')
            f.write(ej_str + '\n')
            f.write(diff_str + '\n')
            f.write(info_str + '\n')

            os.chdir(os.pardir)



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

    rkt = rocket.Rocket(
        utils.Settings.RKT_MASS_KG,
        utils.Settings.RKT_DROGUE_DRAG_COEFF,
        utils.Settings.RKT_DROGUE_AREA,
        utils.Settings.RKT_MAIN_DRAG_COEFF,
        utils.Settings.RKT_MAIN_AREA,
        utils.Settings.RKT_LAUNCH_ANGLE,
    )
    rkt.assign_thrust_curve(utils.Settings.RKT_THRUST_CURVE_FILE)

    sim = Simulation()
    sim.set_timestep_ms(utils.Settings.SIMULATION_TIMESTEP_MS)

    if utils.Settings.USE_HARDWARE_TARGET == True:
        ser = open_COM_port()
        ser.reset_input_buffer()
    else:
        ser = None

    hw = hardware_interface.Hardware_Interface(ser, sim.sim_flight_state, use_hw_target=utils.Settings.USE_HARDWARE_TARGET)

    if utils.Settings.USE_ALTOS_FLIGHT_DATA == True:
        afd = aosfd.AltOS_Flight_Data(filename=utils.Settings.ALTOS_FLIGHT_DATA_FILENAME, 
                                      requested_column_names=utils.Settings.ALTOS_FLIGHT_DATA_TYPES)

    # start simulation
    # log initial conditions as a datapoint
    sim.log_datapoint()
    while sim.rkt_flight_state != utils.FLIGHT_STATES.LANDED:
        start_time_ns = time.time_ns()
        
        # select which data source to use: sim engine or altus metrum flight data
        if utils.Settings.USE_ALTOS_FLIGHT_DATA == True and afd.data_index_last_accessed <= afd.apogee_index:
            data = afd.get_datapoint(sim.time - sim.launch_time)
            # TODO: determine how to automatically update simulation variables
            sim.rkt_acc_x = data.get('accel_x')
            sim.rkt_acc_z = data.get('accel_z')
            sim.rkt_pos_z = data.get('altitude')
            sim.update_kinematics(rkt, sim.rkt_acc_x * rkt.dry_mass_kg, sim.rkt_acc_z * rkt.dry_mass_kg)
        else:
            sim.simulate_tick(rkt)
        
        if (sim.time % utils.Settings.HARDWARE_UPDATE_TIMESTEP_MS == 0):
            # send data to controller
            hw.send(sim.datalog[-1])
            hw_flight_state = hw.read_hw_state()
            if utils.Settings.USE_HARDWARE_TARGET == False:
                if sim.time > utils.Settings.SIMULATION_SW_TARGET_LAUNCH_TIME_MS and sim.launched == False:
                    sim.sim_flight_state = utils.FLIGHT_STATES.BOOST
                    sim.launch_time = sim.time
                    sim.launched = True
            elif hw_flight_state == utils.FLIGHT_STATES.LAUNCH_COMMAND_START_SIM and sim.launched == False:
                sim.sim_flight_state = utils.FLIGHT_STATES.BOOST
                sim.launch_time = sim.time
                sim.launched = True
            
            sim.rkt_flight_state = hw_flight_state
            if (hw_flight_state == utils.FLIGHT_STATES.DROGUE_DESCENT 
                    or hw_flight_state == utils.FLIGHT_STATES.MAIN_DESCENT):
                # simulation in these states depends on the FC's control, so must match sim state to rocket state
                if sim.rkt_acc_z != 0 and sim.rkt_vel_z != 0: # simulated rocket is still moving
                    sim.sim_flight_state = hw_flight_state
                else:
                    # simulated rocket has landed, start countdown window for hardware to detect landing
                    sim.sim_flight_state = utils.FLIGHT_STATES.LANDED
                    if sim.sim_landed == False:
                        sim.sim_landed = True
                        sim.sim_landed_time = sim.time
                    if sim.time - sim.sim_landed_time > utils.Settings.SIMULATION_TIME_ALLOWED_FOR_HW_TO_DETECT_LANDING_MS:
                        print('Hardware failed to detect landing within allowed simulation. Terminating simulation.')
                        break
            
            # test simulation behavior with CATO due to early ejection (only testable with software mock FC)
            if utils.Settings.TEST_EARLY_EJECTION_CATO and utils.Settings.USE_HARDWARE_TARGET == False:
                if sim.rkt_pos_z > utils.Settings.EJECTION_CATO_ALTITUDE_M:
                    sim.sim_flight_state = utils.FLIGHT_STATES.DROGUE_DESCENT
                    # override mock FC's state
                    sim.rkt_flight_state = utils.FLIGHT_STATES.DROGUE_DESCENT
                    hw.mock_fc.flight_state = utils.FLIGHT_STATES.DROGUE_DESCENT
                    hw.flight_state = utils.FLIGHT_STATES.DROGUE_DESCENT

        if (sim.time % utils.Settings.PRINT_UPDATE_TIMESTEP_MS == 0):
            # print('t = %d ms\tpos z = %.3f m\tsim state = %d\trkt state = %d' % (sim.time, sim.rkt_pos_z, sim.sim_flight_state.value, sim.rkt_flight_state.value))
            print(sim.datalog[-1].format_z() + ', simState = ' + str(sim.sim_flight_state.value))

        sim.log_datapoint()
        sim.increment_tick()
        
        # sleep time :)
        end_time_ns = time.time_ns()
        sleep_time_ns = sim.timestep_ms * 1000000 - (end_time_ns - start_time_ns)
        if utils.Settings.USE_HARDWARE_TARGET:
            if sleep_time_ns > 0:
                time.sleep(sleep_time_ns / 100000000000) # sleep argument is in seconds
    
    if utils.Settings.USE_HARDWARE_TARGET == True:
        hw.com_port.close()
    
    # final datapoint after exiting simulation
    sim.log_datapoint()
    sim.analyze_sim_results()
    # sim.save_sim_log_to_file()
    sim.plot_simulation_results()
    

if __name__ == '__main__':
    main()