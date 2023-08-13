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
import mach_dip
import rocket
import utils


class Simulation:

    def __init__(self) -> None:
        self.time = 0
        self.launch_time = 0
        self.launched = False
        self.sim_landed = False # sim may land before hardware detects it
        self.sim_landed_time = 0
        self.sim_flight_state = utils.FLIGHT_STATES.PAD
        self.ground_altitude = utils.Settings.GROUND_ALTITUDE_M
        self.simulate_mach_dip = utils.Settings.SIMULATE_TRANSONIC_MACH_DIP
        self.altitude_noise_amplitude = utils.Settings.ALTITUDE_NOISE_MAX_AMPLITUDE_M

        # rocket states
        self.rkt_pos_x = 0
        self.rkt_pos_z = self.ground_altitude
        self.rkt_pos_z_noisy = self.ground_altitude
        self.rkt_vel_x = 0
        self.rkt_vel_z = 0
        self.rkt_acc_x = 0
        self.rkt_acc_z = 0
        self.rkt_flight_state = utils.FLIGHT_STATES.PAD

        self.datalog = [] # list of utils.Sim_DataPoint objects
        self.datatable = pd.DataFrame(columns=utils.Settings.data_column_names) # empty, to be filled in after sim ends
        
        base_filename = utils.Settings.get_log_filename_format() + time.strftime('%Y-%m-%d_%H-%M-%S')
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

            # rocket hits the ground
            if self.rkt_pos_z <= self.ground_altitude:
                force_z = 0
        
        elif self.sim_flight_state == utils.FLIGHT_STATES.MAIN_DESCENT:
            force_x = 0
            # in off-nominal cases, motor may still be burning while hardware detected a different flight state
            thrust = get_thrust(rkt, self.time)
            # account for parachute drag, assume only in z direction for now. parachute drag force acts opposite to direction of motion
            air_mass_density = ambiance.Atmosphere(self.rkt_pos_z).density[0]
            force_main = rkt.main_drag_coeff * air_mass_density * (self.rkt_vel_z ** 2) * rkt.main_area_m2 / 2
            force_z = thrust + (-1 * np.sign(self.rkt_vel_z) * force_main) - rocket_weight
        
            # rocket hits the ground
            if self.rkt_pos_z <= self.ground_altitude:
                force_z = 0
        
        elif self.sim_flight_state == utils.FLIGHT_STATES.LANDED:
            # nothing to do
            force_x = 0
            force_z = 0
        
        self.update_kinematics(rkt, force_x, force_z)

    def update_kinematics(self, rocket : rocket.Rocket, force_x, force_z):
        if force_z <= 0 and self.rkt_pos_z <= self.ground_altitude:
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

        if self.simulate_mach_dip == True:
            dp = utils.Sim_DataPoint(self.time, self.rkt_pos_x, self.rkt_pos_z, self.rkt_pos_z_noisy, 
                                 self.rkt_vel_x, self.rkt_vel_z, self.rkt_acc_x, self.rkt_acc_z, self.rkt_flight_state)
            altitude = mach_dip.get_mach_dip_altitude(dp)
            self.rkt_pos_z = altitude
            self.rkt_pos_z_noisy = self.rkt_pos_z + (np.random.rand(1, 1)[0][0] - 0.5) * utils.Settings.ALTITUDE_NOISE_MAX_AMPLITUDE_M # update with new z position

    def log_datapoint(self):
        dp = utils.Sim_DataPoint(self.time, self.rkt_pos_x, self.rkt_pos_z, self.rkt_pos_z_noisy, 
                                 self.rkt_vel_x, self.rkt_vel_z, self.rkt_acc_x, self.rkt_acc_z, self.rkt_flight_state)
        self.datalog.append(dp)

    def convert_log_to_df(self, stride):
        print('Converting simulation session data...')
        conversion_progress_percent = 0

        datatable = pd.DataFrame(columns=utils.Settings.data_column_names) # empty, to be filled in
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
        self.datatable.to_csv(filename, sep=',', columns=utils.Settings.data_column_names, mode='a', index=False)
        f.close()
        os.chdir(os.pardir)

    def get_state_changes(self):
        # find state change time indices
        states = self.datatable.iloc[:, utils.DATA_INDEX.FLIGHT_STATE.value]
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

    def plot_simulation_results(self, show_plots=True):
        state_change_indices = self.get_state_changes()
        
        plt.rc('lines', linewidth=2)
        plt.rc('axes', grid=True)
        plt.rc('grid', linestyle='--')

        figX, axX = plt.subplots(3, 1)
        time_values_s = self.datatable.iloc[:, utils.DATA_INDEX.TIME.value] / 1000
        axX[0].plot(time_values_s, self.datatable.iloc[:, utils.DATA_INDEX.POS_X.value])
        for idx in state_change_indices:
            axX[0].axvline(time_values_s[idx], color='red')
        axX[0].set_ylabel('position x (m)')
        axX[1].plot(time_values_s, self.datatable.iloc[:, utils.DATA_INDEX.VEL_X.value])
        axX[1].set_ylabel('velocity x (m/s)')
        axX[2].plot(time_values_s, self.datatable.iloc[:, utils.DATA_INDEX.ACC_X.value])
        axX[2].set_ylabel('acceleration x (m/s2)')
        axX[2].set_xlabel('time (s)')

        figZ, axZ = plt.subplots(3, 1)
        axZ[0].plot(time_values_s, self.datatable.iloc[:, utils.DATA_INDEX.POS_Z_NOISY.value])
        axZ[0].plot(time_values_s, self.datatable.iloc[:, utils.DATA_INDEX.POS_Z.value])
        axZ[0].legend(['noisy', 'true'])
        for idx in state_change_indices:
            axZ[0].axvline(time_values_s[idx], color='red')
        axZ[0].set_ylabel('position z (m)')
        axZ[1].plot(time_values_s, self.datatable.iloc[:, utils.DATA_INDEX.VEL_Z.value])
        axZ[1].set_ylabel('velocity z (m/s)')
        axZ[2].plot(time_values_s, self.datatable.iloc[:, utils.DATA_INDEX.ACC_Z.value])
        axZ[2].set_ylabel('acceleration z (m/s2)')
        axZ[2].set_xlabel('time (s)')

        # save plots
        if os.path.basename(os.getcwd()) != self.session_folder:
            os.chdir(self.session_folder)
        figX.savefig(self.session_folder + '_x_axis_plots.png', dpi=300)
        figZ.savefig(self.session_folder + '_z_axis_plots.png', dpi=300)
        os.chdir(os.pardir)
        if show_plots:
            plt.show()

    def analyze_sim_results(self):
        # want to know:
        #   - altitude of all state changes
        #   - time + altitude at ejection vs true apogee (shows lead/lag of algorithm)
        state_change_indices = self.get_state_changes()
        # get true apogee
        apogee_index = np.argmax(self.datatable.iloc[:, utils.DATA_INDEX.POS_Z.value], axis=0)
        altitude_apogee = self.datatable.iloc[apogee_index, utils.DATA_INDEX.POS_Z.value]
        time_apogee = self.datatable.iloc[apogee_index, utils.DATA_INDEX.TIME.value]

        # get ejection stats
        ej_idx = 0
        for idx in state_change_indices:
            if self.datatable.iloc[idx, utils.DATA_INDEX.FLIGHT_STATE.value] == utils.FLIGHT_STATES.DROGUE_DESCENT.name:
                ej_idx = idx
                break
        altitude_ej = self.datatable.iloc[ej_idx, utils.DATA_INDEX.POS_Z.value]
        time_ej = self.datatable.iloc[ej_idx, utils.DATA_INDEX.TIME.value]
        
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
            
            time_final = self.datatable['time (ms)'][len(self.datatable) - 1]
            altitude_final = self.datatable['position z (m)'][len(self.datatable) - 1]
            state_final = self.datatable['flight state'][len(self.datatable) - 1]
            final_str =  'Final state:\ttime (ms) = %.3f\taltitude (m) = %.3f\tstate = %s' % (time_final, altitude_final, state_final)
            print(apogee_str)
            print(ej_str)
            print(diff_str)
            print(info_str)
            print(final_str)
            f.write(apogee_str + '\n')
            f.write(ej_str + '\n')
            f.write(diff_str + '\n')
            f.write(info_str + '\n')
            f.write(final_str + '\n')
            os.chdir(os.pardir)

