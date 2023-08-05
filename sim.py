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
        self.datatable = None # datalog to be converted into pandas DataFrame in this variable after sim ends
        
        base_filename = utils.Settings.SIMULATION_LOG_FILENAME_FORMAT + time.strftime('%Y-%m-%d_%H-%M-%S')
        self.session_folder = base_filename # folder name where outputs will be saved
        os.mkdir(self.session_folder)
        
    def set_timestep_ms(self, timestep_ms):
        self.timestep_ms = timestep_ms
    
    def increment_tick(self):
        self.time += self.timestep_ms

    def simulate_tick(self, rkt : rocket):
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
            # get thrust at this time
            if (self.time - self.launch_time) / 1000 < rkt.tc_burn_time:
                thrust = rkt.get_thrust_N((self.time - self.launch_time) / 1000)
            else:
                thrust = 0
            # calculate forces
            la_rad = rkt.launch_angle_deg * np.pi / 180
            force_x = np.cos(la_rad) * thrust
            force_z = np.sin(la_rad) * thrust - rocket_weight
        
        elif self.sim_flight_state == utils.FLIGHT_STATES.COAST:
            force_x = 0
            force_z = -1 * rocket_weight

        elif self.sim_flight_state == utils.FLIGHT_STATES.DROGUE_DESCENT:
            force_x = 0
            # account for drogue parachute drag, assume only in z direction for now
            air_mass_density = ambiance.Atmosphere(self.rkt_pos_z).density[0]
            # air_mass_density = utils.Settings.AIR_MASS_DENSITY
            force_drogue = rkt.drogue_drag_coeff * air_mass_density * (self.rkt_vel_z ** 2) * rkt.drogue_area_m2 / 2
            force_z = force_drogue - rocket_weight
        
        elif self.sim_flight_state == utils.FLIGHT_STATES.MAIN_DESCENT:
            force_x = 0
            # account for main parachute drag, assume only in z direction for now
            air_mass_density = ambiance.Atmosphere(self.rkt_pos_z).density[0]
            # air_mass_density = utils.Settings.AIR_MASS_DENSITY
            force_main = rkt.main_drag_coeff * air_mass_density * (self.rkt_vel_z ** 2) * rkt.main_area_m2 / 2
            force_z = force_main - rocket_weight
            
            if self.rkt_pos_z <= utils.Settings.GROUND_ALTITUDE_M:
                force_z = 0
        
        elif self.sim_flight_state == utils.FLIGHT_STATES.LANDED:
            # nothing to do
            force_x = 0
            force_z = 0

        if force_z <= 0 and self.rkt_pos_z <= utils.Settings.GROUND_ALTITUDE_M: # can't go down, only up
            # rocket is not moving
            self.rkt_acc_z = 0
            self.rkt_acc_x = 0
            self.rkt_vel_z = 0
            self.rkt_vel_x = 0
        else:
            # calculate resultant accelerations (F = m*a)
            self.rkt_acc_x = force_x / rkt.dry_mass_kg
            self.rkt_acc_z = force_z / rkt.dry_mass_kg
        
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

    def plot_simulation_results(self):
        self.convert_log_to_df(stride=utils.Settings.DATA_SAVE_STRIDE_PLOT)

        # find state change time indices
        states = self.datatable['flight state']
        state_change_indices = []
        previous_state = utils.FLIGHT_STATES.PAD.name
        for i in range(len(states)):
            if states[i] != previous_state:
                state_change_indices.append(i)
                previous_state = states[i]
        
        plt.rc('lines', linewidth=2)
        plt.rc('axes', grid=True)
        plt.rc('grid', linestyle='--')

        figX, axX = plt.subplots(3, 1)
        time_values_s = self.datatable['time (ms)'] / 1000
        axX[0].plot(time_values_s, self.datatable['position x (m)'])
        for idx in state_change_indices:
            axX[0].axvline(time_values_s[idx], color='red')
        if self.datalog[-1].rkt_flight_state == utils.FLIGHT_STATES.LANDED:
            axX[0].axvline(time_values_s[len(time_values_s) - 1], color='red')
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
        if self.datalog[-1].rkt_flight_state == utils.FLIGHT_STATES.LANDED:
            axZ[0].axvline(time_values_s[len(time_values_s) - 1], color='red')
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

    # start simulation
    # log initial conditions as a datapoint
    sim.log_datapoint()
    while sim.rkt_flight_state != utils.FLIGHT_STATES.LANDED:
        start_time_ns = time.time_ns()
        
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
            
        if (sim.time % utils.Settings.PRINT_UPDATE_TIMESTEP_MS == 0):
            # print('t = %d ms\tpos z = %.3f m\tsim state = %d\trkt state = %d' % (sim.time, sim.rkt_pos_z, sim.sim_flight_state.value, sim.rkt_flight_state.value))
            print(sim.datalog[-1].format_z() + '\tsimState = ' + str(sim.sim_flight_state.value))

        sim.log_datapoint()
        sim.increment_tick()
        
        # sleep time :)
        end_time_ns = time.time_ns()
        sleep_time_ns = sim.timestep_ms * 1000000 - (end_time_ns - start_time_ns)
        if utils.Settings.USE_HARDWARE_TARGET:
            if sleep_time_ns > 0:
                time.sleep(sleep_time_ns / 100000000000) # sleep argument is in seconds
    
    # final datapoint after exiting simulation
    sim.log_datapoint()
    sim.save_sim_log_to_file()
    sim.plot_simulation_results()
    

if __name__ == '__main__':
    main()