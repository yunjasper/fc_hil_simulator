"""
sim.py

calculates the trajectory of the rocket.

author: jasper yun
"""

# "big" libraries
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

# todo: add logging of all data for the simulation session

# for now, hardcode the port. todo: make flexible 
SERIAL_PORT = 'COM7'
BAUD_RATE = 115200
SERIAL_PORT_TIMEOUT = 5 # seconds

class Simulation:
    def __init__(self) -> None:
        self.time = 0
        self.launch_time = 0
        self.launched = False
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
        
    def set_timestep_ms(self, timestep_ms):
        self.timestep_ms = timestep_ms
    
    def increment_tick(self):
        self.time += self.timestep_ms

    def simulate_tick(self, rkt : rocket):
        # variables for forces
        force_x = 0
        force_z = 0

        # calculate forces in x and z directions based on the flight phase
        rocket_weight = utils.Settings.GRAVITY * (rkt.dry_mass_kg + rkt.get_engine_mass_kg((self.time - self.launch_time) / 1000))

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
            force_drogue = rkt.drogue_drag_coeff * utils.Settings.AIR_MASS_DENSITY * (self.rkt_vel_z ** 2) * rkt.drogue_area_m2 / 2
            force_z = force_drogue - rocket_weight
        
        elif self.sim_flight_state == utils.FLIGHT_STATES.MAIN_DESCENT:
            force_x = 0
            # account for main parachute drag, assume only in z direction for now
            force_main = rkt.main_drag_coeff * utils.Settings.AIR_MASS_DENSITY * (self.rkt_vel_z ** 2) * rkt.main_area_m2 / 2
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

        if self.time % 100 == 0:
            a = 1 # added code to enable faster debugging
            b = 2

    def log_data(self):
        dp = utils.Sim_DataPoint(self.time, self.rkt_pos_x, self.rkt_pos_z, self.rkt_pos_z_noisy, 
                                 self.rkt_vel_x, self.rkt_vel_z, self.rkt_acc_x, self.rkt_acc_z, self.rkt_flight_state)
        self.datalog.append(dp)
        
    def plot_simulation_results(self):
        cols = ['time (ms)', 'position x (m)', 'position z (m)', 'position z (noisy) (m)', 'velocity x (m/s)', 
                'velocity z (m/s)', 'acceleration x (m/s2)', 'acceleration z (m/s2)', 'flight state']
        datatable = pd.DataFrame(columns=cols) # empty
        for idx in np.arange(len(self.datalog), step=1000):
            point = self.datalog[idx]
            datatable.loc[len(datatable)] = point.unwrap()

        # find state change time indices
        states = datatable['flight state']
        state_change_indices = []
        previous_state = utils.FLIGHT_STATES.PAD
        for i in range(len(states)):
            if states[i] != previous_state:
                state_change_indices.append(i)
                previous_state = states[i]
        
        plt.rc('lines', linewidth=2)
        plt.rc('axes', grid=True)
        plt.rc('grid', linestyle='--')

        fig, ax = plt.subplots(3, 2)
        time_values_s = datatable['time (ms)'] / 1000
        ax[0][0].plot(time_values_s, datatable['position x (m)'])
        for idx in state_change_indices:
            ax[0][0].axvline(time_values_s[idx], color='red')
        ax[0][0].set_ylabel('position x (m)')
        ax[1][0].plot(time_values_s, datatable['velocity x (m/s)'])
        ax[1][0].set_ylabel('velocity x (m/s)')
        ax[2][0].plot(time_values_s, datatable['acceleration x (m/s2)'])
        ax[2][0].set_ylabel('acceleration x (m/s2)')
        ax[2][0].set_xlabel('time (s)')

        ax[0][1].plot(time_values_s, datatable['position z (noisy) (m)'])
        ax[0][1].plot(time_values_s, datatable['position z (m)'])
        for idx in state_change_indices:
            ax[0][1].axvline(time_values_s[idx], color='red')
        ax[0][1].set_ylabel('position z (m)')
        ax[1][1].plot(time_values_s, datatable['velocity z (m/s)'])
        ax[1][1].set_ylabel('velocity z (m/s)')
        ax[2][1].plot(time_values_s, datatable['acceleration z (m/s2)'])
        ax[2][1].set_ylabel('acceleration z (m/s2)')
        ax[2][1].set_xlabel('time (s)')

        plt.show()

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
    while sim.sim_flight_state != utils.FLIGHT_STATES.LANDED:
        start_time_ns = time.time_ns()
        
        sim.simulate_tick(rkt)
        sim.log_data()
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
                    or hw_flight_state == utils.FLIGHT_STATES.MAIN_DESCENT 
                    or hw_flight_state == utils.FLIGHT_STATES.LANDED):
                sim.sim_flight_state = hw_flight_state  
            
        
        if (sim.time % utils.Settings.PRINT_UPDATE_TIMESTEP_MS == 0):
            print('t = %d ms\tpos z = %.3f m\tsim state = %d\trkt state = %d' % (sim.time, sim.rkt_pos_z, sim.sim_flight_state.value, sim.rkt_flight_state.value))

        # if hw_flight_state == utils.FLIGHT_STATES.BOOST and sim.flight_state == utils.FLIGHT_STATES.PAD:
        #     sim.launch_time = sim.time # needed because thrust curve time starts at zero
        sim.increment_tick()
        
        # sleep time :)
        end_time_ns = time.time_ns()
        sleep_time_ns = sim.timestep_ms * 1000000 - (end_time_ns - start_time_ns)
        if sleep_time_ns > 0:
            time.sleep(sleep_time_ns / 1000000000) # sleep argument is in seconds
    
    # save data to file (todo)
    
    # plot results
    sim.plot_simulation_results()
    

if __name__ == '__main__':
    main()