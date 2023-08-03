"""
sim.py

calculates the trajectory of the rocket.

author: jasper yun
"""

# "big" libraries
from enum import Enum
import numpy as np
from matplotlib import pyplot as plt

# "local" project libraries
import rocket
import settings

# to simulate the flight, let's do 2D (x and z) for now.
# assume that the rocket is a point particle
# thrust (function of time) operates directly on the point
# drag components impact trajectory through acceleration in x and z, potentially different drag coefficients
# no wind to start
# 

# used to track states of the simulation
class FLIGHT_STATES(Enum):
    PAD = 0
    BOOST = 1
    COAST = 2
    DROGUE_DESCENT = 3
    MAIN_DESCENT = 4
    LANDED = 5

class Simulation:
    def __init__(self) -> None:
        self.time = 0
        self.flight_state = FLIGHT_STATES.PAD
        
        # rocket states
        self.rkt_pos_x = 0
        self.rkt_pos_z = 0
        self.rkt_vel_x = 0
        self.rkt_vel_z = 0
        self.rkt_acc_x = 0
        self.rkt_acc_z = 0
    
    def set_timestep_ms(self, timestep_ms):
        self.timestep_ms = timestep_ms
    
    def increment_tick(self):
        self.time += self.timestep_ms

    def simulate_tick(self, rkt : rocket):
        # variables for forces
        force_x = 0
        force_z = 0

        # calculate forces in x and z directions based on the flight phase
        rocket_weight = settings.GRAVITY * rkt.mass_kg
        if self.flight_state == FLIGHT_STATES.PAD:
            # rocket is stationary on the launch rail, all forces are balanced --> zero acceleration
            force_x = 0
            force_z = 0
        
        elif self.flight_state == FLIGHT_STATES.BOOST:
            # get thrust at this time
            thrust = rkt.thrust_curve(self.time)
            # calculate forces
            la = rkt.launch_angle_deg
            force_x = np.cos(la) * thrust
            force_z = np.sin(la) * thrust - rocket_weight
        
        elif self.flight_state == FLIGHT_STATES.COAST:
            force_x = 0
            force_z = -1 * rocket_weight

        elif self.flight_state == FLIGHT_STATES.DROGUE_DESCENT:
            force_x = 0
            # account for drogue parachute drag, assume only in z direction for now
            force_drogue = rkt.drogue_drag_coeff * settings.AIR_MASS_DENSITY * (self.rkt_vel_z ** 2) * rkt.drogue_area_m2 / 2
            force_z = force_drogue - rocket_weight
        
        elif self.flight_state == FLIGHT_STATES.MAIN_DESCENT:
            force_x = 0
            # account for main parachute drag, assume only in z direction for now
            force_main = rkt.main_drag_coeff * settings.AIR_MASS_DENSITY * (self.rkt_vel_z ** 2) * rkt.main_area_m2 / 2
            force_z = force_main - rocket_weight
        
        elif self.flight_state == FLIGHT_STATES.LANDED:
            # nothing to do
            force_x = 0
            force_z = 0

        # calculate resultant accelerations (F = m*a)
        self.rkt_acc_x = force_x / rkt.mass_kg
        self.rkt_acc_z = force_z / rkt.mass_kg
        # integrate to get velocities and positions
        self.rkt_vel_x += self.rkt_acc_x * self.timestep_ms
        self.rkt_vel_z += self.rkt_acc_z * self.timestep_ms
        self.rkt_pos_x += self.rkt_vel_x * self.timestep_ms
        self.rkt_pos_z += self.rkt_vel_z * self.timestep_ms


def get_FC_input():
    # todo
    a = 1

def main():
    # instantiate classes
    rkt = rocket.Rocket(mass_kg=50, drogue_drag_coeff=10, main_drag_coeff=50, launch_angle=90)
    sim = Simulation()
    sim.set_timestep_ms(1)

    # start simulation
    while sim.flight_state != FLIGHT_STATES.LANDED:
        # simulate one tick
        sim.simulate_tick(rkt)

        # get input from FC
        get_FC_input()
        # increment tick
        sim.increment_tick()
    

if __name__ == '__main__':
    main()