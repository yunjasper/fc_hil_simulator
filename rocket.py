"""
rocket.py

rocket object with attributes:
- mass (kg)
- drogue parachute drag coefficient
- main parachute drag coefficient
- launch rail angle from horizontal (degrees)
- thrust curve (N)

author: jasper yun
"""

import numpy as np
import thrustcurve

class Rocket:
    def __init__(self, dry_mass_kg, drogue_drag_coeff, drogue_area_m2, main_drag_coeff, main_area_m2, launch_angle_deg) -> None:
        self.dry_mass_kg = dry_mass_kg
        self.drogue_drag_coeff = drogue_drag_coeff
        self.drogue_area_m2 = drogue_area_m2
        self.main_drag_coeff = main_drag_coeff
        self.main_area_m2 = main_area_m2
        self.launch_angle_deg = launch_angle_deg

    def assign_thrust_curve(self, file_rse) -> None:
        # accepts an rse file (RockSim) thrust curve
        engines = thrustcurve.load(file_rse)
        e = engines[0]
        self.tc_time_s = e.data['time']
        self.tc_force_N = e.data['force']
        self.tc_mass_kg = e.data['mass'] / 1000 # convert g to kg
        self.tc_burn_time = np.max(e.data['time'])
    
    def get_thrust_N(self, time_s) -> float:
        if time_s < self.tc_burn_time:
            return np.interp(time_s, self.tc_time_s, self.tc_force_N)
        else:
            return 0
    
    def get_engine_propellant_mass_kg(self, time_s) -> float:
        if time_s < self.tc_burn_time:
            return np.interp(time_s, self.tc_time_s, self.tc_mass_kg)
        else:
            return 0
