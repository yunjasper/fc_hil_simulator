"""
rocket.py

rocket object with attributes:
- mass (kg)
- drogue parachute drag coefficient
- main parachute drag coefficient
- launch rail angle from horizontal (degrees)
- thrust curve (N) (interpolated scipy function)

author: jasper yun
"""

class Rocket:
    def __init__(self, mass_kg, drogue_drag_coeff, drogue_area_m2, main_drag_coeff, main_area_m2, launch_angle_deg) -> None:
        self.mass_kg = mass_kg
        self.drogue_drag_coeff = drogue_drag_coeff
        self.drogue_area_m2 = drogue_area_m2
        self.main_drag_coeff = main_drag_coeff
        self.main_area_m2 = main_area_m2
        self.launch_angle_deg = launch_angle_deg
    
        # todo: account for propellant mass
        
    def assign_thrust_curve(self, thrust_fn) -> None:
        self.thrust_curve_N = thrust_fn