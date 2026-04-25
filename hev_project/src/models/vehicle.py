# src/models/vehicle.py
import math

class Vehicle:

    
    #1-D longitudinal vehicle model (Prius).
    #Units: meters, seconds, kg, W

    def __init__(self,
                 mass_kg=1410.0,
                 Cd=0.24,
                 A_m2=2.34,
                 Cr=0.0095,
                 rho_air=1.225,
                 driveline_eff=0.92):
        self.m = mass_kg
        self.Cd = Cd
        self.A = A_m2
        self.Cr = Cr
        self.rho = rho_air
        self.g = 9.81
        self.driveline_eff = driveline_eff

    def aero_drag_force(self, v_mps):
        return 0.5 * self.rho * self.Cd * self.A * v_mps * v_mps

    def rolling_force(self):
        # approximation: constant rolling at all speeds
        return self.m * self.g * self.Cr

    def grade_force(self, slope_deg=0.0):
        return self.m * self.g * math.sin(math.radians(slope_deg))

    def traction_force(self, v_mps, acc_mps2=0.0, slope_deg=0.0):

        F = (self.aero_drag_force(v_mps)
             + self.rolling_force()
             + self.grade_force(slope_deg)
             + self.m * acc_mps2)
        
        return F

    def power_required_at_wheels(self, v_mps, acc_mps2=0.0, slope_deg=0.0):
        #negative (when braking).
        
        F = self.traction_force(v_mps, acc_mps2, slope_deg)
        return F * v_mps

    def power_required_at_shaft(self, v_mps, acc_mps2=0.0, slope_deg=0.0):
  
        P_wheels = self.power_required_at_wheels(v_mps, acc_mps2, slope_deg)
        P_shaft = P_wheels / max(1e-9, self.driveline_eff)
        return max(0.0, P_shaft)

    def braking_power(self, v_mps, acc_mps2=0.0, slope_deg=0.0):

        P_wheels = self.power_required_at_wheels(v_mps, acc_mps2, slope_deg)
        # P_wheels negative => braking energy available (abs value)
        P_brake_wheels = max(0.0, -P_wheels)
        # convert wheels => shaft (approx)
        return P_brake_wheels / max(1e-9, self.driveline_eff)
