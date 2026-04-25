# src/models/motor.py

class Motor:
    def __init__(self,
                 P_max_kW=53.0,
                 motoring_eff=0.92,
                 regen_eff=0.65):
        self.P_max_W = P_max_kW * 1000.0
        self.motoring_eff = motoring_eff
        self.regen_eff = regen_eff

    def clamp_mechanical(self, P_mech_W):
        return max(-self.P_max_W, min(self.P_max_W, P_mech_W))

    def mech_to_batt(self, P_mech_W):

        if P_mech_W >= 0:
            # battery must supply more due to motor inefficiency
            return P_mech_W / max(1e-9, self.motoring_eff)
        else:
            # regen: only part of mechanical is recovered
            return P_mech_W * self.regen_eff
