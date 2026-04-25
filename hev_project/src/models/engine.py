# src/models/engine.py

class Engine:
    def __init__(self,
                 P_max_kW=72.0,
                 bsfc_g_per_kwh=240.0,
                 fuel_density_kg_per_l=0.745):
        # max mechanical power (W)
        self.P_max_W = P_max_kW * 1000.0
        # BSFC: grams of fuel per kWh mechanical (typical mid/peak values)
        self.bsfc = bsfc_g_per_kwh
        # fuel density (kg/L)
        self.fuel_density = fuel_density_kg_per_l

        # idle fuel expressed as small baseline when engine_on and producing near-zero mechanical
        self.idle_fuel_L_per_h = 0.9  # L/h typical idle
        self.idle_fuel_L_per_s = self.idle_fuel_L_per_h / 3600.0

    def clamp_power(self, P_req_W):
        return max(0.0, min(self.P_max_W, P_req_W))

    def fuel_flow_L_per_s(self, P_mech_W, engine_on=True):

        if not engine_on:
            return 0.0

        P_mech_W = max(0.0, P_mech_W)
        if P_mech_W < 1.0:
            return self.idle_fuel_L_per_s

        # mechanical power in kW
        P_kW = P_mech_W / 1000.0

        # fuel mass flow (g/s) = P_kW * BSFC (g/kWh) / 3600
        g_per_s = (P_kW * self.bsfc) / 3600.0

        # convert g/s to L/s
        kg_per_s = g_per_s / 1000.0
        L_per_s = kg_per_s / max(1e-9, self.fuel_density)
        return L_per_s
