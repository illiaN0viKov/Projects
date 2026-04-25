# src/models/battery.py

class Battery:
    def __init__(self,
                 capacity_kwh=0.75,
                 soc_init=0.6,
                 soc_min=0.4,
                 soc_max=0.8,
                 eta_ch=0.95,
                 eta_dis=0.95,
                 max_discharge_kW=25.0,
                 max_charge_kW=20.0):
        self.capacity_Wh = capacity_kwh * 1000.0
        self.E_Wh = self.capacity_Wh * soc_init
        self.soc_min = soc_min
        self.soc_max = soc_max
        self.eta_ch = eta_ch
        self.eta_dis = eta_dis
        self.max_discharge_W = max_discharge_kW * 1000.0
        self.max_charge_W = max_charge_kW * 1000.0

    @property
    def soc(self):
        return self.E_Wh / max(1e-9, self.capacity_Wh)

    def apply_battery_power(self, P_batt_W, dt_s):

        # enforce instantaneous power limits
        P_applied = P_batt_W
        if P_batt_W > self.max_discharge_W:
            # request to discharge more than max -> deficit (W) to be covered by engine
            deficit = P_batt_W - self.max_discharge_W
            P_applied = self.max_discharge_W
        elif P_batt_W < -self.max_charge_W:
            # request to charge beyond max -> surplus (wasted)
            surplus = P_batt_W + self.max_charge_W  # negative + positive => negative surplus value
            P_applied = -self.max_charge_W
        else:
            surplus = 0.0
            deficit = 0.0

        # energy change in Wh. For discharging (P_applied>0) battery loses more due to inefficiency
        if P_applied >= 0:
            delta_Wh = (P_applied * dt_s) / 3600.0 / max(1e-9, self.eta_dis)
        else:
            # charging: stored energy is less due to charge efficiency
            delta_Wh = (P_applied * dt_s) / 3600.0 * max(0.0, self.eta_ch)

        self.E_Wh -= delta_Wh

        # upper bound
        if self.E_Wh > self.capacity_Wh * self.soc_max:
            surplus_Wh = self.E_Wh - self.capacity_Wh * self.soc_max
            self.E_Wh = self.capacity_Wh * self.soc_max
            # convert Wh surplus -> W over dt_s
            return surplus_Wh * 3600.0 / max(1e-9, dt_s)

        # lower bound
        if self.E_Wh < self.capacity_Wh * self.soc_min:
            deficit_Wh = self.capacity_Wh * self.soc_min - self.E_Wh
            self.E_Wh = self.capacity_Wh * self.soc_min
            return -deficit_Wh * 3600.0 / max(1e-9, dt_s)

        return 0.0
