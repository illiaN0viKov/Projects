# src/controllers/rule_based.py
import time

class RuleBasedController:

    def __init__(self,
                 soc_min=0.4,
                 soc_max=0.8,
                 motor_max_W=25000.0,
                 engine_max_W=72000.0,
                 engine_min_on_time_s=8.0):
        self.soc_min = soc_min
        self.soc_max = soc_max
        self.motor_max_W = motor_max_W
        self.engine_max_W = engine_max_W
        self.engine_min_on_time_s = engine_min_on_time_s

        self.engine_on = False
        self._engine_last_switch = -1e9

    def _turn_engine_on(self, t_now):
        if not self.engine_on:
            self.engine_on = True
            self._engine_last_switch = t_now

    def _can_turn_engine_off(self, t_now):
        # enforce minimum on-time to prevent hunting
        if not self.engine_on:
            return True
        return (t_now - self._engine_last_switch) >= self.engine_min_on_time_s

    def _turn_engine_off(self, t_now):
        if self.engine_on and self._can_turn_engine_off(t_now):
            self.engine_on = False
            self._engine_last_switch = t_now

    def decide(self, P_req_shaft_W, soc, t_now):
        # if braking/regenerative demand (handled externally), controller is consulted with P_req_shaft_W==0
        P_req = max(0.0, float(P_req_shaft_W))

        # basic SOC rules
        if soc <= self.soc_min:
            # force engine on to protect battery
            self._turn_engine_on(t_now)
        elif soc >= self.soc_max:
            # allow engine off if min on time satisfied
            if self._can_turn_engine_off(t_now):
                self._turn_engine_off(t_now)

        # Decision logic:
        # - Low demand: motor only (if SOC allows)
        # - Medium demand: if SOC healthy -> motor only; else engine on
        # - High demand: engine on + motor assist up to motor_max
        LOW_W = 8000.0   # below ~8 kW prefer motor-only
        MID_W = 25000.0  # above ~25 kW engine assist recommended

        P_engine = 0.0
        P_motor = 0.0

        if P_req <= LOW_W:
            # Small loads handled by motor only if SOC > min
            if soc > self.soc_min + 0.02:
                P_motor = min(P_req, self.motor_max_W)
                P_engine = 0.0
            else:
                # battery low → engine run to supply low-power loads and charge
                self._turn_engine_on(t_now)
                P_engine = min(P_req, self.engine_max_W)
                P_motor = max(0.0, P_req - P_engine)

        elif P_req <= MID_W:
            # medium demand
            if soc > (self.soc_min + 0.05):
                # prefer motor-only when possible (depending on motor_max)
                if P_req <= self.motor_max_W:
                    P_motor = P_req
                    P_engine = 0.0
                else:
                    # motor saturates -> engine supplies remainder
                    P_motor = self.motor_max_W
                    P_engine = P_req - P_motor
                    self._turn_engine_on(t_now)
            else:
                # SOC low -> engine supplies most
                self._turn_engine_on(t_now)
                # choose an engine operating power to keep engine in reasonable range (attempt to run moderate load)
                desired_engine_frac = 0.6
                P_engine = min(self.engine_max_W, desired_engine_frac * P_req)
                P_motor = max(0.0, P_req - P_engine)
                P_motor = min(P_motor, self.motor_max_W)

        else:
            # high demand: use engine (up to max) + motor assist
            self._turn_engine_on(t_now)
            P_engine = min(P_req, self.engine_max_W)
            P_motor = max(0.0, P_req - P_engine)
            P_motor = min(P_motor, self.motor_max_W)

        return {
            "P_engine_mech": float(max(0.0, P_engine)),
            "P_motor_mech": float(P_motor),
            "engine_on": bool(self.engine_on)
        }
