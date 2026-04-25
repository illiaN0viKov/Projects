# tests/run_model_test.py
from src.models.engine import Engine
from src.models.motor import Motor
from src.models.battery import Battery
from src.models.vehicle import Vehicle

def run_quick_checks():
    eng = Engine()
    mot = Motor()
    bat = Battery()
    veh = Vehicle()

    # Vehicle: power at 20 m/s (~72 km/h)
    v = 20.0
    acc = 0.5
    P_shaft = veh.power_required_at_shaft(v, acc)
    print("Power required at shaft (kW):", P_shaft/1000.0)

    # Engine: fuel for 20 kW mechanical
    P_eng = 20000.0
    print("Engine fuel (L/s) for 20kW:", eng.fuel_flow_L_per_s(P_eng))

    # Motor: electrical draw for 30 kW mech
    P_mot = 30000.0
    print("Motor electrical (W) for 30kW mech:", mot.mech_to_batt(P_mot))

    # Battery: apply 10 kW discharge for 10 s
    print("Initial SOC:", bat.soc)
    res = bat.apply_battery_power(10000.0, 10.0)
    print("Battery apply 10kW x10s returned:", res, "SOC now:", bat.soc)
    # try regen beyond charge limit
    res2 = bat.apply_battery_power(-50000.0, 1.0)  # -50kW charging request (bigger than max charge)
    print("Attempted huge regen returned:", res2, "SOC:", bat.soc)

if __name__ == '__main__':
    run_quick_checks()
