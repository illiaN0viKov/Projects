# src/simulation/simulate.py

#Fixed HEV simulator with correct regenerative braking handling.

#Run: python -m src.simulation.simulate

"""
Notes:
- Uses Vehicle.power_required_at_shaft() for positive propulsion demand.
- Uses Vehicle.braking_power() to compute available mechanical braking power for regen.
- Motor.mechanics->battery conversion and Battery.apply_battery_power handle charging/discharging.
- If battery cannot absorb full regen, surplus is considered wasted (mechanical brakes).
- If battery cannot discharge requested motor power, engine must cover deficit.
- Engine idle fuel considered when engine_on True but producing zero mechanical power.
"""

import os
import math
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from src.models.vehicle import Vehicle
from src.models.engine import Engine
from src.models.motor import Motor
from src.models.battery import Battery
from src.controllers.rule_based import RuleBasedController  # or RuleBasedEMS depending on your controller file

OUT_DIR = os.path.join('.', 'results')
PLOT_DIR = os.path.join(OUT_DIR, 'plots')
LOG_DIR = os.path.join(OUT_DIR, 'logs')
os.makedirs(PLOT_DIR, exist_ok=True)

SIM_PARAMS = {
    "dt": 1.0,   # seconds
    "T": 600.0,  # simulation duration if no CSV provided
}

def load_drive_cycle(filepath=None, dt=1.0):
    #Return (t_array, v_mps_array). If no file, return a smooth synthetic cycle
    if filepath and os.path.isfile(filepath):
        df = pd.read_csv(filepath)
        if "speed_kph" in df.columns:
            v = df["speed_kph"].values / 3.6
        elif "speed_mps" in df.columns:
            v = df["speed_mps"].values
        elif "speed_kmh" in df.columns:
            v = df["speed_kmh"].values / 3.6
        else:
            raise ValueError("Drive cycle CSV must contain speed_kph or speed_mps column")
        t = df["time_s"].values if "time_s" in df.columns else np.arange(len(v)) * dt
        return t, v
    
    # synthetic smooth cycle (city + short highway)
    T = int(SIM_PARAMS["T"])
    t = np.arange(0, T, dt)
    v_kph = np.zeros_like(t, dtype=float)
    for i, ti in enumerate(t):
        if ti < 60:
            v_kph[i] = ti * (30/60)       # accelerate 0->30
        elif ti < 120:
            v_kph[i] = 30
        elif ti < 150:
            v_kph[i] = 30 - (ti-120)      # slow to stop
        elif ti < 200:
            v_kph[i] = 0
        elif ti < 260:
            v_kph[i] = (ti-200)*(50/60)   # accelerate to 50
        else:
            v_kph[i] = 50
    return t, v_kph / 3.6

def run_simulation(drive_cycle_path=None, save_prefix="run_fixed"):
    # instantiate models
    vehicle = Vehicle()
    engine = Engine()   # fuel_flow_L_per_s(P_mech_W, engine_on)
    motor = Motor()
    battery = Battery()

    controller = RuleBasedController(soc_min=battery.soc_min,
                                     soc_max=battery.soc_max,
                                     motor_max_W=motor.P_max_W,
                                     engine_min_on_time_s=5.0)

    t_arr, v_mps = load_drive_cycle(drive_cycle_path, dt=SIM_PARAMS["dt"])
    N = len(t_arr)
    dt = SIM_PARAMS["dt"]

    rows = []
    cumulative_fuel_L = 0.0

    for i in range(N):
        t_now = float(t_arr[i])
        v = float(v_mps[i])
        # acceleration forward difference
        if i == 0:
            acc = 0.0
            v_prev = v
        else:
            v_prev = float(v_mps[i-1])
            acc = (v - v_prev) / dt

        # positive propulsion demand at shaft (W)
        P_req_shaft = vehicle.power_required_at_shaft(v, acc, slope_deg=0.0)  # >= 0

        # available braking mechanical power at wheels (positive value in W)
        P_brake_mech_wheels = vehicle.braking_power(v, acc, slope_deg=0.0)  # >= 0 (wheels)
        # convert wheel braking power to shaft braking (approx divide by driveline_eff)
        P_brake_available_shaft = P_brake_mech_wheels / max(1e-9, vehicle.driveline_eff)

        # Controller decision
        decision = controller.decide(P_req_shaft, battery.soc, t_now)

        P_engine_mech = max(0.0, float(decision.get("P_engine_mech", 0.0)))
        P_motor_mech = float(decision.get("P_motor_mech", 0.0))
        engine_on = bool(decision.get("engine_on", False))


        P_motor_mech = max(-motor.P_max_W, min(motor.P_max_W, P_motor_mech))

        # If there is braking energy available, we can't regenerate more than available braking power
        if P_motor_mech < 0:

            regen_mech_abs = min(abs(P_motor_mech), P_brake_available_shaft)
            P_motor_mech = -regen_mech_abs

        # Now convert motor mechanical to battery electrical power
        P_batt_from_motor_W = motor.mech_to_batt(P_motor_mech)

        # Enforce battery instantaneous power limits inside battery.apply_battery_power
        surplus_or_deficit_W = battery.apply_battery_power(P_batt_from_motor_W, dt)

        # Handle battery deficits/surplus:
        # If battery couldn't discharge enough (deficit < 0), engine must supply the missing mechanical power.
        if surplus_or_deficit_W < 0:
            # deficit (negative): battery couldn't supply requested discharge (W)
            needed_from_engine_W = -surplus_or_deficit_W
            # engine must increase mechanical contribution by this amount at shaft
            P_engine_mech += needed_from_engine_W


        # Compute fuel for engine mechanical power
        fuel_L_per_s = engine.fuel_flow_L_per_s(P_engine_mech, engine_on=engine_on)
        fuel_this_step_L = fuel_L_per_s * dt
        cumulative_fuel_L += fuel_this_step_L

        # Log row
        rows.append({
            "t": t_now,
            "v_mps": v,
            "v_kph": v * 3.6,
            "acc_mps2": acc,
            "P_req_shaft_W": P_req_shaft,
            "P_brake_avail_shaft_W": P_brake_available_shaft,
            "P_engine_mech_W": P_engine_mech,
            "P_motor_mech_W": P_motor_mech,
            "P_batt_W": P_batt_from_motor_W,
            "SOC": battery.soc,
            "engine_on": engine_on,
            "fuel_total_L": cumulative_fuel_L
        })

    df = pd.DataFrame(rows)
    out_csv = os.path.join(LOG_DIR, f"{save_prefix}_results.csv")
    df.to_csv(out_csv, index=False)
    print("Saved results to:", out_csv)

    # Plotting
    fig, axs = plt.subplots(5, 1, figsize=(10, 14), sharex=True)
    axs[0].plot(df["t"], df["v_kph"]); axs[0].set_ylabel("Speed (km/h)"); axs[0].grid(True)
    axs[1].plot(df["t"], df["P_req_shaft_W"] / 1000.0); axs[1].set_ylabel("P_req (kW)"); axs[1].grid(True)
    axs[2].plot(df["t"], df["P_engine_mech_W"] / 1000.0, label="Engine"); axs[2].plot(df["t"], df["P_motor_mech_W"] / 1000.0, label="Motor"); axs[2].legend(); axs[2].set_ylabel("Power split (kW)"); axs[2].grid(True)
    axs[3].plot(df["t"], df["P_batt_W"] / 1000.0); axs[3].set_ylabel("Battery P (kW)"); axs[3].grid(True)
    axs[4].plot(df["t"], df["SOC"]); axs[4].set_ylabel("SOC"); axs[4].set_xlabel("Time (s)"); axs[4].grid(True)
    plt.tight_layout()
    plot_file = os.path.join(PLOT_DIR, f"{save_prefix}_summary.png")
    fig.suptitle("HEV Simulation (fixed regen)", y=1.02)
    fig.savefig(plot_file, bbox_inches="tight")
    print("Saved plot to:", plot_file)
    plt.show()

    print("Final fuel consumed (L):", df["fuel_total_L"].iloc[-1])

    # Total distance [km]
    distance_km = (df["v_mps"] * dt).sum() / 1000.0

    # Electrical energy [kWh] (net)
    E_elec_kWh = (df["P_batt_W"] * dt).sum() / 3.6e6

    # Fuel consumption [L]
    fuel_L = df["fuel_total_L"].iloc[-1]

    # Fuel-equivalent liters (1 Le = 8.9 kWh gasoline LHV)
    fuel_equiv_L = fuel_L + (E_elec_kWh / 8.9)

    # Metrics
    elec_kWh_per_km = E_elec_kWh / distance_km
    total_eff_km_per_Le = distance_km / fuel_equiv_L

    print("\n=== Energy Metrics ===")
    print(f"Distance driven: {distance_km:.3f} km")
    print(f"Electrical energy consumption: {elec_kWh_per_km:.3f} kWh/km")
    print(f"Total energy efficiency: {total_eff_km_per_Le:.2f} km/L_e")
    
    return df

if __name__ == "__main__":
    run_simulation()

# if __name__ == "__main__":
#     run_simulation(drive_cycle_path = "./data/drive_cycles/wltp_like.csv")

