"""
plant_sim.py — reference discrete-time simulation of the cruise control loop.

Independent re-implementation of the control law (PI + back-calculation
anti-windup) used to cross-check the generated C in sil_test.c. Run with:

    python3 plant_sim.py
"""

# Plant: first-order longitudinal vehicle
M, B, KF, TS = 1500.0, 50.0, 60.0, 0.01

# Controller gains (must match CruiseControl.c)
KP, KI, KB = 5.0, 1.5, 1.0
UMIN, UMAX = 0.0, 100.0


def run(setpoint, load=0.0, anti_windup=True, release_at=None, T=40.0):
    n = int(T / TS)
    v, I, peak = 0.0, 0.0, 0.0
    u = 0.0
    for k in range(n):
        t = k * TS
        spd = v * 3.6
        e = setpoint - spd
        pre = KP * e + I
        u = min(max(pre, UMIN), UMAX)
        I += (KI * e + (KB if anti_windup else 0.0) * (u - pre)) * TS
        ld = load if (release_at is None or t < release_at) else 0.0
        v += TS * (KF * u - B * v - ld) / M
        if release_at is not None and t >= release_at:
            peak = max(peak, v * 3.6)
    final = v * 3.6
    return final, u, peak


if __name__ == "__main__":
    f, u, _ = run(100.0)
    print(f"T1 flat   : final={f:6.2f} km/h  u={u:5.1f} %")
    f, u, _ = run(100.0, load=2500.0)
    print(f"T2 hill   : final={f:6.2f} km/h  u={u:5.1f} %")
    _, _, pk = run(100.0, load=6000.0, anti_windup=True, release_at=20.0)
    print(f"T3 windup : with AW peak={pk:6.1f} km/h  overshoot={(pk-100)/100*100:4.1f} %")
    _, _, pk = run(100.0, load=6000.0, anti_windup=False, release_at=20.0)
    print(f"T3 windup : no AW   peak={pk:6.1f} km/h  overshoot={(pk-100)/100*100:4.1f} %")
