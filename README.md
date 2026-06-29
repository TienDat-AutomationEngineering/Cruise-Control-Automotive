# Cruise Control — Model-Based Design (Simulink → AUTOSAR Classic → STM32F407)

![MATLAB/Simulink](https://img.shields.io/badge/MATLAB%2FSimulink-R2025a-orange)
![AUTOSAR](https://img.shields.io/badge/AUTOSAR-Classic%20R23--11-blue)
![Target](https://img.shields.io/badge/Target-STM32F407%20(Cortex--M4F)-brightgreen)
![Language](https://img.shields.io/badge/Code-C99%20(auto--generated)-lightgrey)
![Build](https://img.shields.io/badge/SIL%20%2B%20C%20compile-passing-success)
![On-target](https://img.shields.io/badge/On--target-STM32F407%20validated-success)
![License](https://img.shields.io/badge/License-MIT-yellow)

A production-style **Model-Based Design (MBD)** demonstration of an automotive **cruise control** software function, taken from control design all the way to embedded integration **and validated on real hardware**:

> **Simulink/Stateflow model → AUTOSAR Blockset code generation (`autosar.tlc`, ARXML R23-11) → STM32F407 HAL integration with a simplified RTE → on-target run monitored live in STM32CubeMonitor.**

The controller is a **PI speed regulator with back-calculation anti-windup and bumpless transfer**, packaged as an AUTOSAR Classic Software Component (SWC). The control law is verified by simulation, by compiling and running the generated C against a vehicle plant model, and by **deploying to an STM32F407 board** where the live signals are observed in real time with STM32CubeMonitor.

---

## Table of contents
- [Why this project](#why-this-project)
- [System architecture](#system-architecture)
- [Control design](#control-design)
- [Verification results](#verification-results)
- [AUTOSAR interface](#autosar-interface)
- [Repository layout](#repository-layout)
- [Build &amp; run](#build--run)
- [Hardware bring-up & on-target validation](#hardware-bring-up--on-target-validation)
- [Engineering highlights](#engineering-highlights)
- [License](#license)

---

## Why this project

Cruise control is a compact but complete example of an automotive control function. It exercises the full skill set expected of a **Software Function / Software Control Developer**:

- closed-loop control theory (PI tuning, integrator anti-windup, bumpless transfer);
- the **AUTOSAR Classic** component model (SWC, ports, runnables, RTE, ARXML);
- the industrial **MBD toolchain** (Simulink model → Embedded Coder/AUTOSAR Blockset → target C);
- bare-metal **integration** on an STM32F407 (timers, ADC, PWM, ISRs);
- and an evidence-based **verification** mindset (SIL + on-target-equivalent C testing).

---

## System architecture

```
            ┌─────────────┐  CcEna / VehSpdSetPt   ┌──────────────────┐  EngTqReq   ┌─────────────┐
  HMI  ───► │  CcModeMgr  │ ─────────────────────► │  CruiseControl   │ ──────────► │  Throttle / │
 buttons    │ state machine│                       │  SWC (PI + AW)   │   [%]       │  Engine     │
            └─────────────┘                        └──────────────────┘             └─────────────┘
                  ▲                                          ▲
                  │ VehSpdLgt                                │ VehSpdLgt (feedback)
                  └──────────────────────────────────────────┘
```

- **`CruiseControl`** — the control SWC (auto-generated C). Holds the PI loop with anti-windup. This is the core deliverable.
- **`CcModeMgr`** — a small state machine (`OFF / ACTIVE / OVERRIDE`) handling the Set / Resume / Cancel / Brake buttons. Modeled as a Stateflow chart; implemented in the integration layer for the demo.
- The control task runs every **10 ms (100 Hz)** on TIM2.

Full block-level specification: [`MODEL_SPEC.md`](MODEL_SPEC.md).

---

## Control design

Plant (simplified longitudinal vehicle, first order): `m·v̇ = Kf·u − b·v − F_load`.

The discrete PI controller with back-calculation anti-windup (Ts = 0.01 s):

```
e        = VehSpdSetPt − VehSpdLgt          // speed error
presat   = Kp·e + I                          // PI output before limiting
EngTqReq = sat(presat, [0, 100] %)           // actuator limit
I       += Ki·Ts·e + Kb·Ts·(EngTqReq − presat)   // integrator update (+ anti-windup)
```

Tuned gains: **Kp = 5.0, Ki = 1.5, Kb = 1.0**, selected by a gain sweep for low overshoot and fast settling.

| Technique | Purpose |
|-----------|---------|
| Integral action | removes steady-state error under persistent load (e.g. driving uphill) |
| Back-calculation anti-windup | prevents integrator wind-up while the throttle is saturated |
| Bumpless transfer | resets the integrator on disengage so re-engaging does not jerk the actuator |

---

## Verification results

The control law is validated two ways: a discrete-time simulation, **and** by compiling the generated C (`-Wall -Wextra`, zero warnings) and running it against the plant. Both agree.

### Closed-loop step response

![Step response](docs/img/step_response.png)

On a flat road the speed reaches the set point with **1.0 % overshoot** and a **9.4 s** 2 % settling time. Uphill, the integral term automatically raises the torque request from ~23 % to ~65 % so the steady-state speed remains exactly at the set point.

### Why anti-windup matters

![Anti-windup comparison](docs/img/antiwindup.png)

After 20 s of an impossible load (actuator pinned at 100 %), the road clears. **Without** anti-windup the integrator has wound up and the speed overshoots by **110 %** (to ~210 km/h). **With** back-calculation anti-windup the overshoot is only **1.0 %**.

| Scenario | Final speed | Overshoot | Steady-state error |
|----------|-------------|-----------|--------------------|
| Flat road → 100 km/h | 100.00 km/h | 1.0 % | ≈ 0 |
| Uphill (+2500 N) | 100.00 km/h | 0.5 % | ≈ 0 (u → 64.8 %) |
| Anti-windup ON vs OFF | — | 1.0 % vs 110 % | — |
| Disable (OFF) | torque → 0, integrator reset | — | bumpless |

Reproduce everything with [`test/`](test/) (see [Build & run](#build--run)).

---

## AUTOSAR interface

Software component `CruiseControl`, runnable `CruiseControl_Step()` triggered by a 10 ms `TIMING-EVENT`. Data is exchanged through the RTE using implicit read/write (`Rte_IRead` / `Rte_IWrite`).

| Port | Direction | Interface (Sender-Receiver) | Type | Meaning |
|------|-----------|-----------------------------|------|---------|
| `VehSpdLgt`   | R (require) | `VehSpdLgt`   | `float32` | measured vehicle speed [km/h] |
| `VehSpdSetPt` | R (require) | `VehSpdSetPt` | `float32` | driver speed set point [km/h] |
| `CcEna`       | R (require) | `CcEna`       | `boolean` | cruise active enable |
| `EngTqReq`    | P (provide) | `EngTqReq`    | `float32` | engine torque request [%] |

Port and interface definitions are in the ARXML files (`CruiseControl_component.arxml`, `CruiseControl_interface.arxml`), following AUTOSAR Classic Platform application-interface naming conventions.

---

## Repository layout

```
.
├── README.md
├── MODEL_SPEC.md                  # Simulink/Stateflow block specification + control theory
├── HARDWARE.md                    # STM32F407 bring-up: peripherals, pin map, wiring, CubeMonitor
├── build_cruise_model.m           # builds the model via the Simulink API + SIL harness
├── main_usercode.c                # STM32F407 integration (TIM2/ADC/PWM, mode mgr, simplified RTE)
├── AUTOSAR_Include/               # Platform_Types.h, Std_Types.h, Compiler.h
├── CruiseControl_autosar_rtw/     # generated AUTOSAR SWC
│   ├── CruiseControl.c / .h
│   ├── CruiseControl_component.arxml
│   ├── CruiseControl_interface.arxml
│   └── stub/                      # Rte_CruiseControl.h, Rte_Type.h, MemMap.h
├── CruiseControl_F407/            # buildable STM32CubeMX/CMake firmware project
│   ├── Core/                      # main.c with the integrated SWC + simplified RTE
│   ├── Drivers/                   # STM32 HAL + CMSIS
│   ├── Cruise_Control/            # SWC sources referenced by the build
│   ├── CMakeLists.txt             # SWC sources/includes added under "user" sections
│   └── CruiseControl_F407.ioc     # CubeMX configuration
├── test/                          # SIL verification harness (C + Python)
└── docs/img/                      # result plots
```

---

## Build & run

### Reproduce the verification (no hardware required)

```bash
cd test
# Option A — C: compile the generated SWC against the plant model
make            # or: see test/README.md for the gcc command
./sil_test

# Option B — Python: identical discrete-time reference simulation
python3 plant_sim.py
```

### Regenerate the model and code (MATLAB)

```matlab
>> build_cruise_model     % builds CruiseControl.slx, configures AUTOSAR, runs SIL
% >> slbuild('CruiseControl')   % regenerates CruiseControl_autosar_rtw/
```
Requires Simulink, Stateflow, Embedded Coder and AUTOSAR Blockset. Without AUTOSAR Blockset the model still simulates; the committed C is the reference output.

### Deploy on STM32F407

The generated SWC is integrated bare-metal via a simplified RTE. Full peripheral configuration, pin map and wiring are documented in [`HARDWARE.md`](HARDWARE.md). In short: CubeMX (CMake toolchain) → VSCode → add the SWC sources/includes to `CMakeLists.txt` → paste the blocks from [`main_usercode.c`](main_usercode.c) into the matching `USER CODE` sections → build and flash with ST-Link.

---

## Hardware bring-up & on-target validation

The function was deployed and **validated on a real STM32F407 board**, not only in simulation.

- **Toolchain:** STM32CubeMX (CMake project) → VSCode + arm-none-eabi-gcc → ST-Link flash.
- **Control task:** the AUTOSAR runnable `CruiseControl_Step()` is called from a 100 Hz TIM2 interrupt — the hardware realisation of the model's 10 ms `TIMING-EVENT`.
- **I/O mapping:** potentiometer on ADC1/PA1 → `VehSpdLgt`; Set/Resume/Cancel/Brake buttons on PC0–PC3 → mode manager; `EngTqReq` → TIM3 PWM (PA6); cruise-active LED on PA5.
- **Live monitoring:** the RTE buffers (`rte_VehSpdLgt`, `rte_EngTqReq`, `rte_CcEna`) are observed in real time with **STM32CubeMonitor** over SWD, confirming the PI law, anti-windup and the OFF/ACTIVE/OVERRIDE state machine behave on-target exactly as in the model.

See [`HARDWARE.md`](HARDWARE.md) for the peripheral configuration table, wiring diagram and bring-up steps.

---

## Engineering highlights

- **From model to silicon** — a complete MBD chain: Simulink model → AUTOSAR C → bare-metal STM32F407, validated live with CubeMonitor.
- **Evidence over assertion** — the controller is not just written, it is proven: simulated, compiled warning-free, run against a plant with quantified overshoot/settling/steady-state error, and exercised on real hardware.
- **Anti-windup with numbers** — a concrete, defensible result (110 % → 1.0 % overshoot) rather than a buzzword.
- **Standard-compliant AUTOSAR** — real SWC/RTE/ARXML structure, not pseudo-code.
- **Reproducible** — anyone can clone and re-run the verification in seconds.

---

## License

Released under the [MIT License](LICENSE).
