# Hardware bring-up — STM32F407

How the generated AUTOSAR SWC is run on a real STM32F407 board, monitored live with STM32CubeMonitor.

## Toolchain

STM32CubeMX (**CMake** toolchain) → VSCode + arm-none-eabi-gcc → flash via ST-Link.
Board: STM32F407VGT6, system clock 72 MHz (APB1 timer clock 72 MHz).

## Peripheral configuration (CubeMX)

| Peripheral | Setting | Maps to |
|------------|---------|---------|
| **Clock** | HSE → PLL (PLLM=4, PLLN=72) → 72 MHz; APB1 timer = 72 MHz | timing base |
| **TIM2** | Prescaler 7199, Period 99 → **100 Hz**; global interrupt **enabled** | runnable trigger (10 ms `TIMING-EVENT`) |
| **TIM3** | CH1 PWM on **PA6**; Prescaler 71, Period 999 → ~1 kHz | `EngTqReq` (throttle %) → duty |
| **ADC1** | IN1 on **PA1**, 12-bit, software trigger, single conversion | `VehSpdLgt` (speed, via potentiometer) |
| **GPIO in** | **PC0–PC3**, input, **pull-up** | Set / Resume / Cancel / Brake buttons |
| **GPIO out** | **PA5**, push-pull | cruise-active LED |
| **SYS** | Debug = Serial Wire (SWD) | flashing + CubeMonitor |

Verify the tick rate: `f = TimerClk / ((PSC+1)(ARR+1)) = 72e6 / (7200 × 100) = 100 Hz`.

## Pin map / wiring

| Signal | Pin | External wiring |
|--------|-----|-----------------|
| Speed input | PA1 | Potentiometer: ends → 3V3 / GND, wiper → PA1 (0–4095 → 0–200 km/h) |
| Throttle PWM | PA6 | LED + 220 Ω (brightness = duty), or scope/logic analyzer |
| Cruise LED | PA5 | LED + 220 Ω → GND |
| Set | PC0 | Button → GND (active-low, internal pull-up) |
| Resume | PC1 | Button → GND |
| Cancel | PC2 | Button → GND |
| Brake | PC3 | Button → GND |
| Debug | SWDIO/SWCLK | ST-Link |

## Build & flash

> The HAL `Drivers/` and `cmake/stm32cubemx/` are auto-generated and kept out of git to keep the repo lean. After cloning, open `CruiseControl_F407.ioc` in STM32CubeMX and **Generate Code** once to restore them before building.

1. In CubeMX set Project Manager → Toolchain/IDE = **CMake**, then **Generate Code**.
2. Add the SWC to `CMakeLists.txt`:
   ```cmake
   target_sources(${CMAKE_PROJECT_NAME} PRIVATE
       Cruise_Control/CruiseControl_autosar_rtw/CruiseControl.c)
   target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
       Cruise_Control/AUTOSAR_Include
       Cruise_Control/CruiseControl_autosar_rtw
       Cruise_Control/CruiseControl_autosar_rtw/stub)
   ```
3. Paste the four blocks from [`main_usercode.c`](main_usercode.c) into the matching `USER CODE` sections of `Core/Src/main.c` (Includes, PV, 0, 2).
4. Build (CMake) and flash with ST-Link from VSCode.

## Simplified RTE

There is no full AUTOSAR RTE stack on the target. The four `Rte_IRead/IWrite` functions the generated code calls are implemented in `main.c` over `volatile` shared buffers (`volatile` because they are touched both in the TIM2 ISR and elsewhere). This is the minimal glue that lets a standalone SWC run bare-metal.

## Live monitoring (STM32CubeMonitor)

Connect over SWD and add `rte_VehSpdLgt`, `rte_EngTqReq`, `rte_CcEna` (read from the `.elf` symbol table) to a real-time plot. Turning the potentiometer below the latched set point shows `EngTqReq` ramp up (integral action); the LED / `rte_CcEna` follows the Set/Cancel/Brake logic — confirming the model behaves identically on-target.

> Note: the physical bench has no plant (the potentiometer *is* the speed input, set by hand), so the loop is closed by the operator. True closed-loop tracking is validated in the Simulink/SIL harness; the board validates the control law, I/O, timing and state machine on real silicon.
