# SIL verification

Two independent checks of the cruise control law — they must agree.

## C (generated SWC against a plant)

Compiles the auto-generated `CruiseControl.c` with a minimal RTE and a
first-order vehicle plant.

```bash
make run
```

Manual command (if `make` is unavailable):

```bash
gcc -Wall -Wextra -std=c99 -O2 \
    -I../CruiseControl_autosar_rtw -I../CruiseControl_autosar_rtw/stub -I../AUTOSAR_Include \
    -o sil_test sil_test.c ../CruiseControl_autosar_rtw/CruiseControl.c -lm
./sil_test
```

## Python (reference simulation)

```bash
python3 plant_sim.py
```

## Expected output

```
T1 flat   : final= 100.00 km/h  u= 23.1 %
T2 hill   : final= 100.00 km/h  u= 64.8 %
T3 windup : with AW ... overshoot ~1.0 %   vs   no AW ... overshoot ~110 %
T4 OFF    : EngTqReq=0.0 -> bumpless reset OK
```
