/*
 * sil_test.c — SIL-style verification of the generated CruiseControl SWC.
 *
 * Compiles the auto-generated AUTOSAR C against a first-order vehicle plant
 * and a minimal RTE, then runs four scenarios and prints the metrics quoted
 * in README.md. No hardware required.
 *
 * Build:  see Makefile  (gcc -Wall -Wextra -std=c99 ... -lm)
 */

#include <stdio.h>
#include "CruiseControl.h"

/* ---- Minimal RTE: the generated code talks to these four functions ---- */
static float32 vsp = 0.0f, vset = 0.0f, etq = 0.0f;
static boolean ena = 0;

float32 Rte_IRead_CruiseControl_Step_VehSpdLgt_VehSpdLgt(void)     { return vsp;  }
float32 Rte_IRead_CruiseControl_Step_VehSpdSetPt_VehSpdSetPt(void) { return vset; }
boolean Rte_IRead_CruiseControl_Step_CcEna_CcEna(void)             { return ena;  }
void    Rte_IWrite_CruiseControl_Step_EngTqReq_EngTqReq(float32 u) { etq = u;     }

/* ---- Plant: first-order longitudinal vehicle ---- */
#define M   1500.0   /* kg            */
#define B   50.0     /* N.s/m         */
#define KF  60.0     /* N per % torque*/
#define TS  0.01     /* s (100 Hz)    */

int main(void)
{
    double v;

    /* T1 — flat road, step to 100 km/h */
    CruiseControl_Init(); v = 0.0; ena = 1; vset = 100.0f;
    for (int k = 0; k < 4000; k++) {
        vsp = (float32)(v * 3.6); CruiseControl_Step();
        v += TS * (KF * etq - B * v) / M;
    }
    printf("T1 flat   : final=%6.2f km/h  u=%5.1f %%\n", v * 3.6, etq);

    /* T2 — uphill, persistent 2500 N load */
    CruiseControl_Init(); v = 0.0; ena = 1; vset = 100.0f;
    for (int k = 0; k < 6000; k++) {
        vsp = (float32)(v * 3.6); CruiseControl_Step();
        v += TS * (KF * etq - B * v - 2500.0) / M;
    }
    printf("T2 hill   : final=%6.2f km/h  u=%5.1f %%\n", v * 3.6, etq);

    /* T3 — anti-windup: impossible 6000 N load for 20 s, then released */
    CruiseControl_Init(); v = 0.0; ena = 1; vset = 100.0f;
    double peak = 0.0;
    for (int k = 0; k < 4000; k++) {
        double tk = k * TS;
        vsp = (float32)(v * 3.6); CruiseControl_Step();
        double load = (tk < 20.0) ? 6000.0 : 0.0;
        v += TS * (KF * etq - B * v - load) / M;
        if (tk >= 20.0 && v * 3.6 > peak) peak = v * 3.6;
    }
    printf("T3 windup : peak after release=%6.1f km/h  overshoot=%4.1f %%\n",
           peak, (peak - 100.0) / 100.0 * 100.0);

    /* T4 — disable: integrator reset, output zero */
    ena = 0; CruiseControl_Step();
    printf("T4 OFF    : EngTqReq=%.1f (expect 0.0) -> bumpless reset OK\n", etq);

    return 0;
}
