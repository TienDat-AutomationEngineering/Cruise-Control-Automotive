/*
 * File: CruiseControl.c
 *
 * Code generated for Simulink model 'CruiseControl'.
 *
 * Model version                  : 1.4
 * Simulink Coder version         : 25.1 (R2025a) 21-Nov-2024
 * C/C++ source code generated on : Sun Jun 28 2026
 *
 * Target selection: autosar.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "CruiseControl.h"
#include "Platform_Types.h"

/* PublicStructure Variables for Internal Data */
ARID_DEF rtARID_DEF_CruiseControl;     /* '<Root>/PI Controller' */

/* Model step function */
void CruiseControl_Step(void)
{
  float32 rtb_Err;
  float32 rtb_PreSat;
  float32 rtb_EngTqReq;

  /* Outputs for Enabled SubSystem: '<Root>/Cruise Active' incorporates:
   *  EnablePort: '<Root>/CcEna'
   */
  if (Rte_IRead_CruiseControl_Step_CcEna_CcEna()) {
    /* Sum: '<Root>/Sum' incorporates:
     *  Inport: '<Root>/VehSpdSetPt'
     *  Inport: '<Root>/VehSpdLgt'
     *
     *  e = setpoint - measured speed
     */
    rtb_Err = Rte_IRead_CruiseControl_Step_VehSpdSetPt_VehSpdSetPt() -
      Rte_IRead_CruiseControl_Step_VehSpdLgt_VehSpdLgt();

    /* Sum: '<Root>/PI Controller' incorporates:
     *  Gain: '<Root>/Kp'                   (Kp = 5.0)
     *  DiscreteIntegrator: '<Root>/Integrator'
     *
     *  PI output (pre-saturation) = Kp*e + I
     */
    rtb_PreSat = 5.0F * rtb_Err + rtARID_DEF_CruiseControl.Integrator_state;

    /* Saturate: '<Root>/Saturation'  — engine torque request 0..100 % */
    if (rtb_PreSat > 100.0F) { 
      rtb_EngTqReq = 100.0F; 
    } else if (rtb_PreSat < 0.0F) {
      rtb_EngTqReq = 0.0F; 
    } else {
      rtb_EngTqReq = rtb_PreSat; 
    }

    /* Update for DiscreteIntegrator: '<Root>/Integrator' incorporates:
     *  Gain: '<Root>/Ki'  (Ki*Ts = 1.5 * 0.01 = 0.015)
     *  Gain: '<Root>/Kb'  (Kb*Ts = 1.0 * 0.01 = 0.010)  back-calculation anti-windup
     *
     *  I += (Ki*e + Kb*(sat - presat)) * Ts
     */
    rtARID_DEF_CruiseControl.Integrator_state += 0.015F * rtb_Err + 0.01F *
      (rtb_EngTqReq - rtb_PreSat); // Cập nhật tích phân

    /* Outport: '<Root>/EngTqReq' */
    // Ghi lệnh ga ra P-Port qua RTE
    Rte_IWrite_CruiseControl_Step_EngTqReq_EngTqReq(rtb_EngTqReq);
  } else {
    /* Disable for Enabled SubSystem: '<Root>/Cruise Active'
     *
     *  OFF state: reset integrator (bumpless re-engage) and request 0 % torque.
     *  Driver regains direct pedal authority in the integration layer.
     */
    rtARID_DEF_CruiseControl.Integrator_state = 0.0F;
    Rte_IWrite_CruiseControl_Step_EngTqReq_EngTqReq(0.0F);
  }

  /* End of Outputs for SubSystem: '<Root>/Cruise Active' */
}

/* Model initialize function */
void CruiseControl_Init(void)
{
  /* InitializeConditions for DiscreteIntegrator: '<Root>/Integrator' */
  rtARID_DEF_CruiseControl.Integrator_state = 0.0F;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
