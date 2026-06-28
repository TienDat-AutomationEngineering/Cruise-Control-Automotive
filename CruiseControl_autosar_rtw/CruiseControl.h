/*
 * File: CruiseControl.h
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

#ifndef CruiseControl_h_
#define CruiseControl_h_
#ifndef CruiseControl_COMMON_INCLUDES_
#define CruiseControl_COMMON_INCLUDES_
#include "Platform_Types.h"
#include "Rte_CruiseControl.h"
#endif                                 /* CruiseControl_COMMON_INCLUDES_ */

/* PublicStructure Variables for Internal Data, for system '<Root>' */
typedef struct {
  float32 Integrator_state;            /* '<Root>/Integrator' — PI integral state */
} ARID_DEF;

/* PublicStructure Variables for Internal Data */
extern ARID_DEF rtARID_DEF_CruiseControl;/* '<Root>/PI Controller' */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S1>')    - opens system 1
 * hilite_system('<S1>/Kp') - opens and selects block Kp which resides in S1
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'CruiseControl'
 * '<S1>'   : 'CruiseControl/Cruise Active'
 */
#endif                                 /* CruiseControl_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
