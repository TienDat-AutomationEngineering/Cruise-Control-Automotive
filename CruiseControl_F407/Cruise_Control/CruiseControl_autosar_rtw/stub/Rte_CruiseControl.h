/* This file contains stub implementations of the AUTOSAR RTE functions.
   The stub implementations can be used for testing the generated code in
   Simulink, for example, in SIL/PIL simulations of the component under
   test. Note that this file should be replaced with an appropriate RTE
   file when deploying the generated code outside of Simulink.

   This file is generated for:
   Atomic software component:  "CruiseControl"
   ARXML schema: "R23-11"
   File generated on: "Sun Jun 28 2026"  */

#ifndef Rte_CruiseControl_h
#define Rte_CruiseControl_h
#include "Rte_Type.h"
#include "Compiler.h"

/* Data access functions */

/* R-Port: VehSpdLgt (float32) — measured vehicle speed [km/h] */
#define Rte_IRead_CruiseControl_Step_VehSpdLgt_VehSpdLgt Rte_IRead_CruiseControl_CruiseControl_Step_VehSpdLgt_VehSpdLgt

float32 Rte_IRead_CruiseControl_Step_VehSpdLgt_VehSpdLgt(void);

/* R-Port: VehSpdSetPt (float32) — driver speed set point [km/h] */
#define Rte_IRead_CruiseControl_Step_VehSpdSetPt_VehSpdSetPt Rte_IRead_CruiseControl_CruiseControl_Step_VehSpdSetPt_VehSpdSetPt

float32 Rte_IRead_CruiseControl_Step_VehSpdSetPt_VehSpdSetPt(void);

/* R-Port: CcEna (boolean) — cruise active enable from mode manager */
#define Rte_IRead_CruiseControl_Step_CcEna_CcEna Rte_IRead_CruiseControl_CruiseControl_Step_CcEna_CcEna

boolean Rte_IRead_CruiseControl_Step_CcEna_CcEna(void);

/* P-Port: EngTqReq (float32) — engine torque request [%] */
#define Rte_IWrite_CruiseControl_Step_EngTqReq_EngTqReq Rte_IWrite_CruiseControl_CruiseControl_Step_EngTqReq_EngTqReq

void Rte_IWrite_CruiseControl_Step_EngTqReq_EngTqReq(float32 u);

#define Rte_IWriteRef_CruiseControl_Step_EngTqReq_EngTqReq Rte_IWriteRef_CruiseControl_CruiseControl_Step_EngTqReq_EngTqReq

float32* Rte_IWriteRef_CruiseControl_Step_EngTqReq_EngTqReq(void);

/* Entry point functions */
extern FUNC(void, CruiseControl_CODE) CruiseControl_Init(void);
extern FUNC(void, CruiseControl_CODE) CruiseControl_Step(void);

#endif
