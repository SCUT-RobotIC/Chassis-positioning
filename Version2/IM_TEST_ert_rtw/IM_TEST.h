/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: IM_TEST.h
 *
 * Code generated for Simulink model 'IM_TEST'.
 *
 * Model version                  : 1.31
 * Simulink Coder version         : 9.9 (R2023a) 19-Nov-2022
 * C/C++ source code generated on : Sun Mar 31 18:34:26 2024
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#ifndef RTW_HEADER_IM_TEST_h_
#define RTW_HEADER_IM_TEST_h_
#ifndef IM_TEST_COMMON_INCLUDES_
#define IM_TEST_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* IM_TEST_COMMON_INCLUDES_ */

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM RT_MODEL;

/* Block signals and states (default storage) for system '<S6>/For Each Subsystem' */
typedef struct {
  real_T VariableIntegerDelay_DSTATE[4096];/* '<S7>/Variable Integer Delay' */
} DW_CoreSubsys;

/* Block signals and states (default storage) for system '<S6>/For Each Subsystem' */
typedef struct {
  DW_CoreSubsys CoreSubsys;            /* '<S6>/For Each Subsystem' */
} DW_ForEachSubsystem;

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  DW_ForEachSubsystem ForEachSubsystem_l[1];/* '<S8>/For Each Subsystem' */
  DW_ForEachSubsystem ForEachSubsystem_o[1];/* '<S6>/For Each Subsystem' */
  real_T Probe[2];                     /* '<S6>/Probe' */
  real_T Probe_f[2];                   /* '<S8>/Probe' */
  real_T DiscreteTimeIntegrator_DSTATE;/* '<S8>/Discrete-Time Integrator' */
  real_T UnitDelay_DSTATE;             /* '<S8>/Unit Delay' */
  real_T UnitDelay1_DSTATE;            /* '<S8>/Unit Delay1' */
  real_T DiscreteTimeIntegrator_DSTATE_d;/* '<S6>/Discrete-Time Integrator' */
  real_T UnitDelay_DSTATE_i;           /* '<S6>/Unit Delay' */
  real_T UnitDelay1_DSTATE_p;          /* '<S6>/Unit Delay1' */
  uint8_T DiscreteTimeIntegrator_SYSTEM_E;/* '<S8>/Discrete-Time Integrator' */
  uint8_T DiscreteTimeIntegrator_SYSTEM_g;/* '<S6>/Discrete-Time Integrator' */
} DW;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real32_T W1;                         /* '<Root>/W1' */
  real32_T W2;                         /* '<Root>/W2' */
  real32_T DEG;                        /* '<Root>/In1' */
  real_T X_ACCIN;                      /* '<Root>/X_ACCIN' */
  real_T Y_ACCIN;                      /* '<Root>/Y_ACCIN' */
} ExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T YOUT;                         /* '<Root>/YOUT' */
  real_T XOUT;                         /* '<Root>/XOUT' */
  real_T X_ACCOUT;                     /* '<Root>/X_ACCOUT' */
  real_T Y_ACCOUT;                     /* '<Root>/Y_ACCOUT' */
} ExtY;

/* Real-time Model Data Structure */
struct tag_RTM {
  const char_T * volatile errorStatus;
};

/* Block signals and states (default storage) */
extern DW rtDW;

/* External inputs (root inport signals with default storage) */
extern ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY rtY;

/*
 * Exported Global Signals
 *
 * Note: Exported global signals are block signals with an exported global
 * storage class designation.  Code generation will declare the memory for
 * these signals and export their symbols.
 *
 */
extern real_T yraw;                    /* '<S1>/Add1' */
extern real_T xraw;                    /* '<S1>/Add2' */

/* Model entry point functions */
extern void IM_TEST_initialize(void);
extern void IM_TEST_step(void);

/* Real-time Model object */
extern RT_MODEL *const rtM;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Note that this particular code originates from a subsystem build,
 * and has its own system numbers different from the parent model.
 * Refer to the system hierarchy for this subsystem below, and use the
 * MATLAB hilite_system command to trace the generated code back
 * to the parent model.  For example,
 *
 * hilite_system('MAIN/IM_TEST')    - opens subsystem MAIN/IM_TEST
 * hilite_system('MAIN/IM_TEST/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'MAIN'
 * '<S1>'   : 'MAIN/IM_TEST'
 * '<S2>'   : 'MAIN/IM_TEST/Degrees to Radians'
 * '<S3>'   : 'MAIN/IM_TEST/MATLAB Function'
 * '<S4>'   : 'MAIN/IM_TEST/Moving Average'
 * '<S5>'   : 'MAIN/IM_TEST/Moving Average1'
 * '<S6>'   : 'MAIN/IM_TEST/Moving Average/Discrete'
 * '<S7>'   : 'MAIN/IM_TEST/Moving Average/Discrete/For Each Subsystem'
 * '<S8>'   : 'MAIN/IM_TEST/Moving Average1/Discrete'
 * '<S9>'   : 'MAIN/IM_TEST/Moving Average1/Discrete/For Each Subsystem'
 */
#endif                                 /* RTW_HEADER_IM_TEST_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
