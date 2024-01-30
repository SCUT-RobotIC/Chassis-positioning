/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: IM_TEST.h
 *
 * Code generated for Simulink model 'IM_TEST'.
 *
 * Model version                  : 1.12
 * Simulink Coder version         : 9.9 (R2023a) 19-Nov-2022
 * C/C++ source code generated on : Mon Jan 29 19:52:11 2024
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

/* External inputs (root inport signals with default storage) */
typedef struct {
  real32_T W1;                         /* '<Root>/W1' */
  real32_T W2;                         /* '<Root>/W2' */
  real32_T DEG;                        /* '<Root>/In1' */
} ExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real32_T YOUT;                       /* '<Root>/YOUT' */
  real32_T XOUT;                       /* '<Root>/XOUT' */
} ExtY;

/* External inputs (root inport signals with default storage) */
extern ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY rtY;

/* Model entry point functions */
extern void IM_TEST_initialize(void);
extern void IM_TEST_step(void);

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
 */
#endif                                 /* RTW_HEADER_IM_TEST_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
