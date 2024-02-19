/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: IM_TEST.c
 *
 * Code generated for Simulink model 'IM_TEST'.
 *
 * Model version                  : 1.15
 * Simulink Coder version         : 9.9 (R2023a) 19-Nov-2022
 * C/C++ source code generated on : Mon Feb 19 14:31:26 2024
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "IM_TEST.h"
#include "mw_cmsis.h"
#include "rtwtypes.h"

/* Block signals and states (default storage) */
DW rtDW;

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

/* Real-time model */
static RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;

/* Model step function */
void IM_TEST_step(void)
{
  real32_T rtb_Add2;
  real32_T rtb_DeadZone;
  real32_T rtb_Sum2;

  /* Outputs for Atomic SubSystem: '<Root>/IM_TEST' */
  /* Gain: '<S2>/Gain1' incorporates:
   *  Inport: '<Root>/In1'
   */
  rtb_Sum2 = 0.0174532924F * rtU.DEG;

  /* Sum: '<S1>/Add1' incorporates:
   *  Constant: '<S1>/Constant'
   *  Constant: '<S1>/Constant1'
   *  Inport: '<Root>/W1'
   *  Inport: '<Root>/W2'
   *  Product: '<S1>/Product1'
   *  Product: '<S1>/Product4'
   *  Sum: '<S1>/Sum1'
   *  Sum: '<S1>/Sum2'
   *  Trigonometry: '<S1>/Sin'
   *  Trigonometry: '<S1>/Sin2'
   */
  rtb_Add2 = arm_sin_f32(rtb_Sum2 + 0.785398185F) * rtU.W1 + arm_sin_f32
    (rtb_Sum2 + 2.3561945F) * rtU.W2;

  /* DeadZone: '<S1>/Dead Zone' */
  if (rtb_Add2 > 0.005F) {
    rtb_DeadZone = rtb_Add2 - 0.005F;
  } else if (rtb_Add2 >= -0.005F) {
    rtb_DeadZone = 0.0F;
  } else {
    rtb_DeadZone = rtb_Add2 - -0.005F;
  }

  /* End of DeadZone: '<S1>/Dead Zone' */

  /* Sum: '<S1>/Add2' incorporates:
   *  Constant: '<S1>/Constant'
   *  Constant: '<S1>/Constant1'
   *  Inport: '<Root>/W1'
   *  Inport: '<Root>/W2'
   *  Product: '<S1>/Product2'
   *  Product: '<S1>/Product3'
   *  Sum: '<S1>/Sum1'
   *  Sum: '<S1>/Sum2'
   *  Trigonometry: '<S1>/Sin1'
   *  Trigonometry: '<S1>/Sin3'
   */
  rtb_Add2 = arm_cos_f32(rtb_Sum2 + 0.785398185F) * rtU.W1 + arm_cos_f32
    (rtb_Sum2 + 2.3561945F) * rtU.W2;

  /* DeadZone: '<S1>/Dead Zone1' */
  if (rtb_Add2 > 0.005F) {
    rtb_Sum2 = rtb_Add2 - 0.005F;
  } else if (rtb_Add2 >= -0.005F) {
    rtb_Sum2 = 0.0F;
  } else {
    rtb_Sum2 = rtb_Add2 - -0.005F;
  }

  /* End of DeadZone: '<S1>/Dead Zone1' */

  /* Outport: '<Root>/XOUT' incorporates:
   *  DiscreteIntegrator: '<S1>/Discrete-Time Integrator'
   */
  rtY.XOUT = rtDW.DiscreteTimeIntegrator_DSTATE;

  /* Outport: '<Root>/YOUT' incorporates:
   *  DiscreteIntegrator: '<S1>/Discrete-Time Integrator1'
   */
  rtY.YOUT = rtDW.DiscreteTimeIntegrator1_DSTATE;

  /* Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator' */
  rtDW.DiscreteTimeIntegrator_DSTATE += 0.0001F * rtb_Sum2;

  /* Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator1' */
  rtDW.DiscreteTimeIntegrator1_DSTATE += 0.0001F * rtb_DeadZone;

  /* End of Outputs for SubSystem: '<Root>/IM_TEST' */

  /* Outport: '<Root>/XV_OUT' */
  rtY.XV_OUT = rtb_Sum2;

  /* Outport: '<Root>/YV_OUT' */
  rtY.YV_OUT = rtb_DeadZone;
}

/* Model initialize function */
void IM_TEST_initialize(void)
{
  /* (no initialization code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
