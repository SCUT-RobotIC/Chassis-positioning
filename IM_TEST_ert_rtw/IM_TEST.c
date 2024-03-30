/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: IM_TEST.c
 *
 * Code generated for Simulink model 'IM_TEST'.
 *
 * Model version                  : 1.18
 * Simulink Coder version         : 9.9 (R2023a) 19-Nov-2022
 * C/C++ source code generated on : Sat Mar 30 18:36:31 2024
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

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

/* Model step function */
void IM_TEST_step(void)
{
  real32_T rtb_Add2;
  real32_T rtb_DeadZone4;
  real32_T rtb_DeadZone5;
  real32_T rtb_Sum2;

  /* Outputs for Atomic SubSystem: '<Root>/IM_TEST' */
  /* DeadZone: '<S1>/Dead Zone4' incorporates:
   *  Inport: '<Root>/W1'
   */
  if (rtU.W1 > 30.0F) {
    rtb_DeadZone4 = rtU.W1 - 30.0F;
  } else if (rtU.W1 >= -30.0F) {
    rtb_DeadZone4 = 0.0F;
  } else {
    rtb_DeadZone4 = rtU.W1 - -30.0F;
  }

  /* End of DeadZone: '<S1>/Dead Zone4' */

  /* Gain: '<S2>/Gain1' incorporates:
   *  Inport: '<Root>/In1'
   */
  rtb_Sum2 = 0.0174532924F * rtU.DEG;

  /* DeadZone: '<S1>/Dead Zone5' incorporates:
   *  Inport: '<Root>/W2'
   */
  if (rtU.W2 > 30.0F) {
    rtb_DeadZone5 = rtU.W2 - 30.0F;
  } else if (rtU.W2 >= -30.0F) {
    rtb_DeadZone5 = 0.0F;
  } else {
    rtb_DeadZone5 = rtU.W2 - -30.0F;
  }

  /* End of DeadZone: '<S1>/Dead Zone5' */

  /* Sum: '<S1>/Add1' incorporates:
   *  Constant: '<S1>/Constant'
   *  Constant: '<S1>/Constant1'
   *  Product: '<S1>/Product1'
   *  Product: '<S1>/Product4'
   *  Sum: '<S1>/Sum1'
   *  Sum: '<S1>/Sum2'
   *  Trigonometry: '<S1>/Sin'
   *  Trigonometry: '<S1>/Sin2'
   */
  rtb_Add2 = arm_sin_f32(rtb_Sum2 + 0.785398185F) * rtb_DeadZone4 + arm_sin_f32
    (rtb_Sum2 + 2.3561945F) * rtb_DeadZone5;

  /* DeadZone: '<S1>/Dead Zone' */
  if (rtb_Add2 > 5.0E-5F) {
    /* Outport: '<Root>/YOUT' */
    rtY.YOUT = rtb_Add2 - 5.0E-5F;
  } else if (rtb_Add2 >= -0.0005F) {
    /* Outport: '<Root>/YOUT' */
    rtY.YOUT = 0.0F;
  } else {
    /* Outport: '<Root>/YOUT' */
    rtY.YOUT = rtb_Add2 - -0.0005F;
  }

  /* End of DeadZone: '<S1>/Dead Zone' */

  /* Sum: '<S1>/Add2' incorporates:
   *  Constant: '<S1>/Constant'
   *  Constant: '<S1>/Constant1'
   *  Product: '<S1>/Product2'
   *  Product: '<S1>/Product3'
   *  Sum: '<S1>/Sum1'
   *  Sum: '<S1>/Sum2'
   *  Trigonometry: '<S1>/Sin1'
   *  Trigonometry: '<S1>/Sin3'
   */
  rtb_Add2 = arm_cos_f32(rtb_Sum2 + 0.785398185F) * rtb_DeadZone4 + arm_cos_f32
    (rtb_Sum2 + 2.3561945F) * rtb_DeadZone5;

  /* DeadZone: '<S1>/Dead Zone1' */
  if (rtb_Add2 > 5.0E-5F) {
    /* Outport: '<Root>/XOUT' */
    rtY.XOUT = rtb_Add2 - 5.0E-5F;
  } else if (rtb_Add2 >= -0.0005F) {
    /* Outport: '<Root>/XOUT' */
    rtY.XOUT = 0.0F;
  } else {
    /* Outport: '<Root>/XOUT' */
    rtY.XOUT = rtb_Add2 - -0.0005F;
  }

  /* End of DeadZone: '<S1>/Dead Zone1' */

  /* DeadZone: '<S1>/Dead Zone2' incorporates:
   *  Inport: '<Root>/X_ACCIN'
   */
  if (rtU.X_ACCIN > 0.1) {
    /* Outport: '<Root>/X_ACCOUT' */
    rtY.X_ACCOUT = rtU.X_ACCIN - 0.1;
  } else if (rtU.X_ACCIN >= -0.1) {
    /* Outport: '<Root>/X_ACCOUT' */
    rtY.X_ACCOUT = 0.0;
  } else {
    /* Outport: '<Root>/X_ACCOUT' */
    rtY.X_ACCOUT = rtU.X_ACCIN - -0.1;
  }

  /* End of DeadZone: '<S1>/Dead Zone2' */

  /* DeadZone: '<S1>/Dead Zone3' incorporates:
   *  Inport: '<Root>/Y_ACCIN'
   */
  if (rtU.Y_ACCIN > 0.1) {
    /* Outport: '<Root>/Y_ACCOUT' */
    rtY.Y_ACCOUT = rtU.Y_ACCIN - 0.1;
  } else if (rtU.Y_ACCIN >= -0.1) {
    /* Outport: '<Root>/Y_ACCOUT' */
    rtY.Y_ACCOUT = 0.0;
  } else {
    /* Outport: '<Root>/Y_ACCOUT' */
    rtY.Y_ACCOUT = rtU.Y_ACCIN - -0.1;
  }

  /* End of DeadZone: '<S1>/Dead Zone3' */
  /* End of Outputs for SubSystem: '<Root>/IM_TEST' */
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
