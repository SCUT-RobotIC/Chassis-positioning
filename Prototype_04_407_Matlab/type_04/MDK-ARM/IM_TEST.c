/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: IM_TEST.c
 *
 * Code generated for Simulink model 'IM_TEST'.
 *
 * Model version                  : 1.30
 * Simulink Coder version         : 9.9 (R2023a) 19-Nov-2022
 * C/C++ source code generated on : Thu Oct 19 18:20:59 2023
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "IM_TEST.h"
#include <math.h>
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
  real_T rtb_Cos;
  real_T rtb_Sin;

  /* Outport: '<Root>/Out1' incorporates:
   *  DiscreteIntegrator: '<S1>/Discrete-Time Integrator1'
   */
  rtY.Out1 = rtDW.DiscreteTimeIntegrator1_DSTATE;

  /* Outport: '<Root>/Out2' incorporates:
   *  DiscreteIntegrator: '<S1>/Discrete-Time Integrator'
   */
  rtY.Out2 = rtDW.DiscreteTimeIntegrator_DSTATE;

  /* Gain: '<S2>/Gain1' incorporates:
   *  Inport: '<Root>/In3'
   */
  rtb_Cos = 0.017453292519943295 * rtU.In3;

  /* Outport: '<Root>/Out3' */
  rtY.Out3 = rtb_Cos;

  /* Trigonometry: '<S1>/Sin' */
  rtb_Sin = sin(rtb_Cos);

  /* Trigonometry: '<S1>/Cos' */
  rtb_Cos = cos(rtb_Cos);

  /* Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator1' incorporates:
   *  Gain: '<S1>/Gain'
   *  Inport: '<Root>/In1'
   *  Inport: '<Root>/In2'
   *  Product: '<S1>/Product2'
   *  Product: '<S1>/Product3'
   *  Sum: '<S1>/Add1'
   */
  rtDW.DiscreteTimeIntegrator1_DSTATE += (rtU.In1 * rtb_Cos + -rtU.In2 * rtb_Sin)
    * 0.01;

  /* Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/In1'
   *  Inport: '<Root>/In2'
   *  Product: '<S1>/Product'
   *  Product: '<S1>/Product1'
   *  Sum: '<S1>/Add'
   */
  rtDW.DiscreteTimeIntegrator_DSTATE += (rtU.In1 * rtb_Sin + rtU.In2 * rtb_Cos) *
    0.01;
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
