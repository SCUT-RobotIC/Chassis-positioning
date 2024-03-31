/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: IM_TEST.c
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

#include "IM_TEST.h"
#include "rtwtypes.h"
#include "mw_cmsis.h"
#include <math.h>
#include <stddef.h>
#define NumBitsPerChar                 8U

/* Exported block signals */
real_T yraw;                           /* '<S1>/Add1' */
real_T xraw;                           /* '<S1>/Add2' */

/* Block signals and states (default storage) */
DW rtDW;

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

/* Real-time model */
static RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;
extern real_T rt_roundd_snf(real_T u);
static void ForEachSubsystem(int32_T NumIters, const real_T *rtu_In1, const
  real_T *rtu_In2, real_T *rty_Out1, DW_ForEachSubsystem localDW[1]);

#define NOT_USING_NONFINITE_LITERALS   1

extern real_T rtInf;
extern real_T rtMinusInf;
extern real_T rtNaN;
extern real32_T rtInfF;
extern real32_T rtMinusInfF;
extern real32_T rtNaNF;
static void rt_InitInfAndNaN(size_t realSize);
static boolean_T rtIsInf(real_T value);
static boolean_T rtIsInfF(real32_T value);
static boolean_T rtIsNaN(real_T value);
static boolean_T rtIsNaNF(real32_T value);
typedef struct {
  struct {
    uint32_T wordH;
    uint32_T wordL;
  } words;
} BigEndianIEEEDouble;

typedef struct {
  struct {
    uint32_T wordL;
    uint32_T wordH;
  } words;
} LittleEndianIEEEDouble;

typedef struct {
  union {
    real32_T wordLreal;
    uint32_T wordLuint;
  } wordL;
} IEEESingle;

real_T rtInf;
real_T rtMinusInf;
real_T rtNaN;
real32_T rtInfF;
real32_T rtMinusInfF;
real32_T rtNaNF;
static real_T rtGetInf(void);
static real32_T rtGetInfF(void);
static real_T rtGetMinusInf(void);
static real32_T rtGetMinusInfF(void);
static real_T rtGetNaN(void);
static real32_T rtGetNaNF(void);

/*
 * Initialize the rtInf, rtMinusInf, and rtNaN needed by the
 * generated code. NaN is initialized as non-signaling. Assumes IEEE.
 */
static void rt_InitInfAndNaN(size_t realSize)
{
  (void) (realSize);
  rtNaN = rtGetNaN();
  rtNaNF = rtGetNaNF();
  rtInf = rtGetInf();
  rtInfF = rtGetInfF();
  rtMinusInf = rtGetMinusInf();
  rtMinusInfF = rtGetMinusInfF();
}

/* Test if value is infinite */
static boolean_T rtIsInf(real_T value)
{
  return (boolean_T)((value==rtInf || value==rtMinusInf) ? 1U : 0U);
}

/* Test if single-precision value is infinite */
static boolean_T rtIsInfF(real32_T value)
{
  return (boolean_T)(((value)==rtInfF || (value)==rtMinusInfF) ? 1U : 0U);
}

/* Test if value is not a number */
static boolean_T rtIsNaN(real_T value)
{
  boolean_T result = (boolean_T) 0;
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  if (bitsPerReal == 32U) {
    result = rtIsNaNF((real32_T)value);
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;

    tmpVal.fltVal = value;
    result = (boolean_T)((tmpVal.bitVal.words.wordH & 0x7FF00000) == 0x7FF00000 &&
                         ( (tmpVal.bitVal.words.wordH & 0x000FFFFF) != 0 ||
                          (tmpVal.bitVal.words.wordL != 0) ));
  }

  return result;
}

/* Test if single-precision value is not a number */
static boolean_T rtIsNaNF(real32_T value)
{
  IEEESingle tmp;
  tmp.wordL.wordLreal = value;
  return (boolean_T)( (tmp.wordL.wordLuint & 0x7F800000) == 0x7F800000 &&
                     (tmp.wordL.wordLuint & 0x007FFFFF) != 0 );
}

/*
 * Initialize rtInf needed by the generated code.
 * Inf is initialized as non-signaling. Assumes IEEE.
 */
static real_T rtGetInf(void)
{
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  real_T inf = 0.0;
  if (bitsPerReal == 32U) {
    inf = rtGetInfF();
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;

    tmpVal.bitVal.words.wordH = 0x7FF00000U;
    tmpVal.bitVal.words.wordL = 0x00000000U;
    inf = tmpVal.fltVal;
  }

  return inf;
}

/*
 * Initialize rtInfF needed by the generated code.
 * Inf is initialized as non-signaling. Assumes IEEE.
 */
static real32_T rtGetInfF(void)
{
  IEEESingle infF;
  infF.wordL.wordLuint = 0x7F800000U;
  return infF.wordL.wordLreal;
}

/*
 * Initialize rtMinusInf needed by the generated code.
 * Inf is initialized as non-signaling. Assumes IEEE.
 */
static real_T rtGetMinusInf(void)
{
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  real_T minf = 0.0;
  if (bitsPerReal == 32U) {
    minf = rtGetMinusInfF();
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;

    tmpVal.bitVal.words.wordH = 0xFFF00000U;
    tmpVal.bitVal.words.wordL = 0x00000000U;
    minf = tmpVal.fltVal;
  }

  return minf;
}

/*
 * Initialize rtMinusInfF needed by the generated code.
 * Inf is initialized as non-signaling. Assumes IEEE.
 */
static real32_T rtGetMinusInfF(void)
{
  IEEESingle minfF;
  minfF.wordL.wordLuint = 0xFF800000U;
  return minfF.wordL.wordLreal;
}

/*
 * Initialize rtNaN needed by the generated code.
 * NaN is initialized as non-signaling. Assumes IEEE.
 */
static real_T rtGetNaN(void)
{
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  real_T nan = 0.0;
  if (bitsPerReal == 32U) {
    nan = rtGetNaNF();
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;

    tmpVal.bitVal.words.wordH = 0xFFF80000U;
    tmpVal.bitVal.words.wordL = 0x00000000U;
    nan = tmpVal.fltVal;
  }

  return nan;
}

/*
 * Initialize rtNaNF needed by the generated code.
 * NaN is initialized as non-signaling. Assumes IEEE.
 */
static real32_T rtGetNaNF(void)
{
  IEEESingle nanF = { { 0.0F } };

  nanF.wordL.wordLuint = 0xFFC00000U;
  return nanF.wordL.wordLreal;
}

/*
 * Output and update for iterator system:
 *    '<S6>/For Each Subsystem'
 *    '<S8>/For Each Subsystem'
 */
static void ForEachSubsystem(int32_T NumIters, const real_T *rtu_In1, const
  real_T *rtu_In2, real_T *rty_Out1, DW_ForEachSubsystem localDW[1])
{
  /* local scratch DWork variables */
  int32_T ForEach_itr;
  int_T idxDelay;

  /* Outputs for Iterator SubSystem: '<S6>/For Each Subsystem' incorporates:
   *  ForEach: '<S7>/For Each'
   */
  for (ForEach_itr = 0; ForEach_itr < NumIters; ForEach_itr++) {
    /* Delay: '<S7>/Variable Integer Delay' */
    if ((rtu_In2[ForEach_itr] < 1.0) || rtIsNaN(rtu_In2[ForEach_itr])) {
      /* ForEachSliceAssignment generated from: '<S7>/Out1' */
      rty_Out1[ForEach_itr] = rtu_In1[ForEach_itr];
    } else {
      uint32_T tmp;
      if (rtu_In2[ForEach_itr] > 4096.0) {
        tmp = 4096U;
      } else {
        tmp = (uint32_T)rtu_In2[ForEach_itr];
      }

      /* ForEachSliceAssignment generated from: '<S7>/Out1' */
      rty_Out1[ForEach_itr] = localDW[ForEach_itr].
        CoreSubsys.VariableIntegerDelay_DSTATE[4096U - tmp];
    }

    /* End of Delay: '<S7>/Variable Integer Delay' */

    /* Update for Delay: '<S7>/Variable Integer Delay' */
    for (idxDelay = 0; idxDelay < 4095; idxDelay++) {
      localDW[ForEach_itr].CoreSubsys.VariableIntegerDelay_DSTATE[idxDelay] =
        localDW[ForEach_itr].CoreSubsys.VariableIntegerDelay_DSTATE[idxDelay + 1];
    }

    localDW[ForEach_itr].CoreSubsys.VariableIntegerDelay_DSTATE[4095] =
      rtu_In1[ForEach_itr];

    /* End of Update for Delay: '<S7>/Variable Integer Delay' */
  }

  /* End of Outputs for SubSystem: '<S6>/For Each Subsystem' */
}

real_T rt_roundd_snf(real_T u)
{
  real_T y;
  if (fabs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

/* Model step function */
void IM_TEST_step(void)
{
  /* local block i/o variables */
  real_T rtb_ImpAsg_InsertedFor_Out1_at_;
  real_T rtb_ImpAsg_InsertedFor_Out1_a_i;
  real_T DiscreteTimeIntegrator;
  real_T rtb_MathFunction;
  real_T rtb_RoundingFunction;
  real_T rtb_RoundingFunction_i;
  real_T rtb_Sum4;
  real_T rtb_Sum4_i;
  real32_T rtb_Sum2_mk;

  /* Outputs for Atomic SubSystem: '<Root>/IM_TEST' */
  /* Math: '<S8>/Math Function' incorporates:
   *  Gain: '<S8>/Gain1'
   *
   * About '<S8>/Math Function':
   *  Operator: reciprocal
   */
  rtb_MathFunction = 1.0 / (500.0 * rtDW.Probe_f[0]);

  /* Rounding: '<S8>/Rounding Function' */
  rtb_RoundingFunction = rt_roundd_snf(rtb_MathFunction);

  /* Gain: '<S2>/Gain1' incorporates:
   *  Inport: '<Root>/In1'
   */
  rtb_Sum2_mk = 0.0174532924F * rtU.DEG;

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
  yraw = arm_sin_f32(rtb_Sum2_mk + 0.785398185F) * rtU.W1 + arm_sin_f32
    (rtb_Sum2_mk + 2.3561945F) * rtU.W2;

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
  xraw = arm_cos_f32(rtb_Sum2_mk + 0.785398185F) * rtU.W1 + arm_cos_f32
    (rtb_Sum2_mk + 2.3561945F) * rtU.W2;

  /* DiscreteIntegrator: '<S8>/Discrete-Time Integrator' */
  if (rtDW.DiscreteTimeIntegrator_SYSTEM_E != 0) {
    /* DiscreteIntegrator: '<S8>/Discrete-Time Integrator' */
    DiscreteTimeIntegrator = rtDW.DiscreteTimeIntegrator_DSTATE;
  } else {
    /* DiscreteIntegrator: '<S8>/Discrete-Time Integrator' incorporates:
     *  MATLAB Function: '<S1>/MATLAB Function'
     */
    DiscreteTimeIntegrator = 5.0E-6 * yraw + rtDW.DiscreteTimeIntegrator_DSTATE;
  }

  /* End of DiscreteIntegrator: '<S8>/Discrete-Time Integrator' */

  /* Sum: '<S8>/Sum4' incorporates:
   *  Gain: '<S8>/Gain2'
   *  MATLAB Function: '<S1>/MATLAB Function'
   */
  rtb_Sum4 = 0.0 * yraw + rtb_RoundingFunction;

  /* Outputs for Iterator SubSystem: '<S8>/For Each Subsystem' */
  ForEachSubsystem(1, &DiscreteTimeIntegrator, &rtb_Sum4,
                   &rtb_ImpAsg_InsertedFor_Out1_at_, rtDW.ForEachSubsystem_l);

  /* End of Outputs for SubSystem: '<S8>/For Each Subsystem' */

  /* Switch: '<S8>/Switch' incorporates:
   *  Constant: '<S8>/Constant2'
   *  Gain: '<S8>/Gain'
   *  MATLAB Function: '<S1>/MATLAB Function'
   *  Product: '<S8>/Product1'
   *  RelationalOperator: '<S8>/Relational Operator'
   *  Sum: '<S8>/Sum'
   *  Sum: '<S8>/Sum1'
   *  Sum: '<S8>/Sum2'
   *  Sum: '<S8>/Sum3'
   *  UnitDelay: '<S8>/Unit Delay'
   *  UnitDelay: '<S8>/Unit Delay1'
   */
  if (rtDW.UnitDelay_DSTATE + 1.0 >= rtb_RoundingFunction) {
    rtb_MathFunction = (rtb_MathFunction - rtb_RoundingFunction) * yraw /
      rtb_MathFunction + (DiscreteTimeIntegrator -
                          rtb_ImpAsg_InsertedFor_Out1_at_) * 500.0;
  } else {
    rtb_MathFunction = rtDW.UnitDelay1_DSTATE;
  }

  /* End of Switch: '<S8>/Switch' */

  /* DiscreteIntegrator: '<S6>/Discrete-Time Integrator' */
  if (rtDW.DiscreteTimeIntegrator_SYSTEM_g != 0) {
    /* DiscreteIntegrator: '<S6>/Discrete-Time Integrator' */
    rtb_RoundingFunction = rtDW.DiscreteTimeIntegrator_DSTATE_d;
  } else {
    /* DiscreteIntegrator: '<S6>/Discrete-Time Integrator' incorporates:
     *  MATLAB Function: '<S1>/MATLAB Function'
     */
    rtb_RoundingFunction = 5.0E-6 * xraw + rtDW.DiscreteTimeIntegrator_DSTATE_d;
  }

  /* End of DiscreteIntegrator: '<S6>/Discrete-Time Integrator' */

  /* Math: '<S6>/Math Function' incorporates:
   *  Gain: '<S6>/Gain1'
   *
   * About '<S6>/Math Function':
   *  Operator: reciprocal
   */
  rtb_Sum4 = 1.0 / (500.0 * rtDW.Probe[0]);

  /* Rounding: '<S6>/Rounding Function' */
  rtb_RoundingFunction_i = rt_roundd_snf(rtb_Sum4);

  /* Sum: '<S6>/Sum4' incorporates:
   *  Gain: '<S6>/Gain2'
   *  MATLAB Function: '<S1>/MATLAB Function'
   */
  rtb_Sum4_i = 0.0 * xraw + rtb_RoundingFunction_i;

  /* Outputs for Iterator SubSystem: '<S6>/For Each Subsystem' */
  ForEachSubsystem(1, &rtb_RoundingFunction, &rtb_Sum4_i,
                   &rtb_ImpAsg_InsertedFor_Out1_a_i, rtDW.ForEachSubsystem_o);

  /* End of Outputs for SubSystem: '<S6>/For Each Subsystem' */

  /* Switch: '<S6>/Switch' incorporates:
   *  Constant: '<S6>/Constant2'
   *  Gain: '<S6>/Gain'
   *  MATLAB Function: '<S1>/MATLAB Function'
   *  Product: '<S6>/Product1'
   *  RelationalOperator: '<S6>/Relational Operator'
   *  Sum: '<S6>/Sum'
   *  Sum: '<S6>/Sum1'
   *  Sum: '<S6>/Sum2'
   *  Sum: '<S6>/Sum3'
   *  UnitDelay: '<S6>/Unit Delay'
   *  UnitDelay: '<S6>/Unit Delay1'
   */
  if (rtDW.UnitDelay_DSTATE_i + 1.0 >= rtb_RoundingFunction_i) {
    rtb_Sum4 = (rtb_Sum4 - rtb_RoundingFunction_i) * xraw / rtb_Sum4 +
      (rtb_RoundingFunction - rtb_ImpAsg_InsertedFor_Out1_a_i) * 500.0;
  } else {
    rtb_Sum4 = rtDW.UnitDelay1_DSTATE_p;
  }

  /* End of Switch: '<S6>/Switch' */

  /* Update for DiscreteIntegrator: '<S8>/Discrete-Time Integrator' incorporates:
   *  MATLAB Function: '<S1>/MATLAB Function'
   */
  rtDW.DiscreteTimeIntegrator_SYSTEM_E = 0U;
  rtDW.DiscreteTimeIntegrator_DSTATE = 5.0E-6 * yraw + DiscreteTimeIntegrator;

  /* Update for UnitDelay: '<S8>/Unit Delay' incorporates:
   *  Constant: '<S8>/Constant2'
   *  Sum: '<S8>/Sum3'
   */
  rtDW.UnitDelay_DSTATE++;

  /* Update for UnitDelay: '<S8>/Unit Delay1' */
  rtDW.UnitDelay1_DSTATE = rtb_MathFunction;

  /* Update for DiscreteIntegrator: '<S6>/Discrete-Time Integrator' incorporates:
   *  MATLAB Function: '<S1>/MATLAB Function'
   */
  rtDW.DiscreteTimeIntegrator_SYSTEM_g = 0U;
  rtDW.DiscreteTimeIntegrator_DSTATE_d = 5.0E-6 * xraw + rtb_RoundingFunction;

  /* Update for UnitDelay: '<S6>/Unit Delay' incorporates:
   *  Constant: '<S6>/Constant2'
   *  Sum: '<S6>/Sum3'
   */
  rtDW.UnitDelay_DSTATE_i++;

  /* Update for UnitDelay: '<S6>/Unit Delay1' */
  rtDW.UnitDelay1_DSTATE_p = rtb_Sum4;

  /* End of Outputs for SubSystem: '<Root>/IM_TEST' */

  /* Outport: '<Root>/YOUT' */
  rtY.YOUT = rtb_MathFunction;

  /* Outport: '<Root>/XOUT' */
  rtY.XOUT = rtb_Sum4;

  /* Outputs for Atomic SubSystem: '<Root>/IM_TEST' */
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
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* SystemInitialize for Atomic SubSystem: '<Root>/IM_TEST' */
  /* Start for Probe: '<S6>/Probe' */
  rtDW.Probe[0] = 1.0E-5;
  rtDW.Probe[1] = 0.0;

  /* Start for Probe: '<S8>/Probe' */
  rtDW.Probe_f[0] = 1.0E-5;
  rtDW.Probe_f[1] = 0.0;

  /* End of SystemInitialize for SubSystem: '<Root>/IM_TEST' */

  /* Enable for Atomic SubSystem: '<Root>/IM_TEST' */
  /* Enable for DiscreteIntegrator: '<S8>/Discrete-Time Integrator' */
  rtDW.DiscreteTimeIntegrator_SYSTEM_E = 1U;

  /* Enable for DiscreteIntegrator: '<S6>/Discrete-Time Integrator' */
  rtDW.DiscreteTimeIntegrator_SYSTEM_g = 1U;

  /* End of Enable for SubSystem: '<Root>/IM_TEST' */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
