/*
 * File: foc_controller.c
 *
 * Code generated for Simulink model 'foc_controller'.
 *
 * Model version                  : 1.13
 * Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
 * C/C++ source code generated on : Tue Apr 21 17:03:24 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "foc_controller.h"
#include <math.h>
#include "rtwtypes.h"

/* Block states (default storage) */
DW_foc_controller_T foc_controller_DW;

/* External inputs (root inport signals with default storage) */
ExtU_foc_controller_T foc_controller_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_foc_controller_T foc_controller_Y;

/* Real-time model */
static RT_MODEL_foc_controller_T foc_controller_M_;
RT_MODEL_foc_controller_T *const foc_controller_M = &foc_controller_M_;

/* Model step function */
void foc_controller_step(void)
{
  real32_T I_c;
  real32_T V_lim;
  real32_T err_d;
  real32_T err_q;
  real32_T ialpha;
  real32_T ibeta;
  real32_T sin_e;
  real32_T v_mag;

  /* MATLAB Function: '<Root>/FOC_Core' incorporates:
   *  Inport: '<Root>/In_I_a'
   *  Inport: '<Root>/In_I_b'
   *  Inport: '<Root>/In_V_bus'
   *  Inport: '<Root>/In_iq_ref'
   *  Inport: '<Root>/In_reset'
   *  Inport: '<Root>/In_theta_e'
   */
  if ((!foc_controller_DW.int_d_not_empty) || foc_controller_U.In_reset) {
    foc_controller_DW.int_d = 0.0F;
    foc_controller_DW.int_d_not_empty = true;
    foc_controller_DW.int_q = 0.0F;
  }

  I_c = -(foc_controller_U.In_I_a + foc_controller_U.In_I_b);
  ialpha = ((2.0F * foc_controller_U.In_I_a - foc_controller_U.In_I_b) - I_c) /
    3.0F;
  ibeta = (foc_controller_U.In_I_b - I_c) * 1.73205078F / 3.0F;
  I_c = cosf(foc_controller_U.In_theta_e);
  sin_e = sinf(foc_controller_U.In_theta_e);
  V_lim = foc_controller_U.In_V_bus / 1.73205078F;
  err_d = 0.0F - (ialpha * I_c + ibeta * sin_e);
  err_q = foc_controller_U.In_iq_ref - (-ialpha * sin_e + ibeta * I_c);
  ialpha = 0.628318489F * err_d + foc_controller_DW.int_d;
  ibeta = 0.628318489F * err_q + foc_controller_DW.int_q;
  v_mag = sqrtf(ialpha * ialpha + ibeta * ibeta);
  if (v_mag > V_lim) {
    ialpha = ialpha * V_lim / v_mag;
    ibeta = ibeta * V_lim / v_mag;
  } else {
    foc_controller_DW.int_d += 0.424115032F * err_d;
    foc_controller_DW.int_q += 0.424115032F * err_q;
  }

  V_lim = ialpha * I_c - ibeta * sin_e;
  sin_e = ialpha * sin_e + ibeta * I_c;
  I_c = -0.5F * V_lim + 0.866025388F * sin_e;
  sin_e = -0.5F * V_lim - 0.866025388F * sin_e;
  err_d = -(fmaxf(fmaxf(V_lim, I_c), sin_e) + fminf(fminf(V_lim, I_c), sin_e)) /
    2.0F;

  /* Outport: '<Root>/Out_duty_a' incorporates:
   *  Inport: '<Root>/In_V_bus'
   *  MATLAB Function: '<Root>/FOC_Core'
   */
  foc_controller_Y.Out_duty_a = fmaxf(0.02F, fminf(0.98F, (V_lim + err_d) /
    foc_controller_U.In_V_bus + 0.5F));

  /* Outport: '<Root>/Out_duty_b' incorporates:
   *  Inport: '<Root>/In_V_bus'
   *  MATLAB Function: '<Root>/FOC_Core'
   */
  foc_controller_Y.Out_duty_b = fmaxf(0.02F, fminf(0.98F, (I_c + err_d) /
    foc_controller_U.In_V_bus + 0.5F));

  /* Outport: '<Root>/Out_duty_c' incorporates:
   *  Inport: '<Root>/In_V_bus'
   *  MATLAB Function: '<Root>/FOC_Core'
   */
  foc_controller_Y.Out_duty_c = fmaxf(0.02F, fminf(0.98F, (sin_e + err_d) /
    foc_controller_U.In_V_bus + 0.5F));
}

/* Model initialize function */
void foc_controller_initialize(void)
{
  /* (no initialization code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
