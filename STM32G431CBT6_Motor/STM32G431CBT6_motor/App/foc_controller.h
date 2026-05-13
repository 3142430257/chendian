/*
 * File: foc_controller.h
 *
 * Code generated for Simulink model 'foc_controller'.
 *
 * Model version                  : 1.13
 * Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
 * C/C++ source code generated on : Wed May 13 09:15:59 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef foc_controller_h_
#define foc_controller_h_
#ifndef foc_controller_COMMON_INCLUDES_
#define foc_controller_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* foc_controller_COMMON_INCLUDES_ */

#include "foc_controller_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real32_T int_d;                      /* '<Root>/FOC_Core' */
  real32_T int_q;                      /* '<Root>/FOC_Core' */
  boolean_T int_d_not_empty;           /* '<Root>/FOC_Core' */
} DW_foc_controller_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real32_T In_I_a;                     /* '<Root>/In_I_a' */
  real32_T In_I_b;                     /* '<Root>/In_I_b' */
  real32_T In_theta_e;                 /* '<Root>/In_theta_e' */
  real32_T In_iq_ref;                  /* '<Root>/In_iq_ref' */
  real32_T In_V_bus;                   /* '<Root>/In_V_bus' */
  boolean_T In_reset;                  /* '<Root>/In_reset' */
} ExtU_foc_controller_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real32_T Out_duty_a;                 /* '<Root>/Out_duty_a' */
  real32_T Out_duty_b;                 /* '<Root>/Out_duty_b' */
  real32_T Out_duty_c;                 /* '<Root>/Out_duty_c' */
} ExtY_foc_controller_T;

/* Real-time Model Data Structure */
struct tag_RTM_foc_controller_T {
  const char_T * volatile errorStatus;
};

/* Block states (default storage) */
extern DW_foc_controller_T foc_controller_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_foc_controller_T foc_controller_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_foc_controller_T foc_controller_Y;

/* Model entry point functions */
extern void foc_controller_initialize(void);
extern void foc_controller_step(void);

/* Real-time Model object */
extern RT_MODEL_foc_controller_T *const foc_controller_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'foc_controller'
 * '<S1>'   : 'foc_controller/FOC_Core'
 */
#endif                                 /* foc_controller_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
