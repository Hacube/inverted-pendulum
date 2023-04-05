/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: PolySim_IPRW_simulink2.h
 *
 * Code generated for Simulink model 'PolySim_IPRW_simulink2'.
 *
 * Model version                  : 1.11
 * Simulink Coder version         : 9.8 (R2022b) 13-May-2022
 * C/C++ source code generated on : Tue Apr  4 15:26:28 2023
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#ifndef RTW_HEADER_PolySim_IPRW_simulink2_h_
#define RTW_HEADER_PolySim_IPRW_simulink2_h_
#ifndef PolySim_IPRW_simulink2_COMMON_INCLUDES_
#define PolySim_IPRW_simulink2_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#endif                             /* PolySim_IPRW_simulink2_COMMON_INCLUDES_ */

#include <string.h>

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                ((rtm)->Timing.t)
#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM RT_MODEL;

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  real_T Noise[3];                     /* '<S1>/Noise' */
  real_T Plus[3];                      /* '<S1>/Plus' */
  real_T NextOutput;                   /* '<S2>/White Noise' */
  uint32_T RandSeed;                   /* '<S2>/White Noise' */
} DW;

/* Continuous states (default storage) */
typedef struct {
  real_T Integrate_CSTATE[3];          /* '<S1>/Integrate' */
} X;

/* State derivatives (default storage) */
typedef struct {
  real_T Integrate_CSTATE[3];          /* '<S1>/Integrate' */
} XDot;

/* State disabled  */
typedef struct {
  boolean_T Integrate_CSTATE[3];       /* '<S1>/Integrate' */
} XDis;

#ifndef ODE3_INTG
#define ODE3_INTG

/* ODE3 Integration Data */
typedef struct {
  real_T *y;                           /* output */
  real_T *f[3];                        /* derivatives */
} ODE3_IntgData;

#endif

/* Constant parameters (default storage) */
typedef struct {
  /* Expression: A
   * Referenced by: '<S1>/A matrix'
   */
  real_T Amatrix_Gain[9];

  /* Expression: B
   * Referenced by: '<S1>/B Matrix'
   */
  real_T BMatrix_Gain[3];
} ConstP;

/* Real-time Model Data Structure */
struct tag_RTM {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;
  X *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  XDis *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[3];
  real_T odeF[3][3];
  ODE3_IntgData intgData;
  DW *dwork;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    int_T numContStates;
    int_T numPeriodicContStates;
    int_T numSampTimes;
  } Sizes;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    time_T stepSize0;
    uint32_T clockTick1;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

/* Constant parameters (default storage) */
extern const ConstP rtConstP;

/* Model entry point functions */
extern void PolySim_IPRW_simulink2_initialize(RT_MODEL *const rtM);
extern void PolySim_IPRW_simulink2_step(RT_MODEL *const rtM);

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<Root>/Scope' : Unused code path elimination
 * Block '<Root>/Voltage' : Unused code path elimination
 * Block '<S2>/Output' : Eliminated nontunable gain of 1
 */

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
 * '<Root>' : 'PolySim_IPRW_simulink2'
 * '<S1>'   : 'PolySim_IPRW_simulink2/System Dynamics'
 * '<S2>'   : 'PolySim_IPRW_simulink2/System Dynamics/Band-Limited White Noise'
 */
#endif                                /* RTW_HEADER_PolySim_IPRW_simulink2_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
