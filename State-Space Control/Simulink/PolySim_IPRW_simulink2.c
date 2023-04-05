

/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: PolySim_IPRW_simulink2.c
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

#include "PolySim_IPRW_simulink2.h"
#include "rtwtypes.h"
#include <emmintrin.h>
#include <math.h>

/* Private macros used by the generated code to access rtModel */
#ifndef rtmIsMajorTimeStep
#define rtmIsMajorTimeStep(rtm)        (((rtm)->Timing.simTimeStep) == MAJOR_TIME_STEP)
#endif

#ifndef rtmIsMinorTimeStep
#define rtmIsMinorTimeStep(rtm)        (((rtm)->Timing.simTimeStep) == MINOR_TIME_STEP)
#endif

#ifndef rtmSetTPtr
#define rtmSetTPtr(rtm, val)           ((rtm)->Timing.t = (val))
#endif

extern real_T rt_urand_Upu32_Yd_f_pw_snf(uint32_T *u);
extern real_T rt_nrand_Upu32_Yd_f_pw_snf(uint32_T *u);

/* private model entry point functions */
extern void PolySim_IPRW_simulink2_derivatives(RT_MODEL *const rtM);

/*
 * This function updates continuous states using the ODE3 fixed-step
 * solver algorithm
 */
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si , RT_MODEL *const
  rtM)
{
  /* Solver Matrices */
  static const real_T rt_ODE3_A[3] = {
    1.0/2.0, 3.0/4.0, 1.0
  };

  static const real_T rt_ODE3_B[3][3] = {
    { 1.0/2.0, 0.0, 0.0 },

    { 0.0, 3.0/4.0, 0.0 },

    { 2.0/9.0, 1.0/3.0, 4.0/9.0 }
  };

  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE3_IntgData *id = (ODE3_IntgData *)rtsiGetSolverData(si);
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T hB[3];
  int_T i;
  int_T nXc = 3;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  /* Save the state values at time t in y, we'll use x as ynew. */
  (void) memcpy(y, x,
                (uint_T)nXc*sizeof(real_T));

  /* Assumes that rtsiSetT and ModelOutputs are up-to-date */
  /* f0 = f(t,y) */
  rtsiSetdX(si, f0);
  PolySim_IPRW_simulink2_derivatives(rtM);

  /* f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*)); */
  hB[0] = h * rt_ODE3_B[0][0];
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[0]);
  rtsiSetdX(si, f1);
  PolySim_IPRW_simulink2_step(rtM);
  PolySim_IPRW_simulink2_derivatives(rtM);

  /* f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*)); */
  for (i = 0; i <= 1; i++) {
    hB[i] = h * rt_ODE3_B[1][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[1]);
  rtsiSetdX(si, f2);
  PolySim_IPRW_simulink2_step(rtM);
  PolySim_IPRW_simulink2_derivatives(rtM);

  /* tnew = t + hA(3);
     ynew = y + f*hB(:,3); */
  for (i = 0; i <= 2; i++) {
    hB[i] = h * rt_ODE3_B[2][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1] + f2[i]*hB[2]);
  }

  rtsiSetT(si, tnew);
  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

real_T rt_urand_Upu32_Yd_f_pw_snf(uint32_T *u)
{
  uint32_T hi;
  uint32_T lo;

  /* Uniform random number generator (random number between 0 and 1)

     #define IA      16807                      magic multiplier = 7^5
     #define IM      2147483647                 modulus = 2^31-1
     #define IQ      127773                     IM div IA
     #define IR      2836                       IM modulo IA
     #define S       4.656612875245797e-10      reciprocal of 2^31-1
     test = IA * (seed % IQ) - IR * (seed/IQ)
     seed = test < 0 ? (test + IM) : test
     return (seed*S)
   */
  lo = *u % 127773U * 16807U;
  hi = *u / 127773U * 2836U;
  if (lo < hi) {
    *u = 2147483647U - (hi - lo);
  } else {
    *u = lo - hi;
  }

  return (real_T)*u * 4.6566128752457969E-10;
}

real_T rt_nrand_Upu32_Yd_f_pw_snf(uint32_T *u)
{
  real_T si;
  real_T sr;
  real_T y;

  /* Normal (Gaussian) random number generator */
  do {
    sr = 2.0 * rt_urand_Upu32_Yd_f_pw_snf(u) - 1.0;
    si = 2.0 * rt_urand_Upu32_Yd_f_pw_snf(u) - 1.0;
    si = sr * sr + si * si;
  } while (si > 1.0);

  y = sqrt(-2.0 * log(si) / si) * sr;
  return y;
}

/* Model step function */
void PolySim_IPRW_simulink2_step(RT_MODEL *const rtM)
{
  DW *rtDW = rtM->dwork;
  X *rtX = rtM->contStates;
  real_T rtb_LQR_Gain_0;
  int32_T i;
  if (rtmIsMajorTimeStep(rtM)) {
    /* set solver stop time */
    rtsiSetSolverStopTime(&rtM->solverInfo,((rtM->Timing.clockTick0+1)*
      rtM->Timing.stepSize0));
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(rtM)) {
    rtM->Timing.t[0] = rtsiGetT(&rtM->solverInfo);
  }

  /* Gain: '<Root>/LQR_Gain' incorporates:
   *  Integrator: '<S1>/Integrate'
   */
  rtb_LQR_Gain_0 = (162.10237269327661 * rtX->Integrate_CSTATE[0] +
                    17.616827548651166 * rtX->Integrate_CSTATE[1]) +
    2.0346154610834 * rtX->Integrate_CSTATE[2];
  if (rtmIsMajorTimeStep(rtM)) {
    /* Gain: '<S1>/Noise' incorporates:
     *  RandomNumber: '<S2>/White Noise'
     */
    rtDW->Noise[0] = rtDW->NextOutput;
    rtDW->Noise[1] = 0.0 * rtDW->NextOutput;
    rtDW->Noise[2] = 0.0 * rtDW->NextOutput;
  }

  for (i = 0; i <= 0; i += 2) {
    __m128d tmp;

    /* Sum: '<S1>/Plus' incorporates:
     *  Gain: '<S1>/A matrix'
     */
    tmp = _mm_loadu_pd(&rtDW->Noise[i]);

    /* Sum: '<S1>/Plus' incorporates:
     *  Gain: '<Root>/LQR_Gain'
     *  Gain: '<S1>/A matrix'
     *  Gain: '<S1>/B Matrix'
     *  Integrator: '<S1>/Integrate'
     */
    _mm_storeu_pd(&rtDW->Plus[i], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
      (_mm_loadu_pd(&rtConstP.Amatrix_Gain[i + 3]), _mm_set1_pd
       (rtX->Integrate_CSTATE[1])), _mm_mul_pd(_mm_loadu_pd
      (&rtConstP.Amatrix_Gain[i]), _mm_set1_pd(rtX->Integrate_CSTATE[0]))),
      _mm_mul_pd(_mm_loadu_pd(&rtConstP.Amatrix_Gain[i + 6]), _mm_set1_pd
                 (rtX->Integrate_CSTATE[2]))), _mm_add_pd(_mm_mul_pd
      (_mm_loadu_pd(&rtConstP.BMatrix_Gain[i]), _mm_set1_pd(rtb_LQR_Gain_0)),
      tmp)));
  }

  for (i = 2; i < 3; i++) {
    /* Sum: '<S1>/Plus' incorporates:
     *  Gain: '<Root>/LQR_Gain'
     *  Gain: '<S1>/A matrix'
     *  Gain: '<S1>/B Matrix'
     *  Integrator: '<S1>/Integrate'
     */
    rtDW->Plus[i] = ((rtConstP.Amatrix_Gain[i + 3] * rtX->Integrate_CSTATE[1] +
                      rtConstP.Amatrix_Gain[i] * rtX->Integrate_CSTATE[0]) +
                     rtConstP.Amatrix_Gain[i + 6] * rtX->Integrate_CSTATE[2]) +
      (rtConstP.BMatrix_Gain[i] * rtb_LQR_Gain_0 + rtDW->Noise[i]);
  }

  if (rtmIsMajorTimeStep(rtM)) {
    if (rtmIsMajorTimeStep(rtM)) {
      /* Update for RandomNumber: '<S2>/White Noise' */
      rtDW->NextOutput = rt_nrand_Upu32_Yd_f_pw_snf(&rtDW->RandSeed);
    }
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep(rtM)) {
    rt_ertODEUpdateContinuousStates(&rtM->solverInfo, rtM);

    /* Update absolute time for base rate */
    /* The "clockTick0" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick0"
     * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
     * overflow during the application lifespan selected.
     */
    ++rtM->Timing.clockTick0;
    rtM->Timing.t[0] = rtsiGetSolverStopTime(&rtM->solverInfo);

    {
      /* Update absolute timer for sample time: [0.1s, 0.0s] */
      /* The "clockTick1" counts the number of times the code of this task has
       * been executed. The resolution of this integer timer is 0.1, which is the step size
       * of the task. Size of "clockTick1" ensures timer will not overflow during the
       * application lifespan selected.
       */
      rtM->Timing.clockTick1++;
    }
  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void PolySim_IPRW_simulink2_derivatives(RT_MODEL *const rtM)
{
  DW *rtDW = rtM->dwork;
  XDot *_rtXdot;
  _rtXdot = ((XDot *) rtM->derivs);

  /* Derivatives for Integrator: '<S1>/Integrate' */
  _rtXdot->Integrate_CSTATE[0] = rtDW->Plus[0];
  _rtXdot->Integrate_CSTATE[1] = rtDW->Plus[1];
  _rtXdot->Integrate_CSTATE[2] = rtDW->Plus[2];
}

/* Model initialize function */
void PolySim_IPRW_simulink2_initialize(RT_MODEL *const rtM)
{
  DW *rtDW = rtM->dwork;
  X *rtX = rtM->contStates;

  /* Registration code */
  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&rtM->solverInfo, &rtM->Timing.simTimeStep);
    rtsiSetTPtr(&rtM->solverInfo, &rtmGetTPtr(rtM));
    rtsiSetStepSizePtr(&rtM->solverInfo, &rtM->Timing.stepSize0);
    rtsiSetdXPtr(&rtM->solverInfo, &rtM->derivs);
    rtsiSetContStatesPtr(&rtM->solverInfo, (real_T **) &rtM->contStates);
    rtsiSetNumContStatesPtr(&rtM->solverInfo, &rtM->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&rtM->solverInfo,
      &rtM->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&rtM->solverInfo,
      &rtM->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&rtM->solverInfo,
      &rtM->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&rtM->solverInfo, (&rtmGetErrorStatus(rtM)));
    rtsiSetRTModelPtr(&rtM->solverInfo, rtM);
  }

  rtsiSetSimTimeStep(&rtM->solverInfo, MAJOR_TIME_STEP);
  rtM->intgData.y = rtM->odeY;
  rtM->intgData.f[0] = rtM->odeF[0];
  rtM->intgData.f[1] = rtM->odeF[1];
  rtM->intgData.f[2] = rtM->odeF[2];
  rtM->contStates = ((X *) rtX);
  rtsiSetSolverData(&rtM->solverInfo, (void *)&rtM->intgData);
  rtsiSetIsMinorTimeStepWithModeChange(&rtM->solverInfo, false);
  rtsiSetSolverName(&rtM->solverInfo,"ode3");
  rtmSetTPtr(rtM, &rtM->Timing.tArray[0]);
  rtM->Timing.stepSize0 = 0.1;

  /* InitializeConditions for Integrator: '<S1>/Integrate' */
  rtX->Integrate_CSTATE[0] = 0.0;
  rtX->Integrate_CSTATE[1] = 0.0;
  rtX->Integrate_CSTATE[2] = 0.0;

  /* InitializeConditions for RandomNumber: '<S2>/White Noise' */
  rtDW->RandSeed = 1529675776U;
  rtDW->NextOutput = rt_nrand_Upu32_Yd_f_pw_snf(&rtDW->RandSeed);
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
