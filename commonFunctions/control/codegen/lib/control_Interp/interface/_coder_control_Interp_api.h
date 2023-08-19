//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// _coder_control_Interp_api.h
//
// Code generation for function 'control_Interp'
//

#ifndef _CODER_CONTROL_INTERP_API_H
#define _CODER_CONTROL_INTERP_API_H

// Include files
#include "emlrt.h"
#include "tmwtypes.h"
#include <algorithm>
#include <cstring>

// Type Definitions
typedef struct {
  real_T maxAngle;
  real_T minAngle;
} struct1_T;

typedef struct {
  struct1_T servo;
  real_T z_final;
} struct0_T;

typedef struct {
  real_T deltaZ;
  real_T Z[301];
  real_T Vz[602];
} struct3_T;

typedef struct {
  struct3_T reference;
  real_T N_forward;
  boolean_T flagFirstControlABK;
  real_T filter_coeff;
  real_T filterMinAltitude;
  real_T filterMaxAltitude;
  real_T criticalAltitude;
  real_T filter_coeff0;
  char_T interpType;
} struct2_T;

// Variable Declarations
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

// Function Declarations
real_T control_Interp(real_T z, real_T Vz, struct0_T *settings,
                      struct2_T *contSettings, real_T alpha0_old);

void control_Interp_api(const mxArray *const prhs[5], int32_T nlhs,
                        const mxArray *plhs[2]);

void control_Interp_atexit();

void control_Interp_initialize();

void control_Interp_terminate();

void control_Interp_xil_shutdown();

void control_Interp_xil_terminate();

#endif
// End of code generation (_coder_control_Interp_api.h)
