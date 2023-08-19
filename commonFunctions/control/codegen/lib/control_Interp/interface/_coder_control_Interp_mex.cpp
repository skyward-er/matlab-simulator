//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// _coder_control_Interp_mex.cpp
//
// Code generation for function 'control_Interp'
//

// Include files
#include "_coder_control_Interp_mex.h"
#include "_coder_control_Interp_api.h"

// Function Definitions
void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  mexAtExit(&control_Interp_atexit);
  // Module initialization.
  control_Interp_initialize();
  // Dispatch the entry-point.
  unsafe_control_Interp_mexFunction(nlhs, plhs, nrhs, prhs);
  // Module termination.
  control_Interp_terminate();
}

emlrtCTX mexFunctionCreateRootTLS()
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1,
                           NULL, "windows-1252", true);
  return emlrtRootTLSGlobal;
}

void unsafe_control_Interp_mexFunction(int32_T nlhs, mxArray *plhs[2],
                                       int32_T nrhs, const mxArray *prhs[5])
{
  emlrtStack st = {
      NULL, // site
      NULL, // tls
      NULL  // prev
  };
  const mxArray *outputs[2];
  int32_T i;
  st.tls = emlrtRootTLSGlobal;
  // Check for proper number of arguments.
  if (nrhs != 5) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 5, 4,
                        14, "control_Interp");
  }
  if (nlhs > 2) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 14,
                        "control_Interp");
  }
  // Call the function.
  control_Interp_api(prhs, nlhs, outputs);
  // Copy over outputs to the caller.
  if (nlhs < 1) {
    i = 1;
  } else {
    i = nlhs;
  }
  emlrtReturnArrays(i, &plhs[0], &outputs[0]);
}

// End of code generation (_coder_control_Interp_mex.cpp)
