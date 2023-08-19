//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// _coder_control_Interp_api.cpp
//
// Code generation for function 'control_Interp'
//

// Include files
#include "_coder_control_Interp_api.h"
#include "_coder_control_Interp_mex.h"

// Variable Definitions
emlrtCTX emlrtRootTLSGlobal = NULL;

emlrtContext emlrtContextGlobal = {
    true,                                                 // bFirstTime
    false,                                                // bInitialized
    131627U,                                              // fVersionInfo
    NULL,                                                 // fErrorFunction
    "control_Interp",                                     // fFunctionName
    NULL,                                                 // fRTCallStack
    false,                                                // bDebugMode
    {2045744189U, 2170104910U, 2743257031U, 4284093946U}, // fSigWrd
    NULL                                                  // fSigMem
};

// Function Declarations
static struct0_T b_emlrt_marshallIn(const emlrtStack *sp,
                                    const mxArray *settings,
                                    const char_T *identifier);

static struct0_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                    const emlrtMsgIdentifier *parentId);

static void b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               real_T y[602]);

static struct1_T c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                    const emlrtMsgIdentifier *parentId);

static void c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               real_T ret[301]);

static void d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               real_T ret[602]);

static boolean_T d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                    const emlrtMsgIdentifier *parentId);

static char_T e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId);

static void emlrt_marshallIn(const emlrtStack *sp, const mxArray *contSettings,
                             const char_T *identifier, struct2_T *y);

static void emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId, struct2_T *y);

static void emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId, struct3_T *y);

static void emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId, real_T y[301]);

static real_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *z,
                               const char_T *identifier);

static real_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId);

static const mxArray *emlrt_marshallOut(const emlrtStack *sp,
                                        const struct2_T *u);

static const mxArray *emlrt_marshallOut(const real_T u);

static real_T f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId);

static boolean_T g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId);

static char_T h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId);

// Function Definitions
static struct0_T b_emlrt_marshallIn(const emlrtStack *sp,
                                    const mxArray *settings,
                                    const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  struct0_T y;
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(sp, emlrtAlias(settings), &thisId);
  emlrtDestroyArray(&settings);
  return y;
}

static struct0_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                    const emlrtMsgIdentifier *parentId)
{
  static const int32_T dims = 0;
  static const char_T *fieldNames[2] = {"servo", "z_final"};
  emlrtMsgIdentifier thisId;
  struct0_T y;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b((emlrtConstCTX)sp, parentId, u, 2,
                         (const char_T **)&fieldNames[0], 0U,
                         (const void *)&dims);
  thisId.fIdentifier = "servo";
  y.servo = c_emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 0, "servo")),
      &thisId);
  thisId.fIdentifier = "z_final";
  y.z_final = emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 1, "z_final")),
      &thisId);
  emlrtDestroyArray(&u);
  return y;
}

static void b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               real_T y[602])
{
  d_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static struct1_T c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                    const emlrtMsgIdentifier *parentId)
{
  static const int32_T dims = 0;
  static const char_T *fieldNames[2] = {"maxAngle", "minAngle"};
  emlrtMsgIdentifier thisId;
  struct1_T y;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b((emlrtConstCTX)sp, parentId, u, 2,
                         (const char_T **)&fieldNames[0], 0U,
                         (const void *)&dims);
  thisId.fIdentifier = "maxAngle";
  y.maxAngle = emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 0, "maxAngle")),
      &thisId);
  thisId.fIdentifier = "minAngle";
  y.minAngle = emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 1, "minAngle")),
      &thisId);
  emlrtDestroyArray(&u);
  return y;
}

static void c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[301])
{
  static const int32_T dims = 301;
  real_T(*r)[301];
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 1U,
                          (const void *)&dims);
  r = (real_T(*)[301])emlrtMxGetData(src);
  for (int32_T i = 0; i < 301; i++) {
    ret[i] = (*r)[i];
  }
  emlrtDestroyArray(&src);
}

static void d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[602])
{
  static const int32_T dims[2] = {301, 2};
  real_T(*r)[602];
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 2U,
                          (const void *)&dims[0]);
  r = (real_T(*)[602])emlrtMxGetData(src);
  for (int32_T i = 0; i < 602; i++) {
    ret[i] = (*r)[i];
  }
  emlrtDestroyArray(&src);
}

static boolean_T d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                    const emlrtMsgIdentifier *parentId)
{
  boolean_T y;
  y = g_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static char_T e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId)
{
  char_T y;
  y = h_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static void emlrt_marshallIn(const emlrtStack *sp, const mxArray *contSettings,
                             const char_T *identifier, struct2_T *y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  emlrt_marshallIn(sp, emlrtAlias(contSettings), &thisId, y);
  emlrtDestroyArray(&contSettings);
}

static void emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId, struct2_T *y)
{
  static const int32_T dims = 0;
  static const char_T *fieldNames[9] = {
      "reference",        "N_forward",         "flagFirstControlABK",
      "filter_coeff",     "filterMinAltitude", "filterMaxAltitude",
      "criticalAltitude", "filter_coeff0",     "interpType"};
  emlrtMsgIdentifier thisId;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b((emlrtConstCTX)sp, parentId, u, 9,
                         (const char_T **)&fieldNames[0], 0U,
                         (const void *)&dims);
  thisId.fIdentifier = "reference";
  emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 0, "reference")),
      &thisId, &y->reference);
  thisId.fIdentifier = "N_forward";
  y->N_forward = emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 1, "N_forward")),
      &thisId);
  thisId.fIdentifier = "flagFirstControlABK";
  y->flagFirstControlABK = d_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 2,
                                     "flagFirstControlABK")),
      &thisId);
  thisId.fIdentifier = "filter_coeff";
  y->filter_coeff =
      emlrt_marshallIn(sp,
                       emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0,
                                                      3, "filter_coeff")),
                       &thisId);
  thisId.fIdentifier = "filterMinAltitude";
  y->filterMinAltitude =
      emlrt_marshallIn(sp,
                       emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0,
                                                      4, "filterMinAltitude")),
                       &thisId);
  thisId.fIdentifier = "filterMaxAltitude";
  y->filterMaxAltitude =
      emlrt_marshallIn(sp,
                       emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0,
                                                      5, "filterMaxAltitude")),
                       &thisId);
  thisId.fIdentifier = "criticalAltitude";
  y->criticalAltitude =
      emlrt_marshallIn(sp,
                       emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0,
                                                      6, "criticalAltitude")),
                       &thisId);
  thisId.fIdentifier = "filter_coeff0";
  y->filter_coeff0 =
      emlrt_marshallIn(sp,
                       emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0,
                                                      7, "filter_coeff0")),
                       &thisId);
  thisId.fIdentifier = "interpType";
  y->interpType = e_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 8, "interpType")),
      &thisId);
  emlrtDestroyArray(&u);
}

static void emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId, struct3_T *y)
{
  static const int32_T dims = 0;
  static const char_T *fieldNames[3] = {"deltaZ", "Z", "Vz"};
  emlrtMsgIdentifier thisId;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b((emlrtConstCTX)sp, parentId, u, 3,
                         (const char_T **)&fieldNames[0], 0U,
                         (const void *)&dims);
  thisId.fIdentifier = "deltaZ";
  y->deltaZ = emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 0, "deltaZ")),
      &thisId);
  thisId.fIdentifier = "Z";
  emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 1, "Z")),
      &thisId, y->Z);
  thisId.fIdentifier = "Vz";
  b_emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 2, "Vz")),
      &thisId, y->Vz);
  emlrtDestroyArray(&u);
}

static void emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId, real_T y[301])
{
  c_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static real_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *z,
                               const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  real_T y;
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = emlrt_marshallIn(sp, emlrtAlias(z), &thisId);
  emlrtDestroyArray(&z);
  return y;
}

static real_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = f_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static const mxArray *emlrt_marshallOut(const emlrtStack *sp,
                                        const struct2_T *u)
{
  static const int32_T iv[2] = {301, 2};
  static const int32_T i = 301;
  static const char_T *sv[9] = {
      "reference",        "N_forward",         "flagFirstControlABK",
      "filter_coeff",     "filterMinAltitude", "filterMaxAltitude",
      "criticalAltitude", "filter_coeff0",     "interpType"};
  static const char_T *sv1[3] = {"deltaZ", "Z", "Vz"};
  const mxArray *b_y;
  const mxArray *c_y;
  const mxArray *d_y;
  const mxArray *e_y;
  const mxArray *f_y;
  const mxArray *m;
  const mxArray *y;
  real_T *pData;
  int32_T i1;
  y = NULL;
  emlrtAssign(&y, emlrtCreateStructMatrix(1, 1, 9, (const char_T **)&sv[0]));
  b_y = NULL;
  emlrtAssign(&b_y, emlrtCreateStructMatrix(1, 1, 3, (const char_T **)&sv1[0]));
  emlrtSetFieldR2017b(b_y, 0, "deltaZ", emlrt_marshallOut(u->reference.deltaZ),
                      0);
  c_y = NULL;
  m = emlrtCreateNumericArray(1, (const void *)&i, mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  for (int32_T b_i = 0; b_i < 301; b_i++) {
    pData[b_i] = u->reference.Z[b_i];
  }
  emlrtAssign(&c_y, m);
  emlrtSetFieldR2017b(b_y, 0, "Z", c_y, 1);
  d_y = NULL;
  m = emlrtCreateNumericArray(2, (const void *)&iv[0], mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  i1 = 0;
  for (int32_T b_i = 0; b_i < 2; b_i++) {
    for (int32_T c_i = 0; c_i < 301; c_i++) {
      pData[i1 + c_i] = u->reference.Vz[c_i + 301 * b_i];
    }
    i1 += 301;
  }
  emlrtAssign(&d_y, m);
  emlrtSetFieldR2017b(b_y, 0, "Vz", d_y, 2);
  emlrtSetFieldR2017b(y, 0, "reference", b_y, 0);
  emlrtSetFieldR2017b(y, 0, "N_forward", emlrt_marshallOut(u->N_forward), 1);
  e_y = NULL;
  m = emlrtCreateLogicalScalar(u->flagFirstControlABK);
  emlrtAssign(&e_y, m);
  emlrtSetFieldR2017b(y, 0, "flagFirstControlABK", e_y, 2);
  emlrtSetFieldR2017b(y, 0, "filter_coeff", emlrt_marshallOut(u->filter_coeff),
                      3);
  emlrtSetFieldR2017b(y, 0, "filterMinAltitude",
                      emlrt_marshallOut(u->filterMinAltitude), 4);
  emlrtSetFieldR2017b(y, 0, "filterMaxAltitude",
                      emlrt_marshallOut(u->filterMaxAltitude), 5);
  emlrtSetFieldR2017b(y, 0, "criticalAltitude",
                      emlrt_marshallOut(u->criticalAltitude), 6);
  emlrtSetFieldR2017b(y, 0, "filter_coeff0",
                      emlrt_marshallOut(u->filter_coeff0), 7);
  f_y = NULL;
  m = emlrtCreateString1R2022a((emlrtCTX)sp, u->interpType);
  emlrtAssign(&f_y, m);
  emlrtSetFieldR2017b(y, 0, "interpType", f_y, 8);
  return y;
}

static const mxArray *emlrt_marshallOut(const real_T u)
{
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateDoubleScalar(u);
  emlrtAssign(&y, m);
  return y;
}

static real_T f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims = 0;
  real_T ret;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 0U,
                          (const void *)&dims);
  ret = *static_cast<real_T *>(emlrtMxGetData(src));
  emlrtDestroyArray(&src);
  return ret;
}

static boolean_T g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims = 0;
  boolean_T ret;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "logical", false, 0U,
                          (const void *)&dims);
  ret = *emlrtMxGetLogicals(src);
  emlrtDestroyArray(&src);
  return ret;
}

static char_T h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims = 0;
  char_T ret;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "char", false, 0U,
                          (const void *)&dims);
  emlrtImportCharR2015b((emlrtCTX)sp, src, &ret);
  emlrtDestroyArray(&src);
  return ret;
}

void control_Interp_api(const mxArray *const prhs[5], int32_T nlhs,
                        const mxArray *plhs[2])
{
  emlrtStack st = {
      NULL, // site
      NULL, // tls
      NULL  // prev
  };
  struct0_T settings;
  struct2_T contSettings;
  real_T Vz;
  real_T alpha0_old;
  real_T z;
  st.tls = emlrtRootTLSGlobal;
  // Marshall function inputs
  z = emlrt_marshallIn(&st, emlrtAliasP(prhs[0]), "z");
  Vz = emlrt_marshallIn(&st, emlrtAliasP(prhs[1]), "Vz");
  settings = b_emlrt_marshallIn(&st, emlrtAliasP(prhs[2]), "settings");
  emlrt_marshallIn(&st, emlrtAliasP(prhs[3]), "contSettings", &contSettings);
  alpha0_old = emlrt_marshallIn(&st, emlrtAliasP(prhs[4]), "alpha0_old");
  // Invoke the target function
  z = control_Interp(z, Vz, &settings, &contSettings, alpha0_old);
  // Marshall function outputs
  plhs[0] = emlrt_marshallOut(z);
  if (nlhs > 1) {
    plhs[1] = emlrt_marshallOut(&st, &contSettings);
  }
}

void control_Interp_atexit()
{
  emlrtStack st = {
      NULL, // site
      NULL, // tls
      NULL  // prev
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  control_Interp_xil_terminate();
  control_Interp_xil_shutdown();
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

void control_Interp_initialize()
{
  emlrtStack st = {
      NULL, // site
      NULL, // tls
      NULL  // prev
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, NULL);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

void control_Interp_terminate()
{
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

// End of code generation (_coder_control_Interp_api.cpp)
