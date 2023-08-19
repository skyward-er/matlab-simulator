//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// control_Interp_types.h
//
// Code generation for function 'control_Interp'
//

#ifndef CONTROL_INTERP_TYPES_H
#define CONTROL_INTERP_TYPES_H

// Include files
#include "rtwtypes.h"

// Type Definitions
struct struct1_T {
  double maxAngle;
  double minAngle;
};

struct struct0_T {
  struct1_T servo;
  double z_final;
};

struct struct3_T {
  double deltaZ;
  double Z[301];
  double Vz[602];
};

struct struct2_T {
  struct3_T reference;
  double N_forward;
  boolean_T flagFirstControlABK;
  double filter_coeff;
  double filterMinAltitude;
  double filterMaxAltitude;
  double criticalAltitude;
  double filter_coeff0;
  char interpType;
};

#endif
// End of code generation (control_Interp_types.h)
