//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// control_Interp.cpp
//
// Code generation for function 'control_Interp'
//

// Include files
#include "control_Interp.h"
#include "control_Interp_types.h"
#include <math.h>

// Function Definitions
double control_Interp(double z, double Vz, const struct0_T *settings,
                      struct2_T *contSettings, double alpha0_old)
{
  double alpha0;
  double d;
  double index_z;
  //  HELP
  //
  //  This function aims to decide the angle to give as reference to the air
  //  braking system in order to guarantee the drag needed.
  //  in order to do so it has speed and altitude as variable inputs, and
  //  altitude and vertical speed references.
  //  Note that it takes speed V as a vector as input, while V_ref is a
  //  reference only the vertical component.
  //
  //  Interpolation algorithm: takes two references (max and
  //  min extension) and decides how much to open with an
  //  interpolation at fixed altitude of the actual velocity
  //  w.r.t. the two references.
  //  INPUTS:
  //  z:        measured altitude
  //  V:        speed (vector of two components)
  //  settings: structure with data of the rocket and other simulation
  //            parameters
  //  contSettings: structure with control configuration parameters
  //  alpha_0_old: old commanded ABK angle, used in the filtering action
  //
  //  OUTPUTS:
  //  alpha0:   commanded angle for the ABK controller
  //  contSettings: control configuration parameters updated
  //  retrieve data
  //  find reference altitude index
  index_z =
      (floor(z / contSettings->reference.deltaZ) + contSettings->N_forward) +
      1.0;
  //  +1 because we are 1-based on matlab, on CPP the formula is just """
  //  floor(z/deltaZ) + N_forward """  (0-based), pay attention!
  if (index_z > 301.0) {
    index_z = 301.0;
  } else if (index_z <= 1.0) {
    index_z = 1.0;
  }
  //  choose points on velocity references
  //  percentage
  d = contSettings->reference.Vz[static_cast<int>(index_z) - 1];
  if (Vz < d) {
    //  use the vertical component of vector V, check if it is the first or
    //  second
    index_z = 0.0;
  } else {
    index_z = contSettings->reference.Vz[static_cast<int>(index_z) + 300];
    if (Vz > index_z) {
      index_z = 1.0;
    } else {
      //      switch contSettings.interpType
      //          case 'linear'
      index_z = (Vz - d) / (index_z - d);
      //  percentage = 0 if completely on trajectory 1, percentage = 1 if
      //  completely on trajectory 2
      //          case 'sinusoidal'
      //              percentage =
      //              0.5+0.5*cos(-pi+pi*(Vz-V_extrema(1))/(V_extrema(2)-V_extrema(1)));
      //              % same as choice 1, but with a sinusoidal approach
      //      end
    }
  }
  alpha0 = settings->servo.minAngle * (1.0 - index_z) +
           settings->servo.maxAngle * index_z;
  //  if we are too high
  if (z > settings->z_final) {
    alpha0 = settings->servo.maxAngle;
  }
  //  filter control action
  if (!contSettings->flagFirstControlABK) {
    //  the first reference is given the fastest possible (unfiltered), then
    //  filter
    alpha0 = alpha0_old + (alpha0 - alpha0_old) * contSettings->filter_coeff;
  }
  //  filter
  contSettings->flagFirstControlABK = false;
  //  after the first control action, set this flag to false in order to
  //  activate filter filter coefficient of 0.9 if below 1000 meters, linear
  //  decrease 0.9 to 0 until 3000, if above open to max
  if (z <= contSettings->filterMinAltitude) {
    contSettings->filter_coeff = contSettings->filter_coeff0;
  } else if ((z > contSettings->filterMinAltitude) &&
             (z <= contSettings->filterMaxAltitude)) {
    contSettings->filter_coeff =
        contSettings->filter_coeff0 - (z - contSettings->filterMinAltitude) /
                                          (contSettings->filterMaxAltitude -
                                           contSettings->filterMinAltitude) *
                                          contSettings->filter_coeff0;
    // linear
  }
  if (z > contSettings->criticalAltitude) {
    alpha0 = settings->servo.maxAngle;
  }
  return alpha0;
}

// End of code generation (control_Interp.cpp)
