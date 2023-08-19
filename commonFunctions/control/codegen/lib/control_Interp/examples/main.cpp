//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// main.cpp
//
// Code generation for function 'main'
//

/*************************************************************************/
/* This automatically generated example C++ main file shows how to call  */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/

// Include files
#include "main.h"
#include "control_Interp.h"
#include "control_Interp_terminate.h"
#include "control_Interp_types.h"

// Function Declarations
static void argInit_301x1_real_T(double result[301]);

static void argInit_301x2_real_T(double result[602]);

static boolean_T argInit_boolean_T();

static char argInit_char_T();

static double argInit_real_T();

static struct0_T argInit_struct0_T();

static struct1_T argInit_struct1_T();

static void argInit_struct2_T(struct2_T *result);

static void argInit_struct3_T(struct3_T *result);

// Function Definitions
static void argInit_301x1_real_T(double result[301])
{
  // Loop over the array to initialize each element.
  for (int idx0 = 0; idx0 < 301; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx0] = argInit_real_T();
  }
}

static void argInit_301x2_real_T(double result[602])
{
  // Loop over the array to initialize each element.
  for (int i = 0; i < 602; i++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[i] = argInit_real_T();
  }
}

static boolean_T argInit_boolean_T()
{
  return false;
}

static char argInit_char_T()
{
  return '?';
}

static double argInit_real_T()
{
  return 0.0;
}

static struct0_T argInit_struct0_T()
{
  struct0_T result;
  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  result.servo = argInit_struct1_T();
  result.z_final = argInit_real_T();
  return result;
}

static struct1_T argInit_struct1_T()
{
  struct1_T result;
  double result_tmp;
  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  result_tmp = argInit_real_T();
  result.minAngle = result_tmp;
  result.maxAngle = result_tmp;
  return result;
}

static void argInit_struct2_T(struct2_T *result)
{
  double result_tmp;
  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  result_tmp = argInit_real_T();
  result->filter_coeff = result_tmp;
  result->filterMinAltitude = result_tmp;
  result->filterMaxAltitude = result_tmp;
  result->criticalAltitude = result_tmp;
  result->filter_coeff0 = result_tmp;
  argInit_struct3_T(&result->reference);
  result->N_forward = result_tmp;
  result->flagFirstControlABK = argInit_boolean_T();
  result->interpType = argInit_char_T();
}

static void argInit_struct3_T(struct3_T *result)
{
  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  result->deltaZ = argInit_real_T();
  argInit_301x1_real_T(result->Z);
  argInit_301x2_real_T(result->Vz);
}

int main(int, char **)
{
  // The initialize function is being called automatically from your entry-point
  // function. So, a call to initialize is not included here. Invoke the
  // entry-point functions.
  // You can call entry-point functions multiple times.
  main_control_Interp();
  // Terminate the application.
  // You do not need to do this more than one time.
  control_Interp_terminate();
  return 0;
}

void main_control_Interp()
{
  struct0_T r;
  struct2_T contSettings;
  double z_tmp;
  // Initialize function 'control_Interp' input arguments.
  z_tmp = argInit_real_T();
  // Initialize function input argument 'settings'.
  // Initialize function input argument 'contSettings'.
  // Call the entry-point 'control_Interp'.
  argInit_struct2_T(&contSettings);
  r = argInit_struct0_T();
  z_tmp = control_Interp(z_tmp, z_tmp, &r, &contSettings, z_tmp);
}

// End of code generation (main.cpp)
