/*
 * Automatically Generated from Mathematica.
 * Fri 8 Mar 2024 11:41:29 GMT-06:00
 */

#ifdef MATLAB_MEX_FILE
#include <stdexcept>
#include <cmath>
#include <math.h>
/**
 * Copied from Wolfram Mathematica C Definitions file mdefs.hpp
 * Changed marcos to inline functions (Eric Cousineau)
 */
inline double Power(double x, double y) { return pow(x, y); }
inline double Sqrt(double x) { return sqrt(x); }

inline double Abs(double x) { return fabs(x); }

inline double Exp(double x) { return exp(x); }
inline double Log(double x) { return log(x); }

inline double Sin(double x) { return sin(x); }
inline double Cos(double x) { return cos(x); }
inline double Tan(double x) { return tan(x); }

inline double ArcSin(double x) { return asin(x); }
inline double ArcCos(double x) { return acos(x); }
inline double ArcTan(double x) { return atan(x); }

/* update ArcTan function to use atan2 instead. */
inline double ArcTan(double x, double y) { return atan2(y, x); }

inline double Sinh(double x) { return sinh(x); }
inline double Cosh(double x) { return cosh(x); }
inline double Tanh(double x) { return tanh(x); }

const double E = 2.71828182845904523536029;
const double Pi = 3.14159265358979323846264;
const double Degree = 0.01745329251994329576924;

inline double Sec(double x) { return 1 / cos(x); }
inline double Csc(double x) { return 1 / sin(x); }

#endif

#ifdef MATLAB_MEX_FILE

#include "mex.h"
/*
 * Main function
 */
void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[])
{
  size_t mrows, ncols;

  double *var1;
  double *p_output1;

  /*  Check for proper number of arguments.  */
  if (nrhs != 1)
  {
    mexErrMsgIdAndTxt("MATLAB:MShaped:invalidNumInputs", "One input(s) required (var1).");
  }
  else if (nlhs > 1)
  {
    mexErrMsgIdAndTxt("MATLAB:MShaped:maxlhs", "Too many output arguments.");
  }

  /*  The input must be a noncomplex double vector or scaler.  */
  mrows = mxGetM(prhs[0]);
  ncols = mxGetN(prhs[0]);
  if (!mxIsDouble(prhs[0]) || mxIsComplex(prhs[0]) ||
      (!(mrows == 7 && ncols == 1) &&
       !(mrows == 1 && ncols == 7)))
  {
    mexErrMsgIdAndTxt("MATLAB:MShaped:inputNotRealVector", "var1 is wrong.");
  }

  /*  Assign pointers to each input.  */
  var1 = mxGetPr(prhs[0]);

  /*  Create matrices for return arguments.  */
  plhs[0] = mxCreateDoubleMatrix((mwSize)3, (mwSize)7, mxREAL);
  p_output1 = mxGetPr(plhs[0]);

  /* Call the calculation subroutine. */
  output1(p_output1, var1);
}

#else // MATLAB_MEX_FILE

#include "../include/frost_codegen/J_Toe.hh"

/*
 * Sub functions
 */
static void output1(double *p_output1, const double *var1)
{
  double t1492;
  double t1510;
  double t1516;
  double t1520;
  double t1518;
  double t1552;
  double t1649;
  double t1650;
  double t1653;
  double t1519;
  double t1571;
  double t1572;
  t1492 = Cos(var1[3]);
  t1510 = Cos(var1[4]);
  t1516 = Cos(var1[5]);
  t1520 = Sin(var1[3]);
  t1518 = Sin(var1[4]);
  t1552 = Sin(var1[5]);
  t1649 = t1492 * t1516;
  t1650 = -1. * t1520 * t1518 * t1552;
  t1653 = t1649 + t1650;
  t1519 = t1492 * t1516 * t1518;
  t1571 = -1. * t1520 * t1552;
  t1572 = t1519 + t1571;
  p_output1[0] = 1.;
  p_output1[1] = 0;
  p_output1[2] = 0;
  p_output1[3] = 0;
  p_output1[4] = 1.;
  p_output1[5] = 0;
  p_output1[6] = 0;
  p_output1[7] = 0;
  p_output1[8] = 1.;
  p_output1[9] = 0;
  p_output1[10] = 0.53531 * t1492 * t1510 + 0.00602 * (-1. * t1516 * t1520 - 1. * t1492 * t1518 * t1552) - 0.007727 * t1572 - 1. * t1492 * t1510 * var1[6];
  p_output1[11] = 0.53531 * t1510 * t1520 - 0.007727 * (t1516 * t1518 * t1520 + t1492 * t1552) + 0.00602 * t1653 - 1. * t1510 * t1520 * var1[6];
  p_output1[12] = -0.53531 * t1510 + 0.007727 * t1516 * t1518 + 0.00602 * t1518 * t1552 + t1510 * var1[6];
  p_output1[13] = -0.007727 * t1510 * t1516 * t1520 - 0.53531 * t1518 * t1520 - 0.00602 * t1510 * t1520 * t1552 + t1518 * t1520 * var1[6];
  p_output1[14] = 0.007727 * t1492 * t1510 * t1516 + 0.53531 * t1492 * t1518 + 0.00602 * t1492 * t1510 * t1552 - 1. * t1492 * t1518 * var1[6];
  p_output1[15] = -0.00602 * t1510 * t1516 + 0.007727 * t1510 * t1552;
  p_output1[16] = 0.00602 * (-1. * t1516 * t1518 * t1520 - 1. * t1492 * t1552) - 0.007727 * t1653;
  p_output1[17] = -0.007727 * (t1516 * t1520 + t1492 * t1518 * t1552) + 0.00602 * t1572;
  p_output1[18] = t1518;
  p_output1[19] = -1. * t1510 * t1520;
  p_output1[20] = t1492 * t1510;
}

namespace SymFunction
{

  void J_Toe_raw(double *p_output1, const double *var1)
  {
    // Call Subroutines
    output1(p_output1, var1);
  }

}

#endif // MATLAB_MEX_FILE
