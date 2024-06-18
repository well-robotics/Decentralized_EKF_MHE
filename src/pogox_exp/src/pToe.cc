/*
 * Automatically Generated from Mathematica.
 * Fri 8 Mar 2024 11:41:28 GMT-06:00
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
  plhs[0] = mxCreateDoubleMatrix((mwSize)1, (mwSize)3, mxREAL);
  p_output1 = mxGetPr(plhs[0]);

  /* Call the calculation subroutine. */
  output1(p_output1, var1);
}

#else // MATLAB_MEX_FILE

#include "../include/frost_codegen/pToe.hh"

/*
 * Sub functions
 */
static void output1(double *p_output1, const double *var1)
{
  double t1501;
  double t1412;
  double t1517;
  double t1467;
  double t1512;
  double t1546;
  t1501 = Sin(var1[4]);
  t1412 = Cos(var1[4]);
  t1517 = Sin(var1[3]);
  t1467 = Cos(var1[5]);
  t1512 = Sin(var1[5]);
  t1546 = Cos(var1[3]);
  p_output1[0] = -0.007727 * t1412 * t1467 - 0.53531 * t1501 - 0.00602 * t1412 * t1512 + var1[0] + t1501 * var1[6];
  p_output1[1] = 0.53531 * t1412 * t1517 + 0.00602 * (-1. * t1501 * t1512 * t1517 + t1467 * t1546) - 0.007727 * (t1467 * t1501 * t1517 + t1512 * t1546) + var1[1] - 1. * t1412 * t1517 * var1[6];
  p_output1[2] = -0.53531 * t1412 * t1546 - 0.007727 * (t1512 * t1517 - 1. * t1467 * t1501 * t1546) + 0.00602 * (t1467 * t1517 + t1501 * t1512 * t1546) + var1[2] + t1412 * t1546 * var1[6];
}

namespace SymFunction
{

  void pToe_raw(double *p_output1, const double *var1)
  {
    // Call Subroutines
    output1(p_output1, var1);
  }

}

#endif // MATLAB_MEX_FILE
