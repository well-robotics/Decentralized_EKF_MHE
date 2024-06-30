/*
 * Automatically Generated from Mathematica.
 * Fri 10 May 2024 15:36:40 GMT-05:00
 */

#include <stdexcept>
#include <cmath>
#include<math.h>
#include "../../include/Expressions/mdefs.hpp"


/*
 * Sub functions
 */
static void output1(double *p_output1,const double *var1)
{
  double t76;
  double t85;
  double t93;
  double t97;
  double t109;
  double t81;
  double t140;
  double t51;
  double t143;
  double t145;
  double t146;
  double t151;
  double t153;
  double t154;
  double t156;
  double t160;
  double t161;
  double t162;
  double t170;
  double t171;
  double t172;
  double t111;
  double t112;
  double t114;
  double t118;
  double t127;
  double t128;
  double t132;
  double t198;
  double t200;
  double t201;
  double t58;
  double t70;
  double t213;
  double t101;
  double t106;
  double t216;
  double t116;
  double t119;
  double t121;
  double t221;
  double t222;
  double t223;
  double t155;
  double t158;
  double t159;
  double t215;
  double t217;
  double t219;
  double t233;
  double t234;
  double t235;
  double t165;
  double t166;
  double t169;
  double t238;
  double t241;
  double t242;
  double t244;
  double t245;
  double t246;
  double t192;
  double t194;
  double t195;
  double t226;
  double t227;
  double t229;
  double t253;
  double t254;
  double t255;
  double t271;
  double t272;
  double t273;
  double t267;
  double t268;
  double t269;
  double t279;
  double t280;
  double t281;
  double t283;
  double t284;
  double t285;
  double t287;
  double t288;
  double t289;
  double t275;
  double t276;
  double t277;
  double t295;
  double t296;
  double t297;
  t76 = Cos(var1[4]);
  t85 = Sin(var1[14]);
  t93 = Sin(var1[4]);
  t97 = Cos(var1[14]);
  t109 = Sin(var1[5]);
  t81 = Cos(var1[5]);
  t140 = Sin(var1[15]);
  t51 = Cos(var1[15]);
  t143 = t97*t93;
  t145 = t76*t85*t109;
  t146 = t143 + t145;
  t151 = Cos(var1[16]);
  t153 = -1.*t151;
  t154 = 1. + t153;
  t156 = Sin(var1[16]);
  t160 = t76*t81*t140;
  t161 = t51*t146;
  t162 = t160 + t161;
  t170 = t51*t76*t81;
  t171 = -1.*t140*t146;
  t172 = t170 + t171;
  t111 = Cos(var1[17]);
  t112 = -1.*t111;
  t114 = 1. + t112;
  t118 = Sin(var1[17]);
  t127 = t85*t93;
  t128 = -1.*t97*t76*t109;
  t132 = t127 + t128;
  t198 = t151*t162;
  t200 = t156*t172;
  t201 = t198 + t200;
  t58 = -1.*t51;
  t70 = 1. + t58;
  t213 = Sin(var1[3]);
  t101 = -1.*t97;
  t106 = 1. + t101;
  t216 = Cos(var1[3]);
  t116 = -0.12675*t114;
  t119 = -0.426*t118;
  t121 = t116 + t119;
  t221 = t216*t81;
  t222 = -1.*t213*t93*t109;
  t223 = t221 + t222;
  t155 = -0.213*t154;
  t158 = -0.1881*t156;
  t159 = t155 + t158;
  t215 = t81*t213*t93;
  t217 = t216*t109;
  t219 = t215 + t217;
  t233 = -1.*t97*t76*t213;
  t234 = -1.*t85*t223;
  t235 = t233 + t234;
  t165 = -0.1881*t154;
  t166 = 0.213*t156;
  t169 = t165 + t166;
  t238 = t140*t219;
  t241 = t51*t235;
  t242 = t238 + t241;
  t244 = t51*t219;
  t245 = -1.*t140*t235;
  t246 = t244 + t245;
  t192 = -0.426*t114;
  t194 = 0.12675*t118;
  t195 = t192 + t194;
  t226 = -1.*t76*t85*t213;
  t227 = t97*t223;
  t229 = t226 + t227;
  t253 = t151*t242;
  t254 = t156*t246;
  t255 = t253 + t254;
  t271 = t81*t213;
  t272 = t216*t93*t109;
  t273 = t271 + t272;
  t267 = -1.*t216*t81*t93;
  t268 = t213*t109;
  t269 = t267 + t268;
  t279 = t97*t216*t76;
  t280 = -1.*t85*t273;
  t281 = t279 + t280;
  t283 = t140*t269;
  t284 = t51*t281;
  t285 = t283 + t284;
  t287 = t51*t269;
  t288 = -1.*t140*t281;
  t289 = t287 + t288;
  t275 = t216*t76*t85;
  t276 = t97*t273;
  t277 = t275 + t276;
  t295 = t151*t285;
  t296 = t156*t289;
  t297 = t295 + t296;
  p_output1[0]=t121*t132 - 0.1881*t140*t146 + t159*t162 + t169*t172 - 0.1881*(-1.*t156*t162 + t151*t172) + t195*t201 - 0.426*(-1.*t118*t132 + t111*t201) - 0.12675*(t111*t132 + t118*t201) + 0.04675*t106*t109*t76 - 0.1881*t70*t76*t81 + 0.04675*t85*t93 + var1[0];
  p_output1[1]=-0.04675*t106*t223 + t121*t229 - 0.1881*t140*t235 + t159*t242 + t169*t246 - 0.1881*(-1.*t156*t242 + t151*t246) + t195*t255 - 0.426*(-1.*t118*t229 + t111*t255) - 0.12675*(t111*t229 + t118*t255) - 0.1881*t219*t70 - 0.04675*t213*t76*t85 + var1[1];
  p_output1[2]=-0.04675*t106*t273 + t121*t277 - 0.1881*t140*t281 + t159*t285 + t169*t289 - 0.1881*(-1.*t156*t285 + t151*t289) + t195*t297 - 0.426*(-1.*t118*t277 + t111*t297) - 0.12675*(t111*t277 + t118*t297) - 0.1881*t269*t70 + 0.04675*t216*t76*t85 + var1[2];
}




// #include "mex.h"
// /*
//  * Main function
//  */
// void mexFunction( int nlhs, mxArray *plhs[],
//                   int nrhs, const mxArray *prhs[] )
// {
//   size_t mrows, ncols;

//   double *var1;
//   double *p_output1;

//   /*  Check for proper number of arguments.  */ 
//   if( nrhs != 1)
//     {
//       mexErrMsgIdAndTxt("MATLAB:MShaped:invalidNumInputs", "One input(s) required (var1).");
//     }
//   else if( nlhs > 1)
//     {
//       mexErrMsgIdAndTxt("MATLAB:MShaped:maxlhs", "Too many output arguments.");
//     }

//   /*  The input must be a noncomplex double vector or scaler.  */
//   mrows = mxGetM(prhs[0]);
//   ncols = mxGetN(prhs[0]);
//   if( !mxIsDouble(prhs[0]) || mxIsComplex(prhs[0]) ||
//     ( !(mrows == 22 && ncols == 1) && 
//       !(mrows == 1 && ncols == 22))) 
//     {
//       mexErrMsgIdAndTxt( "MATLAB:MShaped:inputNotRealVector", "var1 is wrong.");
//     }

//   /*  Assign pointers to each input.  */
//   var1 = mxGetPr(prhs[0]);
   


   
//   /*  Create matrices for return arguments.  */
//   plhs[0] = mxCreateDoubleMatrix((mwSize) 1, (mwSize) 3, mxREAL);
//   p_output1 = mxGetPr(plhs[0]);


//   /* Call the calculation subroutine. */
//   output1(p_output1,var1);


// }

#include "../../include/Expressions/RR_foot.hh"

namespace SymFunction
{

void RR_foot_raw(double *p_output1, const double *var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}

}

