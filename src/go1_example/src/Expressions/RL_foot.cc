/*
 * Automatically Generated from Mathematica.
 * Fri 10 May 2024 15:36:38 GMT-05:00
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
  double t48;
  double t55;
  double t57;
  double t59;
  double t65;
  double t50;
  double t94;
  double t27;
  double t95;
  double t97;
  double t98;
  double t102;
  double t104;
  double t105;
  double t107;
  double t111;
  double t112;
  double t113;
  double t121;
  double t122;
  double t125;
  double t72;
  double t73;
  double t74;
  double t80;
  double t85;
  double t87;
  double t89;
  double t149;
  double t151;
  double t152;
  double t36;
  double t43;
  double t164;
  double t62;
  double t63;
  double t167;
  double t76;
  double t81;
  double t82;
  double t172;
  double t173;
  double t175;
  double t106;
  double t109;
  double t110;
  double t166;
  double t169;
  double t170;
  double t184;
  double t185;
  double t186;
  double t116;
  double t118;
  double t119;
  double t189;
  double t192;
  double t193;
  double t195;
  double t196;
  double t197;
  double t143;
  double t145;
  double t146;
  double t177;
  double t178;
  double t180;
  double t204;
  double t205;
  double t206;
  double t222;
  double t223;
  double t224;
  double t218;
  double t219;
  double t220;
  double t230;
  double t231;
  double t232;
  double t234;
  double t235;
  double t236;
  double t238;
  double t239;
  double t240;
  double t226;
  double t227;
  double t228;
  double t246;
  double t247;
  double t248;
  t48 = Cos(var1[4]);
  t55 = Sin(var1[18]);
  t57 = Sin(var1[4]);
  t59 = Cos(var1[18]);
  t65 = Sin(var1[5]);
  t50 = Cos(var1[5]);
  t94 = Sin(var1[19]);
  t27 = Cos(var1[19]);
  t95 = t59*t57;
  t97 = t48*t55*t65;
  t98 = t95 + t97;
  t102 = Cos(var1[20]);
  t104 = -1.*t102;
  t105 = 1. + t104;
  t107 = Sin(var1[20]);
  t111 = t48*t50*t94;
  t112 = t27*t98;
  t113 = t111 + t112;
  t121 = t27*t48*t50;
  t122 = -1.*t94*t98;
  t125 = t121 + t122;
  t72 = Cos(var1[21]);
  t73 = -1.*t72;
  t74 = 1. + t73;
  t80 = Sin(var1[21]);
  t85 = t55*t57;
  t87 = -1.*t59*t48*t65;
  t89 = t85 + t87;
  t149 = t102*t113;
  t151 = t107*t125;
  t152 = t149 + t151;
  t36 = -1.*t27;
  t43 = 1. + t36;
  t164 = Sin(var1[3]);
  t62 = -1.*t59;
  t63 = 1. + t62;
  t167 = Cos(var1[3]);
  t76 = 0.12675*t74;
  t81 = -0.426*t80;
  t82 = t76 + t81;
  t172 = t167*t50;
  t173 = -1.*t164*t57*t65;
  t175 = t172 + t173;
  t106 = -0.213*t105;
  t109 = -0.1881*t107;
  t110 = t106 + t109;
  t166 = t50*t164*t57;
  t169 = t167*t65;
  t170 = t166 + t169;
  t184 = -1.*t59*t48*t164;
  t185 = -1.*t55*t175;
  t186 = t184 + t185;
  t116 = -0.1881*t105;
  t118 = 0.213*t107;
  t119 = t116 + t118;
  t189 = t94*t170;
  t192 = t27*t186;
  t193 = t189 + t192;
  t195 = t27*t170;
  t196 = -1.*t94*t186;
  t197 = t195 + t196;
  t143 = -0.426*t74;
  t145 = -0.12675*t80;
  t146 = t143 + t145;
  t177 = -1.*t48*t55*t164;
  t178 = t59*t175;
  t180 = t177 + t178;
  t204 = t102*t193;
  t205 = t107*t197;
  t206 = t204 + t205;
  t222 = t50*t164;
  t223 = t167*t57*t65;
  t224 = t222 + t223;
  t218 = -1.*t167*t50*t57;
  t219 = t164*t65;
  t220 = t218 + t219;
  t230 = t59*t167*t48;
  t231 = -1.*t55*t224;
  t232 = t230 + t231;
  t234 = t94*t220;
  t235 = t27*t232;
  t236 = t234 + t235;
  t238 = t27*t220;
  t239 = -1.*t94*t232;
  t240 = t238 + t239;
  t226 = t167*t48*t55;
  t227 = t59*t224;
  t228 = t226 + t227;
  t246 = t102*t236;
  t247 = t107*t240;
  t248 = t246 + t247;
  p_output1[0]=t110*t113 + t119*t125 - 0.1881*(-1.*t107*t113 + t102*t125) + t146*t152 - 0.1881*t43*t48*t50 - 0.04675*t55*t57 - 0.04675*t48*t63*t65 + t82*t89 + 0.12675*(t152*t80 + t72*t89) - 0.426*(t152*t72 - 1.*t80*t89) - 0.1881*t94*t98 + var1[0];
  p_output1[1]=t110*t193 + t119*t197 - 0.1881*(-1.*t107*t193 + t102*t197) + t146*t206 - 0.1881*t170*t43 + 0.04675*t164*t48*t55 + 0.04675*t175*t63 - 0.426*(t206*t72 - 1.*t180*t80) + 0.12675*(t180*t72 + t206*t80) + t180*t82 - 0.1881*t186*t94 + var1[1];
  p_output1[2]=t110*t236 + t119*t240 - 0.1881*(-1.*t107*t236 + t102*t240) + t146*t248 - 0.1881*t220*t43 - 0.04675*t167*t48*t55 + 0.04675*t224*t63 - 0.426*(t248*t72 - 1.*t228*t80) + 0.12675*(t228*t72 + t248*t80) + t228*t82 - 0.1881*t232*t94 + var1[2];
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

#include "../../include/Expressions/RL_foot.hh"

namespace SymFunction
{

void RL_foot_raw(double *p_output1, const double *var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}

}

