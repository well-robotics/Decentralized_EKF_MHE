/*
 * Automatically Generated from Mathematica.
 * Fri 10 May 2024 15:36:36 GMT-05:00
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
  double t10;
  double t28;
  double t34;
  double t40;
  double t42;
  double t20;
  double t58;
  double t59;
  double t60;
  double t15;
  double t61;
  double t67;
  double t68;
  double t69;
  double t71;
  double t76;
  double t77;
  double t79;
  double t63;
  double t65;
  double t66;
  double t44;
  double t45;
  double t47;
  double t49;
  double t54;
  double t55;
  double t56;
  double t101;
  double t102;
  double t103;
  double t24;
  double t26;
  double t31;
  double t33;
  double t117;
  double t115;
  double t48;
  double t50;
  double t51;
  double t122;
  double t123;
  double t124;
  double t135;
  double t136;
  double t137;
  double t116;
  double t118;
  double t119;
  double t70;
  double t72;
  double t73;
  double t80;
  double t81;
  double t82;
  double t146;
  double t147;
  double t148;
  double t140;
  double t143;
  double t144;
  double t95;
  double t97;
  double t98;
  double t128;
  double t129;
  double t131;
  double t155;
  double t156;
  double t157;
  double t172;
  double t173;
  double t174;
  double t181;
  double t182;
  double t183;
  double t168;
  double t169;
  double t170;
  double t189;
  double t190;
  double t191;
  double t185;
  double t186;
  double t187;
  double t177;
  double t178;
  double t179;
  double t197;
  double t198;
  double t199;
  t10 = Cos(var1[4]);
  t28 = Cos(var1[6]);
  t34 = Sin(var1[5]);
  t40 = Sin(var1[4]);
  t42 = Sin(var1[6]);
  t20 = Cos(var1[7]);
  t58 = t28*t40;
  t59 = t10*t34*t42;
  t60 = t58 + t59;
  t15 = Cos(var1[5]);
  t61 = Sin(var1[7]);
  t67 = Cos(var1[8]);
  t68 = -1.*t67;
  t69 = 1. + t68;
  t71 = Sin(var1[8]);
  t76 = t10*t15*t20;
  t77 = -1.*t60*t61;
  t79 = t76 + t77;
  t63 = t20*t60;
  t65 = t10*t15*t61;
  t66 = t63 + t65;
  t44 = Cos(var1[9]);
  t45 = -1.*t44;
  t47 = 1. + t45;
  t49 = Sin(var1[9]);
  t54 = -1.*t10*t28*t34;
  t55 = t40*t42;
  t56 = t54 + t55;
  t101 = t67*t66;
  t102 = t79*t71;
  t103 = t101 + t102;
  t24 = -1.*t20;
  t26 = 1. + t24;
  t31 = -1.*t28;
  t33 = 1. + t31;
  t117 = Cos(var1[3]);
  t115 = Sin(var1[3]);
  t48 = -0.12675*t47;
  t50 = -0.426*t49;
  t51 = t48 + t50;
  t122 = t117*t15;
  t123 = -1.*t115*t40*t34;
  t124 = t122 + t123;
  t135 = -1.*t10*t28*t115;
  t136 = -1.*t124*t42;
  t137 = t135 + t136;
  t116 = t15*t115*t40;
  t118 = t117*t34;
  t119 = t116 + t118;
  t70 = -0.213*t69;
  t72 = 0.1881*t71;
  t73 = t70 + t72;
  t80 = 0.1881*t69;
  t81 = 0.213*t71;
  t82 = t80 + t81;
  t146 = t20*t119;
  t147 = -1.*t137*t61;
  t148 = t146 + t147;
  t140 = t20*t137;
  t143 = t119*t61;
  t144 = t140 + t143;
  t95 = -0.426*t47;
  t97 = 0.12675*t49;
  t98 = t95 + t97;
  t128 = t28*t124;
  t129 = -1.*t10*t115*t42;
  t131 = t128 + t129;
  t155 = t67*t144;
  t156 = t148*t71;
  t157 = t155 + t156;
  t172 = t15*t115;
  t173 = t117*t40*t34;
  t174 = t172 + t173;
  t181 = t117*t10*t28;
  t182 = -1.*t174*t42;
  t183 = t181 + t182;
  t168 = -1.*t117*t15*t40;
  t169 = t115*t34;
  t170 = t168 + t169;
  t189 = t20*t170;
  t190 = -1.*t183*t61;
  t191 = t189 + t190;
  t185 = t20*t183;
  t186 = t170*t61;
  t187 = t185 + t186;
  t177 = t28*t174;
  t178 = t117*t10*t42;
  t179 = t177 + t178;
  t197 = t67*t187;
  t198 = t191*t71;
  t199 = t197 + t198;
  p_output1[0]=0.1881*t10*t15*t26 + 0.04675*t10*t33*t34 + 0.04675*t40*t42 + t51*t56 - 0.12675*(t103*t49 + t44*t56) - 0.426*(t103*t44 - 1.*t49*t56) + 0.1881*t60*t61 + t66*t73 + 0.1881*(-1.*t66*t71 + t67*t79) + t79*t82 + t103*t98 + var1[0];
  p_output1[1]=0.1881*t119*t26 - 0.04675*t124*t33 - 0.04675*t10*t115*t42 - 0.426*(t157*t44 - 1.*t131*t49) - 0.12675*(t131*t44 + t157*t49) + t131*t51 + 0.1881*t137*t61 + 0.1881*(t148*t67 - 1.*t144*t71) + t144*t73 + t148*t82 + t157*t98 + var1[1];
  p_output1[2]=0.1881*t170*t26 - 0.04675*t174*t33 + 0.04675*t10*t117*t42 - 0.426*(t199*t44 - 1.*t179*t49) - 0.12675*(t179*t44 + t199*t49) + t179*t51 + 0.1881*t183*t61 + 0.1881*(t191*t67 - 1.*t187*t71) + t187*t73 + t191*t82 + t199*t98 + var1[2];
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


#include "../../include/Expressions/FR_foot.hh"

namespace SymFunction
{

void FR_foot_raw(double *p_output1, const double *var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}

}

