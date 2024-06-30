/*
 * Automatically Generated from Mathematica.
 * Fri 10 May 2024 15:36:34 GMT-05:00
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
  double t8;
  double t13;
  double t14;
  double t16;
  double t19;
  double t9;
  double t32;
  double t5;
  double t33;
  double t34;
  double t35;
  double t37;
  double t38;
  double t39;
  double t41;
  double t44;
  double t45;
  double t46;
  double t51;
  double t52;
  double t53;
  double t21;
  double t22;
  double t23;
  double t25;
  double t28;
  double t29;
  double t30;
  double t62;
  double t63;
  double t64;
  double t6;
  double t7;
  double t75;
  double t17;
  double t18;
  double t78;
  double t24;
  double t26;
  double t27;
  double t82;
  double t83;
  double t84;
  double t40;
  double t42;
  double t43;
  double t77;
  double t79;
  double t80;
  double t90;
  double t91;
  double t92;
  double t48;
  double t49;
  double t50;
  double t94;
  double t95;
  double t96;
  double t98;
  double t99;
  double t100;
  double t59;
  double t60;
  double t61;
  double t86;
  double t87;
  double t88;
  double t106;
  double t107;
  double t108;
  double t124;
  double t125;
  double t126;
  double t120;
  double t121;
  double t122;
  double t132;
  double t133;
  double t134;
  double t136;
  double t137;
  double t138;
  double t140;
  double t141;
  double t142;
  double t128;
  double t129;
  double t130;
  double t148;
  double t149;
  double t150;
  t8 = Cos(var1[4]);
  t13 = Sin(var1[10]);
  t14 = Sin(var1[4]);
  t16 = Cos(var1[10]);
  t19 = Sin(var1[5]);
  t9 = Cos(var1[5]);
  t32 = Sin(var1[11]);
  t5 = Cos(var1[11]);
  t33 = t16*t14;
  t34 = t8*t13*t19;
  t35 = t33 + t34;
  t37 = Cos(var1[12]);
  t38 = -1.*t37;
  t39 = 1. + t38;
  t41 = Sin(var1[12]);
  t44 = t8*t9*t32;
  t45 = t5*t35;
  t46 = t44 + t45;
  t51 = t5*t8*t9;
  t52 = -1.*t32*t35;
  t53 = t51 + t52;
  t21 = Cos(var1[13]);
  t22 = -1.*t21;
  t23 = 1. + t22;
  t25 = Sin(var1[13]);
  t28 = t13*t14;
  t29 = -1.*t16*t8*t19;
  t30 = t28 + t29;
  t62 = t37*t46;
  t63 = t41*t53;
  t64 = t62 + t63;
  t6 = -1.*t5;
  t7 = 1. + t6;
  t75 = Sin(var1[3]);
  t17 = -1.*t16;
  t18 = 1. + t17;
  t78 = Cos(var1[3]);
  t24 = 0.12675*t23;
  t26 = -0.426*t25;
  t27 = t24 + t26;
  t82 = t78*t9;
  t83 = -1.*t75*t14*t19;
  t84 = t82 + t83;
  t40 = -0.213*t39;
  t42 = 0.1881*t41;
  t43 = t40 + t42;
  t77 = t9*t75*t14;
  t79 = t78*t19;
  t80 = t77 + t79;
  t90 = -1.*t16*t8*t75;
  t91 = -1.*t13*t84;
  t92 = t90 + t91;
  t48 = 0.1881*t39;
  t49 = 0.213*t41;
  t50 = t48 + t49;
  t94 = t32*t80;
  t95 = t5*t92;
  t96 = t94 + t95;
  t98 = t5*t80;
  t99 = -1.*t32*t92;
  t100 = t98 + t99;
  t59 = -0.426*t23;
  t60 = -0.12675*t25;
  t61 = t59 + t60;
  t86 = -1.*t8*t13*t75;
  t87 = t16*t84;
  t88 = t86 + t87;
  t106 = t37*t96;
  t107 = t41*t100;
  t108 = t106 + t107;
  t124 = t9*t75;
  t125 = t78*t14*t19;
  t126 = t124 + t125;
  t120 = -1.*t78*t9*t14;
  t121 = t75*t19;
  t122 = t120 + t121;
  t132 = t16*t78*t8;
  t133 = -1.*t13*t126;
  t134 = t132 + t133;
  t136 = t32*t122;
  t137 = t5*t134;
  t138 = t136 + t137;
  t140 = t5*t122;
  t141 = -1.*t32*t134;
  t142 = t140 + t141;
  t128 = t78*t8*t13;
  t129 = t16*t126;
  t130 = t128 + t129;
  t148 = t37*t138;
  t149 = t41*t142;
  t150 = t148 + t149;
  p_output1[0]=-0.04675*t13*t14 + t27*t30 + 0.1881*t32*t35 + t43*t46 + t50*t53 + 0.1881*(-1.*t41*t46 + t37*t53) + t61*t64 - 0.426*(-1.*t25*t30 + t21*t64) + 0.12675*(t21*t30 + t25*t64) - 0.04675*t18*t19*t8 + 0.1881*t7*t8*t9 + var1[0];
  p_output1[1]=t100*t50 + t108*t61 + 0.04675*t13*t75*t8 + 0.1881*t7*t80 + 0.04675*t18*t84 + t27*t88 + 0.12675*(t108*t25 + t21*t88) - 0.426*(t108*t21 - 1.*t25*t88) + 0.1881*t32*t92 + t43*t96 + 0.1881*(t100*t37 - 1.*t41*t96) + var1[1];
  p_output1[2]=0.04675*t126*t18 - 0.426*(t150*t21 - 1.*t130*t25) + 0.12675*(t130*t21 + t150*t25) + t130*t27 + 0.1881*t134*t32 + 0.1881*(t142*t37 - 1.*t138*t41) + t138*t43 + t142*t50 + t150*t61 + 0.1881*t122*t7 - 0.04675*t13*t78*t8 + var1[2];
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


#include "../../include/Expressions/FL_foot.hh"

namespace SymFunction
{

void FL_foot_raw(double *p_output1, const double *var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}

}

