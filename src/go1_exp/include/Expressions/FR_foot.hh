/*
 * Automatically Generated from Mathematica.
 * Fri 10 May 2024 15:36:36 GMT-05:00
 */

#ifndef FR_FOOT_HH
#define FR_FOOT_HH

#ifdef MATLAB_MEX_FILE
// No need for external definitions
#else // MATLAB_MEX_FILE


#include "math2mat.hpp"
#include "mdefs.hpp"

namespace SymFunction
{

  void FR_foot_raw(double *p_output1, const double *var1);

  inline void FR_foot(Eigen::MatrixXd &p_output1, const Eigen::VectorXd &var1)
  {
    // Check
    // - Inputs
    assert_size_matrix(var1, 22, 1);

	
    // - Outputs
    assert_size_matrix(p_output1, 1, 3);


    // set zero the matrix
    p_output1.setZero();


    // Call Subroutine with raw data
    FR_foot_raw(p_output1.data(), var1.data());
    }
  
  
}

#endif // MATLAB_MEX_FILE

#endif // FR_FOOT_HH
