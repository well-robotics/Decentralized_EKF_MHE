/*
 * Automatically Generated from Mathematica.
 * Fri 10 May 2024 15:36:34 GMT-05:00
 */

#ifndef FL_FOOT_HH
#define FL_FOOT_HH

#ifdef MATLAB_MEX_FILE
// No need for external definitions
#else // MATLAB_MEX_FILE


#include "math2mat.hpp"
#include "mdefs.hpp"

namespace SymFunction
{

  void FL_foot_raw(double *p_output1, const double *var1);

  inline void FL_foot(Eigen::MatrixXd &p_output1, const Eigen::VectorXd &var1)
  {
    // Check
    // - Inputs
    assert_size_matrix(var1, 22, 1);

	
    // - Outputs
    assert_size_matrix(p_output1, 1, 3);


    // set zero the matrix
    p_output1.setZero();


    // Call Subroutine with raw data
    FL_foot_raw(p_output1.data(), var1.data());
    }
  
  
}

#endif // MATLAB_MEX_FILE

#endif // FL_FOOT_HH
