/*
 * Automatically Generated from Mathematica.
 * Fri 8 Mar 2024 11:41:28 GMT-06:00
 */

#ifndef PTOE_HH
#define PTOE_HH

#ifdef MATLAB_MEX_FILE
// No need for external definitions
#else // MATLAB_MEX_FILE


#include "../include/frost_codegen/math2mat.hpp"
#include "../include/frost_codegen/mdefs.hpp"

namespace SymFunction
{

  void pToe_raw(double *p_output1, const double *var1);

  inline void pToe(Eigen::MatrixXd &p_output1, const Eigen::VectorXd &var1)
  {
    // Check
    // - Inputs
    assert_size_matrix(var1, 7, 1);

	
    // - Outputs
    assert_size_matrix(p_output1, 1, 3);


    // set zero the matrix
    p_output1.setZero();


    // Call Subroutine with raw data
    pToe_raw(p_output1.data(), var1.data());
    }
  
  
}

#endif // MATLAB_MEX_FILE

#endif // PTOE_HH
