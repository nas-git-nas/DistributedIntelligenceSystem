#pragma once 

#include <iostream>
#include <string>

#include "Eigen/Dense" 
typedef double dtype;  // data type to use for matrix computations 

#define DIM 6                                      // State dimension 

typedef Eigen::Matrix<dtype,DIM,DIM>   Mat;        // DIMxDIM matrix  
typedef Eigen::Matrix<dtype, -1, -1>   MatX;       // Arbitrary size matrix 
typedef Eigen::Matrix<dtype,DIM,  1>   Vec;        // DIMx1 column vector  
typedef Eigen::Matrix<dtype, -1,  1>   VecX;       // Arbitrary size column vector  

const Mat I = MatX::Identity(DIM,DIM);             // DIMxDIM identity matrix  

/**
 * @brief Print a matrix
 * @param m matrix to print
 * @param name (optional) name of the matrix
 */
void printm(const MatX& m, const std::string name="");

/**
 * @brief Simple demonstration function illustrating 
 * how to perform matrix operations. 
 */
void matrix_demo();