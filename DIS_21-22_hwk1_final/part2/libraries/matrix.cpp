#include "matrix.hpp"

void printm(const MatX& m, const std::string name){
    std::cout << (name == "" ? "Matrix:\n":name+":\n") << m << std::endl; 
}

void matrix_demo(){

    /* Matrix initialization */

    Mat Mz = Mat::Zero();   // a DIMxDIM zero matrix
    printm(Mz, "Zero matrix");

    Mat Mr = Mat::Random(); // a DIMxDIM random matrix
    printm(Mr, "Random matrix");

    Mat Me {{1,24,3,4,54,6},  // a DIMxDIM matrix with brace initialization
            {91,2,39,4,5,6},  // Note there is NO "=" sign!! 
            {1,21,3,4,59,6},
            {1,2,3,44,5,6},
            {1,42,3,4,5,61},
            {11,2,3,4,59,6}};
    printm(Me, "Explicit matrix");

    MatX Mx {{1,5,3},   // A 2x3 matrix
             {9,3,6}};  // Note the dimension is set by specifying the elements of the matrix
    printm(Mx, "Arbitrary size matrix");

    /* Vector initialization */

    Vec vz = Vec::Zero();    // a DIMx1 zero vector
    printm(vz, "Zero vector");

    Vec ve {{1},    // a DIMx1 vector with brace initialization
            {2},    // Note there is NO "=" sign!! 
            {3},
            {4},
            {5},
            {6}};
    printm(ve, "Explicit vector");

    VecX vx {{1},   // A 3x1 vector 
             {2},
             {3}};
    printm(vx, "Arbitrary size vector");

    /* Access */

    printf("Element at location 2,3 in Mr is %lf\n", Mr(1,2)); // access the element of a matrix 
    Mr(1,2) = 1000.0;                                          // set a single element of a matrix
    printf("Element at location 2,3 in Mr is now %lf\n", Mr(1,2));
    printf("Element at index 2 in vx is %lf\n", vx(1));        // access the element of a vector

    /* Matrix operations */

    printm(Mx*vx, "Mx*vx");             // multiplication
    printm(Me.transpose(), "Me^T");     // transpose
    printm(Me.inverse(), "Me⁻¹");       // inverse
    printm(Me.inverse()*Me*ve, "Me⁻¹*Me*ve"); // combined operations
}
