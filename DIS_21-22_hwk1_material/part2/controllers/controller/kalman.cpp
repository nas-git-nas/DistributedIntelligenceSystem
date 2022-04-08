
#include "kalman.h"
#include <math.h>
#include <memory.h>

#include "matrix.hpp"

#define MATRIX_DEMO false // whether to run the matrix_demo() function and exit 

/**
 * Check if the state has a NaN value. 
 * Exit if NaN detected. 
 */
void kal_check_nan();

/**
 * @brief Perform the update step of the Kalman filter.
 */
void kal_update(VecX z, MatX C, MatX Q);

static double _T = 1.0;

// State mu (x,y,heading,dx,dy,dheading)
static Vec mu; 

// State covariance sigma 
static Mat sigma;
						 

void kal_reset(int timestep, pose_t origin){
    
    // Run the matrix_demo function and exit 
    if(MATRIX_DEMO){
        matrix_demo(); 
        exit(EXIT_SUCCESS);
    }	
	
    _T = (double)timestep/1000.;
	
    // To Do : Initialize mu and sigma (assume perfect initial knowledge of pose)
    

    // To Do : Set mu with the origin 
    
}

void kal_check_nan(){
    for (int i=0;i<DIM;i++){
		if(isnan(mu(i))){
            printf("FATAL: kalman state is NaN, exiting...\n");
            exit(EXIT_FAILURE);
        }
	}
}


void kal_predict(pose_t* odo, double* acc, double* acc_mean){

    // To Do : Declare the matrices A and B 
    // Hint: use the types Mat and MatX respectively. 
    

    // To Do : Declare and compute the input vector u 
    // Hint: use the type VecX 
    

    // To Do : Declare and compute the matrix R
    // Hint: use the type Mat
    

	///****************** state vector update ******************//
	// To Do : compute the prediction step μ = A*μ + B*u
	
	
	///****************** noise vector update ******************//
	// To Do : compute the predicted covariance Σ = A*Σ*A^T + R
	

	// write result back to struct
	odo->x = mu(0);
	odo->y = mu(1);
	odo->heading = mu(2);

    kal_check_nan();
}

void kal_update(VecX z, MatX C, MatX Q){

    ///********* Kalman gain *********//
	// To Do : Compute the Kalman gain K = Σ*C^T*(C*Σ*C^T + Q)⁻¹
    
    
    ///********* Update state mu *********//
    // To Do : Compute the state update μ = μ + K*(z - C*μ)
    

    ///********* Update state covariance sigma *********//
    // To Do : Compute the state covariance update Σ = (I - K*C)*Σ
    

    kal_check_nan();
}

void kal_update_gyro(pose_t* odo, double* gyro){
    
    // To Do : Declare and initialize the measurement vector z (use VecX type)
    

    // To Do : Declare the matrix C (use MatX type)
    

    // To Do : Declare the matrix Q (use MatX type)
    

    // To Do : Call kal_update 
    

    // write result back to struct
	odo->x = mu(0);
	odo->y = mu(1);
	odo->heading = mu(2);

    kal_check_nan();
}

void kal_update_enc(pose_t* odo, double Aleft_enc, double Aright_enc)
{
	// To Do : Declare and initialize the measurement vector z (use VecX type)
    

    // To Do : Declare the matrix C (use MatX type)
    

    // To Do : Declare the matrix Q (use MatX type)
    

    // To Do : Call kal_update 


    // write result back to struct
	odo->x = mu(0);
	odo->y = mu(1);
	odo->heading = mu(2);

    kal_check_nan();
}

void kal_update_gps(pose_t* odo, double* gps){

    // To Do : Declare and initialize the measurement vector z (use VecX type)
    

    // To Do : Declare the matrix C (use MatX type)
    

    // To Do : Declare the matrix Q (use MatX type)
    

    // To Do : Call kal_update 
    

    // write result back to struct
	odo->x = mu(0);
	odo->y = mu(1);
	odo->heading = mu(2);

    kal_check_nan();
}

int kal_get_dim(){
    return DIM;
}
void kal_get_state(double* state){
    for(int i=0; i<DIM; i++){
        state[i] = mu(i);
    }
}
void kal_get_state_covariance(double* cov){
    for(int i=0; i<DIM; i++){
        cov[i] = sigma(i,i);
    }
}