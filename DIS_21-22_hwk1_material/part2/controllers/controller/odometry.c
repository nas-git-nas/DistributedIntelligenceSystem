#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include "odometry.h"

//-----------------------------------------------------------------------------------//

/*VERBOSE_FLAGS*/
#define VERBOSE_ODO_ACC 	true		// Print odometry values computed with accelerometer 
#define VERBOSE_ODO_GYRO 	false		// Print odometry values computed with gyroscope  
#define VERBOSE_ODO_ENC 	false     	// Print odometry values computed with wheel encoders 
//-----------------------------------------------------------------------------------//
/*GLOBAL*/
static double _T;

static pose_t _odo;
//-----------------------------------------------------------------------------------//

/**
 * @brief      Compute the position using the accelerometer
 *
 * @param      odo       The current odometry
 * @param[in]  acc       The acceleration
 * @param[in]  acc_mean  The acc mean
 */
void odo_compute_acc(pose_t* odo, const double acc[3], const double acc_mean[3])
{
	memcpy(&_odo,odo,sizeof(_odo));

	// To Do: Implement your accelerometer based odometry using the current state
	// estimate stored in _odo and the provided measurements passed as arguments,
	// and update the values stored in _odo accordingly.
	
	
	double ax = 0;
	double ay = 0;
	static double vx = 0;
	static double vy = 0;

           // calc. acceleration in global reference frame
	ax = cos(_odo.heading)*acc[0] - sin(_odo.heading)*acc[1];
	ay = sin(_odo.heading)*acc[0] + cos(_odo.heading)*acc[1];
	//calc. speed in global reference frame
	vx += _T*ax;
	vy += _T*ay;
	// calc. position in global reference frame
	_odo.x += _T*vx + 0.5*_T*_T*ax;
	_odo.y += _T*vy + 0.5*_T*_T*ay;
	
	memcpy(odo, &_odo, sizeof(_odo));
	
	if(VERBOSE_ODO_ACC)
	{
    	  //printf("x = %.3lf, y = %.3lf, vx = %.3lf, vy = %.3lf\n", x, y, vx, vy);
    	  printf("x = %.3lf, y = %.3lf, vx = %.3lf, vy = %.3lf\n", _odo.x, _odo.y, vx, vy);
    	  //printf("ODO with accelerometer : %.3lf %.3lf [m], (%.3lf [deg])\n", odo->x , odo->y , RAD2DEG(odo->heading));
    	  //printf("acceleration : %.3lf %.3lf [m]\n", acc[0] , acc[1]);
            }
}


/**
 * @brief      Compute the heading using the gyroscope 
 *
 * @param      odo       The current odometry
 * @param[in]  gyro      The angular velocity 
 */
void odo_compute_gyro(pose_t* odo, const double gyro[3])
{
	memcpy(&_odo,odo,sizeof(_odo));

	// To Do : Update the heading (_odo.heading)
	
	_odo.heading += _T*gyro[2];

	memcpy(odo, &_odo, sizeof(_odo));
	
	if(VERBOSE_ODO_GYRO)
	{
	  printf("ODO with gyroscope : (%.3lf %.3lf [m]), %.3lf [deg]\n", odo->x , odo->y , RAD2DEG(odo->heading));
	  //printf("Gyroscope : [%.3lf, %.3lf, %.3lf]\n", gyro[0], gyro[1], gyro[2]);
	  
	}
}

/**
 * @brief      Compute the odometry using the encoders
 *
 * @param      odo         The odometry pose to update 
 * @param[in]  Aleft_enc   The delta of the left encoder
 * @param[in]  Aright_enc  The delta of the right encoder
 */
void odo_compute_encoders(pose_t* odo, double Aleft_enc, double Aright_enc)
{
	// Copy the current odometry 
	memcpy(&_odo, odo, sizeof(_odo));

	///////////////////////////////////////////////////////////////////////////////
	// To Do : Update the variable _odo that stores the current odometry estimate:
	_odo.x += (WHEEL_RADIUS/2)*(Aleft_enc+Aright_enc)
      	          *cos(_odo.heading + (WHEEL_RADIUS/(2*WHEEL_AXIS))*(Aright_enc-Aleft_enc));
	_odo.y += (WHEEL_RADIUS/2)*(Aleft_enc+Aright_enc)
      	          *sin(_odo.heading + (WHEEL_RADIUS/(2*WHEEL_AXIS))*(Aright_enc-Aleft_enc));
      	_odo.heading += (WHEEL_RADIUS/WHEEL_AXIS)*(Aright_enc-Aleft_enc);

	///////////////////////////////////////////////////////////////////////////////

	memcpy(odo, &_odo, sizeof(_odo));

	if(VERBOSE_ODO_ENC)
    	printf("ODO with wheel encoders : %.3lf %.3lf [m], %.3lf [deg]\n", odo->x , odo->y , RAD2DEG(odo->heading) );
}

/**
 * @brief      Reset the odometry to zeros
 *
 * @param[in]  time_step  The time step used in the simulation in miliseconds
 */
void odo_reset(int time_step)
{
	memset(&_odo, 0 , sizeof(_odo));

	_T = time_step / 1000.0;
}
