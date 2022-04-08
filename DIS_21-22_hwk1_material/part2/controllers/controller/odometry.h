#ifndef ODOMETRY_H
#define ODOMETRY_H 

/*CONSTANTES*/
#define WHEEL_AXIS 		0.052 		// Distance between the two wheels in meter
#define WHEEL_RADIUS 	0.0205		// Radius of the wheel in meter

#define RAD2DEG(X)      X / M_PI * 180.0

typedef struct 
{
  double x;
  double y;
  double heading;
} pose_t;

void odo_compute_acc(pose_t* odo, const double acc[3], const double acc_mean[3]);
void odo_compute_gyro(pose_t* odo, const double gyro[3]);
void odo_compute_encoders(pose_t* odo, double Aleft_enc, double Aright_enc);
void odo_reset(int time_step);

#endif