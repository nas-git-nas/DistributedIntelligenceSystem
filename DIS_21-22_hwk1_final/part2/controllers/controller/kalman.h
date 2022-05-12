#ifndef KALMAN_H
#define KALMAN_H 

#include "odometry.h"

// These methods must be callable from the C controller 
#ifdef __cplusplus
extern "C" {
#endif

    /**
     * @brief   Initializes/resets the kalman filter. 
     * @param   timestep (int) simulation timestep in ms
     * @param   origin (pose_t) initial pose in the world 
     */
    void kal_reset(int timestep, pose_t origin);

    void kal_predict(pose_t* odo, double* acc, double* acc_mean);

    void kal_update_gyro(pose_t* odo, double* gyro);
    void kal_update_enc(pose_t* odo, double Aleft_enc, double Aright_enc);
    void kal_update_gps(pose_t* odo, double* gps);

    int kal_get_dim();
    void kal_get_state(double* state);
    void kal_get_state_covariance(double* cov);

#ifdef __cplusplus
}
#endif

#endif //KALMAN_H