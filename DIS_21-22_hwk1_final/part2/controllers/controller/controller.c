#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// base webots libraries
#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/keyboard.h>
#include <webots/motor.h>

// webots sensor libraries 
#include <webots/accelerometer.h> 
#include <webots/position_sensor.h>
#include <webots/distance_sensor.h>
#include <webots/accelerometer.h>
#include <webots/gyro.h>
#include <webots/gps.h> 

#include "kalman.h" // this includes odometry.h 

//-----------------------------------------------------------------------------------//
// You can play with these values!

/*NAVIGATION OPTION*/
enum NavTypes {BRAITENBERG=0, STRAIGHT_LINE, CURVE, WAYPOINTS};
//const static int nav = BRAITENBERG;   // Choose the type of navigation to perform 
const static int nav = STRAIGHT_LINE;
//const static int nav = WAYPOINTS;
//const static int nav = CURVE;

/*ACCELEROMETER CALIBRATION*/
#define TIME_INIT_ACC         5       // Time in second for accelerometer calibration (disable by setting -1)

/*VERBOSE FLAGS*/
#define VERBOSE_GROUND_TRUTH  false   // Print ground truth 
#define VERBOSE_ENC           false   // Print encoder values
#define VERBOSE_ACC           false   // Print accelerometer values 
#define VERBOSE_ACC_MEAN      false   // Print mean accelerometer values 
#define VERBOSE_GYRO          false   // Print gyroscope values 
#define VERBOSE_GPS           false   // Print gps values 
#define VERBOSE_BRAITENBERG   false    // Print correction if symmetry is detected

/*KALMAN FILTER FLAGS*/
#define FUSE_GYRO             true    // Fuse the gyroscope data 
#define FUSE_ENC              true    // Fuse the encoder data 
#define FUSE_GPS              true    // Fuse the gps data 
//-----------------------------------------------------------------------------------//

/*MACRO*/
#define CATCH(X,Y)      X = X || Y
#define CATCH_ERR(X,Y)  controller_error(X, Y, __LINE__, __FILE__)

/*DISTANCE SENSORS*/
#define NB_SENSORS 8

/*CONSTANTS*/
#define MAX_SPEED_WEB 6.279     // Maximum speed webots
#define MAX_DIST_RESP 4095      // Maximum distance sensor response 

/*BRAITENBERG CONTROLLER*/
#define IR_SENSOR_THRESHOLD 80
#define INITIAL_SPEED 3.0
#define IR_SENSOR_BRAKE_SYMMETRY 90
#define BRAKE_SYMMETRY 2.0

//-----------------------------------------------------------------------------------//
/*DEFINITIONS*/

typedef struct
{
  int time_step;
  WbNodeRef ref; // a reference to the robot itself 
  WbDeviceTag ps[NB_SENSORS];
  WbDeviceTag left_encoder;
  WbDeviceTag right_encoder;
  WbDeviceTag left_motor; 
  WbDeviceTag right_motor; 
  WbDeviceTag accelerometer;
  WbDeviceTag gyroscope; 
  WbDeviceTag gps; 
} simulation_t;

typedef struct 
{
  double ds_values[NB_SENSORS];
  double ds_ranges[NB_SENSORS];
  double prev_left_enc;
  double left_enc;
  double prev_right_enc;
  double right_enc;
  double acc[3];
  double acc_mean[3];
  double gyro[3];
  double gps[3];
} measurement_t;

//-----------------------------------------------------------------------------------//
/*VARIABLES*/
static simulation_t   _robot;
static measurement_t  _meas;
static pose_t         _ground_truth, _odo_imu, _odo_enc, _kalman;

static FILE *fp;

//-----------------------------------------------------------------------------------//
/*FUNCTIONS DECLARATION*/

static void constraint_heading(double* heading);

static bool controller_init();
static bool controller_init_time_step();
static bool controller_init_distance_sensors();
static bool controller_init_encoder();
static bool controller_init_motors();
static bool controller_init_accelerometer();
static bool controller_init_gyroscope(); 
static bool controller_init_gps();
static bool controller_init_log(const char* filename);

static void controller_get_distance();
static void controller_get_ground_truth(bool init);
static void controller_get_encoder();
static void controller_get_accelerometer();
static void controller_get_gyroscope();
static void controller_get_gps();

static void controller_compute_mean_acc();
static void controller_set_speed();

static void controller_print_log(double time);

static bool controller_error(bool test, const char * message, int line, const char * fileName);

//-----------------------------------------------------------------------------------//

int main() 
{

  // initialize the webots controller library
  wb_robot_init();
  
  if(CATCH_ERR(controller_init(), "Controller failed to init\n"))
    return 1;

  while (wb_robot_step(_robot.time_step) != -1) 
  {
    // Read the sensors
    controller_get_ground_truth(false);
    controller_get_distance();
    controller_get_encoder();
    controller_get_accelerometer();
    controller_get_gyroscope();
    controller_get_gps();

    // Compute mean acceleration 
    if( wb_robot_get_time() < TIME_INIT_ACC ){
      controller_compute_mean_acc();
    }
    else{
      // Odometry with IMU (acc + gyro) 
      odo_compute_acc(&_odo_imu, _meas.acc, _meas.acc_mean);
      odo_compute_gyro(&_odo_imu, _meas.gyro);
      constraint_heading(&_odo_imu.heading);

      // Odometry with encoders 
      odo_compute_encoders(&_odo_enc, _meas.left_enc - _meas.prev_left_enc, _meas.right_enc - _meas.prev_right_enc);
      constraint_heading(&_odo_enc.heading);
      
      // Kalman filter prediction
      kal_predict(&_kalman, _meas.acc, _meas.acc_mean);

      // Kalman filter updates
      if(FUSE_GYRO)
        kal_update_gyro(&_kalman, _meas.gyro);
      
      if(FUSE_ENC)
        kal_update_enc(&_kalman, _meas.left_enc - _meas.prev_left_enc, _meas.right_enc - _meas.prev_right_enc);
      
      if(FUSE_GPS){
        static int gps_counter = 0;
        if(gps_counter > 100){
          kal_update_gps(&_kalman, _meas.gps);
          gps_counter = 0;
        }else gps_counter++;
      }
      constraint_heading(&_kalman.heading);
      
      // Set the wheels' speed 
      controller_set_speed();
    }
	  // Log the data 
	  controller_print_log(wb_robot_get_time());
  }

  // Close the log file
  if(fp != NULL)
    fclose(fp);

  // End of the simulation
  wb_robot_cleanup();

  return 0;
}

//--------------------------------------ACCESS SENSORS---------------------------------------------//

/**
 * @brief   Get the ground truth. The data retrieved here is comparable to what an MCS could provide. 
 *          The data returned is expressed in the world frame. 
 * @param   init (bool) whether to set the estimated poses with the acquired measurement. 
 */
void controller_get_ground_truth(bool init){

  // Get the robot orientation in the World frame 
  const double* o = wb_supervisor_node_get_orientation(_robot.ref); 
  if(o != NULL){
    // Assume pure rotation around world z-axis (FLU). 
    // Use the x-projection of the 3D rotation matrix (1st column)
    // heading = atan2(cos,sin) (because atan2 expects y,x)
    // Result in range ]-pi,pi]
    _ground_truth.heading = atan2(o[3],o[0]); 
  }

  // Get the robot position in the world frame  
  const double* p = wb_supervisor_node_get_position(_robot.ref);
  if(p != NULL){
    _ground_truth.x = p[0];  // X in world frame 
    _ground_truth.y = p[1];  // Y in world frame 
  }

  constraint_heading(&_ground_truth.heading);

  // Set the initial pose 
  if(init){
    memcpy(&_odo_imu,&_ground_truth,sizeof(_odo_imu));
    memcpy(&_odo_enc,&_ground_truth,sizeof(_odo_enc));
    memcpy(&_kalman, &_ground_truth,sizeof(_kalman ));
  }
  if(VERBOSE_GROUND_TRUTH)
    printf("Ground truth : %.3lf %.3lf [m], %.3lf [deg]\n", _ground_truth.x, _ground_truth.y,RAD2DEG(_ground_truth.heading));
}

/**
 * @brief     Get the distance measurements 
 */
void controller_get_distance(){
  
  // read sensor values
  for (int i = 0; i < NB_SENSORS; i++){

    // To Do: read the range sensor ranges and store them into _meas.ds_values[i].  
    // Use wb_distance_sensor_get_value and the sensor handles _robot.ps[i]. 
    _meas.ds_values[i] = wb_distance_sensor_get_value(_robot.ps[i]); // range: 0 (far) to 4095 (0 distance (in theory))
  }
}


/**
 * @brief      Read the encoders values from the sensors
 */
void controller_get_encoder()
{
  // To Do : Store previous value of the left encoder
  _meas.prev_left_enc = _meas.left_enc;

  // To Do : Read the angle value of the left wheel with wb_position_sensor_get_value
  // Use the device tag _robot.left_encoder
  _meas.left_enc = wb_position_sensor_get_value(_robot.left_encoder);

  // To Do : Store previous value of the right encoder
  _meas.prev_right_enc = _meas.right_enc;
  
  // To Do : Read the angle value of the right wheel with wb_position_sensor_get_value
  // Use the device tag _robot.right_encoder
  _meas.right_enc = wb_position_sensor_get_value(_robot.right_encoder);

  if(VERBOSE_ENC)
    printf("ROBOT enc : %.3lf %.3lf [rad]\n", _meas.left_enc, _meas.right_enc);
}

/**
 * @brief      Read the acclerometer data 
 */
void controller_get_accelerometer(){
  // To Do : Call the function wb_accelerometer_get_values to get the accelerometer measurements. 
  // Uncomment and complete the following line. Note : Use _robot.accelerometer
  const double * acc_values = wb_accelerometer_get_values(_robot.accelerometer);

  // To Do : Copy the acc_values into the measurment structure _meas.acc (use memcpy)
  memcpy(_meas.acc, acc_values, sizeof(_meas.acc));

  if(VERBOSE_ACC)
    printf("ROBOT acc : %.3lf %.3lf %.3lf [m/s2]\n", _meas.acc[0], _meas.acc[1] , _meas.acc[2]);
}


/**
 * @brief   Read the gyroscope data. 
 *          Following looktable applies [ -13.315805 -100000 0.003 13.315805 100000 0.003 ]  
 */
void controller_get_gyroscope(){

  // To Do : Call the function wb_gyro_get_values to get the gyroscope measurements. 
  // Uncomment and complete the following line. Note : Use _robot.gyroscope
  const double * gyro_values = wb_gyro_get_values(_robot.gyroscope);

  // To Do : Copy the gyro_values into the measurment structure _meas.gyro (use memcpy)
  memcpy(_meas.gyro, gyro_values, sizeof(_meas.gyro));

  // Apply lookup table (linear function)
  static const double c_ = 13.315805/100000.;
  for (int i=0; i<3; i++) _meas.gyro[i] *= c_;

  if(VERBOSE_GYRO)
    printf("ROBOT gyro : %.3lf %.3lf %.3lf [rad/s]\n", _meas.gyro[0], _meas.gyro[1] , _meas.gyro[2]);
}

/**
 * @brief      Read the gps data 
 */
void controller_get_gps(){
  
  // To Do : Call the function wb_gps_get_values to get the gps measurements. 
  // Uncomment and complete the following line. Note : Use _robot.gps
  const double * gps_values = wb_gps_get_values(_robot.gps);


  // To Do : Copy the gps_values into the measurment structure _meas.gps (use memcpy)
  memcpy(_meas.gps, gps_values, sizeof(_meas.gps));

  if(VERBOSE_GPS)
    printf("ROBOT gps : %.3lf %.3lf %.3lf [m]\n", _meas.gps[0], _meas.gps[1] , _meas.gps[2]);
}

//-------------------------------------CONTROL----------------------------------------------//

/**
 * @brief      Compute the mean of the 3-axis accelerometer. The result is stored in array _meas.acc
 */
void controller_compute_mean_acc()
{
  static int count = 0, n = 0;
  
  count++;
  
  if( count > 20 ) // Remove the effects of strong acceleration at the begining
  {
    n++;
    for(int i = 0; i < 3; i++)  
        _meas.acc_mean[i] = (_meas.acc_mean[i] * (n - 1) + _meas.acc[i]) / (double) n;
  }
  
  if( count == (int) (TIME_INIT_ACC / (double) _robot.time_step * 1000) )
    printf("Accelerometer initialization Done ! (%d measurements) \n", n);

  if(VERBOSE_ACC_MEAN)
        printf("ROBOT acc mean : %.3lf %.3lf %.3lf\n", _meas.acc_mean[0], _meas.acc_mean[1] , _meas.acc_mean[2]);
}

/**
 * @brief   Use a braitenberg controller to set the speed of the motors 
 */
void controller_set_speed(){

  float msl_w = 0., msr_w = 0.; // wheel speeds 

  switch (nav)
  {
  case BRAITENBERG:
    {
      // To Do: Implement you Braitenberg solution here and 
      // set the left and right wheel velocities in rad/s. 
      // Use the _meas.ds_values variable. 
      const double l_weight[NB_SENSORS] = {-0.035, -0.05, -0.03, 0.025, 0.025, 0.03, 0.05, 0.035};
      const double r_weight[NB_SENSORS] = {0.035, 0.05, 0.03, 0.025, 0.025, -0.03, -0.05, -0.035};
      
      double left_speed = INITIAL_SPEED;
      double right_speed = INITIAL_SPEED;
      static double left_correction = 0;
      static double right_correction = 0;
           
      for (int i = 0; i < NB_SENSORS; i++)
      {
        if(_meas.ds_values[i]>IR_SENSOR_THRESHOLD)
        {
          left_speed += l_weight[i]*_meas.ds_values[i];
          right_speed += r_weight[i]*_meas.ds_values[i];
        }
      }
      
      if(_meas.ds_values[0]>IR_SENSOR_BRAKE_SYMMETRY && _meas.ds_values[7]>IR_SENSOR_BRAKE_SYMMETRY)
      {
        left_correction -= BRAKE_SYMMETRY;
        right_correction += BRAKE_SYMMETRY;
        
        if(VERBOSE_BRAITENBERG)
        {
          printf("correction necessary");
          printf("left speed = %f, right speed = %f\n", left_speed, right_speed);
        }
      }
      else
      {
        left_correction = left_correction/2;
        right_correction = right_correction/2;
      }
      
      msl_w =  left_speed + left_correction; // left  wheel velocity in rad/s
      msr_w =  right_speed + right_correction; // right wheel velocity in rad/s
    }
    break;
  
  case STRAIGHT_LINE:
    msl_w = 5.;
    msr_w = 5.;
    if(fabs(_ground_truth.x) > .4 || fabs(_ground_truth.y) > .4){
      msl_w = 0.;
      msr_w = 0.;
    }
    break;

  case CURVE:
    msl_w = .5;
    msr_w = 1.;
    if(fabs(_ground_truth.heading) > M_PI/2.){
      msl_w = 0.;
      msr_w = 0.;
      // Force pause the simulation to prevent the state from diverging
      // because of integrated measurement error.  
      wb_supervisor_simulation_set_mode(WB_SUPERVISOR_SIMULATION_MODE_PAUSE);
    }
    break;

  case WAYPOINTS:
    {
      // follow a predefined path 
      static int p = 0;
      static bool tour_complete = false;

      static const double waypoints[][2] = {
                                              {0,0},
                                              {0.3,0.},
                                              {0.4,0.4},
                                              {0.,0.3},
                                              {-0.4,0.4},
                                              {-0.3,0.},
                                              {-0.4,-0.4},
                                              {0,-0.3},
                                              {0.4,-0.4},
                                              {0.3,0.}
                                            };
      
      static int num_waypoints = sizeof(waypoints)/sizeof(double)/2; 

      double x = _ground_truth.x, y = _ground_truth.y;
      double vec[2] = {waypoints[p][0]-x, waypoints[p][1]-y}; // vector from robot to waypoint 
      double heading_correction = atan2(vec[1],vec[0]) - _ground_truth.heading;
      constraint_heading(&heading_correction);
      double distance = sqrt(pow(vec[0],2)+pow(vec[1],2)); 

      static const double ref_speed = 5.;
      msl_w = ref_speed*(-heading_correction*.5 + distance);
      msr_w = ref_speed*( heading_correction*.5 + distance);

      if(distance < 0.01){p++; printf("Reached waypoint %d\n",p-1);};

      if(p >= num_waypoints){
        p = 0;
        tour_complete = true;
      }else if(p > 0 && tour_complete){
        msl_w = 0.; 
        msr_w = 0.;
      }
    }
    break;
  
  default:
    printf("FATAL: navigation type not supported\n");
    exit(EXIT_FAILURE);
    break;
  }

  // constrain speeds within feasible values 
  msl_w = msl_w > MAX_SPEED_WEB ? MAX_SPEED_WEB : 
            (msl_w < -MAX_SPEED_WEB ? -MAX_SPEED_WEB : msl_w);
  msr_w = msr_w > MAX_SPEED_WEB ? MAX_SPEED_WEB : 
            (msr_w < -MAX_SPEED_WEB ? -MAX_SPEED_WEB : msr_w);
  wb_motor_set_velocity(_robot.left_motor,  msl_w);
  wb_motor_set_velocity(_robot.right_motor, msr_w);
}

//---------------------------------------INIT----------------------------------------------//

/**
 * @brief      Run the initialization. Set the variables and structure to 0. Try to initialize the Webots components. 
 *
 * @return     Return true if it rise and error
 */
bool controller_init()
{

  bool err = false;

  memset(&_robot, 0 , sizeof(simulation_t));

  memset(&_meas, 0 , sizeof(measurement_t));

  memset(&_ground_truth, 0 , sizeof(pose_t));
  
  memset(&_odo_imu, 0 , sizeof(pose_t));

  memset(&_odo_enc, 0 , sizeof(pose_t));

  memset(&_kalman, 0 , sizeof(pose_t));

  _robot.ref = wb_supervisor_node_get_self();

  CATCH(err,controller_init_time_step());

  controller_get_ground_truth(true);

  CATCH(err,controller_init_distance_sensors());

  CATCH(err,controller_init_encoder());

  CATCH(err,controller_init_motors());

  CATCH(err,controller_init_accelerometer());

  CATCH(err,controller_init_gyroscope());

  CATCH(err,controller_init_gps());

  CATCH(err, controller_init_log("data.csv"));
  
  wb_keyboard_enable(_robot.time_step);

  odo_reset(_robot.time_step);

  kal_reset(_robot.time_step, _kalman);

  return err;
}

/**
 * @brief      Initialize the simulation time step on Webots
 *
 * @return     return true if it fails
 */
bool controller_init_time_step()
{
  _robot.time_step =  wb_robot_get_basic_time_step();

  return CATCH_ERR(_robot.time_step == 0,"time step is not set\n");
}

/**
 * @brief      Initialize the distance sensors from Webots
 *
 * @return     return true if it fails
 */
bool controller_init_distance_sensors(){

  bool err = false; 
  // get and enable each distance sensor
  char name[] = "ps0";
  for(int i = 0; i < NB_SENSORS; i++) {

    _robot.ps[i] = wb_robot_get_device(name);

    err = CATCH_ERR(_robot.ps[i] == 0, "No distance sensor node found in the current robot file\n");

    if( !err ){
      wb_distance_sensor_enable(_robot.ps[i], _robot.time_step);
    }
    else
      return err; 
    
    name[2]++; // increase the device name to "ps1", "ps2", etc.
  }
  return err; 
}


/**
 * @brief      Initiliaze the the wheel encoders (position sensors) from Webots
 *
 * @return     return true if it fails
 */
bool controller_init_encoder()
{
  // Get the device from Webots
  _robot.left_encoder = wb_robot_get_device("left wheel sensor");
  // Write an error message if the initialization fails
  bool err = CATCH_ERR(_robot.left_encoder == 0, "No left wheel sensor node found in the current robot file\n");

  _robot.right_encoder = wb_robot_get_device("right wheel sensor");

  CATCH(err,CATCH_ERR(_robot.right_encoder == 0, "No right wheel sensor node found in the current robot file\n"));
  
  if( !err ) // if no error initialize the sensors
  {
    wb_position_sensor_enable(_robot.left_encoder,  _robot.time_step);
    wb_position_sensor_enable(_robot.right_encoder, _robot.time_step);
  }

  return err;
}


/**
 * @brief      Initiliaze the the wheel motors from Webots
 *
 * @return     return true if it fails
 */
bool controller_init_motors()
{
  // To Do : Get the left motors device from webots. Note : the name of the gps is "left wheel motor"
  _robot.left_motor = wb_robot_get_device("left wheel motor");

  bool err = CATCH_ERR(_robot.left_motor == 0, "No left wheel motor node found in the current robot file\n");

  // To Do : Get the right motors device from webots. Note : the name of the gps is "right wheel motor"
  _robot.right_motor = wb_robot_get_device("right wheel motor");

  CATCH(err,CATCH_ERR(_robot.left_motor == 0, "No right wheel motor node found in the current robot file\n"));
  
  if( !err )
  {
    wb_motor_set_position(_robot.left_motor, INFINITY);   // To Do : Set the left motor position to INFINITY.    Note : use _robot.left_motor
    wb_motor_set_position(_robot.right_motor, INFINITY);  // To Do : Set the right motor position to INFINITY.   Note : use _robot.right_motor
    wb_motor_set_velocity(_robot.left_motor, 0.0);        // To Do : Set the left motor speed to 0.              Note : use _robot.left_motor
    wb_motor_set_velocity(_robot.right_motor, 0.0);       // To Do : Set the right motor speed to 0.             Note : use _robot.right_motor
  }

  return err;
}

/**
 * @brief      Initialize the accelerometer 
 *
 * @return     return true if it fails
 */
bool controller_init_accelerometer(){

  // Get the device from Webots
  _robot.accelerometer = wb_robot_get_device("accelerometer");

  // Write an error message if the initialization fails
  bool err = CATCH_ERR(_robot.accelerometer == 0, "No accelerometer node found in the current robot file\n");

  if( !err ) // if no error initialize the sensors
  {
    wb_accelerometer_enable(_robot.accelerometer, _robot.time_step);
  }

  return err;
}

/**
 * @brief      Initialize the gyroscope 
 *
 * @return     return true if it fails
 */
bool controller_init_gyroscope(){

  // Get the device from Webots
  _robot.gyroscope = wb_robot_get_device("gyro");

  // Write an error message if the initialization fails
  bool err = CATCH_ERR(_robot.gyroscope == 0, "No gyroscope node found in the current robot file\n");

  if( !err ) // if no error initialize the sensors
  {
    wb_gyro_enable(_robot.gyroscope, _robot.time_step);
  }

  return err;
}

/**
 * @brief      Initialize the gps  
 *
 * @return     return true if it fails
 */
bool controller_init_gps(){

  // Get the device from Webots
  _robot.gps = wb_robot_get_device("gps");

  // Write an error message if the initialization fails
  bool err = CATCH_ERR(_robot.gps == 0, "No gps node found in the current robot file\n");

  if( !err ) // if no error initialize the sensors
  {
    wb_gps_enable(_robot.gps, _robot.time_step);
  }

  return err;
}

/**
 * @brief      Do an error test if the result is true write the message in the stderr.
 *
 * @param[in]  test     The error test to run
 * @param[in]  message  The error message
 *
 * @return     true if there is an error
 */
bool controller_error(bool test, const char * message, int line, const char * fileName)
{
  if (test) 
  {
    char buffer[256];

    sprintf(buffer, "file : %s, line : %d,  error : %s", fileName, line, message);

    fprintf(stderr,"%s",buffer);

    return(true);
  }

  return false;
}

//---------------------------------------LOGGING---------------------------------------//

/**
 * @brief      Initialize the logging of the file
 *
 * @param[in]  filename  The filename to write
 *
 * @return     return true if it fails
 */
bool controller_init_log(const char* filename)
{

  fp = fopen(filename,"w");
  
  bool err = CATCH_ERR(fp == NULL, "Fails to create a log file\n");

  if( !err )
  {
    // published data headers 
    fprintf(fp, "time; \
gt_x; gt_y; gt_heading; \
odo_imu_x; odo_imu_y; odo_imu_heading; \
odo_enc_x; odo_enc_y; odo_enc_heading; \
kalman_x; kalman_y; kalman_heading; \
acc_x; acc_y; acc_z; \
gyro_x; gyro_y; gyro_z; \
gps_x; gps_y; gps_z; \
right_enc; left_enc; \
state_x; state_y; state_h; state_dx; state_dy; state_dh; \
cov_x; cov_y; cov_h; cov_dx; cov_dy; cov_dh;\n");
  }

  return err;
}

/**
 * @brief      Log the usefull informations about the simulation
 *
 * @param[in]  time  The time
 */
void controller_print_log(double time)
{

  if( fp != NULL)
  {

    // number of columns 
    static const int num_cols = 36; 

    // get the state and covariance
    static int dim; dim = kal_get_dim();
    double state[dim], cov[dim];
    kal_get_state(state);
    kal_get_state_covariance(cov);

    // build format string  
    static const char format[] = "%lf; "; 
    char str[100] = "";
    for(int i=0; i<num_cols; i++){
      strcat(str,format);
    }strcat(str,"\n");

    // log data at given time 
    fprintf(fp, str, 
          time,                                                     // 1
          _ground_truth.x, _ground_truth.y, _ground_truth.heading,  // 3
          _odo_imu.x, _odo_imu.y, _odo_imu.heading,                 // 3
          _odo_enc.x, _odo_enc.y, _odo_enc.heading,                 // 3
          _kalman.x, _kalman.y, _kalman.heading,                    // 3
          _meas.acc[0],_meas.acc[1],_meas.acc[2],                   // 3
          _meas.gyro[0],_meas.gyro[1],_meas.gyro[2],                // 3
          _meas.gps[0],_meas.gps[1],_meas.gps[2],                   // 3
          _meas.right_enc, _meas.left_enc,                          // 2
          state[0],state[1],state[2],state[3],state[4],state[5],    // 6
          cov[0],cov[1],cov[2],cov[3],cov[4],cov[5]                 // 6
    );
  }

}


/*--------------------------------UTILS---------------------------------------*/

/**
 * @brief   Constrain an angle within [-pi,pi]
 * 
 * @param   heading value to constrain
 */
void constraint_heading(double* heading){
  while(*heading > M_PI){
    *heading -= 2.0*M_PI; 
  }
  while(*heading < -M_PI){
    *heading += 2.0*M_PI; 
  }
}
