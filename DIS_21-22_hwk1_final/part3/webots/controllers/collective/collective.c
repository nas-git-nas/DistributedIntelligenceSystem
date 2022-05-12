/*****************************************************************************/
/* File:         reynolds.c                                                  */
/* Version:      1.0                                                         */
/* Date:         10-Oct-14                                                   */
/* Description:  Reynolds flocking control 									 */
/*                                                                           */
/* Author:        10-Oct-14 by Ali marjovi			           				 */
/* Last revision: 16-March-22 by Lucas Waelti					 			 */
/*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/emitter.h>
#include <webots/receiver.h>

#include "../flock_param.h"

#define TIME_STEP			16	  	// [ms] Length of time step

#define NB_SENSORS	  		8	  	// Number of distance sensors
#define MAX_SENS          	4096    // Maximum sensibility value

#define MAX_SPEED_WEB      	6.28    // Maximum wheel speed webots

#define AXLE_LENGTH 		0.052	// Distance between wheels of robot (meters)
#define WHEEL_RADIUS		0.0205	// Wheel radius (meters)
#define DELTA_T	  (TIME_STEP/1000.0)	// Timestep (seconds)

#define VERBOSE              1
#define VERBOSE_BRAITENBERG  0
#define VERBOSE_REYNOLD      0

WbDeviceTag left_motor; 	// Handle for left wheel of the robot
WbDeviceTag right_motor; 	// Handle for the right wheel of the robot
WbDeviceTag ds[NB_SENSORS];	// Handle for the infrared distance sensors
WbDeviceTag receiver;		// Handle for the receiver node
WbDeviceTag emitter;		// Handle for the emitter node


int robot_id_u, robot_id;		// Unique and normalized (between 0 and FLOCK_SIZE-1) robot ID
float loc[FLOCK_SIZE][3];		// X, Y, Theta of all robots
float prev_loc[FLOCK_SIZE][3];	// Previous X, Y, Theta values
float speed[FLOCK_SIZE][2];		// Speeds in world frame calculated with Reynolds or Laplacian 
int initialized[FLOCK_SIZE];	// != 0 if initial positions have been received
float migr[2];	                // Migration vector

/*
 * Reset the robot's devices and get its ID
 *
 */
static void reset() {
	
	wb_robot_init();

	receiver = wb_robot_get_device("receiver");
	emitter = wb_robot_get_device("emitter");
	
	//get motors
	left_motor = wb_robot_get_device("left wheel motor");
	right_motor = wb_robot_get_device("right wheel motor");
	wb_motor_set_position(left_motor, INFINITY);
	wb_motor_set_position(right_motor, INFINITY);

	int i;
	char s[4]="ps0";
	for(i=0; i<NB_SENSORS;i++) {
		ds[i]=wb_robot_get_device(s);	// the device name is specified in the world file
		s[2]++;				// increases the device number
	}
	char* robot_name; 
	robot_name=(char*) wb_robot_get_name(); 

	for(i=0;i<NB_SENSORS;i++) {
		wb_distance_sensor_enable(ds[i],64); // limit update frequency
	}
	wb_receiver_enable(receiver,64); // limit update frequency

	//Reading the robot's name. Pay attention to name specification when adding robots to the simulation!
	sscanf(robot_name,"epuck%d",&robot_id_u); // read robot id from the robot's name
	robot_id = robot_id_u%FLOCK_SIZE;	  // normalize between 0 and FLOCK_SIZE-1

	// If the robot's index is beyond the flock size, turn leds on and exit 
	if(robot_id_u >= FLOCK_SIZE && NUM_FLOCKS == 1){
		printf("Robot %d not part of the flock, exited\n",robot_id_u);
		char led_name[5] = "led0";
		for(i=0;i<NB_SENSORS;i++){
			WbDeviceTag led = wb_robot_get_device(led_name);
			wb_led_set(led,1);
			led_name[3]++;
		}
		wb_motor_set_velocity(left_motor,  0);
		wb_motor_set_velocity(right_motor, 0);
		wb_robot_step(TIME_STEP);
		exit(EXIT_SUCCESS);	
	}

	for(i=0; i<FLOCK_SIZE; i++) {
		initialized[i] = 0; 		  // Set initialization to 0 (= not yet initialized)
	}

	printf("Reset: robot %d\n",robot_id_u);
}


/*
 * Keep given float number within interval {-limit, limit}
 */
void limitf(float *number, int limit) {

	if (*number > limit)
		*number = (float)limit;
	if (*number < -limit)
		*number = (float)-limit;
}

/*
 * Keep given int number within interval {-limit, limit}
 */
void limit(int *number, int limit) {

	if (*number > limit)
		*number = limit;
	if (*number < -limit)
		*number = -limit;
}

/*
 * recenter_angle to [-PI, PI]
 */
void recenter_angle(double* angle){
	if (*angle > M_PI){
		*angle -= 2*M_PI;
	}
	if (*angle < -M_PI){
		*angle += 2*M_PI;
	}
}

/**
 * Get the position and velocity of the robots 
 */
void get_robot_positions()
{
	int rob_nb;						// Robot number
	float rob_x, rob_y, rob_theta;  // Robot position and orientation
	float mx, my;					// Migratory direction 
	char *inbuffer;					// Buffer for the receiver node

	/* Get own position */
	int count = 0;
	while (wb_receiver_get_queue_length(receiver) > 0 && count < FLOCK_SIZE) 
	{
		inbuffer = (char*) wb_receiver_get_data(receiver);
		//sscanf(inbuffer,"%d#%f#%f#%f",&rob_nb,&rob_x,&rob_y,&rob_theta);
		sscanf(inbuffer,"%d#%f#%f#%f##%f#%f",&rob_nb,&rob_x,&rob_y,&rob_theta,&mx,&my);
		
		if ((int) rob_nb/FLOCK_SIZE == (int) robot_id_u/FLOCK_SIZE) {
			//printf("%d received info about %d (mig: %.2lf,%.2lf)\n",robot_id_u,rob_nb,migr[0],migr[1]);
			migr[0] = mx;
			migr[1] = my;
			rob_nb %= FLOCK_SIZE;
			if (initialized[rob_nb] == 0) {
				// Get initial positions
				loc[rob_nb][0] = rob_x; //x-position
				loc[rob_nb][1] = rob_y; //y-position
				loc[rob_nb][2] = rob_theta; //theta
				prev_loc[rob_nb][0] = loc[rob_nb][0];
				prev_loc[rob_nb][1] = loc[rob_nb][1];
				initialized[rob_nb] = 1;
			} else {
				// Get position update
				prev_loc[rob_nb][0] = loc[rob_nb][0];
				prev_loc[rob_nb][1] = loc[rob_nb][1];
				loc[rob_nb][0] = rob_x; //x-position
				loc[rob_nb][1] = rob_y; //y-position
				loc[rob_nb][2] = rob_theta; //theta
			}
			speed[rob_nb][0] = (1/DELTA_T)*(loc[rob_nb][0]-prev_loc[rob_nb][0]);
			speed[rob_nb][1] = (1/DELTA_T)*(loc[rob_nb][1]-prev_loc[rob_nb][1]);
			count++;
		}
		wb_receiver_next_packet(receiver);
	}
}

/**
 * @brief 	Update speed (world frame) according to Reynold's rules
 */
void reynolds_rules() {

       int i, j, k;			// Loop counters
	float avg_loc[2] 	= {0,0};	// Flock average location
	float avg_speed[2] 	= {0,0};	// Flock average speeds
	float cohesion[2] 	= {0,0};	// Cohesion command
	float dispersion[2] = {0,0};	// Dispersion command
	float alignment[2] 	= {0,0};	// Alignment command 
	float dist  = 0; // Use it to define distance between robots
	float n_robots = 1; // Use to assign initial number of robots
	
	
	// Compute the average speed and loc of the neighbourhood/flock 
	if (NEIGHBORHOOD) {
		// To Do : compute the average speed and location of the local neighbourhood.
		// Use the NEIGH_THRESHOLD constant as a maximal distance to consider neighbours. 
		for(i=0; i<FLOCK_SIZE; i++) {
			if (i == robot_id) {	
          			// don't consider yourself for the average 
				continue;
			}
			dist = sqrt(pow(loc[i][0]-loc[robot_id][0],2.0)+pow(loc[i][1]-loc[robot_id][1],2.0));
			if (dist < NEIGH_THRESHOLD){
				for (j=0;j<2;j++) {      		         
					avg_speed[j] += speed[i][j];
					avg_loc[j] += loc[i][j];
				}
				n_robots= n_robots+1;
			}	
		}

		for (j=0;j<2;j++) {
			if (n_robots >1){
				avg_speed[j] /= (n_robots-1);
				avg_loc[j] /= (n_robots-1) ;
			}
			else{
				avg_speed[j] = speed[robot_id][j];
				avg_loc[j] = loc[robot_id][j];
			}
		}
		
	} else {
		// To Do : compute the average speed and location of the whole flock. 
		for(i=0; i<FLOCK_SIZE; i++) {
			if (i == robot_id) {	
          			// don't consider yourself for the average 
				continue;
			}
			for (j=0;j<2;j++) {
				avg_speed[j] += speed[i][j];
				avg_loc[j] += loc[i][j];
				//printf("id = %d, speed_x = %f\n", i, speed[i][0]);
			}
		}

		for (j=0;j<2;j++) {
			avg_speed[j] /= FLOCK_SIZE-1;
			avg_loc[j] /= FLOCK_SIZE-1;
		}
	}
	
	/* Reynold's rules */
	
	/* To Do : Implement Rule 1 - Aggregation/Cohesion: move towards the center of mass */
	for (j=0;j<2;j++) {
		// If center of mass is too far
		if (sqrt(pow(loc[robot_id][0]-avg_loc[0],2)+pow(loc[robot_id][1]-avg_loc[1],2)) > RULE1_THRESHOLD) {
         		cohesion[j] = avg_loc[j] - loc[robot_id][j];   // Relative distance to the center of the swarm
         	}
         }
	
	/* To Do : Implement Rule 2 - Dispersion/Separation: keep far enough from flockmates */
       for (k=0;k<FLOCK_SIZE;k++) {
		if (k != robot_id) {        // Loop on flockmates only
			// If neighbor k is too close (Euclidean distance)
			if (pow(loc[robot_id][0]-loc[k][0],2)+pow(loc[robot_id][1]-loc[k][1],2) < RULE2_THRESHOLD) {
				for (int j=0;j<2;j++) {
					dispersion[j] += 1/(loc[robot_id][j] -loc[k][j]);	// Relative distance to k
				}
			}
		}
	}
	
	/* To Do : Implement Rule 3 - Alignment: match the speeds of flockmates */
	alignment[0] = 0;
	alignment[1] = 0;
	
	for (j=0;j<2;j++) {
		alignment[j] = avg_speed[j] - speed[robot_id][j];
	}
	
	if (VERBOSE_REYNOLD) {
              printf("Reynolds: id = %d, coh_x:%f coh_y:%f, dis_x:%f dis_y:%f, align_x:%f align_y:%f\n", 
			robot_id, cohesion[0], cohesion[1], dispersion[0], dispersion[1], alignment[0], alignment[1]);
	}
	
	for (j=0;j<2;j++) {
		speed[robot_id][j]  = cohesion[j]   * RULE1_WEIGHT;
		speed[robot_id][j] += dispersion[j] * RULE2_WEIGHT;
		speed[robot_id][j] += alignment[j]  * RULE3_WEIGHT;
	}
		
	// move the robot according to some migration rule
	if(MIGRATORY_URGE == 0){
		/* To Do : Implement migratory urge */
		speed[robot_id][0] += 0*0.01*cos(loc[robot_id][2] + M_PI/2);
		speed[robot_id][1] += 0*0.01*sin(loc[robot_id][2] + M_PI/2);
	} else {
		speed[robot_id][0] += MIGRATION_WEIGHT*(migr[0]-loc[robot_id][0]);
		speed[robot_id][1] += MIGRATION_WEIGHT*(migr[1]-loc[robot_id][1]); 
	}
	
	if (VERBOSE_REYNOLD) {
	   printf("After Migratory urge: id = %d, speed_x = %f, speed_y = %f\n", robot_id, speed[robot_id][0], speed[robot_id][1]);
	}

	
}

#ifdef LAPLACIAN 
void laplacian_formation(){

	//To Do: Implement the Laplacian formation controller
	//Hint: find the correct values of x_next and y_next

	//Compute the graph dependent velocity references in WF
	double x_next = 0; 
	double y_next = 0; 
	int i;
       
       for (i=0;i<FLOCK_SIZE;i++){
            x_next += -L[robot_id][i]*(loc[i][0]-bias[i][0]);
            y_next += -L[robot_id][i]*(loc[i][1]-bias[i][1]);
       }
       
       // printf("id = %d, x_next = %f, y_next = %f \n", robot_id, x_next, y_next);
             
       speed[robot_id][0] = x_next;
       speed[robot_id][1] = y_next;

	// To Do: Include the migratory urge in the controller
	if(MIGRATORY_URGE == 0){
		/* Implement migratory urge */
		speed[robot_id][0] += 0*0.01*cos(loc[robot_id][2] + M_PI/2);
		speed[robot_id][1] += 0*0.01*sin(loc[robot_id][2] + M_PI/2);
	} else {
		speed[robot_id][0] += MIGRATION_WEIGHT*(migr[0]-loc[robot_id][0]);
		speed[robot_id][1] += MIGRATION_WEIGHT*(migr[1]-loc[robot_id][1]); 
       }
}
#endif 
/**
 * @brief	Computes wheel speed given a certain X,Y speed
 * @param	left	left wheel speed command in rad/s
 * @param	right 	right wheel speed command in rad/s
 */
void compute_wheel_speeds(double *left, double *right) {

	// To Do : Update the left and right wheel speeds based on speed[robot_id]
	// You can use the current robot's heading stored in loc[robot_id][2]. 
	
	float x = speed[robot_id][0]*cosf(loc[robot_id][2]) + speed[robot_id][1]*sinf(loc[robot_id][2]); // x in robot coordinates
	float y = -speed[robot_id][0]*sinf(loc[robot_id][2]) + speed[robot_id][1]*cosf(loc[robot_id][2]); // y in robot coordinates
        
           //printf("6 - compute wheel: robot_id = %d, speed_x = %f,  speed_y = %f\n", robot_id, speed[robot_id][0], speed[robot_id][1]);

	float Ku = 0.2;   // Forward control coefficient
	float Kw = 0.5;  // Rotational control coefficient
	float range = sqrtf(x*x + y*y);	  // Distance to the wanted position
	float bearing = atan2(y, x);	  // Orientation of the wanted position
	
	// Compute forward control
	float u = Ku*range*cosf(bearing);
	// Compute rotational control
	float w = Kw*bearing;
	
	// Convert to wheel speeds!
	*left = (u - AXLE_LENGTH*w/2.0) * (1000.0 / WHEEL_RADIUS); // left wheel speed command in rad/s
	*right = (u + AXLE_LENGTH*w/2.0) * (1000.0 / WHEEL_RADIUS); // right wheel speed command in rad/s
}

/**
 * @brief 	Compute left and right speed for obstacle avoidance 
 * @param	left	left wheel speed command in rad/s
 * @param	right 	right wheel speed command in rad/s
 * @param	avoid	whether avoidance is required 
 */
void compute_braitenberg_speed(double *left, double *right, bool *avoid){

	// To Do : Implement a braitenberg based obstacle avoidance solution.
	// You can access distance values with wb_distance_sensor_get_value(ds[0])  
	*avoid = false;	// indicate whether avoidance should be applied 
	*left  = 4.0;	// left wheel speed command in rad/s
	*right = 4.0;	// right wheel speed command in rad/s
	
	
	const double l_weight[NB_SENSORS] = {-0.035, -0.05, -0.03, 0.025, 0.025, 0.03, 0.05, 0.035};
       const double r_weight[NB_SENSORS] = {0.035, 0.05, 0.03, 0.025, 0.025, -0.03, -0.05, -0.035};
      
            
           // read sensor values
           double ds_value[NB_SENSORS];
           double max_sens = 0;
           for (int i = 0; i < NB_SENSORS; i++) {
               ds_value[i] = wb_distance_sensor_get_value(ds[i]); // range: 0 (far) to 4095 (0 distance (in theory))
               if(ds_value[i] > max_sens) {
                   max_sens = ds_value[i];
               }
           }
           
           if((ds_value[0]>85) || (ds_value[7]>85) || (ds_value[1]>90) || (ds_value[6]>90)) { // enable obstacle avoidance              
               *avoid = true;
               
               // calc. Braitenberg speeds
               for (int i = 0; i < NB_SENSORS; i++) {
                 *left += l_weight[i]*ds_value[i];
                 *right += r_weight[i]*ds_value[i];
               }
            } else if((ds_value[0]<75) && (ds_value[7]<75) && (ds_value[1]<80) && (ds_value[6]<80)) { // disable obstacle avoidance
                *avoid = false;
            }
            
           if(VERBOSE_BRAITENBERG) {
               printf("max_sens = %f, left_speed = %f, right_speed = %f\n", max_sens, *left, *right);
           }
}	
	

/*
 * Main function
 */
int main(){ 	

	float wsl, wsr;					// Wheel speed left and right 
	double ws[2]		= {0.,0.}; 	// Wheel speed computed with reynolds or laplacian  
	double ws_brait[2]	= {0.,0.}; 	// Wheel speed computed with braitenberg 
	bool avoid; 					// Whether avoidance is required 

 	reset();						// Resetting the robot
	
	// Forever
	for(;;){

		/* Get information */
		get_robot_positions();
		
#ifdef 	LAPLACIAN
		// Laplacian formation (updates the speed[robot_id][] variable)
		laplacian_formation();
#else 
		// Reynold's rules (updates the speed[robot_id][] variable)
		reynolds_rules();
#endif
		// Compute wheels speed from the speed[robot_id][] variable 
		compute_wheel_speeds(&ws[0], &ws[1]);

		// Add Braitenberg
		compute_braitenberg_speed(&ws_brait[0], &ws_brait[1], &avoid);

		// Compute wheel velocities 
		if(avoid){
			// apply avoidance 
			wsl = ws_brait[0];
			wsr = ws_brait[1];
		}
		else{
			// apply reynolds or laplacian 
			wsl = ws[0];
			wsr = ws[1];
		}
		
		// Set speeds 
		limitf(&wsl, MAX_SPEED_WEB);
		limitf(&wsr, MAX_SPEED_WEB);
		wb_motor_set_velocity(left_motor, wsl); 
		wb_motor_set_velocity(right_motor, wsr);

		// Continue one step
		wb_robot_step(TIME_STEP);
	}
}  

