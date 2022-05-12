#pragma once 

#define FLOCK_SIZE	        10		// Number of robots in flock
#define NUM_FLOCKS	        1		// How many flocks are considered, 1 or 2 (warning: both flocks must have equal sizes)

/* REYNOLDS */
#define RULE1_THRESHOLD     0.20   		// Threshold to activate aggregation rule. default 0.20
#define RULE1_WEIGHT        (0.6/10)	        // Weight of aggregation rule. 
#define RULE2_THRESHOLD     0.015   		// Threshold to activate dispersion rule. default 0.15
#define RULE2_WEIGHT        (0.01/10)	    // Weight of dispersion rule. 
#define RULE3_WEIGHT        (5.0/10) 	    // Weight of alignment rule. 
#define MIGRATION_WEIGHT    (1.0/10) 	    // Wheight of attraction towards the common goal.
#define MIGRATORY_URGE 		1 			// Tells the robots if they should just go forward or move towards a specific migratory direction

#define NEIGHBORHOOD 		1 	// Tells the robot considering neighbors or all robots during flocking, either 0 or 1
#define NEIGH_THRESHOLD 	0.4 // Threshold to consider neighborhood

/* LAPLACIAN */
//#define LAPLACIAN // <-------------- Uncomment to use the Laplacian controller 

#ifdef LAPLACIAN 
    typedef double Laplacian[FLOCK_SIZE][FLOCK_SIZE];
    // To Do: Complete the laplacian and bias matrices
    Laplacian L ={{4.0, -1.0, -1.0, -1.0, -1.0},
                  {-1.0, 1.0, 0.0, 0.0, 0.0},
                  {-1.0, 0.0, 1.0, 0.0, 0.0},
                  {-1.0, 0.0, 0.0, 1.0, 0.0},
                  {-1.0, 0.0, 0.0, 0.0, 1.0}};
    double bias[FLOCK_SIZE][2] = {{0.15, 0.0},
                                  {0.0, 0.15},
                                  {0.0, -0.15},
                                  {0.3, 0.15},
                                  {0.3, -0.15}};
#endif
