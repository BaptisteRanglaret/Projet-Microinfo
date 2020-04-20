#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <arm_math.h>

#include <main.h>
#include <mouvement.h>
#include <pi_regulator.h>
//#include <process_image.h>
#include <sensors/proximity.h>
#include <motors.h>

float calcul_angle (float dist2, float dist4);

//simple PID regulator implementation
int16_t pi_regulator(float angle, float goal){

	float error = 0;
	float speed_correction = 0;

	//static float sum_error = 0;
	//static float error_prec=0;

	error = angle - goal;

	//float error_diff = (error-error_prec); // D of PID



	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and 
	//the camera is a bit noisy
	if(fabs(error) < ERROR_THRESHOLD)
	{
		return 0;
	}

	//sum_error += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	// COMPRENDRE CE QU'IL SE PASSE SI OVERFLOW
	/*if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}*/


	/*if(error_diff> MAX_ERROR_DIFF)
	{
		error_diff = MAX_ERROR_DIFF;
	}
	else if(error_diff < -MAX_ERROR_DIFF)
	{
		error_diff = -MAX_ERROR_DIFF;
	}*/

	/*if(error> MAX_ERROR)
	{
		error = MAX_ERROR;
	}
	else if(error < -MAX_ERROR)
	{
		error= -MAX_ERROR;
	}*/

	// On met deux Kp pour contrer la non-linéarité des valeurs de l'erreur

	speed_correction = KP * error ; //+ KD* error_diff + KI * sum_error;



	//chprintf((BaseSequentialStream *)&SDU1, "KP*ERROR = %f\n",KP*error);
	//chprintf((BaseSequentialStream *)&SDU1, "KI*SUM_ERROR = %f\n",KI*sum_error);
	//chThdSleepMilliseconds(1000);
	//error_prec = error;
    return (int16_t)speed_correction;
}

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    //int16_t speed_correction = 0;
    float angle , angle_correction, dist_correction =0; //, norme, diff_norme=0;

    while(1){
        time = chVTGetSystemTime();
        

        float dist3 = convertisseur_value_dist(get_calibrated_prox(2));
        float dist2 = convertisseur_value_dist(get_calibrated_prox(1));
        float dist4 = convertisseur_value_dist(get_calibrated_prox(3));

        //chprintf((BaseSequentialStream *)&SDU1, "DISTANCE capteur 3 = %f\n",dist3);
        //chprintf((BaseSequentialStream *)&SDU1, "DISTANCE capteur 2 = %f\n",dist2);
        //chprintf((BaseSequentialStream *)&SDU1, "DISTANCE capteur 4 = %f\n",dist4);

        	//Angle calculation
        	angle = calcul_angle(dist2, dist4);

        //chprintf((BaseSequentialStream *)&SDU1, "Angle = %f\n",angle);

        //computes the angle correction
        //The angle is determined above
        angle_correction = pi_regulator(angle, GOAL_ANGLE);
        dist_correction = dist3-GOAL_DIST;

        //applies the speed from the PID regulator
		right_motor_set_speed(SPEED - (5*dist_correction) + angle_correction);
		left_motor_set_speed(SPEED + (5*dist_correction) - angle_correction);

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}



// ATTENTION AUX MAGIC NUMBERS
float convertisseur_value_dist(float value)
{
	if (value>124)
	{
		float distance=6*log((13760/(value-120))-2);
		return distance;
	}
	else
	{
		return 50;
	}
}

// ATTENTION AUX MAGIC NUMBERS
float calcul_angle (float dist2, float dist4)
{
	double alpha = 0; //,x, y, l1 =0; //initializes local variables


	alpha = asin(((dist4+15)-(dist2+25))/DIST_CAPT);
	//l1 = 25 + dist4;


	//x= DIST_CAPT*cos(alpha);
	//y=l1*cos(alpha) + DIST_CAPT*sin(alpha);		//creates 2 coordinates of a vector

	//float angle = alpha;
	float angle= (180/M_PI)*alpha;		// computes the angle of this vector to the wall in degrees
	return angle;  						// angle négatif si robot s'éloigne du mur
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}

