#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <arm_math.h>

#include <main.h>
#include <mouvement.h>
#include <pi_regulator.h>
#include <sensors/proximity.h>
#include <motors.h>

float calcul_angle (float dist2, float dist4);

//simple P regulator implementation
int16_t pi_regulator(float angle, float goal){

	float error, speed_correction =OFF;

	error = angle - goal;

	/*disables the PI regulator if the error is to small, this avoids to always
	 * move as we cannot exactly be where we want and the sensors are a bit noisy
	*/
	if(fabs(error) < ERROR_THRESHOLD)
	{
		return OFF;
	}

	speed_correction = KP * error;

    return (int16_t)speed_correction;
}

float convertisseur_value_dist(float value)
{
	if (value>CAPT_TRESHOLD)
	{
		float distance=6*log((13760/(value-120))-2);
		return distance;
	}
	else
	{
		return MAX_DISTANCE;
	}
}

float calcul_angle (float dist2, float dist4)
{
	float angle=0 ;

	if(dist2==MAX_DISTANCE && dist4==MAX_DISTANCE)
	{
		return 0;
	}
	else
	{
		double alpha = asin(((dist4+DIST_CAPT_4)-(dist2+DIST_CAPT_2))/DIST_CAPT);

		angle= (DEG_CONV/M_PI)*alpha;	// computes the angle of this vector to the wall in degrees
	}
	return angle;  						// angle négatif si robot s'éloigne du mur
}
/*
float return_angle()
{
	return angle;
}
*/
