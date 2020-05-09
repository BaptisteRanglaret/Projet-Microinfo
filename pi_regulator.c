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
int16_t p_regulator(float angle, float goal){

	float error, speed_correction =0;

	error = angle - goal;

	/*disables the P regulator if the error is to small, this avoids to always
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
	if (value>CAPT_TRESHOLD) //A partir d'une certaine valeur, on considère que le capteur est assez précis pour traiter son information
	{
		float distance=CONST_EQ_DIST_2*log((CONST_EQ_DIST_4/(value-CONST_EQ_DIST_3))-CONST_EQ_DIST_1);
		return distance;
	}
	else //Si la valeur reçue est en dessous du threshold on renvoie une valeur normalisée
	{
		return MAX_DISTANCE;
	}
}

float calcul_angle (float dist2, float dist4)
{
	float angle=0 ;

	if(dist2==MAX_DISTANCE && dist4==MAX_DISTANCE) //si les deux capteurs ne détecte aucun obstacle
	{
		return 0;
	}
	else // sinon effectue le calcul de l'angle
	{
		double alpha = asin(((dist4+DIST_CAPT_4)-(dist2+DIST_CAPT_2))/DIST_CAPT);  //Voi rapport

		angle= (DEG_CONV/M_PI)*alpha;	// convertit l'angle de radians en degrés
	}
	return angle;  						// angle négatif si robot s'éloigne du mur
}

