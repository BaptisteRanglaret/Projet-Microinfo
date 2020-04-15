#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <arm_math.h>

#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
//#include <process_image.h>
#include <sensors/proximity.h>

float calcul_angle (float dist1, float dist2, float dist_capt, bool var);

//simple PID regulator implementation
int16_t pi_regulator(float distance, float goal){

	float error = 0;
	float speed_correction = 0;

	static float sum_error = 0;
	static float error_prec=0;

	error = distance - goal;

	float error_diff = (error-error_prec);



	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and 
	//the camera is a bit noisy
	if(fabs(error) < ERROR_THRESHOLD)
	{
		return 0;
	}

	sum_error += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	// COMPRENDRE CE QU'IL SE PASSE SI OVERFLOW
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}


	if(error_diff> MAX_ERROR_DIFF)
	{
		error_diff = MAX_ERROR_DIFF;
	}
	else if(error_diff < -MAX_ERROR_DIFF)
	{
		error_diff = -MAX_ERROR_DIFF;
	}

	if(error> MAX_ERROR)
		{
			error = MAX_ERROR;
		}
		else if(error < -MAX_ERROR)
		{
			error= -MAX_ERROR;
		}

	// On met deux Kp pour contrer la non-linéarité des valeurs de l'erreur
	if(fabs(error)> 300)
	{
		speed_correction = KP * error + KD* error_diff + KI * sum_error;
	}
	else
	{
		speed_correction = KP * error + KD* error_diff + KI * sum_error;
	}


	//chprintf((BaseSequentialStream *)&SDU1, "KP*ERROR = %f\n",KP*error);
	//chprintf((BaseSequentialStream *)&SDU1, "KI*SUM_ERROR = %f\n",KI*sum_error);
	//chThdSleepMilliseconds(1000);
	error_prec = error;
    return (int16_t)speed_correction;
}

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    //int16_t speed_correction = 0;
    float angle, diff_angle, norme, diff_norme=0;

    while(1){
        time = chVTGetSystemTime();
        

        float dist3 = convertisseur_value_dist(get_calibrated_prox(2));
        float dist2 = convertisseur_value_dist(get_calibrated_prox(1));
        float dist4 = convertisseur_value_dist(get_calibrated_prox(3));

        chprintf((BaseSequentialStream *)&SDU1, "DISTANCE capteur 3 = %f\n",dist3);
        chprintf((BaseSequentialStream *)&SDU1, "DISTANCE capteur 2 = %f\n",dist2);
        chprintf((BaseSequentialStream *)&SDU1, "DISTANCE capteur 4 = %f\n",dist4);

        if((dist4 == 0) || (dist4!=0 && dist2!=0))
        {
        		if (dist2==0)
        		{
        			angle = 0;
        		}
        		else
        		{
        			//Angle calculation
        			angle = calcul_angle(dist3, dist2, DIST_CAPT1, ANGLE);
        			norme = calcul_angle(dist3, dist2, DIST_CAPT1, NORME);
        			diff_angle=angle-GOAL_ANGLE1;
        			diff_norme=norme-GOAL_NORME1;
        		}
        }
        else if(dist2 == 0)
        {
        		//Angle calculation
        		angle = calcul_angle(dist4, dist3, DIST_CAPT2, ANGLE);
        		norme = calcul_angle(dist4, dist3, DIST_CAPT2, NORME);
        		diff_angle=angle-GOAL_ANGLE2;
        		diff_norme=norme-GOAL_NORME2;
        }
        else
        {
        		angle = 0;
        		// faire un truc pour qu'il tourne en rond ou cherche le mur ?
        }

        chprintf((BaseSequentialStream *)&SDU1, "Angle = %f\n",angle);
        //chprintf((BaseSequentialStream *)&SDU1, "Diff angle = %f\n",diff_angle);
        //chprintf((BaseSequentialStream *)&SDU1, "Norme = %f\n",norme);
        //chprintf((BaseSequentialStream *)&SDU1, "Diff norme = %f\n",diff_norme);

        //computes the speed to give to the motors
        //The angle is determined above
        //speed_correction = pi_regulator(angle, GOAL_ANGLE);


        //applies the speed from the PID regulator
		//right_motor_set_speed(SPEED + speed_correction);
		//left_motor_set_speed(SPEED - speed_correction);

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}



// ATTENTION AUX MAGIC NUMBERS, faire une macro svp
float convertisseur_value_dist(float value)
{
	if (value>124)
	{
		float distance=6*log((13760/(value-120))-2);
		return distance;
	}
	else
	{
		return 0;
	}
}

float calcul_angle (float dist1, float dist2, float dist_capt, bool var)
{
	double x,y,alpha, l1 =0; //initializes local variables

	if(dist_capt == DIST_CAPT1) // if we work with IR2 and IR3
	{
		alpha = atan(((dist2+50)-(dist1+70))/dist_capt);
		l1 = 70 + dist1;
	}
	else // if we work with IR3 and IR4
	{
		alpha = atan(((dist2+70)-(dist1+25))/dist_capt);
		l1 = 25 + dist1;
	}



	x= dist_capt*cos(alpha);
	y=l1*cos(alpha) + dist_capt*sin(alpha);		//creates 2 coordinates of a vector

	float angle = alpha;
	//float angle= (180/M_PI)*atan(y/x);		// computes the angle of this vector to the wall
	float norme= sqrt(x*x+y*y);

	if(var==ANGLE){
		return angle;
	}

	return norme;
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}

