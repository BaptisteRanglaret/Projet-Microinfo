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

//simple PI regulator implementation
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
		speed_correction = KP_PROCHE * error + KD_PROCHE* error_diff + KI * sum_error;
	}
	else
	{
		speed_correction = KP_LOIN * error + KD_LOIN* error_diff + KI * sum_error;
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

    int16_t speed_correction = 0;

    while(1){
        time = chVTGetSystemTime();
        

        float dist1 = convertisseur_value_dist(get_calibrated_prox(2));
        float dist2 = convertisseur_value_dist(get_calibrated_prox(1));
        float dist3 = convertisseur_value_dist(get_calibrated_prox(3));

        chprintf((BaseSequentialStream *)&SDU1, "DISTANCE capteur 3 = %f\n",dist1);
        chprintf((BaseSequentialStream *)&SDU1, "DISTANCE capteur 2 = %f\n",dist2);
        chprintf((BaseSequentialStream *)&SDU1, "DISTANCE capteur 4 = %f\n",dist3);


        //vector implementation
        ////////////////////////////////////////////////////////////////////

        //Angle calculation
        ////////////////////////////////////////////////////////////////////

        //computes the speed to give to the motors
        //The angle is determined above
        //speed_correction = pi_regulator(dist1, GOAL_VALUE); //changer dist en angle


        //applies the speed from the PID regulator
		//right_motor_set_speed(SPEED + speed_correction);
		//left_motor_set_speed(SPEED - speed_correction);

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(1000));
    }
}


// ATTENTION AUX MAGIC NUMBERS, faire une macro svp
float convertisseur_value_dist(float value)
{
	if (value>120)
	{
		float distance=6*log((13760/(value-120))-2);
		return distance;
	}
	else
	{
		return 0;
	}
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
