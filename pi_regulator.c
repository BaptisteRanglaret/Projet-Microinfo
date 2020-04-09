#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


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
        //time = chVTGetSystemTime();
        
        //computes the speed to give to the motors
        //distance_cm is modified by the image processing thread
        speed_correction = pi_regulator((float)get_calibrated_prox(2), GOAL_VALUE);
        //computes a correction factor to let the robot rotate to be in front of the line
        //speed_correction = (get_line_position() - (IMAGE_BUFFER_SIZE/2));

        //if the line is nearly in front of the camera, don't rotate
        /*if(abs(speed_correction) < ROTATION_THRESHOLD){
        	speed_correction = 0;
        }*/

        //applies the speed from the PI regulator and the correction for the rotation
		right_motor_set_speed(SPEED + speed_correction);
		left_motor_set_speed(SPEED - speed_correction);

        //100Hz
        //chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
