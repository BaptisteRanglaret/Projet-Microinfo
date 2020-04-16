#include <ch.h>
#include <hal.h>
#include <mouvement.h>
#include "leds.h"
#include "sensors/proximity.h"
#include "pal.h"
#include <motors.h>


static int cligno=0;  // variable globale pour transmettre l'état du clignotant entre les threads
static int mobile=1;	 //	variable globale pour activer/désactiver le mouvement en ligne droite de base en fonction des autres threads


/****************************PUBLIC FUNCTIONS*************************************/

static THD_WORKING_AREA(waManoeuvre, 128);
static THD_FUNCTION(Manoeuvre, arg)
{
	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	systime_t time;

	while(1)
	{
		time = chVTGetSystemTime();

		if((get_calibrated_prox(5)>= 200) && mobile==1 ) // si il détecte une place et si il n'est pas en train de faire un autre manoeuvre
		{
			mobile=0; // désactive la thread de déplacement de base

			left_motor_set_speed(0);				// s'arrête si détecte une place
			right_motor_set_speed(0);
			cligno=1;							// Allumer le cligno sur les leds 2 et 4
			chThdSleepMilliseconds(1000);


			left_motor_set_speed(-1000);
			right_motor_set_speed(-1000); // recule pour se mettre au niveau de la place
			chThdSleepMilliseconds(1000);

			left_motor_set_speed(-1000); 		// quart de tour sur lui même
			right_motor_set_speed(1000);
			chThdSleepMilliseconds(325);

			left_motor_set_speed(-1000);
			right_motor_set_speed(-1000);		// TOUT DROIT en marche arrière
			chThdSleepMilliseconds(750);

			//eteindre les leds 2 et 4
			cligno=0;

			left_motor_set_pos(0);
			while(left_motor_get_pos()<= 325)
			{
				left_motor_set_speed(1000); 		// quart de tour sur lui même
				right_motor_set_speed(-1000);
			}

			left_motor_set_speed(0);
			right_motor_set_speed(0);
		}

		//100Hz
		chThdSleepUntilWindowed(time, time+MS2ST(10));
	}
}

static THD_WORKING_AREA(waClignotant, 128);
static THD_FUNCTION(Clignotant, arg)
{
	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	systime_t time;

	while(1)
	{
		time = chVTGetSystemTime();

		if(cligno==1) // cligno gauche activé
		{
			set_rgb_led(LED6, 0,0,0);
			set_rgb_led(LED8, 0,0,0);

			toggle_rgb_led(LED4,0, 255 );
			toggle_rgb_led(LED4,1, 165 );


			toggle_rgb_led(LED2,0, 255 );
			toggle_rgb_led(LED2,1, 165 );
		}
		else if(cligno==2) // cligno droit activé
		{
			set_rgb_led(LED2, 0,0,0);
			set_rgb_led(LED4, 0,0,0);

			toggle_rgb_led(LED6,0, 255 );
			toggle_rgb_led(LED6,1, 165 );


			toggle_rgb_led(LED8,0, 255 );
			toggle_rgb_led(LED8,1, 165 );
		}
		else			// si cligno désactivé, éteint toutes les leds
		{
			set_rgb_led(LED2, 0,0,0);
			set_rgb_led(LED4, 0,0,0);
			set_rgb_led(LED6, 0,0,0);
			set_rgb_led(LED8, 0,0,0);
		}


        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(250));
    }

}

static THD_WORKING_AREA(waDepassement, 128);
static THD_FUNCTION(Depassement, arg)
{


	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	systime_t time;

	while(1)
	{
		time = chVTGetSystemTime();

		if(((get_calibrated_prox(0)>= 200)|| (get_calibrated_prox(7)>=200)) && mobile==1)
		{
			mobile=0;			//désactive la thread de déplacement

			left_motor_set_speed(0);
			right_motor_set_speed(0);		//Arrete les deux moteurs
			cligno=2; 						//Allumer cligno sur la led 6 et led 8
			chThdSleepMilliseconds(500);

			left_motor_set_speed(0);
			right_motor_set_speed(0);
			chThdSleepMilliseconds(10);
			left_motor_set_speed(-1000);			// tourne à gauche sur lui même
			right_motor_set_speed(1000);
			chThdSleepMilliseconds(325);

			left_motor_set_speed(0);
			right_motor_set_speed(0);
			chThdSleepMilliseconds(10);
			left_motor_set_speed(1000);
			right_motor_set_speed(1000);		// TOUT DROIT sur une longueur de robot
			chThdSleepMilliseconds(750);


			left_motor_set_speed(0);
			right_motor_set_speed(0);
			chThdSleepMilliseconds(10);
			left_motor_set_speed(1000);          // tourne à droite
			right_motor_set_speed(-1000);
			chThdSleepMilliseconds(325);
			cligno=0;							//eteindre la led6 et 8

			left_motor_set_speed(0);
			right_motor_set_speed(0);
			chThdSleepMilliseconds(10);
			left_motor_set_speed(1000);
			right_motor_set_speed(1000);		// TOUT DROIT sur deux longueurs et demi de robot
			chThdSleepMilliseconds(1500);
			cligno=1;						//Allumer le cligno sur la led 2 et 4
			left_motor_set_speed(1000);
			right_motor_set_speed(1000);		// TOUT DROIT sur deux longueurs et demi de robot
			chThdSleepMilliseconds(1000);


			left_motor_set_speed(0);
			right_motor_set_speed(0);
			chThdSleepMilliseconds(10);
			left_motor_set_speed(1000);			// tourne à droite à nouveau
			right_motor_set_speed(-1000);
			chThdSleepMilliseconds(325);


			left_motor_set_speed(0);
			right_motor_set_speed(0);
			chThdSleepMilliseconds(10);
			left_motor_set_speed(1000);
			right_motor_set_speed(1000);			// TOUT DROIT
			chThdSleepMilliseconds(750);

			left_motor_set_speed(0);
			right_motor_set_speed(0);
			chThdSleepMilliseconds(10);
			left_motor_set_speed(-1000); 		// enfin, tourne à gauche pour se remettre droit
			right_motor_set_speed(1000);
			chThdSleepMilliseconds(325);
			cligno=0;							// eteindre cigno droit

			mobile=1; 							// réactive la thread de déplacement

		}

		 //100Hz
		 chThdSleepUntilWindowed(time, time + MS2ST(10));
	}
}

static THD_WORKING_AREA(waDeplacement, 128);
static THD_FUNCTION(Deplacement, arg)
{
	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	systime_t time;

	while(1)
	{
		time = chVTGetSystemTime();

		if(get_calibrated_prox(4)>= 500)
		{
			mobile=1;
		}
		if(mobile)
		{
			left_motor_set_speed(1000);
			right_motor_set_speed(1000);
		}

		 //100Hz
		 chThdSleepUntilWindowed(time, time + MS2ST(10));
	}

}

static THD_WORKING_AREA(waAsmr, 128);
static THD_FUNCTION(Asmr, arg)
{
	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	systime_t time;

	while(1)
	{
		time = chVTGetSystemTime();

		//if(son)
		//{
		//	toggle_rgb_led(LED4,0, 255 );
		//	toggle_rgb_led(LED4,1, 165 );
		//}

		//100Hz
		chThdSleepUntilWindowed(time, time + MS2ST(10));
	}


}


void clignotant_start(void)
{
	chThdCreateStatic(waClignotant, sizeof(waClignotant), NORMALPRIO, Clignotant, NULL);
}

void depassement_start(void)
{
	chThdCreateStatic(waDepassement, sizeof(waDepassement), HIGHPRIO, Depassement, NULL);
}

void manoeuvre_start(void)
{
	chThdCreateStatic(waManoeuvre, sizeof(waManoeuvre), NORMALPRIO+1, Manoeuvre, NULL);
}

void deplacement_start(void)
{
	chThdCreateStatic(waDeplacement, sizeof(waDeplacement), NORMALPRIO, Deplacement, NULL);
}

void asmr_start(void)
{
	chThdCreateStatic(waAsmr, sizeof(waAsmr), NORMALPRIO, Asmr, NULL);
}
/**************************END PUBLIC FUNCTIONS***********************************/
