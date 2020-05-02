#include <ch.h>
#include <hal.h>
#include <mouvement.h>
#include "leds.h"
#include "sensors/proximity.h"
#include "pal.h"
#include <motors.h>
#include <audio_processing.h>
#include <chprintf.h>
#include <usbcfg.h>
#include <pi_regulator.h>
#include <main.h>

static int cligno=0;  			// variable globale pour transmettre l'état du clignotant entre les threads
static int mobile=MOVE_START;	// variable globale pour activer/désactiver le mouvement en ligne droite de base en fonction des autres threads
static int compteur_place=0;


/****************************PUBLIC FUNCTIONS*************************************/



static THD_WORKING_AREA(waManoeuvre, 64);
static THD_FUNCTION(Manoeuvre, arg)
{
	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	systime_t time;
	int son, gare=0;
	bool en_cours=0;

	while(1)
	{
		time = chVTGetSystemTime();

		if((compteur_place>80 && mobile==MOVE_ON) || en_cours) // si il détecte une place et si il n'est pas en train de faire un autre manoeuvre
		{
			if(en_cours==0)		//INSTRUCTIONS EFFECTUEES UNE SEULE FOIS AU DEBUT
			{
				cligno=1;		// Allumer le cligno sur les leds 2 et 4
				mobile=MOVE_OFF; // désactive la thread de déplacement de base et interdit les autres manoeuvres
				en_cours=1;
				left_motor_set_speed(0);				// s'arrête si détecte une place
				right_motor_set_speed(0);
			}

			son = return_signal();
			if(son && gare==0)			//Si il capte un son, commence sa manoeuvre
			{
				left_motor_set_speed(-1000);
				right_motor_set_speed(-1000); // recule pour se mettre au niveau de la place
				while (get_calibrated_prox(2)>=100) //tant qu'il voit quelque chose sur le capteur 3 avance le long de l'obstacle
				{
					chThdSleepMilliseconds(10);
				}
				chThdSleepMilliseconds(125);

				left_motor_set_speed(-1000); 		// quart de tour sur lui même
				right_motor_set_speed(1000);
				chThdSleepMilliseconds(325);

				left_motor_set_speed(-1000);
				right_motor_set_speed(-1000);		// TOUT DROIT en marche arrière
				while (get_calibrated_prox(3)<=200) //tant qu'il ne voit rien sur le capteur 5 (derrière)
				{
					chThdSleepMilliseconds(10);
				}

				cligno=0;							//eteindre les leds 2 et 4

				left_motor_set_pos(0);
				while(left_motor_get_pos()<= 325)
				{
					left_motor_set_speed(1000); 		// quart de tour sur lui même
					right_motor_set_speed(-1000);
				}

				left_motor_set_speed(0);
				right_motor_set_speed(0);
				gare=1;						// permet la suite de la manoeuvre sans refaire toute la manoeuvre
			}

			if(gare)  						//si il est autorisé à ressortir de sa place
			{
				son = return_signal();		//si il capte un son reprend un mouvement
				if(son)
				{
					mobile=MOVE_MANOEUVRE;
					gare=0;
					en_cours=0;
					compteur_place=0;
					left_motor_set_speed(1000);
					right_motor_set_speed(1000);
				}
			}
		}
		//100Hz
		chThdSleepUntilWindowed(time, time+MS2ST(10));
	}
}


static THD_WORKING_AREA(waClignotant, 128); //////// !!! TAILLE
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


        //4Hz
        chThdSleepUntilWindowed(time, time + MS2ST(250));
    }

}

static THD_WORKING_AREA(waDepassement, 128); //////// !!! TAILLE
static THD_FUNCTION(Depassement, arg)
{


	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	systime_t time;
	int etat_mobile;

	while(1)
	{
		time = chVTGetSystemTime();

		if(( get_calibrated_prox(7)>=400) && (mobile==MOVE_ON || mobile == MOVE_MANOEUVRE))
		{
			etat_mobile=mobile;  // sauvegarde la valeur d'entrée de mobile pour définir la manoeuvre à effectuer
			mobile=MOVE_OFF;			 //désactive les autres thread de déplacement

			left_motor_set_speed(0);
			right_motor_set_speed(0);		//Arrete les deux moteurs
			cligno=2; 						//Allumer cligno sur la led 6 et led 8
			chThdSleepMilliseconds(1000);

			left_motor_set_speed(-1000);			// tourne à gauche sur lui même
			right_motor_set_speed(1000);
			chThdSleepMilliseconds(325);

			left_motor_set_speed(0);			//
			right_motor_set_speed(0);		//evite un décalage de déplacement entre les 2 moteurs
			chThdSleepMilliseconds(10);		//

			//debut mini_depassement
			mobile=EN_DEPASSEMENT;
			while (get_calibrated_prox(2)>=100) //tant qu'il voit quelque chose sur le capteur 3 avance le long de l'obstacle
			{
				if (convertisseur_value_dist(get_calibrated_prox(1))==MAX_DISTANCE)
				{
					mobile=MOVE_OFF;
					left_motor_set_speed(1000);
					right_motor_set_speed(1000);

				}
				chThdSleepMilliseconds(10);
			}

			chThdSleepMilliseconds(125);
			/*left_motor_set_pos(0);
			while(left_motor_get_pos()<= 300)
			{
				left_motor_set_speed(1000); 		// continue un peu le dépassement à partir du moment où le capteur 3 ne capte plus rien
				right_motor_set_speed(1000);
			}*/


			//fin mini_depassement

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
			right_motor_set_speed(1000);
			chThdSleepMilliseconds(500);

			//debut 2eme mini_depassement
			mobile=EN_DEPASSEMENT;
			while (get_calibrated_prox(2)>=100) //tant qu'il voit quelque chose sur le capteur 3
			{
				if (convertisseur_value_dist(get_calibrated_prox(1))==MAX_DISTANCE)
				{
					mobile=MOVE_OFF;
					left_motor_set_speed(1000);
					right_motor_set_speed(1000);
				}
				chThdSleepMilliseconds(10);
			}

			chThdSleepMilliseconds(125);

			/*left_motor_set_pos(0);
			while(left_motor_get_pos()<= 300)
			{
				left_motor_set_speed(1000);
				right_motor_set_speed(1000);
			}*/

			// fin 2eme mini_depassement

			if(etat_mobile==MOVE_ON)
			{
				cligno=1;						//Allumer le cligno sur la led 2 et 4


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

				while (get_calibrated_prox(7)<=300) //tant qu'il ne voit rien sur le capteur 3
				{
					chThdSleepMilliseconds(10);
				}

				left_motor_set_speed(0);
				right_motor_set_speed(0);
				chThdSleepMilliseconds(10);
				left_motor_set_speed(-1000); 		// enfin, tourne à gauche pour se remettre droit
				right_motor_set_speed(1000);
				chThdSleepMilliseconds(325);
				cligno=0;							// eteindre cligno droit
			}

			mobile=MOVE_ON;						// réactive la thread de déplacement

		}
		 //100Hz
		 chThdSleepUntilWindowed(time, time + MS2ST(10));
	}
}

static THD_WORKING_AREA(waDeplacement, 256); //256 nécessaire pour toutes ces variables !
static THD_FUNCTION(Deplacement, arg)
{
	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	systime_t time;

	 float angle , angle_correction, dist_correction =0; // définition des variables locales
	 float dist2, dist3, dist4 =0;
	 int son =0;
	while(1)
	{
		time = chVTGetSystemTime();

		if(mobile==MOVE_ON || mobile==EN_DEPASSEMENT)
		{
			//Calcule les distances des capteurs IR2, 3 et 4 par rapport aux premiers obstacles
			dist3 = convertisseur_value_dist(get_calibrated_prox(2));
			//chprintf((BaseSequentialStream *)&SDU1, "DISTANCE capteur 3 = %f\n",dist3);
			dist2 = convertisseur_value_dist(get_calibrated_prox(1));

			if(dist3 != MAX_DISTANCE && dist2 != MAX_DISTANCE ) //si les deux capteurs voient qqch
			{
				//dist2 = convertisseur_value_dist(get_calibrated_prox(1));
				dist4 = convertisseur_value_dist(get_calibrated_prox(3));

	        		angle = calcul_angle(dist2, dist4);		//Angle calculation

	        		//Computes the angle and the distance correction
	        		angle_correction = pi_regulator(angle, GOAL_ANGLE);
	        		dist_correction = dist3-GOAL_DIST;

	        		//applies the speed from the PID regulator
	        		right_motor_set_speed(SPEED - (10*dist_correction) + angle_correction); //initial SPEED - (*dist_correction) + angle_correction
	        		left_motor_set_speed(SPEED + (10*dist_correction) - angle_correction);

	        		compteur_place=0;
			}
			else
			{
				if(dist2 == MAX_DISTANCE)
				{
					right_motor_set_speed(SPEED);
					left_motor_set_speed(SPEED);
				}

				if (dist3 == MAX_DISTANCE)
				{
					compteur_place++;
				}

			}

		}

		if(mobile==MOVE_START)
		{
			son = return_signal();
			if(son)
			{
				mobile=MOVE_ON;
			}
		}
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

/**************************END PUBLIC FUNCTIONS***********************************/
