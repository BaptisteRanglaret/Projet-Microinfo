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

// variable globale pour transmettre l'état du clignotant entre les threads
static int cligno=0;

// variable globale pour activer/désactiver le mouvement de base en fonction des autres threads (voir schéma rapport)
static int mobile=MOVE_START;

// variable globale qui permet de transmettre le signal de la détection d'une place pour se garer
static int compteur_place=0;


/****************************PUBLIC FUNCTIONS*************************************/



static THD_WORKING_AREA(waManoeuvre, 128);
static THD_FUNCTION(Manoeuvre, arg)
{
	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	systime_t time;


	int son, gare=0;

	//Variable différente de compteur_place permettant au e-puck de se garer au milieu de la place
	int compteur=0;
	//Booleen évitant de refaire les initialisations à chaque fois que la thread est appelée
	bool en_cours=0;

	while(1)
	{
		time = chVTGetSystemTime();

		if((compteur_place>PLACE_DISPO && mobile==MOVE_ON) || en_cours) 	// s'il détecte une place et s'il n'est pas en train de faire une autre manoeuvre
		{
			if(en_cours==OFF)					//INSTRUCTIONS EFFECTUEES UNE SEULE FOIS AU DEBUT
			{
				cligno=CLIGNO_DROIT;				// Allumer le cligno sur les leds 2 et 4
				mobile=MOVE_OFF; 				// désactive la thread de déplacement de base et interdit les autres manoeuvres
				en_cours=ON;

				left_motor_set_speed(SPEED_POS);
				right_motor_set_speed(SPEED_POS);

				while (convertisseur_value_dist(get_calibrated_prox(2))>DISTANCE_30MM) //tant qu'il ne voit rien sur le capteur 3 avance le long de la place pour s'arreter à la fin de la place
				{
					compteur_place++;   			//pour obtenir la taille de la place et pouvoir se garer précisément
					chThdSleepMilliseconds(10);
				}

				left_motor_set_speed(OFF);		// s'arrête s'il détecte une place
				right_motor_set_speed(OFF);
			}

			son = return_signal();
			if(son && gare==OFF)					//s'il capte un son, commence sa manoeuvre
			{
				left_motor_set_speed(SPEED_NEG);
				right_motor_set_speed(SPEED_NEG); // recule pour se mettre au niveau de la place

				while (compteur<(CONST_MULT_COMPTEUR_1*compteur_place/CONST_MULT_COMPTEUR_2))//tant qu'il n'est pas aux deux tiers de la place
				{
					compteur++;
					chThdSleepMilliseconds(10);
				}

				left_motor_set_speed(OFF);
				right_motor_set_speed(OFF);
				chThdSleepMilliseconds(500);

				left_motor_set_speed(SPEED_ROT_NEG); 		// quart de tour sur lui même
				right_motor_set_speed(SPEED_ROT_POS);
				chThdSleepMilliseconds(MS_QUART_TOUR);

				left_motor_set_speed(SPEED_NEG);
				right_motor_set_speed(SPEED_NEG);			// Tout droit en marche arrière
				while (convertisseur_value_dist(get_calibrated_prox(3))>DISTANCE_20MM) //tant qu'il ne voit rien sur le capteur IR4 pour arriver au fond de la place
				{
					chThdSleepMilliseconds(10);
				}

				cligno=OFF;									//eteindre les leds 2 et 4

				left_motor_set_speed(SPEED_ROT_POS); 		// quart de tour sur lui même
				right_motor_set_speed(SPEED_ROT_NEG);
				chThdSleepMilliseconds(MS_QUART_TOUR);

				left_motor_set_speed(OFF);
				right_motor_set_speed(OFF);
				gare=ON;										// permet la suite de la manoeuvre sans en refaire le début
			}

			if(gare)  										//s'il est garé
			{
				son = return_signal();						//s'il capte un son reprend un mouvement
				if(son)
				{
					gare=OFF;								//réinitialise les compteurs et variables
					en_cours=OFF;
					compteur_place=0;
					compteur=0;

					left_motor_set_speed(SPEED_POS);
					right_motor_set_speed(SPEED_POS);
					mobile=MOVE_MANOEUVRE;					//réactive les autres threads
				}
			}
		}
		//100Hz
		chThdSleepUntilWindowed(time, time+MS2ST(10));
	}
}

static THD_WORKING_AREA(waClignotant, 32);
static THD_FUNCTION(Clignotant, arg)
{
	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	systime_t time;

	while(1)
	{
		time = chVTGetSystemTime();

		if(cligno==CLIGNO_DROIT) 				// cligno droit activé
		{
			set_rgb_led(LED6, 0,0,0);			//éteint les LEDs 6 et 8
			set_rgb_led(LED8, 0,0,0);

			toggle_rgb_led(LED4,0, 255 );		//règle les LEDs 2 et 4 en orange RGB
			toggle_rgb_led(LED4,1, 165 );


			toggle_rgb_led(LED2,0, 255 );
			toggle_rgb_led(LED2,1, 165 );
		}
		else if(cligno==CLIGNO_GAUCHE)		// cligno gauche activé
		{
			set_rgb_led(LED2, 0,0,0);		//éteint les LEDs 2 et 4
			set_rgb_led(LED4, 0,0,0);

			toggle_rgb_led(LED6,0, 255 );	//règle les LEDs 6 et 8 en orange RGB
			toggle_rgb_led(LED6,1, 165 );


			toggle_rgb_led(LED8,0, 255 );
			toggle_rgb_led(LED8,1, 165 );
		}
		else									// si cligno désactivé, éteint toutes les leds
		{
			set_rgb_led(LED2, 0,0,0);
			set_rgb_led(LED4, 0,0,0);
			set_rgb_led(LED6, 0,0,0);
			set_rgb_led(LED8, 0,0,0);
		}


        //4Hz pour ressembler à un clignotant existant
        chThdSleepUntilWindowed(time, time + MS2ST(250));
    }

}

static THD_WORKING_AREA(waDepassement, 64);
static THD_FUNCTION(Depassement, arg)
{


	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	systime_t time;
	int etat_mobile;

	while(1)
	{
		time = chVTGetSystemTime();

		//S'il détecte un obstacle pendant le déplacement en ligne droite ou pour sortir de sa place de parking
		if((convertisseur_value_dist(get_calibrated_prox(7))<DISTANCE_30MM) && (mobile==MOVE_ON || mobile == MOVE_MANOEUVRE))
		{
			etat_mobile=mobile;								// sauvegarde la valeur d'entrée de mobile pour définir la manoeuvre à effectuer
			mobile=MOVE_OFF;			 						//désactive les autres thread de déplacement

			left_motor_set_speed(OFF);
			right_motor_set_speed(OFF);						//Arrete les deux moteurs pour simuler un contrôle des rétroviseurs
			cligno=CLIGNO_GAUCHE; 							//Allumer clignotant gauche
			chThdSleepMilliseconds(1000);

			left_motor_set_speed(SPEED_ROT_NEG);				// tourne à gauche sur lui même
			right_motor_set_speed(SPEED_ROT_POS);
			chThdSleepMilliseconds(MS_QUART_TOUR);

			left_motor_set_speed(OFF);						//
			right_motor_set_speed(OFF);						//évite un décalage de déplacement entre les 2 moteurs
			chThdSleepMilliseconds(10);						//


			//structure permettant de longer et dépasser efficacement un obstacle sur sa droite
			mobile=MOVE_DEPASSEMENT;
			while (convertisseur_value_dist(get_calibrated_prox(2))<DISTANCE_30MM) //tant qu'il voit quelque chose sur le capteur 3 avance le long de l'obstacle avec correction
			{
				if (convertisseur_value_dist(get_calibrated_prox(1))==MAX_DISTANCE)// Si il ne voit rien sur le capteur 2, finit de dépasser l'obstacle sur la même trajectoire et désactive la correction
				{
					mobile=MOVE_OFF;
					left_motor_set_speed(SPEED_POS);
					right_motor_set_speed(SPEED_POS);
				}
				chThdSleepMilliseconds(10);
			}
			chThdSleepMilliseconds(400); 					//ce temps permet de bien dépasser la fin de l'obstacle sans trop s'en éloigner


			left_motor_set_speed(OFF);
			right_motor_set_speed(OFF);
			chThdSleepMilliseconds(10);

			left_motor_set_speed(SPEED_ROT_POS);          	// tourne à droite
			right_motor_set_speed(SPEED_ROT_NEG);
			chThdSleepMilliseconds(MS_QUART_TOUR);
			cligno=OFF;										//éteindre le clignotant gauche

			left_motor_set_speed(OFF);
			right_motor_set_speed(OFF);
			chThdSleepMilliseconds(10);

			left_motor_set_speed(SPEED_POS);					//permet de remettre le capteur IR3 en face d'un obstacle en vue de reprendre la correction de trajectoire
			right_motor_set_speed(SPEED_POS);
			chThdSleepMilliseconds(500);


			if(etat_mobile==MOVE_ON)
			{
				//meme structure que ci-dessus
				mobile=MOVE_DEPASSEMENT;
				while (convertisseur_value_dist(get_calibrated_prox(2))<DISTANCE_30MM)
				{
					if (convertisseur_value_dist(get_calibrated_prox(1))==MAX_DISTANCE)
					{
						mobile=MOVE_OFF;
						left_motor_set_speed(SPEED_POS);
						right_motor_set_speed(SPEED_POS);
					}
					chThdSleepMilliseconds(10);
				}
				chThdSleepMilliseconds(400);


				cligno=CLIGNO_DROIT;							//Allumer le clignotant droit
				left_motor_set_speed(OFF);
				right_motor_set_speed(OFF);
				chThdSleepMilliseconds(10);

				left_motor_set_speed(SPEED_ROT_POS);			// tourne à droite à nouveau d'un quart de tour
				right_motor_set_speed(SPEED_ROT_NEG);
				chThdSleepMilliseconds(MS_QUART_TOUR);


				left_motor_set_speed(OFF);
				right_motor_set_speed(OFF);
				chThdSleepMilliseconds(10);

				left_motor_set_speed(SPEED_POS);
				right_motor_set_speed(SPEED_POS);			// Tout droit
				chThdSleepMilliseconds(500);

				//structure similaire aux autres ci-dessus, seulement on attend juste de voir un obstacle sur le capteur IR 8 pour procéder à la suite du code
				mobile=MOVE_DEPASSEMENT;
				while (convertisseur_value_dist(get_calibrated_prox(7))>DISTANCE_30MM) //tant qu'il ne voit rien sur le capteur 3
				{
					chThdSleepMilliseconds(10);
				}
				mobile=MOVE_OFF;

				left_motor_set_speed(OFF);
				right_motor_set_speed(OFF);
				chThdSleepMilliseconds(10);

				left_motor_set_speed(SPEED_ROT_NEG); 		// enfin, tourne à gauche pour se remettre droit
				right_motor_set_speed(SPEED_ROT_POS);
				chThdSleepMilliseconds(MS_QUART_TOUR);
				cligno=OFF;									// eteindre clignotant droit
			}

			mobile=MOVE_ON;									// réactive les autres threads

		}
		 //100Hz
		 chThdSleepUntilWindowed(time, time + MS2ST(10));
	}
}

static THD_WORKING_AREA(waDeplacement, 256);
static THD_FUNCTION(Deplacement, arg)
{
	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	systime_t time;
	float angle , angle_correction, dist_correction =0;
	float dist2, dist3, dist4 =0;
	int son =0;

	while(1)
	{
		time = chVTGetSystemTime();

		if(mobile==MOVE_ON || mobile==MOVE_DEPASSEMENT)
		{
			//Calcule les distances des capteurs IR2, 3 par rapport aux premiers obstacles pour tester immédiatement leur valeur
			dist3 = convertisseur_value_dist(get_calibrated_prox(2));
			dist2 = convertisseur_value_dist(get_calibrated_prox(1));

			//Si les deux capteurs détectent un obstacle
			if(dist3 != MAX_DISTANCE && dist2 != MAX_DISTANCE)
			{
				dist4 = convertisseur_value_dist(get_calibrated_prox(3));

	        		angle = calcul_angle(dist2, dist4);		//Calcul de l'angle

	        		//Calculs des corrections à appliquer aux vitesses des moteurs
	        		angle_correction = p_regulator(angle, GOAL_ANGLE);
	        		dist_correction = dist3-GOAL_DIST;

	        		//Corrections des vitesses des moteurs
	        		right_motor_set_speed(SPEED - (CONST_CORRECTION_DIST*dist_correction) + angle_correction);
	        		left_motor_set_speed(SPEED + (CONST_CORRECTION_DIST*dist_correction) - angle_correction);

	        		compteur_place=0;
			}
			else
			{
				if(dist2 == MAX_DISTANCE) // Si le capteur 2 ne capte plus rien, arrête la correction pour éviter qu'il se tourne vers la place libre
				{
					right_motor_set_speed(SPEED_POS);
					left_motor_set_speed(SPEED_POS);
				}

				if (dist3 == MAX_DISTANCE)// Si le capteur 3 ne renvoie plus rien, incrémente le compteur pour déterminer si il a la place de se garer
				{
					compteur_place++;
				}
			}
		}

		if(mobile==MOVE_START) 			// Si nous sommes au début du programme, attend un son entre 1300Hz et 1560Hz pour démarrer
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
