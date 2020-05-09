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

// variable globale pour activer/désactiver le mouvement de base en fonction des autres threads
static int mobile=MOVE_START;

// variable globale qui permet de transmettre le signal de la détection d'une place pour se garer
static int compteur_place=0;


/****************************PUBLIC FUNCTIONS*************************************/



static THD_WORKING_AREA(waManoeuvre, 64);
static THD_FUNCTION(Manoeuvre, arg)
{
	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	systime_t time;
	int son, gare=0;
	int compteur=0;

	bool en_cours=0;

	while(1)
	{
		time = chVTGetSystemTime();

		if((compteur_place>PLACE_DISPO && mobile==MOVE_ON) || en_cours) // si il détecte une place et si il n'est pas en train de faire un autre manoeuvre
		{
			if(en_cours==0)		//INSTRUCTIONS EFFECTUEES UNE SEULE FOIS AU DEBUT
			{
				cligno=CLIGNO_DROIT;		// Allumer le cligno sur les leds 2 et 4
				mobile=MOVE_OFF; // désactive la thread de déplacement de base et interdit les autres manoeuvres
				en_cours=ON;

				left_motor_set_speed(1000);				// s'arrête si détecte une place
				right_motor_set_speed(1000);

				while (get_calibrated_prox(2)<200) //tant qu'il ne voit rien sur le capteur 3 avance le long de la place
				{
					compteur_place++;   //pour obtenir la taille de la place et pouvoir se garer précisément
					chThdSleepMilliseconds(10);
				}
				left_motor_set_speed(0);				// s'arrête si détecte une place
				right_motor_set_speed(0);
			}

			son = return_signal();
			if(son && gare==OFF)			//Si il capte un son, commence sa manoeuvre
			{
				left_motor_set_speed(-1000);
				right_motor_set_speed(-1000); // recule pour se mettre au niveau de la place

				while (compteur<(2*compteur_place/3))//tant qu'il n'est pas au milieu de la place
				{
					compteur++;
					chThdSleepMilliseconds(10);
				}

				left_motor_set_speed(0);
				right_motor_set_speed(0);
				chThdSleepMilliseconds(500);
				left_motor_set_speed(-1000); 		// quart de tour sur lui même
				right_motor_set_speed(1000);
				chThdSleepMilliseconds(325);

				left_motor_set_speed(-1000);
				right_motor_set_speed(-1000);		// TOUT DROIT en marche arrière
				while (get_calibrated_prox(3)<=200) //tant qu'il ne voit rien sur le capteur 5 (derrière)
				{
					chThdSleepMilliseconds(10);
				}

				cligno=OFF;							//eteindre les leds 2 et 4

				left_motor_set_pos(0);
				while(left_motor_get_pos()<= 325)
				{
					left_motor_set_speed(1000); 		// quart de tour sur lui même
					right_motor_set_speed(-1000);
				}

				left_motor_set_speed(0);
				right_motor_set_speed(0);
				gare=ON;						// permet la suite de la manoeuvre sans refaire toute la manoeuvre
			}

			if(gare)  						//si il est autorisé à ressortir de sa place
			{
				son = return_signal();		//si il capte un son reprend un mouvement
				if(son)
				{
					mobile=MOVE_MANOEUVRE;
					gare=OFF;
					en_cours=OFF;
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

		if(cligno==CLIGNO_DROIT) // cligno droit activé
		{
			set_rgb_led(LED6, 0,0,0);
			set_rgb_led(LED8, 0,0,0);

			toggle_rgb_led(LED4,0, 255 );
			toggle_rgb_led(LED4,1, 165 );


			toggle_rgb_led(LED2,0, 255 );
			toggle_rgb_led(LED2,1, 165 );
		}
		else if(cligno==CLIGNO_GAUCHE) // cligno gauche activé
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
			cligno=CLIGNO_GAUCHE; 			//Allumer cligno gauche
			chThdSleepMilliseconds(1000);

			left_motor_set_speed(-1000);			// tourne à gauche sur lui même
			right_motor_set_speed(1000);
			chThdSleepMilliseconds(325);

			left_motor_set_speed(0);			//
			right_motor_set_speed(0);		//evite un décalage de déplacement entre les 2 moteurs
			chThdSleepMilliseconds(10);		//

			//debut mini_depassement
			mobile=EN_DEPASSEMENT;
			while (get_calibrated_prox(2)>=100) //tant qu'il voit quelque chose sur le capteur 3 avance le long de l'obstacle avec correction
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
			//fin mini_depassement

			left_motor_set_speed(0);
			right_motor_set_speed(0);
			chThdSleepMilliseconds(10);
			left_motor_set_speed(1000);          // tourne à droite
			right_motor_set_speed(-1000);
			chThdSleepMilliseconds(325);
			cligno=OFF;							//eteindre le cligno gauche

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

			chThdSleepMilliseconds(50);

			// fin 2eme mini_depassement

			if(etat_mobile==MOVE_ON)
			{
				cligno=CLIGNO_DROIT;						//Allumer le cligno sur la led 2 et 4


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
				cligno=OFF;							// eteindre cligno droit
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

	//float angle_reste=0;
	//int temps_sleep=0;
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
			dist2 = convertisseur_value_dist(get_calibrated_prox(1));

			if((dist3 != MAX_DISTANCE && dist2 != MAX_DISTANCE) /*|| (mobile==EN_DEPASSEMENT)*/  ) //si les deux capteurs voient qqch ou en depassement où il est plus susceptible d'avoir des fortes variations d'angle à corriger
			{
				dist4 = convertisseur_value_dist(get_calibrated_prox(3));

	        		angle = calcul_angle(dist2, dist4);		//Angle calculation

	        		//Computes the angle and the distance correction
	        		angle_correction = pi_regulator(angle, GOAL_ANGLE);
	        		dist_correction = dist3-GOAL_DIST;

	        		//applies the speed from the P regulator
	        		right_motor_set_speed(SPEED - (10*dist_correction) + angle_correction);
	        		left_motor_set_speed(SPEED + (10*dist_correction) - angle_correction);

	        		compteur_place=0;
	        		//condition=0;
			}
			else
			{
				if(dist2 == MAX_DISTANCE) // Si le capteur 2 ne capte plus rien, arrête la correction pour éviter qu'il se tourne vers la place libre
				{
					/*
					angle_reste=return_angle();
					if (condition == 0 && (angle_reste >1 || angle_reste<-1)) //pour qu'il ne le fasse su'une fois dès qu'il détecte un espace vide
					{
						left_motor_set_speed(0);				// s'arrête si détecte une place
						right_motor_set_speed(0);
						chThdSleepMilliseconds(1000);

						//angle_reste=return_angle();
						temps_sleep=(10*angle_reste);
						//chprintf((BaseSequentialStream*)&SDU1, "angle_reste =%f\n", angle_reste);
						//chprintf((BaseSequentialStream*)&SDU1, "temps_sleep =%d\n", temps_sleep);
						if(angle_reste<0)
						{
							left_motor_set_pos(0);
							while(left_motor_get_pos()<=(temps_sleep))
							{
								left_motor_set_speed(-1000);				// s'arrête si détecte une place
								right_motor_set_speed(1000);
							}
						}
						if(angle_reste>0)
						{
							left_motor_set_pos(0);
							while(left_motor_get_pos()>=(-temps_sleep))
							{
								left_motor_set_speed(1000);				// s'arrête si détecte une place
								right_motor_set_speed(-1000);
							}
						}
						condition=1;

					}

					*/
					//right_motor_set_speed(SPEED+25);
					//left_motor_set_speed(SPEED-25);
					//chThdSleepMilliseconds(25);
					right_motor_set_speed(1000);
					left_motor_set_speed(1000);
				}

				if (dist3 == MAX_DISTANCE)// Si le capteur 3 ne renvoie plus rien, regarde si il a la place de se garer
				{
					compteur_place++;
				}

			}

		}

		if(mobile==MOVE_START) // Si nous sommes au début du programme, attend un son pour démarrer
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
