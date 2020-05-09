#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"


//constants for the differents parts of the project
#define ON						1
#define OFF						0
#define DEG_CONV					180 //valeur utilisée pour la conversion radians/degrés
#define PLACE_DISPO				40 //valeur de seuil à partir de laquelle le robot considère qu'il peut se garer
#define DIST_CAPT_4				15 //Distance entre le capteur 4 et l'axe du robot
#define DIST_CAPT_2				25 //Distance entre le capteur 2 et l'axe du robot

#define GOAL_ANGLE				0.0f 	//angle cible de l'axe du robot p.r. au mur
#define GOAL_DIST				25.0f	//distance cible entre le robot et le mur en mm
#define DIST_CAPT				110.0f	// distance entre les capteurs IR 2 et 4
#define MAX_DISTANCE				50.0f 	// distance limite renvoyée si le capteur ne "voit rien"
#define ERROR_THRESHOLD			0.5f		// experimental value
#define CAPT_TRESHOLD			124		//valeur renvoyée par le capteur IR à partir de laquelle on traite l'information
#define CONST_MULT_COMPTEUR_1	2		//Sert à ajuster la valeur de compteur_place pour que le e-puck se gare au bon endroit
#define CONST_MULT_COMPTEUR_2	3		//Sert à ajuster la valeur de compteur_place pour que le e-puck se gare au bon endroit

//Constantes de correction
#define KP						20.0f	//Pour la correction de l'angle
#define CONST_CORRECTION_DIST	10		//Pour la correction de la distance

//Constantes de vitesse
#define SPEED					800   // default step/s pour le déplacement en ligne droite
#define SPEED_POS				1000	  //Vitesse lors de la manoeuvre ou du depassement en avançant
#define SPEED_NEG				-1000 //Vitesse lors de la manoeuvre ou du depassement en reculant
#define SPEED_ROT_POS			1000	  //Vitesse de la roue lors de rotation
#define SPEED_ROT_NEG			-1000 //Vitesse de la roue opposée lors de rotation

//Différentes valeur de la variable globale mobile (Schéma explicatif dans le rapport)
#define MOVE_OFF					0
#define MOVE_ON					1
#define MOVE_MANOEUVRE			2
#define MOVE_DEPASSEMENT			3
#define MOVE_START				4

//Différentes valeur de la variable globale cligno
#define CLIGNO_DROIT				1
#define CLIGNO_GAUCHE			2

//Constantes de distance
#define DISTANCE_20MM			20
#define DISTANCE_30MM			30
#define DISTANCE_40MM			40
#define DISTANCE_50MM			50
#define MS_QUART_TOUR 325			//Temps nécessaire à faire une rotation d'un quart de tour en millisecondes

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
