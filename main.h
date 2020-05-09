#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"


//constants for the differents parts of the project
#define IMAGE_BUFFER_SIZE		640
#define WIDTH_SLOPE				5
#define MIN_LINE_WIDTH			40
#define ON						1
#define OFF						0
#define DEG_CONV					180 //valeur utilisée pour la conversion radians/degrés
#define PLACE_DISPO				40
#define CLIGNO_DROIT				1
#define CLIGNO_GAUCHE			2
#define DIST_CAPT_4				15 //Distance entre le capteur 4 et l'axe du robot
#define DIST_CAPT_2				25 //Distance entre le capteur 2 et l'axe du robot

#define GOAL_ANGLE				0.0f 	//angle cible de l'axe du robot p.r. au mur
#define GOAL_DIST				25.0f	//distance cible entre le robot et le mur en mm
#define DIST_CAPT				110.0f	// distance entre les capteurs IR 2 et 4
#define MAX_DISTANCE				50.0f 	// distance limite renvoyée si le capteur ne "voit rien"
#define ERROR_THRESHOLD			0.5f		// experimental value
#define CAPT_TRESHOLD			124		//valeur renvoyée par le capteur IR à partir de laquelle on traite l'information

#define KP						20.0f
#define KD						25.0f

#define SPEED					800  // default step/s

//Différentes valeur de la variable globale mobile
#define MOVE_OFF					0
#define MOVE_ON					1
#define MOVE_MANOEUVRE			2
#define EN_DEPASSEMENT			3
#define MOVE_START				4

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
