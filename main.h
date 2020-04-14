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
#define PXTOCM					1570.0f //experimental value
#define ANGLE					1
#define NORME					0

//PID pour la distance avec le mur
#define GOAL_ANGLE1				63.4f //angle cible de l'axe du robot p.r. au mur si capteur 2 et 3
#define GOAL_ANGLE2				59.0f //angle cible de l'axe du robot p.r. au mur si capteur 3 et 4
#define GOAL_NORME1				111.8f
#define GOAL_NORME2				116.6f

#define DIST_CAPT1 				50.0f
#define DIST_CAPT2				60.0f
#define MAX_DISTANCE 			25.0f
#define ERROR_THRESHOLD			100.0f	// experimental value

#define KP						0.3f

#define KI 						0.005f	//must not be zero

#define KD						1.0f

#define MAX_SUM_ERROR 			2000.0f
#define MAX_ERROR_DIFF			250.0f
#define MAX_ERROR				1000.0f
#define SPEED					1000  // default step/s

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
