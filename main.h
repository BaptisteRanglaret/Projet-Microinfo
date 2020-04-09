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
#define ROTATION_THRESHOLD		10
#define ROTATION_COEFF			2 
#define PXTOCM					1570.0f //experimental value
#define GOAL_VALUE				300.0f
#define MAX_DISTANCE 			25.0f
#define ERROR_THRESHOLD			100.0f	// experimental value
#define KP						0.3f
#define KP_PROCHE				0.1f
#define KP_LOIN					0.8f
#define KI 						0.005f	//must not be zero
#define KD						1.0f
#define KD_PROCHE				0.1f
#define KD_LOIN					1.0f
#define MAX_SUM_ERROR 			2000.0f
#define MAX_ERROR_DIFF			250.0f
#define SPEED					700  // default step/s

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
