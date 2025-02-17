#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <stdio.h>
#include <unistd.h>

#include <audio/microphone.h>
#include <audio_processing.h>
#include <fft.h>
#include <arm_math.h>
#include <motors.h>


//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];

//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];

static int signal=0; // variable globale à transmettre aux threads de mouvements

#define MIN_VALUE_THRESHOLD	100000

#define MIN_FREQ		10	//we don't analyze before this index to not use resources for nothing
#define FREQ_18		18	//1050Hz
#define FREQ_20		20	//1180Hz
#define FREQ_22		22	//1300HZ
#define FREQ_24		24	//1440Hz
#define FREQ_26		26	//1560Hz
#define MAX_FREQ		30	//we don't analyze after this index to not use resources for nothing


/*
*	Simple function used to detect the highest value in a buffer
*	and to execute a motor command depending on it
*/
void sound_remote(float* data)
{
	float max_norm = MIN_VALUE_THRESHOLD;
	int16_t max_norm_index = -1; 

	//search for the highest peak
	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++)
	{
		if(data[i] > max_norm)
		{
			max_norm = data[i];
			max_norm_index = i;
		}
	}

	if(max_norm_index >= FREQ_22 && max_norm_index <= FREQ_26) //si le son détecté est compris entre 1300 et 1560 Hz
	{
		signal=ON;
	}
	else
	{
		signal=OFF;
	}
}

/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*	
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/
void processAudioData(int16_t *data, uint16_t num_samples){

	/*
	*
	*	We get 160 samples per mic every 10ms
	*	So we fill the samples buffers to reach
	*	1024 samples, then we compute the FFTs.
	*
	*/

	static uint16_t nb_samples = 0;

	//loop to fill the buffers
	for(uint16_t i = 0 ; i < num_samples ; i++){
		//construct an array of complex numbers. Put 0 to the imaginary part
		micLeft_cmplx_input[nb_samples] = (float)data[i + MIC_LEFT];

		nb_samples++;

		micLeft_cmplx_input[nb_samples] = 0;

		nb_samples++;

		//stop when buffer is full
		if(nb_samples >= (2 * FFT_SIZE)){
			break;
		}
	}

	if(nb_samples >= (2 * FFT_SIZE)){
		/*	FFT proccessing
		*
		*	This FFT function stores the results in the input buffer given.
		*	This is an "In Place" function. 
		*/

		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);

		/*	Magnitude processing
		*
		*	Computes the magnitude of the complex numbers and
		*	stores them in a buffer of FFT_SIZE because it only contains
		*	real numbers.
		*
		*/
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);

		nb_samples = 0;

		sound_remote(micLeft_output);
	}
}


float* get_audio_buffer_ptr(BUFFER_NAME_t name){
	if(name == LEFT_CMPLX_INPUT){
		return micLeft_cmplx_input;
	}
	else{
		return NULL;
	}
}

int return_signal(void) 		// Fonction qui permet de renvoyer la valeur de signal dans d'autres fichiers
{
	return signal;
}
