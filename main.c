#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <chprintf.h>

#include <pi_regulator.h>
#include <sensors/proximity.h>

#include <audio/microphone.h>

#include <leds.h>

#include <arm_math.h>
#include <motors.h>
#include <mouvement.h>

#include <pal.h>
#include <spi_comm.h>

#include <audio_processing.h>


messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

void SendUint8ToComputer(uint8_t* data, uint16_t size) 
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}


int main(void)
{
    halInit();
    chSysInit();
    mpu_init();

    //starts the serial communication
    serial_start();

    //start the USB communication
    usb_start();

	//inits the motors
	motors_init();

	//starts spi communication
	spi_comm_start();

	messagebus_init(&bus, &bus_lock, &bus_condvar);

	//starts everything
	proximity_start();
	clignotant_start();
	depassement_start();
	manoeuvre_start();
	deplacement_start();

	mic_start(&processAudioData);


    /* Infinite loop. */
    while (1)
    {
    	 	//calcul_angle (convertisseur_value_dist(get_calibrated_prox(1)), convertisseur_value_dist(get_calibrated_prox(3)));
    		//chprintf((BaseSequentialStream*)&SDU1, "Angle=%f\n", return_angle());
    		//chprintf((BaseSequentialStream*)&SDU1, "Distance capt 2=%f\n", convertisseur_value_dist(get_calibrated_prox(1)));
        chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
