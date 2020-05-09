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

    //Démarre la communication sériale
    serial_start();

    //Démarre la communication USB
    usb_start();

	//Initialise les moteurs
	motors_init();

	//Démarre la communication spi nécessaire pour les LEDs RGB
	spi_comm_start();

	messagebus_init(&bus, &bus_lock, &bus_condvar);

	//Initialise les capteurs IR
	proximity_start();
	//Démarre la thread pour les clignotants
	clignotant_start();
	//Démarre la thread pour le dépassement
	depassement_start();
	//Démarre la thread pour la manoeuvre
	manoeuvre_start();
	//Démarre la thread pour le déplacement
	deplacement_start();

	//Initialise le microphone
	mic_start(&processAudioData);


    /* Infinite loop. */
    while (1)
    {
        chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
