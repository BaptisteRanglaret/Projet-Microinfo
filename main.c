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

/*static void timer12_start(void)
{
    //General Purpose Timer configuration
    //timer 12 is a 16 bit timer so we can measure time
    //to about 65ms with a 1Mhz counter
    static const GPTConfig gpt12cfg =
    {
        1000000,        // 1MHz timer clock in order to measure uS.
        NULL,           // Timer callback.
        0,
        0
    };

    gptStart(&GPTD12, &gpt12cfg);
    //let the timer count to max value
    gptStartContinuous(&GPTD12, 0xFFFF);
}*/

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();
    //timer12_start();

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
    		//chprintf((BaseSequentialStream *)&SDU1, "DISTANCE capteur 2 = %f\n", convertisseur_value_dist(get_calibrated_prox(1)));
    		//chprintf((BaseSequentialStream *)&SDU1, "DISTANCE capteur 3 = %f\n", convertisseur_value_dist(get_calibrated_prox(2)));
    		//right_motor_set_speed(1000);
    		//left_motor_set_speed(1000);
        chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
