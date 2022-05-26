/*
 ChibiOS - Copyright (C) 2006..2016 Giovanni Di Sirio

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

 http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
 */

#include <stdio.h>
#include <string.h>

#include "ch.h"
#include "hal.h"

//#include "shell.h"
#include "chprintf.h"

//#include "usbcfg.h"

static void pwmpcb(PWMDriver *pwmp) {
  (void)pwmp;

  palTogglePad(GPIOB, 6);
  // palSetPad(IOPORT3, GPIOC_LED);
}


static PWMConfig pwmcfg = {
  10000,
  10000,
  NULL,
  //pwmpcb,
  {
//    {PWM_OUTPUT_ACTIVE_HIGH | PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH, NULL},
    //{PWM_OUTPUT_ACTIVE_HIGH | PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH, NULL},
    //{PWM_OUTPUT_ACTIVE_HIGH | PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH, NULL},
	//{PWM_OUTPUT_ACTIVE_HIGH , NULL},
	//{PWM_OUTPUT_ACTIVE_HIGH , NULL},
		  {PWM_OUTPUT_ACTIVE_LOW | PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH, NULL},
		  {PWM_OUTPUT_ACTIVE_LOW | PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH, NULL},
		  {PWM_OUTPUT_ACTIVE_LOW | PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH, NULL},
		  {PWM_OUTPUT_ACTIVE_HIGH | PWM_COMPLEMENTARY_OUTPUT_ACTIVE_LOW, NULL}
    //{PWM_OUTPUT_DISABLED, NULL},
	//{PWM_OUTPUT_DISABLED, NULL},
	//{PWM_OUTPUT_DISABLED, NULL}
  },
  0,
  0
};


/*
 * Low speed SPI configuration (281.250kHz, CPHA=0, CPOL=0, MSb first).
 */
static const SPIConfig ls_spicfg = {
NULL, GPIOA, 1, SPI_CR1_CPOL | SPI_CR1_CPHA , 0 };
//SPI_CR1_BR_0|SPI_CR1_BR_1
//SPI_CR1_BR_2 | SPI_CR1_BR_1

/*
 * SPI TX and RX buffers.
 */
static uint8_t txbuf[2];
static uint8_t rxbuf[2];

/*===========================================================================*/
/* Command line related.                                                     */
/*===========================================================================*/

#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(2048)

/* Can be measured using dd if=/dev/xxxx of=/dev/null bs=512 count=10000.*/
//static void cmd_write(BaseSequentialStream *chp, int argc, char *argv[]) {
//  static uint8_t buf[] =
//      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
//      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
//      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
//      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
//      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
//      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
//      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
//      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
//      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
//      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
//      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
//      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
//      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
//      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
//      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
//      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";
//
//  (void)argv;
//  if (argc > 0) {
//    chprintf(chp, "Usage: write\r\n");
//    return;
//  }
//  while (chnGetTimeout((BaseChannel *)chp, TIME_IMMEDIATE) == Q_TIMEOUT) {
//#if 1
//    /* Writing in channel mode.*/
//    chnWrite(&SDU1, buf, sizeof buf - 1);
//#else
//    /* Writing in buffer mode.*/
//    (void) obqGetEmptyBufferTimeout(&SDU1.obqueue, TIME_INFINITE);
//    memcpy(SDU1.obqueue.ptr, buf, SERIAL_USB_BUFFERS_SIZE);
//    obqPostFullBuffer(&SDU1.obqueue, SERIAL_USB_BUFFERS_SIZE);
//#endif
//  }
//  chprintf(chp, "\r\n\nstopped\r\n");
//}
//static const ShellCommand commands[] = {
//  {"write", cmd_write},
//  {NULL, NULL}
//};
//
//static const ShellConfig shell_cfg1 = {
//  (BaseSequentialStream *)&SDU1,
//  commands
//};
/*===========================================================================*/
/* Generic code.                                                             */
/*===========================================================================*/

/*
 * Blinker thread, times are in milliseconds.
 */
static THD_WORKING_AREA(waThread1, 128);
static __attribute__((noreturn)) THD_FUNCTION(Thread1, arg) {

	(void) arg;
	chRegSetThreadName("blinker");
	//int counter = 0;
	//while (counter++ < 10) {
	BaseSequentialStream *chp = (BaseSequentialStream*)(&SD3);
	while(true){
		//systime_t time = serusbcfg.usbp->state == USB_ACTIVE ? 1000 : 1500;
		//palSetPad(GPIOB, GPIOB_LED);
//    sdWrite(&SD1, (uint8_t *)"Example: 1\r\n", 12);
//    chThdSleepMilliseconds(time);
//    //palClearPad(GPIOB, GPIOB_LED);
		//sdWrite(&SD3, (uint8_t *)"Example: 1\r\n", 12);
		//sdWrite(&SD1, (uint8_t *)"Example: 1\r\n", 12);
		//palSetPad(GPIOB, 6);
		//chThdSleepMilliseconds(1500);
		//palClearPad(GPIOB, 6);
		//chThdSleepMilliseconds(1500);
		//sdWrite(&SD3, (uint8_t *)"Example: 1\r\n", 12);

		spiAcquireBus(&SPID1); /* Acquire ownership of the bus.    */
		//palSetPad(IOPORT3, GPIOC_LED); /* LED OFF.                         */
		spiStart(&SPID1, &ls_spicfg); /* Setup transfer parameters.       */
		spiSelect(&SPID1); /* Slave Select assertion.          */
		spiExchange(&SPID1, 2, txbuf, rxbuf); /* Atomic transfer operations.      */
		spiUnselect(&SPID1); /* Slave Select de-assertion.       */
		spiReleaseBus(&SPID1); /* Ownership release.               */
		chprintf(chp, "first byte : %x\n", rxbuf[0]);
		chprintf(chp, "Recond byte : %x\n", rxbuf[1]);
		chThdSleepMilliseconds(1500);
	}
}

//static SerialConfig sd2cfg = {
//    115200,                                 /* 115200 baud rate */
//    USART_CR1_9BIT_WORD | USART_CR1_PARITY_SET | USART_CR1_EVEN_PARITY,
//    USART_CR2_STOP1_BITS | USART_CR2_LINEN,
//    0
//};

/*
 * Application entry point.
 */
int main(void) {

	/*
	 * System initializations.
	 * - HAL initialization, this also initializes the configured device drivers
	 *   and performs the board-specific initializations.
	 * - Kernel initialization, the main() function becomes a thread and the
	 *   RTOS is active.
	 */
	halInit();
	chSysInit();

	palSetPadMode(GPIOA, 5, PAL_MODE_STM32_ALTERNATE_PUSHPULL);     /* SCK. */
	  palSetPadMode(GPIOA, 6, PAL_MODE_STM32_ALTERNATE_PUSHPULL);     /* MISO.*/
	  palSetPadMode(GPIOA, 7, PAL_MODE_STM32_ALTERNATE_PUSHPULL);     /* MOSI.*/
	  palSetPadMode(GPIOA, 1, PAL_MODE_OUTPUT_PUSHPULL);
	  palSetPad(GPIOA, 1);

	//palSetPadMode(GPIOB, 6, PAL_MODE_ALTERNATE(1));
	//palSetPadMode(GPIOB, 7, PAL_MODE_ALTERNATE(1));
//  palSetPadMode(GPIOB, 10, PAL_MODE_ALTERNATE(1));
//  palSetPadMode(GPIOB, 11, PAL_MODE_ALTERNATE(1));

	/*
	 * Initializes a serial-over-USB CDC driver.
	 */
	//sduObjectInit(&SDU1);
	//sduStart(&SDU1, &serusbcfg);
	SerialConfig usartconfig;
	usartconfig.speed = 57600;

	//sduObjectInit(&SDU1);
	//sdObjectInit(sdp, inotify, onotify)
	//sdStart(&SD1, &usartconfig);

	//sdStart(&SD3, &usartconfig);
	sdStart(&SD3, NULL);
	palSetPadMode(GPIOB, 10, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
	//palSetPadMode(GPIOB, 10, PAL_MODE_STM32_ALTERNATE_OPENDRAIN);
	//palSetPadMode(GPIOB, 11, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
	//sdStart(&SD3, NULL);



	palSetPadMode(GPIOA, 8, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
	palSetPadMode(GPIOA, 9, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
	palSetPadMode(GPIOA, 10, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
	palSetPadMode(GPIOB, 13, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
	palSetPadMode(GPIOB, 14, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
	palSetPadMode(GPIOB, 15, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
	pwmStart(&PWMD1, &pwmcfg);
	//pwmEnableChannel(&PWMD1, 2, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 5));
	pwmEnableChannel(&PWMD1, 0, 5000);
	pwmEnableChannel(&PWMD1, 1, 5000);
	pwmEnableChannel(&PWMD1, 2, 5000);

	//sdStart(&SD1, &usartconfig);

	/*
	 * Activates the USB driver and then the USB bus pull-up on D+.
	 * Note, a delay is inserted in order to not have to disconnect the cable
	 * after a reset.
	 */
	//usbDisconnectBus(serusbcfg.usbp);
	chThdSleepMilliseconds(1500);
	//usbStart(serusbcfg.usbp, &usbcfg);
	//usbConnectBus(serusbcfg.usbp);

	/*
	 * Shell manager initialization.
	 */
//  shellInit();
	/*
	 * Creates the blinker thread.
	 */
	chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);
	//BaseSequentialStream *chp = (BaseSequentialStream*)(&SD3);
	/*
	 * Normal main() thread activity, spawning shells.
	 */
	while (true) {
//    if (SDU1.config->usbp->state == USB_ACTIVE) {
//      thread_t *shelltp = chThdCreateFromHeap(NULL, SHELL_WA_SIZE,
//                                              "shell", NORMALPRIO + 1,
//                                              shellThread, (void *)&shell_cfg1);
//      chThdWait(shelltp);               /* Waiting termination.             */
//    }

		chThdSleepMilliseconds(500);
		//(uint8_t *)
		//sdWrite(&SD3, (uint8_t* )"Example: 4\r\n", 12);
		//sdPut(&SD3, &rxbuf[100]);
		//chprintf(chp, "Second byte : %x\n", txbuf[1]);
		//chprintf(chp, "first byte : %x\n", rxbuf[0]);
		//chprintf(chp, "Recond byte : %x\n", rxbuf[1]);
		//sdPut(&SD3, rxbuf[510]);
		//sdWrite(&SD3, (uint8_t* )rxbuf, 8);
		//palSetPad(GPIOB, 6);
		chThdSleepMilliseconds(500);
		//sdWrite(&SD1, (uint8_t *)"Example: 1\r\n", 12);
		//    chThdSleepMilliseconds(time);
		//palClearPad(GPIOB, 6);
		//sdWrite(&SD3, (uint8_t *)"Example: 1\r\n", 12);
	}
}
