/* Copyright 2015, Pablo Ridolfi
 * All rights reserved.
 *
 * This file is part of lpc1769_template.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** @brief Blinky using FreeRTOS.
 *
 *
 * NOTE: It's interesting to check behavior differences between standard and
 * tickless mode. Set @ref configUSE_TICKLESS_IDLE to 1, increment a counter
 * in @ref vApplicationTickHook and print the counter value every second
 * inside a task. In standard mode the counter will have a value around 1000.
 * In tickless mode, it will be around 25.
 *
 */

/** \addtogroup rtos_blink FreeRTOS blink example
 ** @{ */

/*==================[inclusions]=============================================*/

#include "board.h"
#include "chip.h"
#include "string.h"

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include "main.h"

//#include "timer.h"
//#include "serie.h"


/*==================[macros and definitions]=================================*/

# define PRIO_T_ERR 1
# define PRIO_T1 2
# define PRIO_T2 3
# define PRIO_T3 4
# define PRIO_T4 5
# define TAM_PILA 256

# define TAM_COLA 20 /* 20 mensajes */
# define TAM_MSG 8 /* cada mensaje : "Tx:Eyy\n\0" ocupa 8 caracteres */

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/** @brief hardware initialization function
 *	@return none
 */
static void initHardware(void);

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

xQueueHandle cola_err;

/*==================[internal functions definition]==========================*/

static void initHardware(void)
{
    SystemCoreClockUpdate();

    Board_Init();
    //Board_LED_Init();

}

uint8_t LeeEntradas(void)
{
	return Buttons_GetStatus();
}

static void InitSerie(void)
{
    //Board_UART_Init (LPC_USART2);
    Chip_UART_Init(LPC_USART2);
	Chip_UART_SetBaud(LPC_USART2, 115200);  /* Set Baud rate */
	Chip_UART_ConfigData(LPC_USART2, (UART_LCR_WLEN8 | UART_LCR_SBS_1BIT));
	Chip_UART_SetupFIFOS(LPC_USART2, UART_FCR_FIFO_EN | UART_FCR_TRG_LEV3); /* Modify FCR (FIFO Control Register)*/
	Chip_UART_TXEnable(LPC_USART2); /* Enable UART Transmission */
	Chip_SCU_PinMux(7, 1, MD_PDN, FUNC6);              /* P7_1,FUNC6: UART2_TXD */
	Chip_SCU_PinMux(7, 2, MD_PLN|MD_EZI|MD_ZI, FUNC6); /* P7_2,FUNC6: UART2_RXD */
}

static void SeriePuts(char *data)
{
	while(*data != 0)
	{
		while ((Chip_UART_ReadLineStatus(LPC_USART2) & UART_LSR_THRE) == 0) {}
		Chip_UART_SendByte(LPC_USART2, *data);
		data++;
	}
}

static void Tarea1 (void * pvParameters)
{
	extern xQueueHandle cola_err ;
	char cad_err [8];
	uint8_t entradas;
	int error_1, error_2;
	
	while (1){
		/* Proceso Tarea1 */
		Board_LED_Set(5, TRUE);
		entradas = LeeEntradas ();
		switch(entradas) {
			case 0x01:
				error_1 = 1;
				break;
			case 0x02:
				error_2 = 1;
		}
		
		if (error_1){
			strcpy (cad_err, "T1:E01\n");
			xQueueSend (cola_err, (void *) cad_err, (portTickType) 100);
		}
		
		/* Continuación proceso Tarea1 */
		if (error_2){
			strcpy (cad_err, "T1:E02\n");
			xQueueSend (cola_err , (void *) cad_err ,(portTickType) 100);
		}
		/* Resto proceso Tarea1 */
	}
}

static void Tarea2 (void * pvParameters)
{
	extern xQueueHandle cola_err ;
	char cad_err [8];
	uint8_t entradas;
	int error_1, error_27;
	
	while (1){
		/* Proceso Tarea2 */
		Board_LED_Toggle(4);
		entradas = LeeEntradas ();
		switch(entradas){
			case 0x04:
				error_1 = 1;
				break;
			case 0x08:
				error_27 = 1;
		}

		if (error_1){
			strcpy (cad_err, "T2:E01\n");
			xQueueSend (cola_err, (void *) cad_err, (portTickType) 0);
			/* El timeout es 0 para no bloquear la tarea
			si la cola está llena */
		}
		/* Continuación proceso Tarea2 */
		if (error_27){
			strcpy (cad_err , "T2:E27\n");
			xQueueSend (cola_err, (void *) cad_err, (portTickType) 0);
		}
		/* Resto proceso Tarea2 */
	}
}

static void Tarea3 (void * pvParameters)
{
	extern xQueueHandle cola_err ;
	char cad_err [8];
	uint8_t entradas;
	int error_10, error_11;
	
	while (1){
		/* Proceso Tarea3 */
		entradas = LeeEntradas ();
		switch(entradas) {
			case 0x09:
				error_10 = 1;
				break;
			case 0x06:
				error_11 = 1;
		}

		if (error_10){
			strcpy (cad_err, "T3:E10\n");
			xQueueSend (cola_err, (void *) cad_err, (portTickType) 0);
			/* El timeout es 0 para no bloquear la tarea
			si la cola está llena */
		}
		/* Continuación proceso Tarea2 */
		if (error_11){
			strcpy (cad_err , "T3:E11\n");
			xQueueSend (cola_err, (void *) cad_err, (portTickType) 0);
		}
		/* Resto proceso Tarea3 */
	}
}

static void Tarea4 (void * pvParameters)
{
	extern xQueueHandle cola_err ;
	char cad_err [8];
	uint8_t entradas;
	int error_20, error_21;
	
	while (1){
		/* Proceso Tarea4 */
		entradas = LeeEntradas ();
		switch(entradas) {
			case 0x04:
				error_20 = 3;
				break;
			case 0x08:
				error_21 = 12;
		}

		if (error_20){
			strcpy (cad_err, "T4:E20\n");
			xQueueSend (cola_err, (void *) cad_err, (portTickType) 0);
			/* El timeout es 0 para no bloquear la tarea
			si la cola está llena */
		}
		/* Continuación proceso Tarea2 */
		if (error_21){
			strcpy (cad_err, "T4:E21\n");
			xQueueSend (cola_err, (void *) cad_err, (portTickType) 0);
		}
		/* Resto proceso Tarea2 */
	}
}

static void TareaErr (void * pvParameters )
{
	extern xQueueHandle cola_err ;
	char cad_rec [8];
	char cadena [100];

	while (1){
		if(xQueueReceive (cola_err , (void *) cad_rec,
				(portTickType ) 0xFFFFFFFF ) == pdTRUE ){
			/* Se ha recibido un dato. Se escribe en EEPROM */
			//EscEEPROM ((void *) cad_rec, 8);
			//sprintf (cadena , "EEPROM: %x\n", cad_rec );
			SeriePuts (cad_rec);
		}
		/* si después de un timeout no se ha recibido nada
		la tarea se vuelve a bloquear a la espera de un
		nuevo dato */
	}
}


/*==================[external functions definition]==========================*/

int main(void)
{
	initHardware(); /* Inicializa el Hardware del microcontrolador */
	//InitTimer();
	InitSerie();
	//InitQueSeYo ();
	
	Board_LED_Set(4, TRUE);
	
	/* Se crea la cola */
	cola_err = xQueueCreate (TAM_COLA, TAM_MSG);
	
	/* Se crean las tareas */
	xTaskCreate (TareaErr, (const char *)"TareaE", TAM_PILA, NULL,
				PRIO_T_ERR, NULL);
	xTaskCreate (Tarea1, (const char *)"Tarea1", TAM_PILA, NULL,
				PRIO_T1, NULL);
	xTaskCreate (Tarea2, (const char *)"Tarea2", TAM_PILA , NULL,
				PRIO_T2, NULL);
	xTaskCreate (Tarea3, (const char *)"Tarea3", TAM_PILA , NULL,
				PRIO_T3, NULL );
	xTaskCreate (Tarea4, (const char *)"Tarea4", TAM_PILA , NULL,
				PRIO_T4, NULL );
	
	vTaskStartScheduler(); /* y por último se arranca el planificador . */
}

/** @} doxygen end group definition */

/*==================[end of file]============================================*/
