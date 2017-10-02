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

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "semphr.h"

#include "main.h"

//#include "timer.h"
//#include "serie.h"


/*==================[macros and definitions]=================================*/

#define PRIO_IMP_HORA 2
#define PRIO_ENV_ENTR 1
#define TAM_PILA 1024

typedef struct {
uint8_t hor;
uint8_t min;
uint8_t seg;
}HORA;

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/** @brief hardware initialization function
 *	@return none
 */
static void initHardware(void);

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

xSemaphoreHandle sem_serie;
xSemaphoreHandle sem_hora;
static HORA hora_act ={0,0,0};

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

static void InitTimer(void)
{
	Chip_RIT_Init(LPC_RITIMER);
	Chip_RIT_SetTimerInterval(LPC_RITIMER,1000);
}

static void ImprimeHora(void * a)
{
	HORA copia_hora ;
	char cadena [10];
	extern xSemaphoreHandle sem_serie ;
	
	while (1){
		if( xSemaphoreTake (sem_hora , ( portTickType ) 2000 ) == pdTRUE ){
			/* Se bloquea hasta que llegue la interrupción de tiempo */
//			DisableInt();
			copia_hora = hora_act ;
//			EnableInt ();
			sprintf (cadena , " %02d: %02d: %02d\n", copia_hora.hor, copia_hora.min, copia_hora .seg );
			if( xSemaphoreTake (sem_serie, ( portTickType ) 1000 ) == pdTRUE ){
				SeriePuts (cadena); /* Se tiene el semáforo : se puede acceder al puerto serie */
			
			xSemaphoreGive ( sem_serie ); /*Se suelta el semáforo */
			}else{
				/* Después de 1000 ticks no se ha obtenido el
				semáforo . Se podría dar un aviso o
				simplemente no hacer nada como en este caso */
			}
		}
	}
}

static void EnviaEntradas(void * a)
{
	char cadena [100]; /* Guarda el mensaje a transmitir */
	uint8_t entradas;
	static uint8_t entradas_ant = 0;
	extern xSemaphoreHandle sem_serie ;

	while (1){
		entradas = LeeEntradas ();
		if( entradas_ant != entradas ){ /* Sólo imprime si cambian las entradas */
			sprintf (cadena , "Entradas: %x\n", entradas );
			if( xSemaphoreTake (sem_serie , ( portTickType ) 1000) == pdTRUE ){
				/* Se tiene el semáforo : se puede acceder al puerto serie */
				SeriePuts (cadena);
				/* Se suelta el semáforo */
				xSemaphoreGive (sem_serie);
			}else{
				/* Después de 1000 ticks no se ha obtenido el
				semáforo . Se podría dar un aviso o
				simplemente no hacer nada como en este caso */
			}
			entradas_ant = entradas;
		}
	}
}

void RIT_IRQHandler(void)
{
	Board_LED_Toggle(5);
	
	portBASE_TYPE xTaskWoken = pdFALSE ;
	
	hora_act .seg ++;
	if( hora_act .seg == 60){
		hora_act .seg = 0;
		hora_act .min ++;
		if( hora_act .min == 60){
			hora_act .min = 0;
			hora_act .hor ++;
			if( hora_act .hor == 24){
				hora_act .hor = 0;
			}
		}
	}
	/* Lanza las tareas */
	xTaskWoken = xSemaphoreGiveFromISR (sem_hora, xTaskWoken);
	
	/* Borra el flag de interrupción */
	Chip_RIT_ClearInt(LPC_RITIMER);

	if( xTaskWoken == pdTRUE ){
		taskYIELD (); /* Si el semáforo ha despertado
						una tarea , se fuerza un cambio
						de contexto */
	}
}

/*==================[external functions definition]==========================*/

int main(void)
{
	initHardware(); /* Inicializa el Hardware del microcontrolador */
	InitTimer();
	InitSerie();
	//InitQueSeYo ();
	/* Se inicializan los semáforos */
	vSemaphoreCreateBinary (sem_serie);
	vSemaphoreCreateBinary (sem_hora);
	/* Se crean las tareas */
	xTaskCreate(ImprimeHora, (const char *)"ImpHora", TAM_PILA, NULL, PRIO_IMP_HORA, NULL );
	xTaskCreate(EnviaEntradas, (const char *)"EnvEntr", TAM_PILA, NULL, PRIO_ENV_ENTR, NULL );

	NVIC_EnableIRQ(RITIMER_IRQn);
	vTaskStartScheduler(); /* y por último se arranca el planificador . */
}

/** @} doxygen end group definition */

/*==================[end of file]============================================*/
