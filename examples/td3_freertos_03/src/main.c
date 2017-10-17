/* Copyright 2017
 */

/*==================[inclusions]=============================================*/

#include "board.h"
#include "chip.h"

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "semphr.h"

#include "main.h"

/*==================[macros and definitions]=================================*/

#define TAM_COLA 100
#define TAM_PILA 1024
#define PRIO_PROC_SER 2

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

static xQueueHandle cola_rec ; /* Cola para recibir */

/*==================[internal functions definition]==========================*/

static void InitHardware(void)
{
    SystemCoreClockUpdate();
    Board_Init();
}

static void InitSerie(void)
{
	/* Primero se crea la cola */
	cola_rec = xQueueCreate (TAM_COLA , sizeof (char));
	if( cola_rec == NULL ){
		Board_LED_Set(3, TRUE); /* LED 3 indica error Fatal */
		while (1); /* Se queda bloqueado el sistema hasta que
					venga el técnico de mantenimiento */
	}
    Chip_UART_Init(LPC_USART2);
	Chip_UART_SetBaud(LPC_USART2, 115200);  /* Set Baud rate */
	Chip_UART_ConfigData(LPC_USART2, (UART_LCR_WLEN8 | UART_LCR_SBS_1BIT));
	Chip_UART_SetupFIFOS(LPC_USART2, UART_FCR_FIFO_EN | UART_FCR_TRG_LEV3); /* Modify FCR (FIFO Control Register)*/
	Chip_UART_TXEnable(LPC_USART2); /* Enable UART Transmission */
	Chip_SCU_PinMux(7, 1, MD_PDN, FUNC6);              /* P7_1,FUNC6: UART2_TXD */
	Chip_SCU_PinMux(7, 2, MD_PLN|MD_EZI|MD_ZI, FUNC6); /* P7_2,FUNC6: UART2_RXD */
	
	Chip_UART_IntEnable(LPC_USART2, UART_IER_RBRINT);
	NVIC_EnableIRQ(USART2_IRQn);
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

static void ProcesaMensaje(char *data)
{
	SeriePuts(data);
}

void ProcesaRecSerie (void * pvParameters)
{
	static char mensaje [100];
	static uint8_t indice = 0;
	char car_rec ;
	
	while (1){		
		if(xQueueReceive (cola_rec , &car_rec ,
			(portTickType) 0x0 ) == pdTRUE){
			/* Se ha recibido un carácter de la cola.
				Se almacena */
			Board_LED_Toggle(0);
			mensaje[indice] = car_rec;
			if(mensaje[indice] == '\r'){
				/* El \n indica el final del mensaje */
				mensaje[indice+1] = '\0';
				ProcesaMensaje(mensaje);
				indice = 0;
				Board_LED_Toggle(4);
			}else{
				indice ++;
			}
		}
	}
}

void UART2_IRQHandler(void)
{
	portBASE_TYPE xTaskWokenByPost = pdFALSE;
	char car_recibido ;

	if(Chip_UART_ReadLineStatus(LPC_USART2) & UART_LSR_RDR){
		/* Llegó un carácter . Se lee del puerto serie */
		car_recibido = Chip_UART_ReadByte(LPC_USART2);
		/* Y se envía a la cola de recepción */
		xTaskWokenByPost = xQueueSendFromISR(cola_rec,
						   &car_recibido, xTaskWokenByPost);
		if( xTaskWokenByPost == pdTRUE ){
			taskYIELD (); /* Si el envío a la cola ha despertado
							una tarea , se fuerza un cambio de
							contexto */
		}
	}
}

/*==================[external functions definition]==========================*/

int main(void)
{
	InitHardware(); /* Inicializa el Hardware del microcontrolador */
	InitSerie();

	/* Se crean las tareas */
	xTaskCreate(ProcesaRecSerie, (const char *)"ProcSerie", TAM_PILA, NULL, PRIO_PROC_SER, NULL );

	vTaskStartScheduler(); /* y por último se arranca el planificador . */
}

/*==================[end of file]============================================*/
