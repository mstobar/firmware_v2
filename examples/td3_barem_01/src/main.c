/*==================[inclusions]=============================================*/

#include "board.h"
#include "chip.h"

#include "main.h"

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

int seg, min, hor;

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

static void initHardware(void)
{
    SystemCoreClockUpdate();
    Board_Init();
}

static void InicializaSerie(void)
{
    Chip_UART_Init(LPC_USART2);
	Chip_UART_SetBaud(LPC_USART2, 300);  /* Set Baud rate */
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

static void InicializaTemporizador(void)
{
	Chip_RIT_Init(LPC_RITIMER);
	Chip_RIT_SetTimerInterval(LPC_RITIMER,1);
}

static void ImprimeHora(void)
{
	char cadena [30];

	sprintf (cadena , " Hora: %02d", hor);
	SeriePuts (cadena);
	sprintf (cadena , " : %02d : ", min);
	SeriePuts (cadena);
	sprintf (cadena , "%02d\n\r", seg);
	SeriePuts (cadena);
}

void RIT_IRQHandler(void)
{
	static int ms;

	ms++;
	if(ms == 1000){
		Board_LED_Toggle(5);
		ms = 0;
		seg++;
		if(seg == 60){
			seg = 0;
			min ++;
			if(min == 60){
				min = 0;
				hor ++;
				if(hor == 24){
					hor = 0;
				}
			}
		}
	}
	Chip_RIT_ClearInt(LPC_RITIMER);
}

/*==================[external functions definition]==========================*/

int main(void)
{
	initHardware(); /* Inicializa el Hardware del microcontrolador */
	InicializaTemporizador(); /* Inicializa timer RIT */
	InicializaSerie(); /* Inicializa puerto serie */

	NVIC_EnableIRQ(RITIMER_IRQn);

	for(;;){
		ImprimeHora();
	}
}

/*==================[end of file]============================================*/
