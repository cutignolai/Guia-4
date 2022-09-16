/***************************************************************************//**
  @file     App.c
  @brief    Application functions
  @author   Pablo Gonzalez
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "board.h"
#include "gpio.h"
#include "uart.h"
#include "MK64F12.h"
#include "hardware.h"
#include <stdio.h>
#include <string.h>

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/
/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/
#define CR	(0x0D)
#define LF	(0x0A)


static const char message[] = "The quick brown fox jumps over the lazy dog";
static int x = 0;

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/



/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/
void UART_Send_Data(unsigned char tx_data);
void UART_SetBaudRate (UART_Type* uart, uint32_t baudrate);

/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

/* Función que se llama 1 vez, al comienzo del programa */
void App_Init (void)
{
	gpioMode(PIN_LED_GREEN, OUTPUT);
	gpioMode(PIN_LED_BLUE, OUTPUT);
	gpioWrite(PIN_LED_BLUE, LOW);
	gpioWrite(PIN_LED_GREEN, LOW);
	uartInit();
}

/* Función que se llama constantemente en un ciclo infinito */
void App_Run (void)
{
	
	if(x < 20)
	{
		gpioWrite(PIN_LED_BLUE, LOW);
		gpioToggle(PIN_LED_GREEN);
		for (int i=0; i<strlen(message); i++)
		{
			UART_Send_Data(message[i]);
		}
		UART_Send_Data(CR);
		UART_Send_Data(LF);
		x += 1;
	}

	else
	{
		gpioWrite(PIN_LED_GREEN, LOW);
		gpioToggle(PIN_LED_BLUE);
	}
}


/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/




/*******************************************************************************
 ******************************************************************************/

