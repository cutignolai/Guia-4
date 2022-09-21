/***************************************************************************//**
  @file     UART.c
  @brief    UART Driver for K64F. Non-Blocking and using FIFO feature
  @author   Oli, Cuty y Micho
 ******************************************************************************/
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "MK64F12.h"
#include "hardware.h"
#include "uart.h"
#include "gpio.h"
#include "board.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define UART_HAL_DEFAULT_BAUDRATE 9600

//TX
#define UART0_TX		DIO_20
#define UART1_TX		DIO_21
#define UART2_TX		DIO_22
#define UART3_TX		DIO_23
#define UART4_TX		DIO_24

//RX
#define UART0_RX		DIO_25
#define UART1_RX		DIO_26
#define UART2_RX		DIO_27
#define UART3_RX		DIO_28
#define UART4_RX		DIO_29

// UART0: TX = PTA2		RX = PTA3	---> UART0 = PORTA
// UART1: TX = PTC4		RX = PTC3	---> UART1 = PORTC
// UART2: TX = PTD3		RX = PTD2	---> UART2 = PORTD
// UART3: TX = PTC17	RX = PTC16	---> UART3 = PORTC
// UART4: TX = PTE25	RX = PTE24	---> UART4 = PORTE


/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

static const char message[] = "The quick brown fox jumps over the lazy dog";


/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

void uartInit (void)
{
	// Clock gating
	SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;

	// Pin Setup
	PORTD->PCR[2] = (uint32_t)0;
	PORTD->PCR[2] = PORT_PCR_MUX(0b11);
	PORTD->PCR[3] = (uint32_t)0;
	PORTD->PCR[3] = PORT_PCR_MUX(0b11);

	UART2->C1 |= UART_C1_PE(1) | UART_C1_PT(0);

	UART2->S2 |= UART_S2_MSBF(0);

	UART2->C5 &= ~UART_C5_TDMAS_MASK;
	UART2->C2 = UART_C2_TE_MASK;

	NVIC_EnableIRQ(UART2_RX_TX_IRQn);

	UART_SetBaudRate(UART2, 9600);

}




void UART_Send_Data(unsigned char tx_data)
{
	 while(((UART2->S1) & UART_S1_TDRE_MASK) == 0);
	 UART2->D = tx_data;
}


void UART_SetBaudRate (UART_Type* uart, uint32_t baudrate) {
	uint16_t sbr, brfa;
	uint32_t clock;
	clock = ((uart==UART0 || uart==UART1))?(__CORE_CLOCK__):(__CORE_CLOCK__>>1);

	sbr = clock / (baudrate<<4);
	brfa = (clock<<1) / baudrate - (sbr<<5);

	uart->BDH = UART_BDH_SBR(sbr>>8);
	uart->BDL = UART_BDL_SBR(sbr);
	uart->C4 = (uart->C4 & ~UART_C4_BRFA_MASK) | UART_C4_BRFA(brfa);
}


__ISR__ UART2_RX_TX_IRQHandler(void) {

	/************************************
	 * 		No olvidar !!				*
	 * 									*
	 * 		Para borrar el Flag RDRF	*
	 * 		1 Read Status				*
	 * 		2 Read Data					*
	 * 									*
	 * 		En ese orden				*
	 *									*
	 ************************************/
	UART2->S1;

	static int i = 0;


	uint32_t len = strlen(message);

	if(i<len){
		UART2->D = message[i]; // Transmito
		i++;
	}
	else if(i==len) {
		UART2->D = 0x00;
		i++;
	}
	else {
		UART2->D = 0x00;
		i=0;
	}
}

