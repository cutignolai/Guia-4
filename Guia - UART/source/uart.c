/***************************************************************************//**
  @file     UART.c
  @brief    UART Driver for K64F. Non-Blocking and using FIFO feature
  @author   Oli, Cuty y Micho
 ******************************************************************************/

#include <stdbool.h>
#include "MK64F12.h"
#include "hardware.h"
#include "uart.h"
#include "gpio.h"
#include "board.h"

/!*******************************************************************************
 * 					CONSTANT AND MACRO DEFINITIONS USING #DEFINE
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


/***********************************************************************************************************
 * 									ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ***********************************************************************************************************/

/***********************************************************************************************************
 * 									VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ***********************************************************************************************************/

void UART_EnableClockGating(uint8_t id);

void UART_DisableTxRx(UART_Type *uart);

void UART_SetBaudRate (UART_Type *uart, uint32_t baudrate);

void UART_SetParity(UART_Type * uartX_ptr, bool want_parity, bool parity);

void UART_SetDataSize(UART_Type * uartX_ptr, bool data_9bits);

void UART_SetStopBit(UART_Type * uartX_ptr, bool double_stop_bit);

void UART_EnableTxRx(UART_Type *uart);

void UART_SetEnableIRQ(uint8_t id);

void UART_Send_Data(uint8_t id, unsigned char tx_data);

unsigned char UART_Recieve_Data(uint8_t id);
 

/***********************************************************************************************************
 * 													VARIABLES
 ***********************************************************************************************************/

static UART_Type* const UART_ptrs[] = UART_BASE_PTRS;		// { UART0, UART1, UART2, UART3, UART4, UART5 } (Ver MK64F12.h)
static PORT_Type * const addr_arrays[] = {PORTA, PORTC, PORTD, PORTC, PORTE};

static const pin_t TX_PINS[] = {UART0_TX, UART1_TX, UART2_TX, UART3_TX, UART4_TX};
static const pin_t RX_PINS[] = {UART0_RX, UART1_RX, UART2_RX, UART3_RX, UART4_RX};
static bool uart_is_used[UART_CANT_IDS] = {false, false, false, false, false};

static bool all_bytes_were_transfered = true;
static unsigned int total = 0;



/***********************************************************************************************************
 * 									FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ***********************************************************************************************************/

void uartInit (uint8_t id, uart_cfg_t config)
{
	if(uart_is_used[id] || id >= UART_CANT_IDS){
		return;
  	}

	//******************************** CLOCK GATING ***********************************//

	UART_EnableClockGating(id);			// Clock gating para la UART
	
	//********************************* PIN SETUP ************************************//

	UART_Type* uartX_ptr = UART_ptrs[id];
	PORT_Type* port_ptr = addr_arrays[id];
	pin_t UARTX_TX_PIN = TX_PINS[id];
	pin_t UARTX_RX_PIN = RX_PINS[id];
	// TX:
	port_ptr->PCR[UARTX_TX_PIN] = 0x0; //Clear all bits
	port_ptr->PCR[UARTX_TX_PIN]|=PORT_PCR_MUX(0b11); //Set MUX to UART
	port_ptr->PCR[UARTX_TX_PIN]|=PORT_PCR_IRQC(0b0000); //Disable Port interrupts
	
	// RX:
	port_ptr->PCR[UARTX_RX_PIN]= 0x0; //Clear all bits
	port_ptr->PCR[UARTX_RX_PIN]|=PORT_PCR_MUX(0b11); //Set MUX to UART0
	port_ptr->PCR[UARTX_RX_PIN]|=PORT_PCR_IRQC(0b0000); //Disable Port interrupts



	//* Deshabilito comunicación
	UART_DisableTxRx(uartX_ptr);

	//* Limpio por si hay basura en C1 (C1 es DATA)
	uartX_ptr->C1 = 0x0;	

	//* Seteo el BaudRate (diapositiva de Dani)
	UART_SetBaudRate(uartX_ptr, config.baud_rate);

	//* Seteo si hay paridad y que tipo
	UART_SetParity(uartX_ptr, config.want_parity, config.parity_type);

	//* Me fijo si la DATA es de 8 o 9 bits 
	UART_SetDataSize(uartX_ptr, config.data_9bits);

	//* Seteo si el stop es de 1 o 2 bits 	
	UART_SetStopBit(uartX_ptr, config.double_stop_bit);

	//* Habilito la interrupcion
	UART_SetEnableIRQ(id);

	//* Indico que se ocupo el UART
	uart_is_used[id] = true;

	//* Habilito comunicación
	UART_EnableTxRx(uartX_ptr);

	//! *********************************
	//! 			TODO: 				*
	//! 		MODOS: bloq vs no		*
	//! *********************************
	
}




uint8_t uartIsRxMsg(uint8_t id){
	return uartGetRxMsgLength(id) > 0;
}


uint8_t uartGetRxMsgLength(uint8_t id){
	return total;
}


uint8_t uartReadMsg(uint8_t id, char* msg, uint8_t cant){
	if (id >= UART_N_IDS){
		return false;
	}
	else{
		total = ( cant / sizeof(char) );
		for(uint8_t i = 0; i < total; i++){
			msg[i] = UART_Recieve_Data(id);
		}
	return true;
	}
}

//mando la data
uint8_t uartWriteMsg(uint8_t id, const char* msg, uint8_t cant){
	for(uint8_t i = 0; i < cant; i++){
		UART_Send_Data(id, msg[i]);
	}
	return true;
}

//TC me dice si se esta mandando datos, si todavia esta escribiendo, entonces TC esta en 0. Si ya no transmite mas entonces esta en 1.
uint8_t uartIsTxMsgComplete(uint8_t id){
	UART_Type* uart = UART_ptrs[id];
	if(((uart->S1) & UART_S1_TC_MASK) == 0){
		all_bytes_were_transfered = false;
	}
	else{
		all_bytes_were_transfered = true;
	}
	return all_bytes_were_transfered;
}



/************************************************************************************************************
* 										FUNCTION PROTOTYPES WITH LOCAL SCOPE
***********************************************************************************************************/


void UART_EnableClockGating(uint8_t id){
	switch(id)
	{
		case 0:
			SIM->SCGC4 |= SIM_SCGC4_UART0_MASK;
			break;
		
		case 1:
			SIM->SCGC4 |= SIM_SCGC4_UART1_MASK;
			break;

		case 2:
			SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
			break;

		case 3:
			SIM->SCGC4 |= SIM_SCGC4_UART3_MASK;
			break;
		
		case 4:
			SIM->SCGC1 |= SIM_SCGC1_UART4_MASK;
			break;
		
		case 5:
			SIM->SCGC1 |= SIM_SCGC1_UART5_MASK;
			break;
	}
}


void UART_DisableTxRx(UART_Type *uart){
	//52.8.3 Initialization sequence (non ISO-7816)  -----> REFERENCE MANUAL
	// C1: DATA
	// C2: ENABLE
	// C4: BAUDRATE

	/*
	 * Deshabilito la UART para configurar sin problemas
	 * FIXME: --> Si no anda, hay que deshabilitar las interrupciones tambien
	*/
	uart->C2 &= !( UART_C2_TE_MASK | UART_C2_RE_MASK);
}


void UART_EnableTxRx(UART_Type *uart){
	uart-> C2 = (UART_C2_TE_MASK | UART_C2_RE_MASK | UART_C2_RIE_MASK);
}


void UART_SetBaudRate(UART_Type *uart, uint32_t baudrate){
	uint16_t sbr, brfa;
	uint32_t clock;

	clock = ((uart == UART0) || (uart == UART1)) ? (__CORE_CLOCK__) : (__CORE_CLOCK__ >> 1);
	
	baudrate = ((baudrate == 0)? (UART_HAL_DEFAULT_BAUDRATE) : ((baudrate > 0x1FFF)? (UART_HAL_DEFAULT_BAUDRATE): (baudrate)));

	sbr = clock / (baudrate << 4);
	brfa = (clock << 1) / baudrate - (sbr << 5);
	uart->BDH = UART_BDH_SBR(sbr >> 8); 
	uart->BDL = UART_BDL_SBR(sbr); 
	uart->C4 = (uart->C4 & ~UART_C4_BRFA_MASK) | UART_C4_BRFA(brfa);
}


void UART_SetParity(UART_Type * uartX_ptr, bool want_parity, bool parity_type){
	if(want_parity == PARITY_YES){
		uartX_ptr->C1 |= UART_C1_PE_MASK;
		if(parity_type == PARITY_EVEN){
			uartX_ptr->C1 |= UART_C1_PT_MASK;
		}
		else{
			uartX_ptr->C1 &= !UART_C1_PT_MASK;
		}
	} 
	else{
		uartX_ptr->C1 &= !UART_C1_PE_MASK;
	}
	
}


void UART_SetDataSize(UART_Type * uartX_ptr, bool data_9bits){
	if(data_9bits){
		uartX_ptr->C1 |= UART_C1_M_MASK;
	}
	else{
		uartX_ptr->C1 &= !UART_C1_M_MASK;
	}
}


void UART_SetStopBit(UART_Type * uartX_ptr, bool double_stop_bit){
	if(double_stop_bit){
		uartX_ptr->BDH |= UART_BDH_SBNS_MASK;
	}
	else{
		uartX_ptr->BDH &= !UART_BDH_SBNS_MASK;
	}
}


void UART_SetEnableIRQ(uint8_t id)
{
	switch(id) { 										//habilito interrupciones y me fijo en cual fue.
		case 0:
			NVIC_EnableIRQ(UART0_RX_TX_IRQn); 			//PPT DE DANI PAG 20
			break;
		case 1:
			NVIC_EnableIRQ(UART1_RX_TX_IRQn);
			break;
		case 2:
			NVIC_EnableIRQ(UART2_RX_TX_IRQn);
			break;
		case 3:
			NVIC_EnableIRQ(UART3_RX_TX_IRQn);
			break;
		case 4:
			NVIC_EnableIRQ(UART4_RX_TX_IRQn);
			break;
		default:
			break;
	}
}



//RE BLOQUEANTE
void UART_Send_Data(uint8_t id, unsigned char tx_data){
	UART_Type* uart = UART_ptrs[id];
	while(((uart->S1)& UART_S1_TDRE_MASK) ==0); //Puedo Transmitir ?
	uart->D = tx_data; // Transmito
}


//RE BLOQUEANTE
unsigned char UART_Recieve_Data(uint8_t id){
	UART_Type* uart = UART_ptrs[id];
	while(((uart -> S1) & UART_S1_RDRF_MASK) == 0); // Espero recibir un caracter
	return(uart->D); //Devuelvo el caracter recibido
}

