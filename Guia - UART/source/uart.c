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

void UART_EnableClockGating(uint8_t id);

void UART_DisableTxRx(UART_Type *uart);

void UART_SetBaudRate (UART_Type *uart, uint32_t baudrate);

void UART_SetParity(UART_Type * uartX_ptr, bool want_parity, bool parity);

void UART_SetDataSize(UART_Type * uartX_ptr, bool data_9bits);

void UART_SetStopBit(UART_Type * uartX_ptr, bool double_stop_bit);

void UART_EnableTxRx(UART_Type *uart);

void UART_SetEnableIRQ(uint8_t id);
 

static UART_Type* const UART_ptrs[] = UART_BASE_PTRS;		// { UART0, UART1, UART2, UART3, UART4, UART5 } (Ver MK64F12.h)
static PORT_Type * const addr_arrays[] = {PORTA, PORTC, PORTD, PORTC, PORTE};

static const pin_t TX_PINS[] = {UART0_TX, UART1_TX, UART2_TX, UART3_TX, UART4_TX};
static const pin_t RX_PINS[] = {UART0_RX, UART1_RX, UART2_RX, UART3_RX, UART4_RX};
static bool uart_is_used[UART_CANT_IDS] = {false, false, false, false, false};

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/**
 * @brief Initialize UART driver
 * @param id UART's number
 * @param config UART's configuration (baudrate, parity, etc.)
*/
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

	//*******************************************************************************//

	//52.8.3 Initialization sequence (non ISO-7816)  -----> REFERENCE MANUAL
	// C1: DATA
	// C2: ENABLE
	// C4: BAUDRATE

	/*
	 * Deshabilito la UART para configurar sin problemas
	 * FIXME: --> Si no anda, hay que deshabilitar las interrupciones tambien
	*/

	// Deshabilito comunicación
	UART_DisableTxRx(uartX_ptr);

	// Limpio por si hay basura en C1 (C1 es DATA)
	uartX_ptr->C1 = 0x0;	

	// Seteo el BaudRate (diapositiva de Dani)
	UART_SetBaudRate(uartX_ptr, config.baud_rate);

	// Seteo si hay paridad y que tipo
	UART_SetParity(uartX_ptr, config.want_parity, config.parity_type);

	// Me fijo si la DATA es de 8 o 9 bits 
	UART_SetDataSize(uartX_ptr, config.data_9bits);

	// Seteo si el stop es de 1 o 2 bits 	
	UART_SetStopBit(uartX_ptr, config.double_stop_bit);

	// Habilito la interrupcion
	UART_SetEnableIRQ(id);


	// TODO: 
	//- MODOS: bloq vs no

	
	uart_is_used[id] = true;

	// Habilito comunicación
	UART_EnableTxRx(uartX_ptr);
}





/**
 * @brief Check if a new byte was received
 * @param id UART's number
 * @return A new byte has being received
*/
uint8_t uartIsRxMsg(uint8_t id){
}

/**
 * @brief Check how many bytes were received
 * @param id UART's number
 * @return Quantity of received bytes
*/
uint8_t uartGetRxMsgLength(uint8_t id){
	//if (id >= UART_N_IDS)
	//	return 0;

	//return rx_q[id].len;
}

/****
 * @brief 
 * @param id UART's number
 * @param msg Buffer to paste the received bytes
 * @param cant Desired quantity of bytes to be pasted
 * @return Real quantity of pasted bytes
*/
uint8_t uartReadMsg(uint8_t id, char* msg, uint8_t cant){



}

/**
 * @brief Write a message to be transmitted. Non-Blocking
 * @param id UART's number
 * @param msg Buffer with the bytes to be transfered
 * @param cant Desired quantity of bytes to be transfered
 * @return Real quantity of bytes to be transfered
*/
uint8_t uartWriteMsg(uint8_t id, const char* msg, uint8_t cant){
	for(uint8_t i = 0; i < cant; i++){
		UART_Send_Data(id, msg[i]);
		return 1;
	}
}

/**
 * @brief Check if all bytes were transfered
 * @param id UART's number
 * @return All bytes were transfered
*/
uint8_t uartIsTxMsgComplete(uint8_t id){




}




























/*******************************************************************************
 * FUNCTION PROTOTYPES WITH LOCAL SCOPE
 ******************************************************************************/

/**
 * @brief Enable Clock Gating for specific UART
 * @param id UART to enable
 */
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

/**
 * @brief Disables UART's serial communication
 * @param uart UART peripheral base pointer
 */
void UART_DisableTxRx(UART_Type *uart){
	uart->C2 &= !( UART_C2_TE_MASK | UART_C2_RE_MASK);
}

/**
 * @brief Enables UART's serial communication
 * @param uart UART peripheral base pointer
 */
void UART_EnableTxRx(UART_Type *uart){
	uart-> C2 = (UART_C2_TE_MASK | UART_C2_RE_MASK | UART_C2_RIE_MASK);
}

/**
 * @brief Set de Baud Rate
 * @param uart UART pointer
 * @param baudrate config.baudrate
*/
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

/**
 * @brief Set communication with or without parity
 * @param uartX_ptr UART peripheral base pointer
 * @param want_parity True to enable parity, false to exclude parity bit
 * @param parity_type If parity enabled, set EVEN if true or ODD if false. Ignored if parity disabled
 */
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

/**
 * @brief Set amount of data bits
 * @param uartX_ptr UART peripheral base pointer
 * @param data_9bits True to use 9 data bits, false to use 8 data bits
 */
void UART_SetDataSize(UART_Type * uartX_ptr, bool data_9bits){
	if(data_9bits){
		uartX_ptr->C1 |= UART_C1_M_MASK;
	}
	else{
		uartX_ptr->C1 &= !UART_C1_M_MASK;
	}
}

/**
 * @brief Set amount of stop bits
 * @param uartX_ptr UART peripheral base pointer
 * @param double_stop_bit True to use 2 stop bits, false to use 1 stop bit
 */
void UART_SetStopBit(UART_Type * uartX_ptr, bool double_stop_bit){
	if(double_stop_bit){
		uartX_ptr->BDH |= UART_BDH_SBNS_MASK;
	}
	else{
		uartX_ptr->BDH &= !UART_BDH_SBNS_MASK;
	}
}


/**
 * @brief Set the interruption
 * @param id UART to enable interruption
 */
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






/*******************************************************************************
*                                                                              *
*                               UART BLOQUEANTE                                *
*                                                                              *
 ******************************************************************************/
void uartInitBloqueante (uint8_t id, uart_cfg_t config)
{
	SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;

	PORTD->PCR[UART2_RX_PIN] = (uint32_t)0;
	PORTD->PCR[UART2_RX_PIN] = PORT_PCR_MUX(0b11); //ALT3
	PORTD->PCR[UART2_TX_PIN] = (uint32_t)0;
	PORTD->PCR[UART2_TX_PIN] = PORT_PCR_MUX(0b11); //ALT3

	UART2->C1 |= UART_C1_PE(1) | UART_C1_PT(0); // activo paridad par
	UART2->S2 |= UART_S2_MSBF(0); //  0 = LSB first ; 1 = MSB first

	UART2->C5 &= ~UART_C5_TDMAS_MASK;
	UART2->C2 = UART_C2_TE_MASK;

	UART_SetBaudRate(UART2, 9600);

}


void UART_Send_Data(unsigned char tx_data, uint8_t id)
{
	 while(((UART2->S1) & UART_S1_TDRE_MASK) == 0); //Puedo Transmitir ? !!bloqueante!!
	 UART2->D = tx_data; // Transmito
}



/*
unsigned char UART_Recieve_Data(void)
{
	while(((UART0->S1)& UART_S1_RDRF_MASK) ==0); // Espero recibir un caracter
	return(UART0->D); //Devuelvo el caracter recibido
}*/
