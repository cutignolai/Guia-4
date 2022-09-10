/***************************************************************************//**
  @file     UART.c
  @brief    UART Driver for K64F. Non-Blocking and using FIFO feature
  @author   Oli, Cuty y Micho
 ******************************************************************************/

#include "MK64F12.h"
#include "uart.h"
#include "gpio.h"
#include "board.h"
#include <stdbool.h>

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/


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
 /**
 * @brief Set de Baud Rate
 * @param uart UART pointer
 * @param baudrate config.baudrate
 * @return Nothing
*/
void UART_SetBaudRate (UART_Type *uart, uint32_t baudrate);


/**
 * @brief Set Parity
 * @param uart UART pointer
 * @param parity parity mode
 * @return Nothing
*/
void UART_SetParity(UART_Type * uartX_ptr, bool want_parity, bool parity);

/**
 * @brief Set data size
 * @param uart UART pointer
 * @param data_9bits set de 9th bit 
 * @return Nothing
*/
void UART_SetDataSize(UART_Type * uartX_ptr, bool data_9bits);


 

static UART_Type* const UART_ptrs[] = UART_BASE_PTRS;		// { UART0, UART1, UART2, UART3, UART4, UART5 } (Ver MK64F12.h)
static PORT_Type * const addr_arrays[] = {PORTA, PORTC, PORTD, PORTC, PORTE};

static const TX_PINS[] = {UART0_TX, UART1_TX, UART2_TX, UART3_TX, UART4_TX}
static const RX_PINS[] = {UART0_RX, UART1_RX, UART2_RX, UART3_RX, UART4_RX}
static bool uart_is_used [UART_CANT_IDS] = {false, false, false, false, false};

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


	/* 
	*	CLOCK GATING
	*
	*	Antes de usar un modulo se debe habilitar el clock de locontrario se genera un error.
	*	De igual forma antes de apagarun clock se deberá deshabilitar el modulo.
	*/
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

	
	// PIN SETUP
	UART_Type* uartX_ptr =	UART_CANT_IDS[id];
	PORT_Type* port_ptr = addr_arrays[id];
	static const UARTX_TX_PIN = TX_PINS[id];
	static const UARTX_RX_PIN = RX_PINS[id];

	// TODO: Cambiar el MUX y IRQC
	// TX:
	port_ptr->PCR[UARTX_TX_PIN]=0x0; //Clear all bits
	port_ptr->PCR[UARTX_TX_PIN]|=PORT_PCR_MUX(PORT_mAlt3); //Set MUX to UART0
	port_ptr->PCR[UARTX_TX_PIN]|=PORT_PCR_IRQC(PORT_eDisabled); //Disable Port interrupts
	
	// RX:
	port_ptr->PCR[UARTX_RX_PIN]=0x0; //Clear all bits
	port_ptr->PCR[UARTX_RX_PIN]|=PORT_PCR_MUX(PORT_mAlt3); //Set MUX to UART0
	port_ptr->PCR[UARTX_RX_PIN]|=PORT_PCR_IRQC(PORT_eDisabled); //Disable Port interrupts

	//52.8.3 Initialization sequence (non ISO-7816)  -----> REFERENCE MANUAL
	// C1: DATA
	// C2: ENABLE
	// C4: BAUDRATE

	/*
	 * Deshabilito la UART para configurar sin problemas
	 * FIXME: --> Si no anda, hay que deshabilidar las interrupciones tambien
	*/
	uartX_ptr->C2 &= !( UART_C2_TE_MASK | UART_C2_RE_MASK);
	/*
	 * Limpio C1 completo, me olvido de la paridad
	*/
	uart->C1 = 0x0;

	//
	UART_SetBaudRate (uartX_ptr, config.baudrate);			//CONFIGURO EL BAUD RATE

	//
	UART_SetParity(uartX_ptr, config.want_parity, config.parity_type);

	//
	UART_SetDataSize(uartX_ptr, config.data_9bits);


	/*
	 * Lo ultimo que hago es habilitar el UART
	 * Activo el transmisor, el receptor y las interrupciones
	*/
	uartX_ptr->C2 = (UART_C2_TE_MASK | UART_C2_RE_MASK | UART_C2_RIE_MASK);


	/*
	switch(id) { // enable interrupts!
	case 0: NVIC_EnableIRQ(UART0_RX_TX_IRQn); break;
	case 1: NVIC_EnableIRQ(UART1_RX_TX_IRQn); break;
	case 2: NVIC_EnableIRQ(UART2_RX_TX_IRQn); break;
	case 3: NVIC_EnableIRQ(UART3_RX_TX_IRQn); break;
	case 4: NVIC_EnableIRQ(UART4_RX_TX_IRQn); break;
	default: break;
	}

	systick_init();
	systick_add_callback(uart_periodic, SYSTICK_HZ_TO_RELOAD(100), PERIODIC);
	uart_is_used[id] = true;
	
	
	*/



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

}

/**
 * @brief Read a received message. Non-Blocking
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
 * @brief Set de Baud Rate
 * @param uart UART pointer
 * @param baudrate config.baudrate
 * @return Nothing
*/
void UART_SetBaudRate (UART_Type *uart, uint32_t baudrate){
	uint16_t sbr, brfa;
	uint32_t clock;

	clock = ((uart == UART0) || (uart == UART1))?(__CORE_CLOCK__):(__CORE_CLOCK__ >> 1);
	
	baudrate = ((baudrate == 0)? (UART HAL DEFAULT BAUDRATE) :
	((baudrate > 0x1FFF)? (UART_HAL_DEFAULT_BAUDRATE): (baudrate)));

	sbr = clock (baudrate << 4);
	brfa = (clock << 1) / baudrate - (sbr << 5);
	uart->BDH = UART_BDH_SBR(sbr >> 8); 
	uart->BDL = UART_BDL_SBR (sbr); 
	uart->C4 = (uart->C4 & ~UART_C4_BRFA_MASK) | UART_C4_BRFA(brfa);
}


/** @brief 
 * @param uart UART pointer
 * @param baudrate config.baudrate
 * @return Nothing
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

void UART_SetDataSize(UART_Type * uartX_ptr, bool data_9bits){
	if(data_9bits){
		uartX_ptr->C1 |= UART_C1_M_MASK;
	}
	else{
		uartX_ptr->C1 &= !UART_C1_M_MASK;
	}
}

//UART2->S2 |= UART_S2_MSBF(0); //  0 = LSB first ; 1 = MSB first