/***************************************************************************//**
  @file     UART.c
  @brief    UART Driver for K64F. Non-Blocking and using FIFO feature
  @author   Nicolas Magliola
 ******************************************************************************/

#ifndef _UART_H_
#define _UART_H_

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "MK64F12.h"


/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#if defined(CPU_MK64FN1M0VLL12) || defined(CPU_MK64FX512VLL12)
	#define UART_CANT_IDS   5
#elif defined(CPU_MK64FN1M0CAJ12) || defined(CPU_MK64FN1M0VDC12) || defined(CPU_MK64FN1M0VLQ12) || defined(CPU_MK64FN1M0VMD12) || \
    defined(CPU_MK64FX512VDC12) || defined(CPU_MK64FX512VLQ12) || defined(CPU_MK64FX512VMD12)
	#define UART_CANT_IDS   6
#endif

#define PARITY_YES		true
#define PARITY_NO		!PARITY_YES

#define PARITY_EVEN		true
#define PARITY_ODD		!PARITY_EVEN

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

typedef struct {
    uint8_t baud_rate;   // Numeros de cambio de la senial por segundo
    bool non_blocking;
    bool want_parity;
	bool data_9bits;
    bool parity_type;     
    bool double_stop_bit;
    bool use_fifo;
    uint8_t bit_rate;    // Bits por segundo
    //etc
} uart_cfg_t;




/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/**
 * @brief Initialize UART driver
 * @param id UART's number
 * @param config UART's configuration (baudrate, parity, etc.)
*/
void uartInit (uint8_t id, uart_cfg_t config);

/**
 * @brief Check if a new byte was received
 * @param id UART's number
 * @return A new byte has being received
*/
uint8_t uartIsRxMsg(uint8_t id);

/**
 * @brief Check how many bytes were received
 * @param id UART's number
 * @return Quantity of received bytes
*/
uint8_t uartGetRxMsgLength(uint8_t id);

/**
 * @brief Read a received message. Non-Blocking
 * @param id UART's number
 * @param msg Buffer to paste the received bytes
 * @param cant Desired quantity of bytes to be pasted
 * @return Real quantity of pasted bytes
*/
uint8_t uartReadMsg(uint8_t id, char* msg, uint8_t cant);

/**
 * @brief Write a message to be transmitted. Non-Blocking
 * @param id UART's number
 * @param msg Buffer with the bytes to be transfered
 * @param cant Desired quantity of bytes to be transfered
 * @return Real quantity of bytes to be transfered
*/
uint8_t uartWriteMsg(uint8_t id, const char* msg, uint8_t cant);

/**
 * @brief Check if all bytes were transfered
 * @param id UART's number
 * @return All bytes were transfered
*/
uint8_t uartIsTxMsgComplete(uint8_t id);


/*******************************************************************************
*                                                                              *
*                               UART BLOQUEANTE                                *
*                                                                              *
 ******************************************************************************/
enum {
	UART2_RTS_b_PIN,
	UART2_CTS_b_PIN,
	UART2_RX_PIN,
	UART2_TX_PIN
};


void uartInitBloqueante (uint8_t id, uart_cfg_t config);


void UART_Send_Data(unsigned char tx_data, uint8_t id);



#endif // _UART_H_
