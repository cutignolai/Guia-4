/***************************************************************************//**
  @file     fifo.h
  @brief    Software FIFO queue implementation
  @author   Olivia De Vincenti
 ******************************************************************************/

#ifndef _FIFO_H_
#define _FIFO_H_

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>


/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

// #define FIFO_VERBOSE				// Uncomment for logs
#define FIFO_DEVELOPMENT_MODE

#define MAX_FIFO_SIZE   256         // Maximum FIFO buffer size in bytes
#define FIFO_MAX_N      16          // Maximum amount of FIFO queues

#define FIFO_INVALID_ID     255     // ID returned when FIFO Array is full

#define FIFO_BUFFER_FULL     true
#define FIFO_BUFFER_EMPTY    true

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

// FIFO ID
typedef uint8_t fifo_id_t;

// FIFO VALUE TYPE
typedef uint8_t fifo_value_t;

/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/**
 * @brief Inititates a FIFO Queue
 * 
 * @return fifo_id_t ID to access FIFO
 */
fifo_id_t FIFO_GetId(void);

/**
 * @brief Returns whether FIFO Buffer is full or not
 * 
 * @param id: FIFO ID
 * @return true: Buffer is full | false: Buffer is not full
 */
bool FIFO_IsBufferFull(fifo_id_t id);

/**
 * @brief Returns whether FIFO Buffer is empty or not
 * 
 * @param id: FIFO ID
 * @return true: Buffer is empty | false: Buffer is not empty
 */
bool FIFO_IsBufferEmpty(fifo_id_t id);

/**
 * @brief Write array of values to FIFO buffer
 * 
 * @param id: FIFO ID
 * @param data_ptr: Pointer to array of data to write
 * @param data_size: Amount of values to write
 * @return Amount of values that values were written into the buffer, if it is lower than data_size then the buffer is full
 */
size_t FIFO_WriteToBuffer(fifo_id_t id, fifo_value_t* data_ptr, size_t data_size);

/**
 * @brief Copy values from buffer into an external array
 * 
 * @param id: FIFO ID
 * @param data_ptr: Pointer to external array values will be copied to
 * @param data_size: Amount of values to read and copy
 * @return Amount of values that were read and copied into the external array, if it is lower than data_size then the buffer is empty
 * */
size_t FIFO_ReadFromBuffer(fifo_id_t id, fifo_value_t* data_ptr, size_t data_size);

/**
 * @brief Copy all remaining values from buffer into an external array. 
 * EXTERNAL ARRAY MUST BE AT LEAST THE SIZE OF THE REMAINING BUFFER, check with FIFO_GetBufferLength()
 * 
 * @param id: FIFO ID
 * @param data_ptr: Pointer to external array values will be copied to
 * @return Amount of values that were read and copied into the external array 
 */
size_t FIFO_ReadAll(fifo_id_t id, fifo_value_t* data_ptr);

/**
 * @brief Add 1 value to FIFO buffer
 * 
 * @param id: FIFO ID
 * @param data: Value to add
 * @return false: Value pushed correctly | true: Buffer full, can't push value
 */
bool FIFO_PushToBuffer(fifo_id_t id, fifo_value_t data);

/**
 * @brief Remove 1 value from FIFO buffer and copy into external adress
 * 
 * @param id: FIFO ID
 * @param data_ptr: Pointer to external variable where the value will be copied
 * @return false: Value pulled correctly | true: Buffer empty, can't pull value
 */
bool FIFO_PullFromBuffer(fifo_id_t id, fifo_value_t* data_ptr);

/**
 * @brief Get amount of taken spaces in FIFO buffer
 * 
 * @param id FIFO ID
 * @return size_t amount of taken spaces in selected FIFO buffer
 */
size_t FIFO_GetBufferLength(fifo_id_t id);

/**
 * @brief Reset a FIFO
 * 
 * @param id FIFO ID
 */
void FIFO_Reset(fifo_id_t id);

/**
 * @brief Reset a FIFO and clear all its values to cero(0)
 * 
 * @param id FIFO ID
 */
void FIFO_ClearBuffer(fifo_id_t id);

/**
 * @brief Stop using FIFO, it is not taken anymore
 * 
 * @param id FIFO ID
 */
void FIFO_FreeId(fifo_id_t id);

/*******************************************************************************
 ******************************************************************************/

#endif // _FIFO_H_