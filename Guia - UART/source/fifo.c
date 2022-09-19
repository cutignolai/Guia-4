/***************************************************************************//**
  @file     fifo.c
  @brief    Software circular FIFO implementation
  @author   Olivia De Vincenti
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <stdio.h>
#include "fifo.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

typedef struct {
	fifo_value_t    queue[MAX_FIFO_SIZE];
	size_t          head;
	size_t          tail;
    bool            is_buffer_full;
} fifo_t;


/*******************************************************************************
 * VARIABLES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/
void init_fifo(fifo_id_t id);

/*******************************************************************************
 * ROM CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

static fifo_t FIFO_Array[FIFO_MAX_N];   // FIFO Array
static fifo_id_t fifo_n = 0;            // Amount of active FIFO queues

/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

fifo_id_t FIFO_GetId(void)
{
#ifdef FIFO_DEVELOPMENT_MODE
    if (fifo_n >= FIFO_MAX_N)
    {
        return FIFO_INVALID_ID;
    }
    else
#endif // FIFO_DEVELOPMENT_MODE
    {
        init_fifo(fifo_n);
#ifdef FIFO_VERBOSE
        printf("fifo verbose - Fifo n° %u in use\n", fifo_n);
#endif
        return fifo_n++;
    }
}

bool FIFO_IsBufferFull(fifo_id_t id){

    return FIFO_Array[id].is_buffer_full;
}

bool FIFO_IsBufferEmpty(fifo_id_t id){

    return FIFO_Array[id].head == FIFO_Array[id].tail && !FIFO_Array[id].is_buffer_full;
}

size_t FIFO_WriteToBuffer(fifo_id_t id, fifo_value_t* data, size_t data_size){
    
    size_t i;

    for (i = 0; i < data_size && i < MAX_FIFO_SIZE; i++){

        if (FIFO_PushToBuffer(id, *(data + i)) == FIFO_BUFFER_FULL){      // Push value to buffer, if buffer is full:
            break;              // Stop writing
        }
    }

#ifdef FIFO_VERBOSE
    printf("fifo verbose - Wrote %u values to FIFO %u: %s\n", i, id, FIFO_Array[id].queue + FIFO_Array[id].head - i);
#endif

    return i;
}

size_t FIFO_ReadFromBuffer(fifo_id_t id, fifo_value_t* data_ptr, size_t data_size){

    size_t i = 0;
    bool b = FIFO_IsBufferEmpty(id);        // Check if buffer is empty

    // Until the amount of values pulled is data_size or buffer is empty
    for (i = 0; b != FIFO_BUFFER_EMPTY && i < data_size && i < MAX_FIFO_SIZE; i++){      
        b = FIFO_PullFromBuffer(id, data_ptr + i) == FIFO_BUFFER_EMPTY;      // Pull value from buffer
    }

#ifdef FIFO_VERBOSE
    printf("fifo verbose - Read %u values from FIFO %u: %.*s\n", i, id, i, FIFO_Array[id].queue + FIFO_Array[id].tail - i);
#endif

    return i;           // Amount of values pulled
}

size_t FIFO_ReadAll(fifo_id_t id, fifo_value_t* data_ptr){

    size_t i = 0;
    bool b = FIFO_IsBufferEmpty(id);        // Check if buffer is empty

    for (i = 0; b != FIFO_BUFFER_EMPTY && i < MAX_FIFO_SIZE; i++){           // Until buffer is empty
        b = FIFO_PullFromBuffer(id, data_ptr + i) == FIFO_BUFFER_EMPTY;      // Pull value from buffer
    }

#ifdef FIFO_VERBOSE
    printf("fifo verbose - Read %u values from FIFO %u: %.*s\n", i, id, i, FIFO_Array[id].queue + FIFO_Array[id].tail - i);
#endif

    return i;           // Amount of values pulled
}

bool FIFO_PushToBuffer(fifo_id_t id, fifo_value_t data){

    if (!FIFO_Array[id].is_buffer_full){                        // If buffer is not full

        *((fifo_value_t*)(&FIFO_Array[id].queue[0] + FIFO_Array[id].head)) = data;   // Write data
#ifdef FIFO_VERBOSE
        printf("fifo verbose - Pushed to FIFO %u: %u\n", id, data);
#endif

        if (FIFO_Array[id].head + 1 < MAX_FIFO_SIZE){           // Overflow?
            FIFO_Array[id].head++;                              // Advance head
        } else {
            FIFO_Array[id].head = 0;                            // Reset head
        }

        if (FIFO_Array[id].head == FIFO_Array[id].tail ){       // If buffer is now full
            FIFO_Array[id].is_buffer_full = FIFO_BUFFER_FULL;   // Set flag
#ifdef FIFO_VERBOSE
            printf("fifo verbose - FIFO %u buffer full\n", id);
#endif
        }
    }

    return FIFO_Array[id].is_buffer_full;       // Return buffer state
}

bool FIFO_PullFromBuffer(fifo_id_t id, fifo_value_t* data_ptr){

    bool r = !FIFO_BUFFER_EMPTY;

    if (FIFO_Array[id].head == FIFO_Array[id].tail && !FIFO_Array[id].is_buffer_full){    // If buffer is empty
#ifdef FIFO_VERBOSE
        printf("fifo verbose - FIFO %u buffer empty\n", id);
#endif
        r = FIFO_BUFFER_EMPTY;       // Return buffer empty state and don't write in data_ptr*
    
    } else {                            // If buffer is not empty

        *(data_ptr) = *((fifo_value_t*) (&FIFO_Array[id].queue[0] + FIFO_Array[id].tail));      // Write tail value to data_ptr
#ifdef FIFO_VERBOSE
        printf("fifo verbose - Pulled from FIFO %u: %u\n", id, *(data_ptr));
#endif

        if (FIFO_Array[id].tail + 1 < MAX_FIFO_SIZE){           // Overflow?
            FIFO_Array[id].tail++;                              // Advance tail
        } else {
            FIFO_Array[id].tail = 0;                            // Reset tail
        }

        FIFO_Array[id].is_buffer_full = !FIFO_BUFFER_FULL;      // Buffer is now not full

        if (FIFO_Array[id].tail == FIFO_Array[id].head){ 
            r = FIFO_BUFFER_EMPTY;                              // Return buffer now empty state
#ifdef FIFO_VERBOSE
            printf("fifo verbose - FIFO %u buffer empty\n", id);
#endif
        }    

    }

    return r;
}

size_t FIFO_GetBufferLength(fifo_id_t id){
    size_t r;
    if (FIFO_Array[id].is_buffer_full){
        r = MAX_FIFO_SIZE;
    } else {
        r = FIFO_Array[id].head - FIFO_Array[id].tail;
    }
#ifdef FIFO_VERBOSE
    printf("fifo verbose - FIFO %u buffer length: %u\n", r);
#endif
    return r;
}

void FIFO_Reset(fifo_id_t id){
    init_fifo(id);
#ifdef FIFO_VERBOSE
    printf("fifo verbose - Reset FIFO %u\n", id);
#endif
}

void FIFO_ClearBuffer(fifo_id_t id){
    uint8_t i;
    for (i = 0; i < MAX_FIFO_SIZE - 1; i++){
        FIFO_Array[id].queue[i] = 0;
    }
#ifdef FIFO_VERBOSE
    printf("fifo verbose - Clear FIFO %u\n", id);
#endif
    FIFO_Reset(id);
}

void FIFO_FreeId(fifo_id_t id){
    if (fifo_n > 0){
#ifdef FIFO_VERBOSE
    printf("fifo verbose - Free FIFO %u\n", id);
#endif
        fifo_n--;
    }
}

/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

void init_fifo(fifo_id_t id){
    FIFO_Array[id].head = 0;
    FIFO_Array[id].tail = 0;
    FIFO_Array[id].is_buffer_full = false;
}

/******************************************************************************/