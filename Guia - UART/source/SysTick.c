/***************************************************************************//**
  @file     SysTick.c
  @brief    SysTick driver
  @author   Ignacio Cutignola
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "SysTick.h"
#include "MK64F12.h"
#include "hardware.h"
#include "board.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/
#define CLK_FREQ 	        __CORE_CLOCK__			//100MHz

#define SYSTICK_TEST_MODE   1
#define TEST_PIN            DIO_19


// Status Registrer
#define CSR_MASK            0x00010007
#define ENABLE_CSR          0x01
#define TICKINT_CSR         0x02
#define CLKSOURCE_CSR       0x04

// Reload Value Registrer
#define LOAD_RVR            (CLK_FREQ/SYSTICK_ISR_FREQUENCY_HZ - 1)
#define RVR_MASK            0x00FFFFFF

// Current Value Registrer
#define CLEAR_CVR           0x0
#define CVR_MASK            0x00FFFFFF

/*******************************************************************************
 * VARIABLES
 ******************************************************************************/
void (*callback_ptr)(void);

/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/
__ISR__ SysTick_Handler(void);


/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/


bool SysTick_Init (void (*callback)(void))
{

    SysTick->CTRL = 0x00; 												

    // Reload Value
    SysTick->LOAD = RVR_MASK & LOAD_RVR;

    // Current value
    SysTick->VAL=   CVR_MASK & CLEAR_CVR;  

    //Enable SysTick interrupt
    SysTick->CTRL = CSR_MASK & (ENABLE_CSR | TICKINT_CSR | CLKSOURCE_CSR);	

    callback_ptr = callback;

    #ifdef SYSTICK_TEST_MODE
        gpioMode(TEST_PIN, OUTPUT);
    #endif
    

    return true;

}

__ISR__ SysTick_Handler(void)
{	
    #ifdef SYSTICK_TEST_MODE
        gpioWrite(TEST_PIN, HIGH);
    #endif

    (*callback_ptr)();

    #ifdef SYSTICK_TEST_MODE
        gpioWrite(TEST_PIN, LOW);
    #endif
}
