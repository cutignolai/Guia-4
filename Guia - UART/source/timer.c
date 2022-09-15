/***************************************************************************//**
  @file     timer.c
  @brief    Timer driver. Advance implementation
  @author   Nicolás Magliola y Olivia De Vincenti
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "timer.h"

#include "SysTick.h"


/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#if TIMER_TICK_MS != (1000U/SYSTICK_ISR_FREQUENCY_HZ)
#error Las frecuencias no coinciden!!
#endif // TIMER_TICK_MS != (1000U/SYSTICK_ISR_FREQUENCY_HZ)

#define TIMER_DEVELOPMENT_MODE    1

#define TIMER_ID_INTERNAL   0


/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

typedef struct {
	ttick_t             period;
	ttick_t             cnt;
    tim_callback_t      callback;
    uint8_t             mode        : 1;
    uint8_t             running     : 1;
    uint8_t             expired     : 1;
    uint8_t             unused      : 5;
} timer_t;


/*******************************************************************************
 * VARIABLES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/

/**
 * @brief Periodic service
 */
void timer_isr(void);


/*******************************************************************************
 * ROM CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

timer_t timers[TIMERS_MAX_CANT];
tim_id_t timers_cant = TIMER_ID_INTERNAL+1;
static bool yaInit = false;


/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

void timerInit(void)
{
    if (yaInit)
        return;
    
    SysTick_Init(timer_isr); // init peripheral
    
    yaInit = true;
}


tim_id_t timerGetId(void)
{
#ifdef TIMER_DEVELOPMENT_MODE
    if (timers_cant >= TIMERS_MAX_CANT)
    {
        return TIMER_INVALID_ID;
    }
    else
#endif // TIMER_DEVELOPMENT_MODE
    {
        return timers_cant++;
    }
}

// Crea un timer apagado
void timerCreate(tim_id_t id, ttick_t ticks, uint8_t mode, tim_callback_t callback){
    timerStart(id, ticks, mode, callback);
    timerReset(id);
}

// Crea un timer encendido
void timerStart(tim_id_t id, ttick_t ticks, uint8_t mode, tim_callback_t callback)
{
#ifdef TIMER_DEVELOPMENT_MODE
    if ((id < timers_cant) && (mode < CANT_TIM_MODES))
#endif // TIMER_DEVELOPMENT_MODE
    {
        timers[id].running = 1;
        timers[id].expired = 0;
        timers[id].cnt = ticks;
        timers[id].mode = mode;
        timers[id].period = ticks;
        timers[id].callback = callback;
    }
}


// Apaga el timer pero no lo reinicia
void timerStop(tim_id_t id){
    timers[id].running = 0;
    timers[id].expired = 0;
}

// Resume el timer
void timerPlay(tim_id_t id){
    timers[id].running = 1;
}

// Reinicia el timer
void timerRestart(tim_id_t id){
    timers[id].running = 0;
    timers[id].cnt = timers[id].period;
    timers[id].expired = 0;
    timers[id].running = 1;
}

// Expira el timer y lo hace llegar a cero
void timerFinish(tim_id_t id){
    timers[id].running = 0;
    timers[id].expired = 1;
    timers[id].cnt = 0;
    timers[id].callback();
}

// Resetea un timer apagado
void timerReset(tim_id_t id){
    timers[id].running = 0;
    timers[id].expired = 0;
    timers[id].cnt = timers[id].period;
}

// Activa un timer que estaba apagado
void timerActivate(tim_id_t id){
    timers[id].cnt = timers[id].period;
    timers[id].expired = 0;
    timers[id].running = 1;
}

// Modifica el tiempo de un timer y lo reinicia
void timerChangePeriod(tim_id_t id, ttick_t ticks){
    // printf("ticks anteriores: %i\n", timers[id].cnt);
    // printf("período anterior: %i\n", timers[id].period);
    timers[id].running = 0;
    timers[id].period = ticks;
    timers[id].cnt = ticks;
    timers[id].expired = 0;
    timers[id].running = 1;
    // printf("ticks nuevos: %i\n", timers[id].cnt);
    // printf("período nuevo: %i\n", timers[id].period);
}

// Modifica la callback de un timer y lo reinicia
void timerChangeCallback(tim_id_t id, tim_callback_t callback){
    timers[id].running = 0;
    timers[id].callback = callback;
    timers[id].expired = 0;
    timers[id].cnt = timers[id].period;
    timers[id].running = 1;
}

// Indica si expiró o no el timer
bool timerExpired(tim_id_t id){
    bool is_expired = timers[id].expired;
    timers[id].expired = 0;
    return is_expired;
}


void timerDelay(ttick_t ticks){
    timerStart(TIMER_ID_INTERNAL, ticks, TIM_MODE_SINGLESHOT, NULL);
    while (!timerExpired(TIMER_ID_INTERNAL)){
        // wait...
    }
}


/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

// Recorre el arreglo de timers y busca si expiró alguno
void timer_isr(void){
    tim_id_t id_idx;
    for (id_idx = timers_cant; id_idx; id_idx--){
        if (timers[id_idx].running && timers[id_idx].cnt){
            timers[id_idx].cnt--;
            if (!timers[id_idx].cnt){
                if (timers[id_idx].mode == TIM_MODE_SINGLESHOT){
                    timers[id_idx].expired = 1;
                    timers[id_idx].running = 0;
                } else {
                	timers[id_idx].cnt = timers[id_idx].period;
                }
                if (timers[id_idx].callback != NULL){
                    timers[id_idx].callback();
                }
            }
        }
    }
}

/******************************************************************************/
