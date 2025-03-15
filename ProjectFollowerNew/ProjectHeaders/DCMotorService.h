/****************************************************************************
  //Header file for StepService

  based on the Gen 2 Events and Services Framework
********************************************************************/

#ifndef DCMotorService_H

#define DCMotorService_H

#include <stdint.h>
#include <stdbool.h>
#include "ES_Events.h"

#include "ES_Port.h"            	// needed for definition of REENTRANT

// Declare the global variable
//extern uint32_t ScaledInterval;

// Public Function Prototypes

bool InitDCMotorService(uint8_t Priority);
bool PostDCMotorService(ES_Event_t ThisEvent);
ES_Event_t RunDCMotorService(ES_Event_t ThisEvent);
void HandleTimeout(void);
uint32_t GetDesiredSpeed(void);
//void GetPeriod(uint32_t current_period);
//void UpdateLEDs(uint32_t current_period, uint32_t current_pulse_count);
//void InitTimer4Interrupt(void);

typedef enum{
    waiting,
            turning,
            scanningBeacon,
            scanningLine,
            end,
            adjust,
            adjust_ccw,
            turning_ccw
}Motor_State_t;

#endif /* DCMotorService_H */