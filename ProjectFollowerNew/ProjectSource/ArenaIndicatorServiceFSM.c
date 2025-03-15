/****************************************************************************
 Module
   ArenaIndicatorServiceFSM.c

 Revision
   1.0.1

 Description
   This is a template file for implementing flat state machines under the
   Gen2 Events and Services Framework.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 01/15/12 11:12 jec      revisions for Gen2 framework
 11/07/11 11:26 jec      made the queue static
 10/30/11 17:59 jec      fixed references to CurrentEvent in RunTemplateSM()
 10/23/11 18:20 jec      began conversion from SMTemplate.c (02/20/07 rev)
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include <proc/p32mx170f256b.h>

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "TemplateFSM.h"
//#include "CalibrationSM.h"
//#include "TopGameHSM.h"
#include "dbprintf.h"
#include "SPIFollowerService.h"
#include "ArenaIndicatorServiceFSM.h"

/*----------------------------- Module Defines ----------------------------*/
/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine.They should be functions
   relevant to the behavior of this state machine
*/

/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well.
// type of state variable should match htat of enum in header file
//static TemplateState_t CurrentState;

// with the introduction of Gen2, we need a module level Priority var as well
static uint8_t MyPriority;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitArenaIndicatorServiceFSM

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, sets up the initial transition and does any
     other required initialization for this state machine
 Notes

 Author
     J. Edward Carryer, 10/23/11, 18:55
****************************************************************************/
bool InitArenaIndicatorServiceFSM(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;
  DB_printf("Starting Arena Indicator Service FSM");
  //Set the output PWM pins
  TRISBbits.TRISB9 = 0; // Set RB12 as output pin
  //ANSELBbits.ANSB9 = 0; // Disable analog on RB12
  
  //Setup PWM
  OC3CONbits.ON = 0; // Turn Off OC3 while performing the setup
  OC3CONbits.SIDL = 0; //Continue module operation when the device enters the idle mode
  OC3CONbits.OC32 = 0; // 16-bit compare mode bit
  OC3CONbits.OCTSEL = 0; //Timer3 is the clock source for output compare module
  OC3CONbits.OCM = 0b110; // PWM mode on OC3, Fault Pin disabled
  RPB9R = 0b0101; // Mapping OC3 to port RPB9
  T2CONbits.ON = 0; // Disable Timer 3
  T2CONbits.TCS = 0; // Select Internal Peripheral Clock
  T2CONbits.TCKPS = 0b011; // Set the timer input clock pre-scale of 1:8                                     
  PR3 = 49999;
  OC3RS = 0; // Set initial duty cycle to zero
  OC3R = 0; // Write to OC3R initial duty cycle of zero
  T2CONbits.ON = 1; //Enable Timer 3
  OC3CONbits.ON = 1; //Enable OC3
  
  // Set LED indicator pin
  TRISBbits.TRISB12 = 0; // set as output
  ANSELBbits.ANSB12 = 0; // set as digital
          
  // put us into the Initial PseudoState
  //CurrentState = InitPState;
  // post the initial transition event
  ThisEvent.EventType = ES_INIT;
  if (ES_PostToService(MyPriority, ThisEvent) == true)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/****************************************************************************
 Function
     PostArenaIndicatorServiceFSM

 Parameters
     EF_Event_t ThisEvent , the event to post to the queue

 Returns
     boolean False if the Enqueue operation failed, True otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:25
****************************************************************************/
bool PostArenaIndicatorServiceFSM(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunArenaIndicatorServiceFSM

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event_t, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here
 Notes
   uses nested switch/case to implement the machine.
 Author
   J. Edward Carryer, 01/15/12, 15:23
****************************************************************************/
ES_Event_t RunArenaIndicatorServiceFSM(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
  
  switch (ThisEvent.EventType){
      case GreenFieldDetected:
          OC3RS = 2500;
          break;
      case BlueFieldDetected:
          OC3RS = 5500;
          break;
      case GoNeutral:
          OC3RS = 4000;
          break;
      case LED_ON:
          LATBbits.LATB12 = 1;
          break;
      case LED_OFF:
          LATBbits.LATB12 = 0;
          break;
      default:
          break;
  }                                // end switch on Current State
  return ReturnEvent;
}

/****************************************************************************
 Function
     QueryTemplateSM

 Parameters
     None

 Returns
     TemplateState_t The current state of the Template state machine

 Description
     returns the current state of the Template state machine
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:21
****************************************************************************/
/*TemplateState_t QueryArenaIndicatorServiceFSM(void)
{
  return CurrentState;
}*/

/***************************************************************************
 private functions
 ***************************************************************************/