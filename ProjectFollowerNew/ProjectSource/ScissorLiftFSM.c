/****************************************************************************
 Module
   ScissorLiftFSM.c

 Revision
   1.0.1

 Description
   This is a template file for implementing a simple service under the
   Gen2 Events and Services Framework.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 01/16/12 09:58 jec      began conversion from TemplateFSM.c
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ScissorLiftFSM.h"
//#include "ADService.h"
//#include "ButtonDebounce.h"
#include "PIC32_SPI_HAL.h"
//#include "PIC32PortHAL.h"
#include "ES_Port.h"

#include "terminal.h"
#include <sys/attribs.h>

//#include "CalibrationSM.h"
//#include "TopGameHSM.h"
#include "dbprintf.h"
//#include "SPIService.h"
/*----------------------------- Module Defines ----------------------------*/


#define DRIVE_MODE MICRO_STEP
#define NUM_STEPS 200
#define PWM_FREQUENCY 1000
#define MAX_MOTOR_SPEED 1000
#define STEP_INTERVAL_MS 5 // 5 ms interval (200 steps per second)

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/
bool InitScissorLiftFSM(uint8_t Priority);
bool PostScissorLiftFSM(ES_Event_t ThisEvent);
ES_Event_t RunScissorLiftFSM(ES_Event_t ThisEvent);
void HandleTimeout1(void);
void SetOutputStates();

/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;
static ScissorLiftState_t CurrentState;
volatile static uint16_t PR_Value;

typedef struct {
    uint8_t RA2State;
    uint8_t RA3State;
    uint8_t RA0State;
    uint8_t RA1State;
} PinState_t;

typedef enum {
    FULL_STEP,
    WAVE_DRIVE,
    HALF_STEP,
    MICRO_STEP
} DriveMode_t;

// Create a static DriveMode_t variable for the current drive mode
static DriveMode_t currentDriveMode = DRIVE_MODE;

// Define static variables for direction and step tracking
typedef enum {
    FORWARD,
    BACKWARD
} currentDirection_t;


static currentDirection_t motorState = FORWARD;

static uint8_t CurrentStep = 0;
static uint32_t StepCount;
static uint8_t LiftLevel;


/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitScissorLiftFSM

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, and does any
     other required initialization for this service
 Notes

 Author
     S Gangopadhyay, 01/16/12, 10:00
****************************************************************************/
bool InitScissorLiftFSM(uint8_t Priority)
{
  ES_Event_t ThisEvent;
  bool ReturnVal = true;

  MyPriority = Priority;
  /********************************************
   in here you write your initialization code
   *******************************************/
     // Initialize the port lines
  //Set the PWM port lines as Output
  //TRISAbits.TRISA3 = 0;
  //TRISAbits.TRISA2 = 0;
  TRISAbits.TRISA1 = 0;
  TRISAbits.TRISA0 = 0;
  TRISAbits.TRISA4 = 0;
  //Disable the analog on PWM port lines
  //ANSELAbits.ANSA3 = 0;
  //ANSELAbits.ANSA2 = 0;
  ANSELAbits.ANSA1 = 0;
  ANSELAbits.ANSA0 = 0;
  //ANSELAbits.ANSA4 = 0;

  DB_printf( "Starting Scissor Lift FSM\r\n");

   // Start the timer for Scissor Lift Service
  ES_Timer_InitTimer(ScissorLiftTimer, STEP_INTERVAL_MS);
  // post the initial transition event
  ThisEvent.EventType = ES_INIT;
  if (ES_PostToService(MyPriority, ThisEvent) == true)
  {
    ReturnVal = true;
    return ReturnVal;
  }
  else
  {
    ReturnVal = false;
    return ReturnVal;

  }
}

/****************************************************************************
 Function
     PostScissorLiftFSM

 Parameters
     EF_Event_t ThisEvent ,the event to post to the queue

 Returns
     bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
     S Gangopadhyay, 10/23/11, 19:25
****************************************************************************/
bool PostScissorLiftFSM(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunScissorLiftFSM

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here
 Notes

 Author
   S Gangopadhyay, 01/15/12, 15:23
****************************************************************************/
ES_Event_t RunScissorLiftFSM(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
  /********************************************
   in here you write your service code
   *******************************************/
 switch (CurrentState)
  {
    case InitPState:        // If current state is initial Psedudo State
    {
      if (ThisEvent.EventType == ES_INIT)    // only respond to ES_Init
      {
        // this is where you would put any actions associated with the
        // transition from the initial pseudo-state into the actual
        // initial state

        // now put the machine into the actual initial state
        CurrentState = Waiting_To_Move;
      }
    }
    break;

    case Waiting_To_Move:        // If current state is state one
    {
        PORTAbits.RA4 = 0;
      switch (ThisEvent.EventType)
      {
        case LIFT_GROUND:  //If event is event one
           // Execute action function for state one : event one
           motorState = FORWARD;
            DB_printf( "Lift will go to ground level\r\n");
          CurrentState = Moving_To_Ground;  //Decide what the next state will be
          ES_Timer_InitTimer(ScissorLiftTimer, STEP_INTERVAL_MS); // Restart timer
        break;
        case LIFT_STACK_BOTTOM:
            motorState = BACKWARD;
            DB_printf( "Lift going to bottom stack\r\n");
            StepCount = 0;
            CurrentState = Moving_To_Bottom_Stack;//Decide what the next state will be
            ES_Timer_InitTimer(ScissorLiftTimer, STEP_INTERVAL_MS); // Restart timer
            break;

        case LIFT_STACK_MIDDLE:
            motorState = BACKWARD;
            DB_printf( "Lift going to middle stack\r\n");
            StepCount = 0;
            CurrentState = Moving_To_Middle_Stack;//Decide what the next state will be
            ES_Timer_InitTimer(ScissorLiftTimer, STEP_INTERVAL_MS); // Restart timer
            break;

        case LIFT_STACK_TOP:
            motorState = BACKWARD;
            DB_printf( "Lift going to top stack\r\n");
            StepCount = 0;
            CurrentState = Moving_To_Top_Stack;//Decide what the next state will be
            ES_Timer_InitTimer(ScissorLiftTimer, STEP_INTERVAL_MS); // Restart timer
            break;
          case ES_NEW_KEY:
              if (ThisEvent.EventParam == 'f'){
                  // Execute action function for state one : event one
                  motorState = FORWARD;
                  DB_printf( "Lift will go forward\r\n");
                  CurrentState = Going_Forward;  //Decide what the next state will be
                  ES_Timer_InitTimer(ScissorLiftTimer, STEP_INTERVAL_MS); // Restart timer
              }
              else if (ThisEvent.EventParam == 'b'){
                  motorState = BACKWARD;
                  DB_printf( "Lift will go backward\r\n");
                  CurrentState = Going_Backward;  //Decide what the next state will be
                  ES_Timer_InitTimer(ScissorLiftTimer, STEP_INTERVAL_MS); // Restart timer
              }
              else if (ThisEvent.EventParam == 'l'){
                  motorState = BACKWARD;
                  DB_printf( "Lift going to bottom stack\r\n");
                  StepCount = 0;
                  CurrentState = Moving_To_Bottom_Stack;  //Decide what the next state will be
                  ES_Timer_InitTimer(ScissorLiftTimer, STEP_INTERVAL_MS); // Restart timer
              }
              else if (ThisEvent.EventParam == 'm'){
                  motorState = BACKWARD;
                  DB_printf( "Lift going to middle stack\r\n");
                  StepCount = 0;
                  CurrentState = Moving_To_Middle_Stack;  //Decide what the next state will be
                  ES_Timer_InitTimer(ScissorLiftTimer, STEP_INTERVAL_MS); // Restart timer
              }
              else if (ThisEvent.EventParam == 't'){
                  motorState = BACKWARD;
                  DB_printf( "Lift going to top stack\r\n");
                  StepCount = 0;
                  CurrentState = Moving_To_Top_Stack;  //Decide what the next state will be
                  ES_Timer_InitTimer(ScissorLiftTimer, STEP_INTERVAL_MS); // Restart timer
              }
              break;

        // repeat cases as required for relevant events
        default:
          ;
      }  // end switch on CurrentEvent
    }
    break;
    // repeat state pattern as required for other states
    case Going_Forward:
        switch (ThisEvent.EventType){
        case ES_TIMEOUT:
            if (ThisEvent.EventParam == ScissorLiftTimer) {
                HandleTimeout1();
                StepCount++;
            }
            break;
        case GROUND_LIMIT_HIT:
            DB_printf( "Lift has reached ground level\r\n");
            DB_printf("%d steps taken to reach ground/r/n", StepCount);
            StepCount = 0;
            LiftLevel = 0;
            motorState = BACKWARD;
            ES_Event_t NewEvent;
            //NewEvent.EventType = GROUND_REACHED;
            //NewEvent.EventParam = LiftLevel;
            //PostTopGameHSM(NewEvent);
            CurrentState = Waiting_To_Move;  //Decide what the next state will be
            break;
        default:
            break;
        }
    break;
    case Going_Backward:
        switch (ThisEvent.EventType){
        case ES_TIMEOUT:
            if (ThisEvent.EventParam == ScissorLiftTimer) {
                HandleTimeout1();
                StepCount++;
            }
            break;
        case TOP_LIMIT_HIT:
            DB_printf( "Lift has reached top level\r\n");
            DB_printf("%d steps taken to reach top/r/n", StepCount);
            StepCount = 0;
            LiftLevel = 3;
            motorState = FORWARD;
            //ES_Event_t NewEvent;
            //NewEvent.EventType = TOP_STACK_REACHED;
            //NewEvent.EventParam = LiftLevel;
            //PostTopGameHSM(NewEvent);
            CurrentState = Waiting_To_Move;  //Decide what the next state will be
            break;
        default:
            break;
        }
    break;
    case Moving_To_Ground:
        switch (ThisEvent.EventType){
        case ES_TIMEOUT:
            if (ThisEvent.EventParam == ScissorLiftTimer) {
                HandleTimeout1();
                StepCount++;
            }
            break;
            case LIFT_STOP:
                DB_printf( "Lift has reached ground level\r\n");
                DB_printf("%d steps taken to reach ground/r/n", StepCount);
                LiftLevel = 0;
                CurrentState = Waiting_To_Move;  //Decide what the next state will be
                break;
        /*case GROUND_LIMIT_HIT:
            DB_printf( "Lift has reached ground level\r\n");
            DB_printf("%d steps taken to reach ground/r/n", StepCount);
            LiftLevel = 0;
            motorState = BACKWARD;
            ES_Event_t NewEvent;
            NewEvent.EventType = GROUND_REACHED;
            NewEvent.EventParam = LiftLevel;
            //PostTopGameHSM(NewEvent);
            CurrentState = Waiting_To_Move;  //Decide what the next state will be
            break;*/
        default:
            break;
        }
    break;
    
    case Moving_To_Bottom_Stack:
        switch (ThisEvent.EventType){
        case ES_TIMEOUT:
            if (ThisEvent.EventParam == ScissorLiftTimer) {
                    StepCount++;
                    if (StepCount > 640){
                        PORTAbits.RA4 = 1;
                        DB_printf( "Lift has reached bottom stack\r\n");
                        DB_printf("%d steps taken to reach bottom stack/r/n", StepCount);
                        LiftLevel = 1;
                        //ES_Event_t NewEvent;
                        //NewEvent.EventType = BOTTOM_STACK_REACHED;
                        //NewEvent.EventParam = LiftLevel;
                        //PostTopGameHSM(NewEvent);
                        CurrentState = Waiting_To_Move;  //Decide what the next state will be
                        }
                    else{
                        HandleTimeout1();
                    }
            }
        break;
        default:
            break;
        }
    break;
     case Moving_To_Middle_Stack:
        switch (ThisEvent.EventType){
        case ES_TIMEOUT:
            if (ThisEvent.EventParam == ScissorLiftTimer) {
                    StepCount++;
                    if (StepCount > 1460){
                        PORTAbits.RA4 = 1;
                        DB_printf( "Lift has reached middle stack\r\n");
                        DB_printf("%d steps taken to reach middle stack/r/n", StepCount);
                        LiftLevel = 2;
                        //ES_Event_t NewEvent;
                        //NewEvent.EventType = MIDDLE_STACK_REACHED;
                        //NewEvent.EventParam = LiftLevel;
                        //PostTopGameHSM(NewEvent);
                        CurrentState = Waiting_To_Move;  //Decide what the next state will be
                        }
                    else{
                        HandleTimeout1();
                    }
            }
        break;
        default:
            break;
        }
    break;
    case Moving_To_Top_Stack:
        switch (ThisEvent.EventType){
        case ES_TIMEOUT:
            if (ThisEvent.EventParam == ScissorLiftTimer) {
                    StepCount++;
                    if (StepCount > 3900){
                        PORTAbits.RA4 = 1;
                        DB_printf( "Lift should have reached top stack\r\n");
                        DB_printf("%d steps taken to reach the top from middle stack/r/n", StepCount);
                        LiftLevel = 3;
                        //ES_Event_t NewEvent;
                        //NewEvent.EventType = TOP_STACK_REACHED;
                        //NewEvent.EventParam = LiftLevel;
                        //PostTopGameHSM(NewEvent);
                        motorState = FORWARD;
                        CurrentState = Waiting_To_Move;  //Decide what the next state will be
                        }
                    else{
                        HandleTimeout1();
                    }
            }
        break;
        case LIFT_STOP:
            DB_printf( "Lift has reached max top limit\r\n");
            //ES_Event_t NewEvent;
            //NewEvent.EventType = TOP_STACK_REACHED;
            //NewEvent.EventParam = LiftLevel;
            //PostTopGameHSM(NewEvent);
            motorState = FORWARD;
            CurrentState = Waiting_To_Move;  //Decide what the next state will be
            break;
        default:
            break;
        }
    break;
    default:
      break;
  }
   return ReturnEvent;
}

/***************************************************************************
 private functions
 ***************************************************************************/
 // HandleTimeout1 function
void HandleTimeout1(void) {
    LATAbits.LATA1 = !motorState; // Set the motor direction
    SetOutputStates();
    ES_Timer_InitTimer(ScissorLiftTimer, STEP_INTERVAL_MS); // Restart timer
}

// SetOutputStates function
void SetOutputStates() {
    
    LATAbits.LATA0 = StepCount % 2; //MODULO
}


/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/
