/****************************************************************************
 Module
   DCMotorService.c

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
//#include "TemplateService.h"
//#include "ADService.h"
#include "PIC32_SPI_HAL.h"
#include "ES_Port.h"

#include "terminal.h"
#include <sys/attribs.h>
#include "DCMotorService.h"
#include "dbprintf.h"
#include "ES_Events.h"
//#include "PIC32_AD_Lib.h"
//#include "IRSensorService.h"
//#inclsude "PWM_PIC32.h"
/*----------------------------- Module Defines ----------------------------*/

#define ENCODER_PULSES_PER_REV 3
#define GEAR_RATIO 298
#define SLOWEST_SPEED 12
#define HIGHEST_PERIOD 59502
#define LOWEST_PERIOD 13817
#define MAX_LEDs 8
#define PWM_FREQUENCY 5291
//#define DUTY_CYCLE 100
#define MAX_MOTOR_SPEED 24.5
#define KP 20
#define KI 0.1
#define KD 0

#define CW_MOTOR_SPEED 6000// 6100
#define CCW_MOTOR_SPEED 6000 // 6100
#define BEACON_MOTOR_SPEED 12000
#define OPTOSENSOR_THRESHOLD 200 // THIS WAS RANDOM ??? 
//#define DUTY_CYCLE_NUMBER 6

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/
bool InitDCMotorService(uint8_t Priority);
bool PostDCMotorService(ES_Event_t ThisEvent);
ES_Event_t RunDCMotorService(ES_Event_t ThisEvent);
void HandleTimeout(void);
//uint32_t GetDesiredSpeed(void);
//void __ISR(_TIMER_3_VECTOR, IPL7SOFT) T3_IntHandler(void);
//void __ISR(_INPUT_CAPTURE_2_VECTOR, IPL7SOFT) IC2Handler(void);
//void __ISR(_TIMER_2_VECTOR, IPL6SOFT) Timer2ISR(void);
//void GetPeriod(uint32_t current_period);
//void __ISR(_TIMER_4_VECTOR, IPL7SOFT) T4_IntHandler(void);


/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;
volatile static uint32_t ScaledSpeed;
volatile static uint32_t pulse_count = 0;
volatile static uint32_t last_time = 0;
volatile static uint32_t current_period = 0;
volatile static uint32_t last_period = 0;
volatile static uint32_t last_pulse_count = 0;
volatile static uint32_t current_pulse_count = 0;
volatile static int32_t motor_speed;
volatile static uint32_t PR_Value;
volatile static float RPM;
volatile static float TargetRPM;
volatile static float RequestedDuty;
volatile static float SumError;
//volatile static int32_t Duty_Cycle[22] = {0, 10, 15, 16, 18, 20, 30, 32, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100};



// Define static variables for direction and step tracking
typedef enum {
    BothFORWARD,
    BothBACKWARD,
    RightFORWARD,
    LeftFORWARD,
            RightSlowFORWARD,
            LeftSlowFORWARD,
    PAUSED
} currentDirection_t;

static currentDirection_t motorState = PAUSED;

//Define Union for pulse count and time track
typedef union {
    volatile uint32_t current_time;
    volatile uint16_t timecount_array[2];
} TimeCount;

//TimeCount encoder;

Motor_State_t CurrentState = waiting; // set the initial state to waiting
ES_Event_t SelectedEvent; 
uint32_t Optosensor_Reading[1]; // optosensor value initialization
uint32_t beaconPeriod;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitDCMotorService

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, and does any
     other required initialization for this service
 Notes

 Author
     J. Edward Carryer, 01/16/12, 10:00
****************************************************************************/
bool InitDCMotorService(uint8_t Priority)
{
  ES_Event_t ThisEvent;
  bool ReturnVal = true;

  MyPriority = Priority;
  /********************************************
   in here you write your initialization code
   *******************************************/
   // Initialize the port lines
  TRISBbits.TRISB3 = 0; //Set RB3 as Output
  TRISBbits.TRISB2 = 0; ////Set RB2 as Output
  ANSELBbits.ANSB3 = 0; //Disable Analog on RB3
  ANSELBbits.ANSB2 = 0; //Disable Analog on RB2

  TRISBbits.TRISB4 = 0; // Set RB4 as output (Right motor other pin)
  TRISAbits.TRISA3 = 0; // Set RA3 as output (Left motor other pin)
  
//  TRISBbits.TRISB14 = 0; // Set RB14 as output
//  TRISBbits.TRISB8 = 0; // Set RB8 as output
//  TRISBbits.TRISB15 = 0; // Set RB15 as output
//  TRISBbits.TRISB10 = 0; // Set RB10 as output
//  TRISBbits.TRISB11 = 0; // Set RB11 as output
//  TRISBbits.TRISB12 = 0; // Set RB12 as output
//  TRISAbits.TRISA1 = 0; // Set RA1 as output

  //Disable analogs
  //ANSELBbits.ANSB4 = 0;
  //ANSELBbits.ANSB5 = 0;
//  ANSELBbits.ANSB14 = 0;
//  //ANSELBbits.ANSB8 = 0;
//  //ANSELbits.ANSB15 = 0;
//  //ANSELBbits.ANSB10 = 0;
//  //ANSELBbits.ANSB11 = 0;
//  ANSELBbits.ANSB12 = 0;
//  ANSELAbits.ANSA1 = 0;

  //DB_printf( "Starting DC Motor Service\r\n");
  
  // Initialize timer2 interrupt and timer2
  // InitTimer2Interrupt();
  //InitTimer4Interrupt();
  

  // Initialize Input Capture from the Encoder
  //InitEncoderPulsePeriod ();

  // Initialize the Analog pin for the optosensor 
  TRISAbits.TRISA0 = 1; // Set RA0 as input 
  ANSELAbits.ANSA0 = 1; // set analog 
  //ADC_ConfigAutoScan(BIT0HI); // RA0, check this later !!?? 
  

  // Initialize PWM configuration
    OC1CONbits.ON = 0; // Turn off the OC1 when performing the setup
    OC4CONbits.ON = 0; // Turn off the OC4 when performing the setup
    OC1CONbits.SIDL = 0; // Continue module operation when device enters the idle mode
    OC4CONbits.SIDL = 0; // Continue module operation when device enters the idle mode
    OC1CONbits.OC32 = 0; // 16-bit compare mode bit
    OC4CONbits.OC32 = 0; // 16-bit compare mode 
    OC1CONbits.OCTSEL = 1; // timer3
    OC4CONbits.OCTSEL = 1; // timer3
    OC1CONbits.OCM = 0b110; // disabling fault pin
    OC4CONbits.OCM = 0b110; // disabling fault pin 
    RPB3R = 0b0101; // Mapping OC1 to Pin RPB3
    RPB2R = 0b0101; // Mapping OC4 to RPB2
    T3CONbits.ON = 0; // Disable Timer3
    T3CONbits.TCS = 0; // Select internal peripheral clock
    T3CONbits.TCKPS = 0x0001; // Set the timer input clock pre-scale of 1:1
    //T3CONbits.T32 = 0; // Set 32-bit timer mode select bit
    PR3 = 19999; // Set Period register to 1919
    PR_Value = PR3+1;
    OC1RS = 0; //Set initial duty cycle to zero
    OC4RS = 0; //Set initial duty cycle to zero
    OC1R = 0; // Write to OC1R the initial duty cycle of zero
    OC4R = 0; // Write to OC4R the initial duty cycle of zero
    T3CONbits.ON = 1; // Enable Timer3
    OC1CONbits.ON = 1; // Enable OC1
    OC4CONbits.ON = 1; // Enable OC4
    //IC2CONbits.ON = 1;
    //T2CONbits.ON = 1; // Enable the Timer2
    OC1RS = 18000; //Set initial duty cycle to zero
    LATBbits.LATB4 = 0;
    OC4RS = 18000; //Set initial duty cycle to zero
    LATAbits.LATA3 = 0;
     
    
    //T4CONbits.ON = 1; // Enable the Timer5
    //ES_Timer_InitTimer(DC_MOTOR_SERVICE_TIMER, 10);
    DB_printf("DC Motor Service Initialized\n");
  //InitButtonDebounce();
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
     PostDCMotorService

 Parameters
     EF_Event_t ThisEvent ,the event to post to the queue

 Returns
     bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:25
****************************************************************************/
bool PostDCMotorService(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunTemplateService

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here
 Notes

 Author
   J. Edward Carryer, 01/15/12, 15:23
****************************************************************************/
ES_Event_t RunDCMotorService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
  /********************************************
   in here you write your service code
   *******************************************/
   // All states: waiting, turning, scanningBeacon, scanningLine
  //ADC_MultiRead(Optosensor_Reading); // check the optosensor value by reading
 //DB_printf("O %d\n",Optosensor_Reading[0] );
  switch (CurrentState){
      case waiting: 
        switch (ThisEvent.EventType) {
            case ES_NEW_KEY:
                DB_printf("%d\n", ThisEvent.EventParam);
                if (ThisEvent.EventParam == 's'){
                    DB_printf("s happened\n");
                    ES_Event_t DummyEvent;
                    DummyEvent.EventType = ES_STOP_HOLD;
                    PostDCMotorService(DummyEvent);
                 }
                if (ThisEvent.EventParam == 'f'){
                    DB_printf("f happened\n");
                    ES_Event_t DummyEvent;
                    DummyEvent.EventType = ES_FORWARD_FULL_SPEED;
                    PostDCMotorService(DummyEvent);
                }
                if (ThisEvent.EventParam == 'q'){
                    DB_printf("q happened\n");
                    ES_Event_t DummyEvent;
                    DummyEvent.EventType = ES_FORWARD_HALF_SPEED;
                    PostDCMotorService(DummyEvent);
                 }
                 if (ThisEvent.EventParam == 'w'){
                    DB_printf("w happened\n");
                    ES_Event_t DummyEvent;
                    DummyEvent.EventType = ES_REVERSE_FULL_SPEED;
                    PostDCMotorService(DummyEvent);
                 }
                 if (ThisEvent.EventParam == 'e'){
                    DB_printf("e happened\n");
                    ES_Event_t DummyEvent;
                    DummyEvent.EventType = ES_REVERSE_HALF_SPEED;
                    PostDCMotorService(DummyEvent);
                 }
                 if (ThisEvent.EventParam == 'r'){
                    DB_printf("r happened\n");
                    ES_Event_t DummyEvent;
                    DummyEvent.EventType = ES_CW;
                    PostDCMotorService(DummyEvent);
                 }
                if (ThisEvent.EventParam == 'a'){
                    DB_printf("a happened\n");
                    ES_Event_t DummyEvent;
                    DummyEvent.EventType = ES_CW_45;
                    PostDCMotorService(DummyEvent);
                 }
                if (ThisEvent.EventParam == 'd'){
                    DB_printf("d happened\n");
                    ES_Event_t DummyEvent;
                    DummyEvent.EventType = ES_CCW;
                    PostDCMotorService(DummyEvent);
                 }
                if (ThisEvent.EventParam == 'g'){
                    DB_printf("g happened\n");
                    ES_Event_t DummyEvent;
                    DummyEvent.EventType = ES_CCW_45;
                    PostDCMotorService(DummyEvent);
                 }
                if (ThisEvent.EventParam == 'h'){
                    DB_printf("h happened\n");
                    ES_Event_t DummyEvent;
                    DummyEvent.EventType = ES_DRIVE_FORWARD_TAPE_DETECTED;
                    PostDCMotorService(DummyEvent);
                    
                }
                if (ThisEvent.EventParam == 'k'){
                    DB_printf("k happened\n");
                    ES_Event_t DummyEvent;
                    DummyEvent.EventType = ES_ALIGN_BEACON;
                    PostDCMotorService(DummyEvent);
                    
                }
                
                break;
            case ES_STOP_HOLD:
                motorState = BothFORWARD; // forward command 
                ScaledSpeed = 0;// set desired speed right to 0, set desired speed left to 0
                HandleTimeout();// call handle timeout 
                break;
            case ES_FORWARD_FULL_SPEED:
                motorState = BothFORWARD; // forward command
                ScaledSpeed = 7000;// set fdesired speed right to full, set desired speed left to full
                HandleTimeout();// call handle timeout 
                //DB_printf("Forward");
                // CLEAR FLAG
                break;
            case ES_FORWARD_HALF_SPEED:
                motorState = BothFORWARD; // forward command
                ScaledSpeed = (int)((float)PR_Value*2.325/4.0);// set desired speed right to half, set desired speed left to half
                HandleTimeout();// call handle timeout 
                // CLEAR FLAG
                break;
            case ES_REVERSE_FULL_SPEED:
                motorState = BothBACKWARD; // backward command
                ScaledSpeed = 7000;// set desired speed sight to full, set desired speed left to full
                HandleTimeout();// call handle timeout 
                // CLEAR FLAG
                break;
            case ES_REVERSE_HALF_SPEED:
                motorState = BothBACKWARD; // backward command
                ScaledSpeed = (int)((float)PR_Value*2.325/4.0);// set desired speed right to half, set desired speed left to half
                HandleTimeout();// call handle timeout 
                // CLEAR FLAG
                break;
            case ES_CW:
                motorState = LeftFORWARD; // backward command
                ScaledSpeed = CW_MOTOR_SPEED; // set desired speed right to half, set desired speed left to half
                HandleTimeout(); // call handle timeout 
                //CurrentState = turning; // turning state
                //ES_Timer_InitTimer(TURN_TIMER, 6000);// start timer for 6 seconds
                break;
            case ES_LF_RIGHT:
                motorState = LeftSlowFORWARD; // backward command
                ScaledSpeed = 7000; // set desired speed right to half, set desired speed left to half
                HandleTimeout(); // call handle timeout 
                //CurrentState = turning; // turning state
                //ES_Timer_InitTimer(TURN_TIMER, 6000);// start timer for 6 seconds
                break;
            case ES_CW_OFFSET: // 
                motorState = LeftFORWARD; // backward command
                ScaledSpeed = 7000;// set desired speed right to half, set desired speed left to half
                HandleTimeout();// call handle timeout 
                CurrentState = turning;
                ES_Timer_InitTimer(TURN_TIMER, 500);// start timer for 3 seconds
                break;
            case ES_CCW:
                motorState = RightFORWARD; // backward command
                ScaledSpeed = CCW_MOTOR_SPEED;// set desired speed right to half, set desired speed left to half
                HandleTimeout();// call handle timeout 
                //CurrentState = turning;
                //ES_Timer_InitTimer(TURN_TIMER, 6000);// start timer for 6 seconds
                break;
            case ES_LF_LEFT:
                motorState = RightSlowFORWARD; // backward command
                ScaledSpeed = 7000;//CW_MOTOR_SPEED; // set desired speed right to half, set desired speed left to half
                HandleTimeout(); // call handle timeout 
                //CurrentState = turning; // turning state
                //ES_Timer_InitTimer(TURN_TIMER, 6000);// start timer for 6 seconds
                
                break;
            case ES_CCW_45:
                motorState = RightFORWARD; // backward command
                ScaledSpeed = CCW_MOTOR_SPEED;// set desired speed right to half, set desired speed left to half
                HandleTimeout();// call handle timeout 
                //CurrentState = turning;
                //ES_Timer_InitTimer(TURN_TIMER, 3000);// start timer for 3 seconds
                break;
            case ES_ADJUST_OFFSET:
                DB_printf("Adjust forward\n");
                motorState = BothFORWARD; // forward command
                ScaledSpeed = 5000;// set fdesired speed right to full, set desired speed left to full
                HandleTimeout();// call handle timeout 
                CurrentState = adjust;
                ES_Timer_InitTimer(TURN_TIMER, 1800);// start timer for 3 seconds
                
                break;
            case ES_ADJUST_OFFSET_CCW:
                DB_printf("Adjust forward\n");
                motorState = BothFORWARD; // forward command
                ScaledSpeed = 5000;// set fdesired speed right to full, set desired speed left to full
                HandleTimeout();// call handle timeout 
                CurrentState = adjust_ccw;
                ES_Timer_InitTimer(TURN_TIMER, 2300);// start timer for 3 seconds
                
                break;
                
            case ES_ALIGN_BEACON:
                // Turn ccw at scanning beacon speed
                //DB_printf("In ES_ALIGN_BEACON.\n");
                //motorState = LeftFORWARD; // backward command
                //ScaledSpeed = BEACON_MOTOR_SPEED;// set desired speed right to half, set desired speed left to half
                //HandleTimeout();// call handle timeout 
                //CurrentState = scanningBeacon;
                //ES_Timer_InitTimer(TURN_TIMER, 5000);// start timer for 3 seconds
                // enable the beacon interrupt
                //SelectedEvent.EventType = ES_SEARCH_BEACON; // event is ES_SEARCH_BEACON 
                //PostDCMotorService(SelectedEvent); // post to this service
                break;
            case ES_DRIVE_FORWARD_TAPE_DETECTED:
                // Move forward 
                //motorState = BothFORWARD; // forward command
                //ScaledSpeed = PR_Value;// set fdesired speed right to full, set desired speed left to full
                //HandleTimeout();// call handle timeout

                //CurrentState = scanningLine;// change state to scanningLine
                
                //SelectedEvent.EventType = ES_SEARCH_LINE; // event is ES_SEARCH_LINE 
                //PostDCMotorService(SelectedEvent); // post to this service
                break; 

     //        case ADServiceTimeout:
     //            
     //             
     //            if (ThisEvent.EventParam == 2) {
     //                //DB_printf("Received AD Service Timer Timeout\n");
     //                
     //                uint32_t DesiredSpeed = GetDesiredSpeed();
     //                DB_printf("Desired Speed: %d\n", DesiredSpeed);
     //                //DesiredSpeed = DesiredSpeed * PR_Value;
     //                //ScaledSpeed = DesiredSpeed / 1023;
     //                TargetRPM = (DesiredSpeed * MAX_MOTOR_SPEED) / 1023;
     //                HandleTimeout();
     //                GetPeriod(current_period);
     //            }
     //            break;
//            case ES_TIMEOUT:
//                if (ThisEvent.EventParam == DC_MOTOR_SERVICE_TIMER) {
//
//                    ES_Timer_InitTimer(DC_MOTOR_SERVICE_TIMER, 1000);
//                    //DB_printf("Requested Duty: %d\n", (int)RequestedDuty);
//                    //DB_printf("Cumulative Error: %d\n", SumError);
//                    //DB_printf("Scaled Duty: %d\n", ScaledSpeed); 
//                 }
//                break;
             default:
                 // No specific action
                 break;
         }
        break;
      case turning:
            switch (ThisEvent.EventType) {
                case ES_TIMEOUT:
                    // Stop the robot maybe ??? this could be changed TODO
                    DB_printf("Timeout Adjust CCW\n");
                    motorState = LeftFORWARD; // forward command 
                    ScaledSpeed = 7000;// set desired speed right to 0, set desired speed left to 0
                    HandleTimeout();// call handle timeout
                    
                    CurrentState = waiting; // set state to waiting state
                    // CLEAR FLAG
                break;
            }
          break;
          case adjust:
            switch (ThisEvent.EventType) {
                case ES_TIMEOUT:
                    // Stop the robot maybe ??? this could be changed TODO
                    DB_printf("Timeout Adjust forward\n");
                    motorState = LeftFORWARD; // forward command 
                    ScaledSpeed = 7000;// set desired speed right to 0, set desired speed left to 0
                    HandleTimeout();// call handle timeout
                    ES_Timer_InitTimer(TURN_TIMER, 1500);// start timer for 3 seconds CHANGED
                    CurrentState = turning; // set state to waiting state
                    SelectedEvent.EventType = ES_ADJUST_OFFSET; // event is ES_STOP_HOLD 
                    PostDCMotorService(SelectedEvent); // post to this service
                    
                    // CLEAR FLAG
                break;
            }
          break;
          case adjust_ccw:
            switch (ThisEvent.EventType) {
                case ES_TIMEOUT:
                    // Stop the robot maybe ??? this could be changed TODO
                    DB_printf("Timeout Adjust tight forward\n");
                    motorState = RightFORWARD; // forward command 
                    ScaledSpeed = 7000;// set desired speed right to 0, set desired speed left to 0
                    HandleTimeout();// call handle timeout
                    ES_Timer_InitTimer(TURN_TIMER, 1500);// start timer for 3 seconds CHANGED
                    CurrentState = turning_ccw; // set state to waiting state
                    SelectedEvent.EventType = ES_ADJUST_OFFSET; // event is ES_STOP_HOLD 
                    PostDCMotorService(SelectedEvent); // post to this service
                    
                    // CLEAR FLAG
                break;
            }
          break;
          case turning_ccw:
            switch (ThisEvent.EventType) {
                case ES_TIMEOUT:
                    // Stop the robot maybe ??? this could be changed TODO
                    DB_printf("Timeout Adjust CCW\n");
                    motorState = RightFORWARD; // forward command 
                    ScaledSpeed = 7000;// set desired speed right to 0, set desired speed left to 0
                    HandleTimeout();// call handle timeout
                    
                    CurrentState = waiting; // set state to waiting state
                    // CLEAR FLAG
                break;
            }
          break;
      case scanningBeacon:
          //DB_printf("scanning beacon case");
            switch (ThisEvent.EventType) {
                case ES_STOP_HOLD:
                    motorState = BothFORWARD; // forward command 
                    ScaledSpeed = 0;// set desired speed right to 0, set desired speed left to 0
                    HandleTimeout();// call handle timeout
                    DB_printf("stop");
                break;
                case ES_SEARCH_BEACON:
                    // check the beacon interrupt period value
                    //beaconPeriod = GetPeriod(); //period is in ns
                    //DB_printf("IR period %d\n", beaconPeriod);
                    if (beaconPeriod == 300){// if equal to target value
                        DB_printf("Beacon Found 300\n");
                        SelectedEvent.EventType = ES_STOP_HOLD; // event is ES_STOP_HOLD 
                        PostDCMotorService(SelectedEvent); // post to this service
                    }else if (beaconPeriod == 500){// if equal to target value
                        DB_printf("Beacon Found 500\n");
                        SelectedEvent.EventType = ES_STOP_HOLD; // event is ES_STOP_HOLD 
                        PostDCMotorService(SelectedEvent); // post to this service
                    }else if (beaconPeriod == 1100){// if equal to target value
                        DB_printf("Beacon Found 1100\n");
                        SelectedEvent.EventType = ES_STOP_HOLD; // event is ES_STOP_HOLD 
                        PostDCMotorService(SelectedEvent); // post to this service
                    }else if (beaconPeriod == 700){// if equal to target value
                        DB_printf("Beacon Found 700\n");
                        SelectedEvent.EventType = ES_STOP_HOLD; // event is ES_STOP_HOLD 
                        PostDCMotorService(SelectedEvent); // post to this service
                    } else{// else
                        //DB_printf("Beacon not found \n");
                        SelectedEvent.EventType = ES_SEARCH_BEACON; // event is ES_SEARCH_BEACON 
                        PostDCMotorService(SelectedEvent); // post to this service
                    }
                break;
                case ES_TIMEOUT:
                    CurrentState = waiting; // change state to waiting
                    DB_printf("timeout 1");
                    // CLEAR FLAG
                break;
            }
          break;
      case scanningLine:
            switch (ThisEvent.EventType) {
                case ES_SEARCH_LINE:
                    
                    //ADC_MultiRead(Optosensor_Reading); // check the optosensor value by reading
                    DB_printf("Optoread %d\n",Optosensor_Reading[0] );
                    if (Optosensor_Reading[0] > OPTOSENSOR_THRESHOLD){// if equal to target value
                        DB_printf("line found\n");
                        CurrentState = end;// change state to waiting 
                        // DON'T CLEAR FLAG ??? we want it to just stop
                        SelectedEvent.EventType = ES_STOP_HOLD; // event is ES_STOP_HOLD 
                        PostDCMotorService(SelectedEvent); // post to this service
                    }else {
                    // else
                        SelectedEvent.EventType = ES_SEARCH_LINE; // event is ES_SEARCH_LINE 
                        PostDCMotorService(SelectedEvent); // post to this service
                    }
                break;
            }
          break;
      case end:
          
          motorState = BothFORWARD; // forward command 
          ScaledSpeed = 0;// set desired speed right to 0, set desired speed left to 0
          HandleTimeout();// call handle timeout
          break;
      default:
          // No specific action 
          break;
  }
   
   return ReturnEvent;
}

/***************************************************************************
 private functions
 ***************************************************************************/
 // HandleTimeout function
void HandleTimeout(void) {
    // Update current step based on the direction
    switch (motorState) {
        case BothFORWARD:
            OC1RS = ScaledSpeed;// Set duty cycle based on pot value
            //DB_printf("forward\n");
            LATBbits.LATB4 = 0; // set other pin to 0
            OC4RS = ScaledSpeed;
            LATAbits.LATA3 = 0;// set other pin to 0
            break;
        case BothBACKWARD:
            OC1RS = PR_Value - ScaledSpeed; // is it PR2? 
            LATBbits.LATB4 = 1;// set other pin to 1
            OC4RS = PR_Value - ScaledSpeed;// Set duty cycle based on pot value
            LATAbits.LATA3 = 1;// set other pin to 1
            break;
        case RightFORWARD:
            OC4RS = ScaledSpeed;// Set duty cycle based on pot value
            LATBbits.LATB4 = 0;// set other pin to 0
            OC1RS = PR_Value - ScaledSpeed;// Set duty cycle based on pot value
            LATAbits.LATA3 = 1;// set other pin to 1
            break;
        case LeftFORWARD:
            OC4RS = PR_Value - ScaledSpeed;
            LATBbits.LATB4 = 1; // set other pin to 1
            OC1RS = ScaledSpeed;// Set duty cycle based on pot value
            LATAbits.LATA3 = 0;// set other pin to 0
            break;
        case RightSlowFORWARD:
            OC4RS = ScaledSpeed - 2000;// Set duty cycle based on pot value
            LATBbits.LATB4 = 0;// set other pin to 0
            OC1RS =  ScaledSpeed;// Set duty cycle based on pot value
            LATAbits.LATA3 = 0;// set other pin to 1
            break;
        case LeftSlowFORWARD:
            OC4RS = ScaledSpeed;
            LATBbits.LATB4 = 0; // set other pin to 1
            OC1RS = ScaledSpeed- 2000;// Set duty cycle based on pot value
            LATAbits.LATA3 = 0;// set other pin to 0
            break;
        case PAUSED: // MIGHT NOT NEED THIS TODO
            OC4RS = 0;
            LATBbits.LATB4 = 0;// set other pin to 0
            OC1RS = 0;// Set duty cycle based on pot value
            LATAbits.LATA3 = 0;// set other pin to 0
    }
}


// Timer 3 Interrupt Flag Handler
void __ISR(_TIMER_3_VECTOR, IPL7SOFT) T3_IntHandler(void)
{
     __builtin_disable_interrupts(); // Disable interrupts globally
    
     IFS0CLR = _IFS0_T3IF_MASK; // Clear Timer3 interrupt flag
    
     __builtin_enable_interrupts(); // Enable interrupts globally
     
}


//void GetPeriod(uint32_t current_period){
//    //DB_printf("Encoder pulse period is %d\n", current_period);
//}





/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/
