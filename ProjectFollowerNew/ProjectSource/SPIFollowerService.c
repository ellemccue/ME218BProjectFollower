        /****************************************************************************
 Module
   TemplateService.c

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
#include "SPIFollowerService.h"
#include "PIC32_SPI_HAL.h"
#include "dbprintf.h"
#include <sys/attribs.h>
#include "ES_Port.h"
#include "DCMotorService.h"
#include "ArenaIndicatorServiceFSM.h"

/*----------------------------- Module Defines ----------------------------*/
#define SPI_CLK_PERIOD 1200
/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/
void SPI_SetFollower_Init();
void SPI_Set_Interrupt_Init();
/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;
static SPIState_t CurrentState; //Module Level State Variable
volatile static uint8_t Command = 3;
static uint32_t counter;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitTemplateService

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
bool InitSPIFollowerService(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;
  /********************************************
   in here you write your initialization code
   *******************************************/
  TRISBbits.TRISB8 = 1; //Set SDI as input pin (connected to SDO pin on other pic)
  TRISBbits.TRISB5 = 0; //Set SDO as input pin (connected to SDI pin on other pic)
  TRISBbits.TRISB15 = 1; //Set SS as Input Pin DIFF (connected to other pic same pin)
  ANSELBbits.ANSB15 = 0; //Disable Analog on SS
  TRISAbits.TRISA4 = 1; // test
  DB_printf("before setup\n");
  //Initialization
    SPI_SetFollower_Init(); //Initialize SPI
    DB_printf("after setup\n");
   //ES_InitDeferralQueueWith(QueuedEvents, 4); //Initialize the deferral queue
   CurrentState = InitStateSPI; 
   
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
     PostTemplateService

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
bool PostSPIFollowerService(ES_Event_t ThisEvent)
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
ES_Event_t RunSPIFollowerService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
  /********************************************
   in here you write your service code
   *******************************************/
  switch (CurrentState) 
    {
      case InitStateSPI:
          DB_printf("Initializing Spi Follower\n");
          CurrentState = main; 
          //Command = SPI1BUF; //Read the Command from the Buffer
          //DB_printf("The Buffer has: %d\n", Command);
      break;
      case main:
          switch (ThisEvent.EventType) 
            {
              case TransferCompleted:
                  DB_printf("Command Recieved is %d\n", Command);
                  if (Command == 0x04){
                    DB_printf("CCW \n");
                    ES_Event_t DummyEvent;
                    DummyEvent.EventType = ES_CCW;
                    PostDCMotorService(DummyEvent);
                    //SPIOperate_SPI1_Send8(0x09); //Write to SPI1BUF to send info to leader pic
                    counter++;
                  } else if (Command == 0x05){
                    DB_printf("STOP \n");
                    ES_Event_t DummyEvent;
                    DummyEvent.EventType = ES_STOP_HOLD;
                    PostDCMotorService(DummyEvent);
                    //SPIOperate_SPI1_Send8(0x20); //Write to SPI1BUF to send info to leader pic
                    counter++;
                  }else if(Command == 0x06){
                    DB_printf("CW \n");
                    ES_Event_t DummyEvent;
                    DummyEvent.EventType = ES_CW;
                    PostDCMotorService(DummyEvent);
                    //SPIOperate_SPI1_Send8(0x20); //Write to SPI1BUF to send info to leader pic
                    counter++;
                  }else if(Command == 0x07){
                    DB_printf("Backwards \n");
                    ES_Event_t DummyEvent;
                    DummyEvent.EventType = ES_REVERSE_FULL_SPEED;
                    PostDCMotorService(DummyEvent);
                    //SPIOperate_SPI1_Send8(0x20); //Write to SPI1BUF to send info to leader pic
                    counter++;}
                  else if(Command == 0x08){
                    DB_printf("FW \n");
                    ES_Event_t DummyEvent;
                    DummyEvent.EventType = ES_FORWARD_FULL_SPEED;
                    PostDCMotorService(DummyEvent);
                    //SPIOperate_SPI1_Send8(0x20); //Write to SPI1BUF to send info to leader pic
                    counter++;}
                    else if(Command == 0x11){
                    DB_printf("GREEN \n");
                    ES_Event_t DummyEvent;
                    DummyEvent.EventType = GreenFieldDetected;
                    PostArenaIndicatorServiceFSM(DummyEvent);
                    //SPIOperate_SPI1_Send8(0x20); //Write to SPI1BUF to send info to leader pic
                    counter++;}
                  else if(Command == 0x12){
                    DB_printf("BLUE \n");
                    ES_Event_t DummyEvent;
                    DummyEvent.EventType = BlueFieldDetected;
                    PostArenaIndicatorServiceFSM(DummyEvent);
                    counter++;}
                     else if(Command == 0x13){
                    DB_printf("Neutral \n");
                    ES_Event_t DummyEvent;
                    DummyEvent.EventType = GoNeutral;
                    PostArenaIndicatorServiceFSM(DummyEvent);
                    //SPIOperate_SPI1_Send8(0x20); //Write to SPI1BUF to send info to leader pic
                    counter++;
                  }
                  else if(Command == 0x14){
                    DB_printf("Line follow by turning left \n");
                    ES_Event_t DummyEvent;
                    DummyEvent.EventType = ES_LF_LEFT;
                    PostDCMotorService(DummyEvent);
                    //SPIOperate_SPI1_Send8(0x20); //Write to SPI1BUF to send info to leader pic
                    counter++;
                  }
                  else if(Command == 0x15){
                    DB_printf("Line follow by turning RIGHT \n");
                    ES_Event_t DummyEvent;
                    DummyEvent.EventType = ES_LF_RIGHT;
                    PostDCMotorService(DummyEvent);
                    //SPIOperate_SPI1_Send8(0x20); //Write to SPI1BUF to send info to leader pic
                    counter++;
                  }
                  else if(Command == 0x10){
                    DB_printf("OFFSET ADJUST \n");
                    ES_Event_t DummyEvent;
                    DummyEvent.EventType = ES_ADJUST_OFFSET;
                    PostDCMotorService(DummyEvent);
                    //SPIOperate_SPI1_Send8(0x20); //Write to SPI1BUF to send info to leader pic
                    counter++;
                  }
                  else if(Command == 0x16){
                    DB_printf("LED ON \n");
                    ES_Event_t DummyEvent;
                    DummyEvent.EventType = LED_ON;
                    PostArenaIndicatorServiceFSM(DummyEvent);
                    //SPIOperate_SPI1_Send8(0x20); //Write to SPI1BUF to send info to leader pic
                    counter++;
                  }
                  else if(Command == 0x17){
                    DB_printf("LED OFF \n");
                    ES_Event_t DummyEvent;
                    DummyEvent.EventType = LED_OFF;
                    PostArenaIndicatorServiceFSM(DummyEvent);
                    //SPIOperate_SPI1_Send8(0x20); //Write to SPI1BUF to send info to leader pic
                    counter++;
                  }
                  else if(Command == 0x18){
                    DB_printf("Ground Level \n");
                    ES_Event_t DummyEvent;
                    DummyEvent.EventType = LIFT_GROUND;
                    PostScissorLiftFSM(DummyEvent);
                    counter++;
                  }
                   else if(Command == 0x19){
                    DB_printf("Bottom Level \n");
                    ES_Event_t DummyEvent;
                    DummyEvent.EventType = LIFT_STACK_BOTTOM;
                    PostScissorLiftFSM(DummyEvent);
                    counter++;
                  }
                   else if(Command == 0x20){
                    DB_printf("Middle Level \n");
                    ES_Event_t DummyEvent;
                    DummyEvent.EventType = LIFT_STACK_MIDDLE;
                    PostScissorLiftFSM(DummyEvent);
                    counter++;
                  }
                  else if(Command == 0x21){
                    DB_printf("Top Level \n");
                    ES_Event_t DummyEvent;
                    DummyEvent.EventType = LIFT_STACK_TOP;
                    PostScissorLiftFSM(DummyEvent);
                    counter++;
                  }
                   else if(Command == 0x22){
                    DB_printf("Lift Stop \n");
                    ES_Event_t DummyEvent;
                    DummyEvent.EventType = LIFT_STOP;
                    PostScissorLiftFSM(DummyEvent);
                    counter++;
                  }
                  else if(Command == 0x23){
                    DB_printf("Adjust CCW \n");
                    ES_Event_t DummyEvent;
                    DummyEvent.EventType = ES_ADJUST_OFFSET_CCW;
                    PostDCMotorService(DummyEvent);
                    counter++;
                  } else if(Command == 0x24){
                    DB_printf("Adjust CW turn \n");
                    ES_Event_t DummyEvent;
                    DummyEvent.EventType = ES_CW_OFFSET;
                    PostDCMotorService(DummyEvent);
                    counter++;
                  }
                    else {
                    //DB_printf("Backward \n");
                    //SPIOperate_SPI1_Send8(0x20); //Write to SPI1BUF to send info to leader pic
                    //counter = 0;
                  }
                  
              break;

          }
      break;
          
      
  }
  return ReturnEvent;
}

/***************************************************************************
 private functions
 ***************************************************************************/

void SPI_Set_Interrupt_Init() {
        __builtin_disable_interrupts();
        IEC1bits.SPI1RXIE = 0; //Diable SPI1 Interrupts
        INTCONbits.MVEC = 1; //Enable the Multi Vector mode
        IPC7bits.SPI1IP = 3; //Set the SPI Interrupt priority IRQ
        IFS1CLR = _IFS1_SPI1RXIF_MASK; //Clear the SPI Interrupt Flag 
        IEC1SET = _IEC1_SPI1RXIE_MASK; //Set the SPI1 Interrupt Flag
        __builtin_enable_interrupts();
    }

 void SPI_SetFollower_Init() {
        SPISetup_BasicConfig(SPI_SPI1); //Setup SPI Basic Configuration
        SPISetup_DisableSPI(SPI_SPI1); //Disable the SPI
        SPI1BUF; //Clear the SPI Buffer
        SPI1STATbits.SPIROV = 0; // Clear the receive overflow flag bit
        SPISetup_SetFollower(SPI_SPI1); //Set SPI as the Follower
        SPISetup_SetBitTime(SPI_SPI1, SPI_CLK_PERIOD); //Set the SPI Clock Period
        SPISetup_MapSSInput(SPI_SPI1, SPI_RPB15); //Map the Slave Select Input pin (DIFF))
        SPISetup_MapSDOutput(SPI_SPI1, SPI_RPB5); //Map the Serial Data Out Pin
        SPISetup_MapSDInput(SPI_SPI1, SPI_RPB8); //Map the Serial Data In Pin
        SPISetup_SetClockIdleState(SPI_SPI1, SPI_CLK_HI); //Set the idle clock state high
        SPISetup_SetActiveEdge(SPI_SPI1, SPI_SECOND_EDGE); //Set the active edge as the second edge
        SPISetup_SetXferWidth(SPI_SPI1, SPI_8BIT); //Set the transfer width as 8 bits
        SPISetEnhancedBuffer(SPI_SPI1, false); //Disable the Enhanced Buffer Mode
        
        //SPI1CONbits.SRXISEL = 0b01; //Set the Receive Buffer Full Interrupt mode bit (Interrupt is generated when the Buffer is not empty)
        SPI_Set_Interrupt_Init(); //ADD BACK
        SPISetup_EnableSPI(SPI_SPI1); //Enable the SPI
        //DB_printf("spi is initialized");
    }
 
 //SPI Transfer Completion Interrupt Function

//    void __ISR(_SPI_1_VECTOR, IPL3SOFT) TRANSFER_COMPLETE_INTERRUPT(void) {
//        //Check if the SPIRXIF flag has been set after completion of the transfer
//        //DB_printf("Testisr\n");
//        LATAbits.LATA4 = 1;
//        if (IFS1bits.SPI1RXIF) {
//            uint8_t Command = SPI1BUF; //Read the Command from the Buffer
//
//            ES_Event_t ThisEvent;
//            ThisEvent.EventType = TransferCompleted;
//            ThisEvent.EventParam = Command;
//            PostSPIFollowerService(ThisEvent); //Post the transfer completion event to the SPI Service
//            
//        }
//        LATAbits.LATA4 = 0;
//        IFS1CLR = _IFS1_SPI1RXIF_MASK; //Clear the SPI Interrupt Flag
//    }
 
void __ISR(_SPI_1_VECTOR, IPL3SOFT) TRANSFER_COMPLETE_INTERRUPT(void) {
      //Check if the SPIRXIF flag has been set after completion of the transfer
      //DB_printf("Testisr\n");
      //LATAbits.LATA4 = 1;
      if (SPI1STATbits.SPIRBF == 1) {
          Command = SPI1BUF; //Read the Command from the Buffer
          //DB_printf("The Buffer has: %d\n", Command);

          ES_Event_t ThisEvent;
          ThisEvent.EventType = TransferCompleted;
          ThisEvent.EventParam = Command;
          PostSPIFollowerService(ThisEvent); //Post the transfer completion event to the SPI Service
          SPI1BUF =0x00;
      }
      //LATAbits.LATA4 = 0;
      IFS1CLR = _IFS1_SPI1RXIF_MASK; //Clear the SPI Interrupt Flag
  }
/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/

