/****************************************************************************

  Header file for template service
  based on the Gen 2 Events and Services Framework

 ****************************************************************************/

#ifndef SPIFollowerService_H
#define SPIFollowerService_H

//#include "ES_Types.h"

// Public Function Prototypes

typedef enum {

    InitStateSPI, main, WaitingforTransfer, Transferring

}SPIState_t;

bool InitSPIFollowerService(uint8_t Priority);
bool PostSPIFollowerService(ES_Event_t ThisEvent);
ES_Event_t RunSPIFollowerService(ES_Event_t ThisEvent);

#endif /* SPIFollowerService_H */

