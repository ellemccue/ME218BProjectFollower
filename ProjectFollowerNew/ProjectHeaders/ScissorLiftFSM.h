/****************************************************************************

  Header file for template service
  based on the Gen 2 Events and Services Framework

 ****************************************************************************/

#ifndef ScissorLiftFSM_H
#define ScissorLiftFSM_H

#include "ES_Types.h"
#include "ES_Configure.h" /* gets us event definitions */

typedef enum
{
  InitPState, Waiting_To_Move, Moving_To_Ground,
  Moving_To_Bottom_Stack, Moving_To_Middle_Stack, Moving_To_Top_Stack, Going_Forward, Going_Backward
}ScissorLiftState_t;

// Public Function Prototypes

bool InitScissorLiftFSM(uint8_t Priority);
bool PostScissorLiftFSM(ES_Event_t ThisEvent);
ES_Event_t RunScissorLiftFSM(ES_Event_t ThisEvent);
void HandleTimeout1(void);
void SetOutputStates();

#endif /* ScissorLiftFSM_H */

