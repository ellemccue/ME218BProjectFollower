/****************************************************************************

  Header file for template Flat Sate Machine
  based on the Gen2 Events and Services Framework

 ****************************************************************************/

#ifndef ArenaIndicatorServiceFSM_H
#define ArenaIndicatorServiceFSM_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */

// typedefs for the states
// State definitions for use with the query function
/*typedef enum
{
  InitPState, UnlockWaiting, _1UnlockPress,
  _2UnlockPresses, Locked
}TemplateState_t;*/

// Public Function Prototypes

bool InitArenaIndicatorServiceFSM(uint8_t Priority);
bool PostArenaIndicatorServiceFSM(ES_Event_t ThisEvent);
ES_Event_t RunArenaIndicatorServiceFSM(ES_Event_t ThisEvent);

#endif /* FSMTemplate_H */
