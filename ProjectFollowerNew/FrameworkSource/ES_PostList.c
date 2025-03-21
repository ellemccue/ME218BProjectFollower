/****************************************************************************
 Module
     EF_PostList.c
 Description
     source file for the module to post events to lists of state
     machines
 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 10/26/17 18:20 jec     moved prototype of PostToList into the conditional to
                        eliminate warning when not using distribution lists
 08/05/13 15:04 jec      added #includes for ES_Port & ES_Types and converted
                         types to match portable types
 01/15/12 15:55 jec      re-coded for Gen2 with conditional declarations
 10/16/11 12:32 jec      started coding
*****************************************************************************/
/*----------------------------- Include Files -----------------------------*/

#include "../FrameworkHeaders/ES_Port.h"
#include "../FrameworkHeaders/ES_Types.h"
#include "../FrameworkHeaders/ES_Configure.h"
#include "../FrameworkHeaders/ES_General.h"
#include "../FrameworkHeaders/ES_PostList.h"
#include "../FrameworkHeaders/ES_ServiceHeaders.h"

/*---------------------------- Module Functions ---------------------------*/

/*---------------------------- Module Variables ---------------------------*/
// Fill in these arrays with the lists of posting funcitons for the state
// machines that will have common events delivered to them.

#if NUM_DIST_LISTS > 0
static bool PostToList(PostFunc_t *const *FuncList, uint8_t ListSize, ES_Event_t NewEvent);
static PostFunc_t *const DistList00[] = {
  DIST_LIST0
};
// the endif for NUM_DIST_LISTS > 0 is at the end of the file
#if NUM_DIST_LISTS > 1
static PostFunc_t *const DistList01[] = {
  DIST_LIST1
};
#endif
#if NUM_DIST_LISTS > 2
static PostFunc_t *const DistList02[] = {
  DIST_LIST2
};
#endif
#if NUM_DIST_LISTS > 3
static PostFunc_t *const DistList03[] = {
  DIST_LIST3
};
#endif
#if NUM_DIST_LISTS > 4
static PostFunc_t *const DistList04[] = {
  DIST_LIST4
};
#endif
#if NUM_DIST_LISTS > 5
static PostFunc_t *const DistList05[] = {
  DIST_LIST5
};
#endif
#if NUM_DIST_LISTS > 6
static PostFunc_t *const DistList06[] = {
  DIST_LIST6
};
#endif
#if NUM_DIST_LISTS > 7
static PostFunc_t *const DistList07[] = {
  DIST_LIST7
};
#endif

/*------------------------------ Module Code ------------------------------*/

// Each of these list-specific functions is a wrapper that calls the generic
// function to walk through the list, calling the listed posting functions

/****************************************************************************
 Function
   PostListxx
 Parameters
   ES_Event NewEvent : the new event to be passed to each of the state machine
   posting functions in list xx
 Returns
   bool: true if all the post functions succeeded, false if any failed
 Description
   Posts NewEvent to all of the state machines listed in the list
 Notes

 Author
   J. Edward Carryer, 10/24/11, 07:48
****************************************************************************/
bool ES_PostList00(ES_Event_t NewEvent)
{
  return PostToList(DistList00, ARRAY_SIZE(DistList00), NewEvent);
}

#if NUM_DIST_LISTS > 1
bool ES_PostList01(ES_Event_t NewEvent)
{
  return PostToList(DistList01, ARRAY_SIZE(DistList01), NewEvent);
}

#endif

#if NUM_DIST_LISTS > 2
bool ES_PostList02(ES_Event_t NewEvent)
{
  return PostToList(DistList02, ARRAY_SIZE(DistList02), NewEvent);
}

#endif

#if NUM_DIST_LISTS > 3
bool ES_PostList03(ES_Event_t NewEvent)
{
  return PostToList(DistList03, ARRAY_SIZE(DistList03), NewEvent);
}

#endif

#if NUM_DIST_LISTS > 4
bool ES_PostList04(ES_Event_t NewEvent)
{
  return PostToList(DistList04, ARRAY_SIZE(DistList04), NewEvent);
}

#endif

#if NUM_DIST_LISTS > 5
bool ES_PostList05(ES_Event_t NewEvent)
{
  return PostToList(DistList05, ARRAY_SIZE(DistList05), NewEvent);
}

#endif

#if NUM_DIST_LISTS > 6
bool ES_PostList06(ES_Event_t NewEvent)
{
  return PostToList(DistList06, ARRAY_SIZE(DistList06), NewEvent);
}

#endif

#if NUM_DIST_LISTS > 7
bool ES_PostList07(ES_Event_t NewEvent)
{
  return PostToList(DistList07, ARRAY_SIZE(DistList07), NewEvent);
}

#endif

// Implementations for private functions
/****************************************************************************
 Function
   PostToList
 Parameters
   PostFunc *const*List : pointer to the list of posting functions
   unsigned char ListSize : number of elements in the list array
   EF_Event NewEvent : the new event to be passed to each of the state machine
   posting functions in the list
 Returns
   bool: true if all the post functions succeeded, false if any failed
 Description
   Posts NewEvent to all of the state machines listed in the list
 Notes

 Author
   J. Edward Carryer, 10/24/11, 07:52
****************************************************************************/
static bool PostToList(PostFunc_t *const *List, uint8_t ListSize, ES_Event_t NewEvent)
{
  uint8_t i;
  // loop through the list executing the post functions
  for (i = 0; i < ListSize; i++)
  {
    if (List[i](NewEvent) != true)
    {
      break; // this is a failed post
    }
  }
  if (i != ListSize)   // if no failures, i = ListSize
  {
    return false;
  }
  else
  {
    return true;
  }
}

#endif /* NUM_DIST_LISTS > 0*/

/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/
