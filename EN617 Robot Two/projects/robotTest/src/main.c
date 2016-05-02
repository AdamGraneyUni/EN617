/* EN617 Robot 2 Implementation
 * Adam Graney - 12009005
 * ----------------------------
 * Robot 2 implementation for the picking up of blocks
 * from the conveyor and placing them on pad 2.
 * 
 */
#include <stdbool.h>
#include <ucos_ii.h>
#include <bsp.h>
#include <osutils.h>
#include <leds.h>
#include <buttons.h>
#include <lcd.h>
#include <interface.h>
#include <robot.h>
#include <can.h>
#include <can_messages.h>

#define BUTTONS_TASK_ID 0
#define N_JOINTS 4

/***************************************************************************
*                       PRIORITIES
***************************************************************************/

enum {
  APP_TASK_EMERGENCY_STOP_PRIO = 4,
  APP_TASK_BUTTONS_PRIO = 5,
  APP_TASK_MOVE_BLOCK_PRIO = 6 
};

/****************************************************************************
*                  APPLICATION TASK STACKS
****************************************************************************/

enum {
  APP_TASK_EMERGENCY_STOP_STK_SIZE = 256,
  APP_TASK_BUTTONS_STK_SIZE = 256,
  APP_TASK_MOVE_BLOCK_STK_SIZE = 256
};

static OS_STK appTaskEmergencyStopStk[APP_TASK_EMERGENCY_STOP_STK_SIZE];
static OS_STK appTaskButtonsStk[APP_TASK_BUTTONS_STK_SIZE];
static OS_STK appTaskMoveBlockStk[APP_TASK_MOVE_BLOCK_STK_SIZE];

/*****************************************************************************
*                APPLICATION FUNCTION PROTOTYPES
*****************************************************************************/

static void appTaskEmergencyStop(void *pdata);
static void appTaskButtons(void *pdata);
static void appTaskMoveBlock(void *pdata);
static void moveJoint(robotJoint_t joint, int position);
static void canHandler(void);
static void canSend(uint32_t id);

static canMessage_t can1RxBuf;
bool moveBlock = false;
bool padClear = false;
bool paused = false;
bool emergencyStop = false;
bool stopped = true;
int pickupAttempts = 0;
extern void __iar_program_start(void);
/*****************************************************************************
*                        GLOBAL FUNCTION DEFINITIONS
*****************************************************************************/

int main() { 
  /* Initialise the hardware */
  bspInit();
  robotInit();

  /* Initialise the OS */
  OSInit();  
  
    
  /* Create emergency stop task */
  OSTaskCreate(appTaskEmergencyStop,                               
               (void *)0,
               (OS_STK *)&appTaskEmergencyStopStk[APP_TASK_EMERGENCY_STOP_STK_SIZE - 1],
               APP_TASK_EMERGENCY_STOP_PRIO);

  /* Create buttons task, used when testing to get robot positions */
  OSTaskCreate(appTaskButtons,                               
               (void *)0,
               (OS_STK *)&appTaskButtonsStk[APP_TASK_BUTTONS_STK_SIZE - 1],
               APP_TASK_BUTTONS_PRIO);
  
  /* Create move block task for the moving of the block from conveyor to pad 2 */
  OSTaskCreate(appTaskMoveBlock,                               
               (void *)0,
               (OS_STK *)&appTaskMoveBlockStk[APP_TASK_MOVE_BLOCK_STK_SIZE - 1],
               APP_TASK_MOVE_BLOCK_PRIO); 

  /* Start the OS */
  OSStart();                                                  
  
  /* Should never arrive here */ 
  return 0;      
}

/**************************************************************************
*                             APPLICATION TASK DEFINITIONS
****************************************************************************/
/*
 * appTaskEmergencyStop - Emergency stop task
 * This task handles the emergency stop and stop tasks in
 * the robot 2 subsystem.
 */
static void appTaskEmergencyStop(void *pdata)
{
   /* Start the OS ticker
   * (must be done in the highest priority task)
   */
  osStartTick();
  canRxInterrupt(canHandler);
  while(true)
  {
    if(emergencyStop)
    {
        ledToggle(USB_LINK_LED);
    } else if (stopped){
        ledToggle(USB_CONNECT_LED);
    } else {
        OSTimeDlyHMSM(0,0,0,500);
    }
  }
}

/*
 * appTaskButtons - Buttons task used for testing
 * The app task buttons task was used for the testing of the robot,
 * when deciding what position the robot should move too. This task
 * is not used for the final implementation of the system.
 */
static void appTaskButtons(void *pdata) {
  /* Removed task contents as not used in implementation */
  while (true) {
   OSTimeDly(20);                    
  }
}

/*
 * moveJoint - Function used for the moving of the robot joints
 * The function move joint is used to avoid code duplication when moving the joints
 * of the robot to there required position. The function checks the current position,
 * comparing it to the desired desitnation and either increments or decrements the 
 * position depending on the position read.
 */
static void moveJoint(robotJoint_t joint, int position){
  while ((robotJointGetState(joint) < position) || (robotJointGetState(joint) > position)){
        if(robotJointGetState(joint) < position){
          robotJointSetState(joint, ROBOT_JOINT_POS_INC);
          OSTimeDly(10);
        }  
        else if(robotJointGetState(joint) > position){
          robotJointSetState(joint, ROBOT_JOINT_POS_DEC);
          OSTimeDly(10);
        }
      }
}

/*
 * appTaskMoveBlock - Task for moving the block from the conveyor to the pad.
 * The task move block is used for the process of moving the block
 * from the conveyor to the pad. The task works by waiting for a block to be on
 * the conveyor end sensor and when it is true it runs. 
 *
 * The task picks up the block and moves it using the moveJoint function to the required
 * position. The paused functionality works by pausing the system after the move is complete
 * as indicated in the system specification. 
 *
 * The pickUpAttempts function checks for a block failing to be picked up. If this
 * fails twice then the full system will emergency stop.
 */
static void appTaskMoveBlock(void *pdata){
  while (true){
      while(moveBlock){
          if (pickupAttempts == 2){
              canSend(EMERGENCY_STOP);
              emergencyStop=true;
           }
          canSend(QUERY_PAD2_STATUS); //Send a message asking if the PAD is clear.
          while(!padClear)
          {
            OSTimeDlyHMSM(0,0,0,10);
          }
          padClear = false;
          //Waist to Neutral
          moveJoint(ROBOT_WAIST, 67250);
          //Elbow to Neutral
          moveJoint(ROBOT_ELBOW, 71750);
          //Wrist to Neutral
          moveJoint(ROBOT_WRIST, 89000);
          //HAND to Neutral
          moveJoint(ROBOT_HAND, 45000);
          //Waist to Convey
          moveJoint(ROBOT_WAIST, 47500);
          pickupAttempts++;
          //Wrist to Convey
          moveJoint(ROBOT_WRIST, 64500);
          //Elbow to Convey
          moveJoint(ROBOT_ELBOW, 94500);
          //HAND to Convey
          moveJoint(ROBOT_HAND, 80000); 
          //Elbow to Neutral
          moveJoint(ROBOT_ELBOW, 72500);
          //Waist to Pad
          moveJoint(ROBOT_WAIST, 84250);
          //Wrist to Pad
          moveJoint(ROBOT_WRIST, 95000);
          //Elbow to Pad
          moveJoint(ROBOT_ELBOW, 81250);
          //HAND to Pad
          moveJoint(ROBOT_HAND, 45000);
          //Elbow to Neutral
          moveJoint(ROBOT_ELBOW, 71750);
          //Wrist to Neutral
          moveJoint(ROBOT_WRIST, 89000);
          //HAND to Neutral
          moveJoint(ROBOT_HAND, 45000);
          //Waist to Neutral
          moveJoint(ROBOT_WAIST, 67250);
          moveBlock = false;
          canSend(OUTPUT_ROBOT_FINISHED);
      }
      
    while(paused)
    {
      OSTimeDlyHMSM(0,0,0,500);
    }
    OSTimeDly(20);
  }
}

/*
 * A simple interrupt handler for CAN message reception on CAN1
 */
static void canHandler(void) {
  if (canReady(CAN_PORT_1)) {
    interfaceLedToggle(D1_LED);
    canRead(CAN_PORT_1, &can1RxBuf);
    canMessage_t msg;
    msg = can1RxBuf;
      if (msg.id == READY_TO_PICKUP_CONVEYOR){
         moveBlock = true;
      }
      if(msg.id == PAD2_CLEAR)
      {
        padClear = true;
      }
      if(msg.id == PICKED_UP_CONVEYOR)
      {
        pickupAttempts = 0;
      }
      if(msg.id == EMERGENCY_STOP)
      {
        emergencyStop = true;
      }
      if(msg.id == RESET)
      {
        __iar_program_start();
      }
      if(msg.id == PAUSE)
      {
        paused = true;
      }
      if(msg.id == RESUME)
      {
        paused = false;
      }
      if(msg.id == START)
      {
        stopped = false;
      }
      if(msg.id == STOP)
      {
        stopped = true;
      }
      
  }
}

/*
 * A can send function for the sending of can messages 
 * on the can bus.
 */
static void canSend(uint32_t id) {
  canMessage_t msg = {0, 0, 0, 0};

  /* Initialise the CAN message structure */
  msg.id = id;  // arbitrary CAN message id
  msg.len = 4;    // data length 4
  msg.dataA = 0;
  msg.dataB = 0;
  
    // Transmit message on CAN 1
    canWrite(CAN_PORT_1, &msg);
}
