/* Test Robot
 * Joystick UP    -> inc current joint coordinate
 * Joystick DOWN  -> dec current joint coordinate
 * Joystick RIGHT -> cycle joint selection HAND -> WRIST -> ELBOW -> WAIST
 * Joystick left  -> cycle joint selection HAND <- WRIST <- ELBOW <- WAIST
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
bool emergencyStop = false;
bool paused = false;
bool blockPickedUp = false;
bool stopped = true;
bool conveyorOccupied = false;
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

  /* Emergency stop task */
  OSTaskCreate(appTaskEmergencyStop,                               
               (void *)0,
               (OS_STK *)&appTaskEmergencyStopStk[APP_TASK_EMERGENCY_STOP_STK_SIZE - 1],
               APP_TASK_EMERGENCY_STOP_PRIO);
			   
  /* Buttons Task  */
  OSTaskCreate(appTaskButtons,                               
               (void *)0,
               (OS_STK *)&appTaskButtonsStk[APP_TASK_BUTTONS_STK_SIZE - 1],
               APP_TASK_BUTTONS_PRIO);
  
  /* Move Block Task */
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
    }else if (stopped){
       ledToggle(USB_CONNECT_LED);
    }else {
       OSTimeDlyHMSM(0,0,0,500);
    }
  }
}

static void appTaskButtons(void *pdata) {
  
    
  /* the main task loop for this task  */
  while (true) {
   OSTimeDly(20);                    
  }
}

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

static void appTaskMoveBlock(void *pdata){
  while (true){
    if (!conveyorOccupied){
      while(moveBlock){
           if (pickupAttempts == 2){
              canSend(EMERGENCY_STOP);
              emergencyStop=true;
           }
          //Waist to Neutral
          moveJoint(ROBOT_WAIST, 67250);
          //Wrist to Neutral
          moveJoint(ROBOT_WRIST, 82250);
          //Elbow to Neutral
          moveJoint(ROBOT_ELBOW, 74000);
          //HAND to Neutral
          moveJoint(ROBOT_HAND, 45000);
          //Waist to Pad
          pickupAttempts++;
          moveJoint(ROBOT_WAIST, 84000);
          //Wrist to Pad
          moveJoint(ROBOT_WRIST, 64250);
          //Elbow to Pad
          moveJoint(ROBOT_ELBOW, 100000);
          //HAND to Pad
          moveJoint(ROBOT_HAND, 75000);
          //Elbow to Safe
          moveJoint(ROBOT_ELBOW, 85000);
          //Waist to Conveyer
          moveJoint(ROBOT_WAIST, 45500);
          //Wrist to Conveyer
          moveJoint(ROBOT_WRIST, 74250);
          //Elbow to Conveyer
          moveJoint(ROBOT_ELBOW, 94500);
          //HAND to Conveyer
          moveJoint(ROBOT_HAND, 45000);
          //Elbow to Safe
          moveJoint(ROBOT_ELBOW, 73000);
          //Waist to Neutral
          moveJoint(ROBOT_WAIST, 67250);
          //Wrist to Neutral
          moveJoint(ROBOT_WRIST, 82250);
          //Elbow to Neutral
          moveJoint(ROBOT_ELBOW, 74000);
          //HAND to Neutral
          moveJoint(ROBOT_HAND, 45000);
      }
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

      if (msg.id == READY_TO_PICKUP_PAD1){
         moveBlock = true;
      } 
      if (msg.id == PICKED_UP_PAD1){
        moveBlock = false;
        pickupAttempts = 0;
      }  
      if(msg.id == EMERGENCY_STOP)
      {
        emergencyStop = true;
      }
      if(msg.id == RESET)
      {
        __iar_program_start(); //Reset
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
      }if(msg.id == CONVEYOR_OCCUPIED)
      {
        conveyorOccupied = true;
      }
      if(msg.id == CONVEYOR_FREE)
      {
        conveyorOccupied = false;
      }
  }
}

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