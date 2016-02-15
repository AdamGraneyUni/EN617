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

#define BUTTONS_TASK_ID 0
#define N_JOINTS 4

/***************************************************************************
*                       PRIORITIES
***************************************************************************/

enum {
  APP_TASK_BUTTONS_PRIO = 4,
  APP_TASK_MOVE_BLOCK_PRIO = 5 
};

/****************************************************************************
*                  APPLICATION TASK STACKS
****************************************************************************/

enum {
  APP_TASK_BUTTONS_STK_SIZE = 256,
  APP_TASK_MOVE_BLOCK_STK_SIZE = 256
};

static OS_STK appTaskButtonsStk[APP_TASK_BUTTONS_STK_SIZE];
static OS_STK appTaskMoveBlockStk[APP_TASK_MOVE_BLOCK_STK_SIZE];

/*****************************************************************************
*                APPLICATION FUNCTION PROTOTYPES
*****************************************************************************/

static void appTaskButtons(void *pdata);
static void appTaskMoveBlock(void *pdata);
static void moveJoint(robotJoint_t joint, int position);
static void canHandler(void);

static canMessage_t can1RxBuf;
bool moveBlock = false;

/*****************************************************************************
*                        GLOBAL FUNCTION DEFINITIONS
*****************************************************************************/

int main() {
  /* Initialise the hardware */
  bspInit();
  robotInit();

  /* Initialise the OS */
  OSInit();                                                   

  /* Create Tasks */
  OSTaskCreate(appTaskButtons,                               
               (void *)0,
               (OS_STK *)&appTaskButtonsStk[APP_TASK_BUTTONS_STK_SIZE - 1],
               APP_TASK_BUTTONS_PRIO);
  
  /* Create Tasks */
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


static void appTaskButtons(void *pdata) {
  /* Start the OS ticker (highest priority task) */
  osStartTick();
  canRxInterrupt(canHandler);
  
  
  
  
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
      while(moveBlock){
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
          //Wrist to Convey
          moveJoint(ROBOT_WRIST, 64500);
          //Elbow to Convey
          moveJoint(ROBOT_ELBOW, 94250);
          //HAND to Convey
          moveJoint(ROBOT_HAND, 80000); 
          //Elbow to Neutral
          moveJoint(ROBOT_ELBOW, 72500);
          //Waist to Pad
          moveJoint(ROBOT_WAIST, 84250);
          //Wrist to Pad
          moveJoint(ROBOT_WRIST, 89000);
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
    if (msg.id == 0x04){
       moveBlock = true;
    }
  }
}
//Waist 00086750
//Elbow 00081750
//Wrist 00085250
//CLAAWW 00058750