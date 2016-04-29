/* Conveyor Test
 * JS UP -> forward
 *    DN -> reverse
 *    L,R -> stop
 * 
 * Conveyor sensor 1 reported  on interface LED 1 and LINK LED
 * Conveyor sensor 2 reported  on interface LED 2 and CONNECT LED
 *
 * item sensed at 1 and moving in reverse -> stop
 * item sensed at 2 and moving forward -> stop
 */

#include <stdbool.h>
#include <ucos_ii.h>
#include <bsp.h>
#include <osutils.h>
#include <leds.h>
#include <buttons.h>
#include <interface.h>
#include <conveyor.h>
#include <can.h>
#include <can_messages.h>

/*************************************************************************
*                  PRIORITIES
*************************************************************************/

enum {
  APP_TASK_EMERGENCY_STOP_PRIO = 4,
  APP_TASK_MONITOR_SENS2_PRIO = 8,
  APP_TASK_MONITOR_SENS1_PRIO
};

/*************************************************************************
*                  APPLICATION TASK STACKS
*************************************************************************/

enum {
  APP_TASK_EMERGENCY_STOP_STK_SIZE = 256,
  APP_TASK_MONITOR_SENS1_STK_SIZE = 256,
  APP_TASK_MONITOR_SENS2_STK_SIZE = 256
};

static OS_STK appTaskEmergencyStopStk[APP_TASK_EMERGENCY_STOP_STK_SIZE];
static OS_STK appTaskMonitorSens1Stk[APP_TASK_MONITOR_SENS1_STK_SIZE];
static OS_STK appTaskMonitorSens2Stk[APP_TASK_MONITOR_SENS2_STK_SIZE];

/*************************************************************************
*                  APPLICATION FUNCTION PROTOTYPES
*************************************************************************/

static void appTaskMonitorSens1(void *pdata);
static void appTaskMonitorSens2(void *pdata);
static void appTaskEmergencyStop(void *pdata);
static void canSend(uint32_t id);
static void canHandler(void);


static canMessage_t can1RxBuf;
bool checkForInputBlock = false;
bool stopConveyorForLoad = false;
int noBlocks = 0;
bool emergencyStop = false;
bool paused = false;
bool stopped = true;
conveyorState_t pausedState;

extern void __iar_program_start(void);

/*************************************************************************
*                    GLOBAL FUNCTION DEFINITIONS
******************************************
*******************************/

int main() {
  /* Initialise the hardware */
  bspInit();
  conveyorInit();
 
  /* Initialise the OS */
  OSInit();                                                   

  /* Create Tasks */
  OSTaskCreate(appTaskEmergencyStop,                               
               (void *)0,
               (OS_STK *)&appTaskEmergencyStopStk[APP_TASK_EMERGENCY_STOP_STK_SIZE - 1],
               APP_TASK_EMERGENCY_STOP_PRIO);
  OSTaskCreate(appTaskMonitorSens1,                               
               (void *)0,
               (OS_STK *)&appTaskMonitorSens1Stk[APP_TASK_MONITOR_SENS1_STK_SIZE - 1],
               APP_TASK_MONITOR_SENS1_PRIO);
  OSTaskCreate(appTaskMonitorSens2,                               
               (void *)0,
               (OS_STK *)&appTaskMonitorSens2Stk[APP_TASK_MONITOR_SENS2_STK_SIZE - 1],
               APP_TASK_MONITOR_SENS2_PRIO);
 
   
  /* Start the OS */
  OSStart();                                                  
  
  /* Should never arrive here */ 
  return 0;      
}

/*************************************************************************
*                   APPLICATION TASK DEFINITIONS
*************************************************************************/
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
      conveyorSetState(CONVEYOR_OFF);
    } else if(paused){ 
      pausedState = conveyorGetState();
      conveyorSetState(CONVEYOR_OFF);
    } else if (stopped){
      ledToggle(USB_CONNECT_LED);
    }
    else{
      OSTimeDlyHMSM(0,0,0,500);
    }
    
  }
}

static void appTaskMonitorSens1(void *pdata) {
   while (true) { 
    if (conveyorItemPresent(CONVEYOR_SENSOR_1)) {
        conveyorSetState(CONVEYOR_OFF);
        canSend(READY_TO_PICKUP_CONVEYOR);
        while(conveyorItemPresent(CONVEYOR_SENSOR_1)){
          canSend(READY_TO_PICKUP_CONVEYOR);
          OSTimeDly(20);
        }
        canSend(PICKED_UP_CONVEYOR);
        noBlocks -= 1;
    }
    if (!conveyorItemPresent(CONVEYOR_SENSOR_1) && (noBlocks > 0) && !stopConveyorForLoad){
      conveyorSetState(CONVEYOR_REVERSE);
    }
    
    OSTimeDly(20);
  }
  
}
static void appTaskMonitorSens2(void *pdata) {
  /* 
   * Now execute the main task loop for this task
   */
  while (true) {
    if (conveyorItemPresent(CONVEYOR_SENSOR_2)){
      canSend(CONVEYOR_OCCUPIED);
    }
    if (!conveyorItemPresent(CONVEYOR_SENSOR_2)){
      canSend(CONVEYOR_FREE);
    }
    if (conveyorItemPresent(CONVEYOR_SENSOR_2) && checkForInputBlock) {
        OSTimeDlyHMSM(0,0,2,0);
        if (conveyorItemPresent(CONVEYOR_SENSOR_2)){
          checkForInputBlock = false;
          stopConveyorForLoad = false;
          noBlocks += 1;
          if (!conveyorItemPresent(CONVEYOR_SENSOR_1)){
            conveyorSetState(CONVEYOR_REVERSE);
          }
        }
    }
    if (stopConveyorForLoad){
      conveyorSetState(CONVEYOR_OFF);
    }
    OSTimeDly(20);
  }
}

static void canSend(uint32_t id) {
  canMessage_t msg = {0, 0, 0, 0};   
  bool txOk = false;        
  
  msg.id = id;  // arbitrary CAN message id   
  msg.len = 4;    // data length 4   
  msg.dataA = 0;   
  msg.dataB = 0;        
  // Transmit message on CAN 1     
  txOk = canWrite(CAN_PORT_1, &msg);  
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
    if (msg.id == PICKED_UP_PAD1){
       checkForInputBlock = true;
       stopConveyorForLoad = true;
    }    
    if(msg.id == EMERGENCY_STOP){
      emergencyStop = true;
    }
    if(msg.id == PAUSE){
      paused = true;
    }
    if(msg.id == RESUME){
      paused = false;
      conveyorSetState(pausedState);
    }
    if(msg.id == RESET){
      __iar_program_start();
    }
    if(msg.id == START){
      stopped = false;
    }
    if(msg.id == STOP){
      stopped = true;
    }
    
  }
}
