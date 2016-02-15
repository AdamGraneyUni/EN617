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

/*************************************************************************
*                  PRIORITIES
*************************************************************************/

enum {
  APP_TASK_MONITOR_SENS_PRIO = 4
};

/*************************************************************************
*                  APPLICATION TASK STACKS
*************************************************************************/

enum {
  APP_TASK_MONITOR_SENS_STK_SIZE = 256
};

static OS_STK appTaskMonitorSensStk[APP_TASK_MONITOR_SENS_STK_SIZE];

/*************************************************************************
*                  APPLICATION FUNCTION PROTOTYPES
*************************************************************************/

static void appTaskMonitorSens(void *pdata);
static void canSend(uint32_t id);
static void canHandler(void);


static canMessage_t can1RxBuf;
bool checkForInputBlock = false;

/*************************************************************************
*                    GLOBAL FUNCTION DEFINITIONS
*************************************************************************/

int main() {
  /* Initialise the hardware */
  bspInit();
  conveyorInit();
 
  /* Initialise the OS */
  OSInit();                                                   

  /* Create Tasks */
  OSTaskCreate(appTaskMonitorSens,                               
               (void *)0,
               (OS_STK *)&appTaskMonitorSensStk[APP_TASK_MONITOR_SENS_STK_SIZE - 1],
               APP_TASK_MONITOR_SENS_PRIO);
 
   
  /* Start the OS */
  OSStart();                                                  
  
  /* Should never arrive here */ 
  return 0;      
}

/*************************************************************************
*                   APPLICATION TASK DEFINITIONS
*************************************************************************/

static void appTaskMonitorSens(void *pdata) {
    
  /* Start the OS ticker
   * (must be done in the highest priority task)
   */
  osStartTick();
  canRxInterrupt(canHandler);
  /* 
   * Now execute the main task loop for this task
   */
  while (true) {
    
    if (conveyorItemPresent(CONVEYOR_SENSOR_2) && checkForInputBlock) {
        OSTimeDlyHMSM(0,0,2,0);
        if (conveyorItemPresent(CONVEYOR_SENSOR_2)){
          conveyorSetState(CONVEYOR_REVERSE);
        }
    } 
    if (conveyorItemPresent(CONVEYOR_SENSOR_1)) {
        conveyorSetState(CONVEYOR_OFF);
        canSend(0x04);
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
    if (msg.id == 0x02){
       checkForInputBlock = true;
    } 
  }
}
