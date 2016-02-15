/* Control Panel Test
 * Emergency stop -> toggle Alarm State
 *   Data LED 3 lit while emergency stop button pressed
 *
 * Pedestal sensor 1 reported  on interface LED 1 and LINK LED
 * Pedestal sensor 2 reported  on interface LED 2 and CONNECT LED
 */

#include <stdbool.h>
#include <ucos_ii.h>
#include <bsp.h>
#include <osutils.h>
#include <leds.h>
#include <buttons.h>
#include <interface.h>
#include <control.h>
#include <can.h>

/*************************************************************************
*                  PRIORITIES
*************************************************************************/

enum {
  APP_TASK_MONITOR_SENS_PRIO = 4,
  APP_TASK_CTRL_PRIO
};

/*************************************************************************
*                     APPLICATION TASK STACKS
*************************************************************************/

enum {
  APP_TASK_MONITOR_SENS_STK_SIZE = 512,
  APP_TASK_CTRL_STK_SIZE = 512,
};

static OS_STK appTaskMonitorSensStk[APP_TASK_MONITOR_SENS_STK_SIZE];
static OS_STK appTaskCtrlStk[APP_TASK_CTRL_STK_SIZE];

/*************************************************************************
*                  APPLICATION FUNCTION PROTOTYPES
*************************************************************************/

static void appTaskMonitorSens(void *pdata);
static void appTaskCtrl(void *pdata);
static void canSend(uint32_t id);

/*************************************************************************
*                    GLOBAL FUNCTION DEFINITIONS
*************************************************************************/

int main() {
  /* Initialise the hardware */
  bspInit();
  controlInit();

  /* Initialise the OS */
  OSInit();                                                   

  /* Create Tasks */
  OSTaskCreate(appTaskMonitorSens,                               
               (void *)0,
               (OS_STK *)&appTaskMonitorSensStk[APP_TASK_MONITOR_SENS_STK_SIZE - 1],
               APP_TASK_MONITOR_SENS_PRIO);
   
  OSTaskCreate(appTaskCtrl,                               
               (void *)0,
               (OS_STK *)&appTaskCtrlStk[APP_TASK_CTRL_STK_SIZE - 1],
               APP_TASK_CTRL_PRIO);
   
  /* Start the OS */
  OSStart();                                                  
  
  /* Should never arrive here */ 
  return 0;      
}

/*************************************************************************
*                   APPLICATION TASK DEFINITIONS
*************************************************************************/

static void appTaskMonitorSens(void *pdata) {
  bool paused = false;
  /* Start the OS ticker
   * (must be done in the highest priority task)
   */
  osStartTick();
  
  /* 
   * Now execute the main task loop for this task
   */
  while (true) {
    ledSetState(USB_LINK_LED, LED_OFF);
    ledSetState(USB_CONNECT_LED, LED_OFF);
    if(isButtonPressed(JS_CENTRE))
    {
      if(paused)
      {
        canSend(0x0A);
      }
      else
      {
        canSend(0x9);
        paused = 1;
      }
    }
    
    if (controlItemPresent(CONTROL_SENSOR_1)) {
        ledSetState(USB_LINK_LED, LED_ON);
        canSend(0x01);
        
        while (controlItemPresent(CONTROL_SENSOR_1)){
          OSTimeDlyHMSM(0,0,0,10);
        }
        canSend(0x02);
    } 
    if (controlItemPresent(CONTROL_SENSOR_2)) {
        ledSetState(USB_CONNECT_LED, LED_ON);
        while (controlItemPresent(CONTROL_SENSOR_2)){
          OSTimeDlyHMSM(0,0,0,10);
        }
        canSend(0x06);
    } 
    
    OSTimeDly(20);
  }
}

static void appTaskCtrl(void *pdata) {
  static bool emergency = false;
  static bool running = false;
  uint32_t btnState;
  interfaceLedSetState(D1_LED | D2_LED | D3_LED | D4_LED, LED_OFF);
  
  while (true) {
    //Emergency/Error Control
    emergency = controlEmergencyStopButtonPressed();
    if (emergency) {
      controlAlarmToggleState();
      interfaceLedSetState(D3_LED, LED_ON);
      canSend(0x08); //Send emergency stop.
      while (controlEmergencyStopButtonPressed()) {
        OSTimeDly(20);
      }
    } else {
      interfaceLedSetState(D3_LED, LED_OFF);
      canSend(0xB);
    }
    OSTimeDly(20);
  } 
}

static void canSend(uint32_t id) {
  canMessage_t msg = {0, 0, 0, 0};
  bool txOk = false;
    
  /* Start the OS ticker
   * (must be done in the highest priority task)
   */
  osStartTick();

  /* Initialise the CAN message structure */
  msg.id = id;  // arbitrary CAN message id
  msg.len = 4;    // data length 4
  msg.dataA = 0;
  msg.dataB = 0;
  
    // Transmit message on CAN 1
    txOk = canWrite(CAN_PORT_1, &msg);
  }

