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
#include <lcd.h>

/*************************************************************************
*                  PRIORITIES
*************************************************************************/

enum {
  APP_TASK_EMERGENCY_STOP_PRIO = 4,
  APP_TASK_MONITOR_SENS1_PRIO,
  APP_TASK_MONITOR_SENS2_PRIO,
  APP_TASK_MONITOR_BUTTON_PRIO,
  APP_TASK_CTRL_PRIO,
  APP_TASK_DISPLAY_PRIO
};

/*************************************************************************
*                     APPLICATION TASK STACKS
*************************************************************************/

enum {
  APP_TASK_EMERGENCY_STOP_STK_SIZE = 256,
  APP_TASK_MONITOR_SENS1_STK_SIZE = 512,
  APP_TASK_MONITOR_SENS2_STK_SIZE = 512,
  APP_TASK_CTRL_STK_SIZE = 512,
  APP_TASK_MONITOR_BUTTON_STK_SIZE = 256,
  APP_TASK_DISPLAY_STK_SIZE = 256,
};

static OS_STK appTaskMonitorButtonStk[APP_TASK_MONITOR_BUTTON_STK_SIZE];
static OS_STK appTaskMonitorSens1Stk[APP_TASK_MONITOR_SENS1_STK_SIZE];
static OS_STK appTaskMonitorSens2Stk[APP_TASK_MONITOR_SENS2_STK_SIZE];
static OS_STK appTaskCtrlStk[APP_TASK_CTRL_STK_SIZE];
static OS_STK appTaskDisplayStk[APP_TASK_DISPLAY_STK_SIZE];
static OS_STK appTaskEmergencyStopStk[APP_TASK_EMERGENCY_STOP_STK_SIZE];
/*************************************************************************
*                  APPLICATION FUNCTION PROTOTYPES
*************************************************************************/

static void appTaskEmergencyStop(void *pdata);
static void appTaskDisplay(void *pdata);
static void appTaskMonitorButton(void *pdata);
static void appTaskMonitorSens1(void *pdata);
static void appTaskMonitorSens2(void *pdata);
static void appTaskCtrl(void *pdata);
static void canSend(uint32_t id);
static void canHandler(void);
static canMessage_t can1RxBuf;

extern void __iar_program_start(void);
bool paused = false;
bool waitForPad2Clear = false;
bool emergencyStop = false;
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
   
  OSTaskCreate(appTaskCtrl,                               
               (void *)0,
               (OS_STK *)&appTaskCtrlStk[APP_TASK_CTRL_STK_SIZE - 1],
               APP_TASK_CTRL_PRIO);
   
  OSTaskCreate(appTaskMonitorButton,                               
               (void *)0,
               (OS_STK *)&appTaskMonitorButtonStk[APP_TASK_MONITOR_BUTTON_STK_SIZE - 1],
               APP_TASK_MONITOR_BUTTON_PRIO);
  
  OSTaskCreate(appTaskDisplay,                               
               (void *)0,
               (OS_STK *)&appTaskDisplayStk[APP_TASK_DISPLAY_STK_SIZE - 1],
               APP_TASK_DISPLAY_PRIO);
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
  controlAlarmSetState(CONTROL_ALARM_OFF);
  while(true)
  {
    if(emergencyStop) //Never sleep if we're stopped to prevent other tasks from running.
    {
      if(controlEmergencyStopButtonPressed())
      {
        canSend(0x0B);
        __iar_program_start();
      }
    }
    else
    {
      OSTimeDlyHMSM(0,0,0,500);
    }
  }
}
static void appTaskMonitorButton(void *pdata)
{
  while(true)
  {
    if(isButtonPressed(JS_CENTRE))
    {
      while(isButtonPressed(JS_CENTRE)) {}
      if(paused == false)
      {
        canSend(0x9);
        paused = true;
        lcdSetTextPos(1,1);
        lcdWrite("Paused: True    ");
      }
      else
      {
        paused = false;
        canSend(0xA);
      }
    }
    
    if(controlEmergencyStopButtonPressed())
    {
      controlAlarmSetState(CONTROL_ALARM_ON);
      canSend(0x08);
      lcdSetTextPos(1,2);
      if(!emergencyStop)
      {
        lcdWrite("Emerg Stop: True    ");
      }
      emergencyStop = true;
      
    }
    OSTimeDlyHMSM(0,0,0,5);
  }
}
static void appTaskMonitorSens2(void *pdata) {
  while(true)
  {
    ledSetState(USB_LINK_LED, LED_OFF);
    if (controlItemPresent(CONTROL_SENSOR_1)) {
        ledSetState(USB_LINK_LED, LED_ON);
        canSend(0x01);
        
        while (controlItemPresent(CONTROL_SENSOR_1)){
          OSTimeDlyHMSM(0,0,0,20);
        }
        canSend(0x02);
    } 
    OSTimeDlyHMSM(0,0,0,20);
  }
}
static void appTaskMonitorSens1(void *pdata) {
  canRxInterrupt(canHandler);
  /* 
   * Now execute the main task loop for this task
   */
  while (true) {
    ledSetState(USB_CONNECT_LED, LED_OFF);

    while(waitForPad2Clear)
    {
      if (controlItemPresent(CONTROL_SENSOR_2)) {
        OSTimeDlyHMSM(0,0,0,20);
      }
      else
      {
        canSend(0x06);
        waitForPad2Clear = false;
      }
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
      controlAlarmSetState(CONTROL_ALARM_ON);
      interfaceLedSetState(D3_LED, LED_ON);
      while (controlEmergencyStopButtonPressed()) {
        OSTimeDly(20);
      }
    } else {
      interfaceLedSetState(D3_LED, LED_OFF);
      controlAlarmSetState(CONTROL_ALARM_OFF);
    }
    OSTimeDly(20);
  } 
}
static void appTaskDisplay(void *pdata)
{
  while(true)
  {
    lcdSetTextPos(1,1);
    if(paused)
    {
      lcdWrite("Paused: True     ");
    }
    else
    {
      lcdWrite("Paused: False    ");
    }
    lcdSetTextPos(1,2);
    if(!emergencyStop)
    {
      lcdWrite("Emerg Stop: False");
    }
    OSTimeDlyHMSM(0,0,0,500);
  }
}
static void canSend(uint32_t id) {
  canMessage_t msg = {0, 0, 0, 0};
  bool txOk = false;
  
  /* Initialise the CAN message structure */
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
    if(msg.id == 0x05)
    {
      waitForPad2Clear = true;
    }
  }
}
