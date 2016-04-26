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
#include <delay.h>
#include "can_messages.h"
/*************************************************************************
*                  PRIORITIES
*************************************************************************/

enum {
  APP_TASK_EMERGENCY_STOP_PRIO = 4,
  APP_TASK_MONITOR_SENS1_PRIO,
  APP_TASK_MONITOR_SENS2_PRIO,
  APP_TASK_MONITOR_BUTTON_PRIO,
  APP_TASK_DISPLAY_PRIO,
  APP_TASK_LEDS_PRIO
};

/*************************************************************************
*                     APPLICATION TASK STACKS
*************************************************************************/

enum {
  APP_TASK_EMERGENCY_STOP_STK_SIZE = 256,
  APP_TASK_MONITOR_SENS1_STK_SIZE = 512,
  APP_TASK_MONITOR_SENS2_STK_SIZE = 512,
  APP_TASK_MONITOR_BUTTON_STK_SIZE = 256,
  APP_TASK_DISPLAY_STK_SIZE = 256,
  APP_TASK_LEDS_STK_SIZE = 256
};

static OS_STK appTaskMonitorButtonStk[APP_TASK_MONITOR_BUTTON_STK_SIZE];
static OS_STK appTaskMonitorSens1Stk[APP_TASK_MONITOR_SENS1_STK_SIZE];
static OS_STK appTaskMonitorSens2Stk[APP_TASK_MONITOR_SENS2_STK_SIZE];
static OS_STK appTaskDisplayStk[APP_TASK_DISPLAY_STK_SIZE];
static OS_STK appTaskEmergencyStopStk[APP_TASK_EMERGENCY_STOP_STK_SIZE];
static OS_STK appTaskLedsStk[APP_TASK_LEDS_STK_SIZE];
/*************************************************************************
*                  APPLICATION FUNCTION PROTOTYPES
*************************************************************************/

static void appTaskEmergencyStop(void *pdata);
static void appTaskLeds(void *pdata);
static void appTaskDisplay(void *pdata);
static void appTaskMonitorButton(void *pdata);
static void appTaskMonitorSens1(void *pdata);
static void appTaskMonitorSens2(void *pdata);
static void canSend(uint32_t id);
static void canHandler(void);
static canMessage_t can1RxBuf;

extern void __iar_program_start(void);
bool paused = false;
bool waitForPad2Clear = false;
bool emergencyStop = false;
int blocks_in_system = 0;
bool stopped = true;
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
   
  OSTaskCreate(appTaskMonitorButton,                               
               (void *)0,
               (OS_STK *)&appTaskMonitorButtonStk[APP_TASK_MONITOR_BUTTON_STK_SIZE - 1],
               APP_TASK_MONITOR_BUTTON_PRIO);
  
  OSTaskCreate(appTaskDisplay,                               
               (void *)0,
               (OS_STK *)&appTaskDisplayStk[APP_TASK_DISPLAY_STK_SIZE - 1],
               APP_TASK_DISPLAY_PRIO);
  
  OSTaskCreate(appTaskLeds,                               
               (void *)0,
               (OS_STK *)&appTaskLedsStk[APP_TASK_LEDS_STK_SIZE - 1],
               APP_TASK_LEDS_PRIO);
  
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
      controlAlarmSetState(CONTROL_ALARM_ON);
      interfaceLedSetState(D3_LED, LED_OFF);
      lcdSetTextPos(1,2);
      lcdWrite("Emerg Stop: True    ");
      if(isButtonPressed(JS_RIGHT))
      {
        canSend(RESET);
        dly100us(50);
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
        canSend(PAUSE);
        paused = true;
        lcdSetTextPos(1,1);
        lcdWrite("Paused: True    ");
      }
      else
      {
        paused = false;
        canSend(RESUME);
      }
    }
    
    if(controlEmergencyStopButtonPressed())
    {
      controlAlarmSetState(CONTROL_ALARM_ON);
      canSend(EMERGENCY_STOP);
      lcdSetTextPos(1,2);
      if(!emergencyStop)
      {
        lcdWrite("Emerg Stop: True    ");
      }
      emergencyStop = true;
      
    }
    
    if(isButtonPressed(BUT_1))
    {
      if(stopped)
      {
        canSend(START);
        stopped = false;
      }
    }
    
    if(isButtonPressed(BUT_2))
    {
      stopped = true;
      while(blocks_in_system > 0)
      {
        OSTimeDlyHMSM(0,0,0,50);
      }
      canSend(STOP);
      
    }
    OSTimeDlyHMSM(0,0,0,5);
  }
}
static void appTaskMonitorSens2(void *pdata) {
  while(true)
  {
    while(stopped)
    {
      OSTimeDlyHMSM(0,0,0,50);
    }
    
    ledSetState(USB_LINK_LED, LED_OFF);
    if (controlItemPresent(CONTROL_SENSOR_1)) {
        ledSetState(USB_LINK_LED, LED_ON);
        canSend(READY_TO_PICKUP_PAD1);
        
        while (controlItemPresent(CONTROL_SENSOR_1)){
          OSTimeDlyHMSM(0,0,0,20);
        }
        canSend(PICKED_UP_PAD1);
        blocks_in_system++;
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
        canSend(PAD2_CLEAR);
        waitForPad2Clear = false;
      }
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
      lcdWrite("State: Paused \t");
    }
    else
    {
      if(stopped)
      {
        lcdWrite("State: Stopped \t");
      }
      else
      {
        lcdWrite("State: Running \t");
      }
    }
    
    lcdSetTextPos(1,2);
    if(!emergencyStop)
    {
      lcdWrite("Emerg Stop: False");
    }
    OSTimeDlyHMSM(0,0,0,500);
  }
}

static void appTaskLeds(void *pdata)
{
  interfaceInit(CONTROL);
  interfaceLedSetState(D1_LED | D2_LED | D3_LED | D4_LED, LED_OFF);
  
  for(int i = 0; i < 5; i++)
  {
    interfaceLedToggle(D4_LED);
    OSTimeDlyHMSM(0,0,0,300);
  }
  while(true)
  {
    
    if(stopped && (blocks_in_system > 0))
    {
      interfaceLedToggle(D1_LED);
      interfaceLedSetState(D4_LED, LED_OFF); //Ready light
    }
    else if(stopped)
    {
      interfaceLedSetState(D1_LED, LED_OFF);
      interfaceLedSetState(D4_LED, LED_OFF); //Ready light
    }
    else
    {
      interfaceLedSetState(D1_LED, LED_ON);
      interfaceLedSetState(D4_LED, LED_ON); //Ready light
    }
    
    if(paused)
    {
      interfaceLedSetState(D2_LED, LED_ON);
    }
    else
    {
      interfaceLedSetState(D2_LED, LED_OFF);
    }
    
    if(emergencyStop || stopped)
    {
      interfaceLedSetState(D3_LED, LED_OFF);
      interfaceLedSetState(D4_LED, LED_OFF); //Ready light
    }
    else
    {
      interfaceLedSetState(D3_LED, LED_ON);
    }
    
    OSTimeDlyHMSM(0,0,0,500);
  }
}

static void canSend(uint32_t id) {
  canMessage_t msg = {0, 0, 0, 0};
  
  /* Initialise the CAN message structure */
  msg.id = id;  // arbitrary CAN message id
  msg.len = 4;    // data length 4
  msg.dataA = 0;
  msg.dataB = 0;
  
  canWrite(CAN_PORT_1, &msg);
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
    if(msg.id == QUERY_PAD2_STATUS)
    {
      waitForPad2Clear = true;
    }
    if(msg.id == OUTPUT_ROBOT_FINISHED)
    {
      if(blocks_in_system > 0)
      {
        blocks_in_system--;
      }
    }
    if(msg.id == EMERGENCY_STOP)
    {
      emergencyStop = true;
    }
  }
}
