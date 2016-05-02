/**
* EN617 Main Controller
* -----------------------
* Jack Dunn 12002001
* -----------------------
* Responsible for:
* - Monitoring PAD 1
* - Monitoring PAD 2
* - Handling button presses for state changes
* - Emergency stop button.
*
*
**/
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
*                  TASK PRIORITIES
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

/*************************************************************************
*                    PROGRAM VARIABLES
*************************************************************************/
extern void __iar_program_start(void); //Pointer to the __iar_program_start assembler label.
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
 
  /* Setup the CAN handler */
  canRxInterrupt(canHandler);

  /* Create Tasks */
  //Create the emergency stop task
  OSTaskCreate(appTaskEmergencyStop,                               
               (void *)0,
               (OS_STK *)&appTaskEmergencyStopStk[APP_TASK_EMERGENCY_STOP_STK_SIZE - 1],
               APP_TASK_EMERGENCY_STOP_PRIO);
  
  //Create the PAD1 sensor task
  OSTaskCreate(appTaskMonitorSens1,                               
               (void *)0,
               (OS_STK *)&appTaskMonitorSens1Stk[APP_TASK_MONITOR_SENS1_STK_SIZE - 1],
               APP_TASK_MONITOR_SENS1_PRIO);

  //Create the PAD2 sensor task
   OSTaskCreate(appTaskMonitorSens2,                               
               (void *)0,
               (OS_STK *)&appTaskMonitorSens2Stk[APP_TASK_MONITOR_SENS2_STK_SIZE - 1],
               APP_TASK_MONITOR_SENS2_PRIO);

  //Create the button monitor task
  OSTaskCreate(appTaskMonitorButton,                               
               (void *)0,
               (OS_STK *)&appTaskMonitorButtonStk[APP_TASK_MONITOR_BUTTON_STK_SIZE - 1],
               APP_TASK_MONITOR_BUTTON_PRIO);

  //Create the display task
  OSTaskCreate(appTaskDisplay,                               
               (void *)0,
               (OS_STK *)&appTaskDisplayStk[APP_TASK_DISPLAY_STK_SIZE - 1],
               APP_TASK_DISPLAY_PRIO);

  //Create the LED task
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

/**
* appTaskEmergencyStop
*
* Task that runs whenever the system is in emergency stop or in the paused state.
* When the system is in emergency stop this task does not sleep which prevents the other tasks from running
*
*/
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

/**
* appTaskMonitorButton
*
* This task monitors the buttons on the board itself (i.e. BUT_1, BUT_2, JS_RIGHT, JS_CENTRE),
* it also monitors the external emergency stop button and enables the alarm if an emergency stop is
* triggered.
*
*/
static void appTaskMonitorButton(void *pdata)
{
  while(true)
  {
    if(isButtonPressed(JS_CENTRE)) //Pause button
    {
      while(isButtonPressed(JS_CENTRE)) {} //Debounce the button press

      if(paused == false) //Send the pause message and set the controller state to paused.
      {
        canSend(PAUSE);
        paused = true;
        lcdSetTextPos(1,1);
        lcdWrite("Paused: True    ");
      }
      else //Resume the system and send the resume message to other nodes.
      {
        paused = false;
        canSend(RESUME);
      }
    }
    
    if(controlEmergencyStopButtonPressed()) //Emergency Stop Button
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
    
    if(isButtonPressed(BUT_1)) //Start button
    {
      if(stopped)
      {
        canSend(START);
        stopped = false;
      }
    }
    
    if(isButtonPressed(BUT_2)) //Stop button.
    {
      //Disable the picking up of new blocks
      stopped = true;

      //Other nodes can continue working until there are no more blocks in the system and then the STOP message will be sent.
      while(blocks_in_system > 0)
      {
        OSTimeDlyHMSM(0,0,0,50);
      }
      canSend(STOP);
      
    }
    OSTimeDlyHMSM(0,0,0,5);
  }
}

/**
* appTaskMonitorSens2
*
* This task monitors in input pad, when a block is present it sends the READY_TO_PICKUP_PAD1 message,
* and then waits until the sensor is cleared, it then sends the PICKED UP message and increments the 
* blocks in system variable.
*
*/
static void appTaskMonitorSens2(void *pdata) {
  while(true)
  {
    //This task does not need to run when the system is stopped.
    while(stopped)
    {
      OSTimeDlyHMSM(0,0,0,50);
    }
    
    ledSetState(USB_LINK_LED, LED_OFF);

    if (controlItemPresent(CONTROL_SENSOR_1)) {
        ledSetState(USB_LINK_LED, LED_ON);
        canSend(READY_TO_PICKUP_PAD1); //Send the pickup message when an item is detected.
        
	//Wait until the item is not detected anymore before continuing, this means the block has been picked up.
        while (controlItemPresent(CONTROL_SENSOR_1)){
          OSTimeDlyHMSM(0,0,0,20);
        }

        canSend(PICKED_UP_PAD1); //Now send out the successful pickup message.
        blocks_in_system++; //Increment the counter of the number of blocks in the system
    } 
    OSTimeDlyHMSM(0,0,0,20);
  }
}

/**
* appTaskMonitorSens1
*
* This task monitors the sensor of the output pad. This task only runs when the output robot has sent a 
* QUERY_PAD2_STATUS message. When this message has been received the controller waits until PAD2 becomes
* clear before sending the PAD2_CLEAR message.
*
*/
static void appTaskMonitorSens1(void *pdata) {
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

/**
* appTaskDisplay
*
* Outputs debugging information on the display such as the current start/stop state and 
* whether an emergency stop has been triggered.
*
*/
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

/**
* appTaskDisplay
*
* Displays system state using the external LED board.
*
*/
static void appTaskLeds(void *pdata)
{
  //Init the LEDs and set them to off.
  interfaceInit(CONTROL);
  interfaceLedSetState(D1_LED | D2_LED | D3_LED | D4_LED, LED_OFF);
  
  for(int i = 0; i < 5; i++)
  {
    interfaceLedToggle(D4_LED);
    OSTimeDlyHMSM(0,0,0,300);
  }

  while(true)
  {
    
    if(stopped && (blocks_in_system > 0)) //If stopped with blocks in the system the started LED should flash indicating shutting down
    {
      interfaceLedToggle(D1_LED);
      interfaceLedSetState(D4_LED, LED_OFF); //Ready light
    }
    else if(stopped) //If stopped the started LED should be off.
    {
      interfaceLedSetState(D1_LED, LED_OFF);
      interfaceLedSetState(D4_LED, LED_OFF); //Ready light
    }
    else //The system must be started so the started LED should be on
    {
      interfaceLedSetState(D1_LED, LED_ON);
      interfaceLedSetState(D4_LED, LED_ON); //Ready light
    }
    
    if(paused)
    {
      interfaceLedSetState(D2_LED, LED_ON); //Paused so turn the pause LED on
    }
    else
    {
      interfaceLedSetState(D2_LED, LED_OFF); //Resumed so turn the pause LED off
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

/*
* canSend
* @param id The CAN id to send on the bus.
*
* Sends a CAN message to the CANbus
**/
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

    //The output robot is querying the PAD2 status
    if(msg.id == QUERY_PAD2_STATUS)
    {
      waitForPad2Clear = true; 
    }

    //The output robot has finished moving the block so decrement the blocks in system counter
    if(msg.id == OUTPUT_ROBOT_FINISHED)
    {
      if(blocks_in_system > 0)
      {
        blocks_in_system--;
      }
    }

    //Another node has triggered an emergency stop.
    if(msg.id == EMERGENCY_STOP)
    {
      emergencyStop = true;
    }
  }
}
