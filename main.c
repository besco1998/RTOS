/*
 * FreeRTOS Kernel V10.2.0
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/* 
	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used.
*/


/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the demo application tasks.
 * 
 * Main.c also creates a task called "Check".  This only executes every three 
 * seconds but has the highest priority so is guaranteed to get processor time.  
 * Its main function is to check that all the other tasks are still operational.
 * Each task (other than the "flash" tasks) maintains a unique count that is 
 * incremented each time the task successfully completes its function.  Should 
 * any error occur within such a task the count is permanently halted.  The 
 * check task inspects the count of each task to ensure it has changed since
 * the last time the check task executed.  If all the count variables have 
 * changed all the tasks are still executing error free, and the check task
 * toggles the onboard LED.  Should any task contain an error at any time 
 * the LED toggle rate will change from 3 seconds to 500ms.
 *
 */

/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "lpc21xx.h"
#include "queue.h"

/* Peripheral includes. */
#include "serial.h"
#include "GPIO.h"


/*-----------------------------------------------------------*/

/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 9600 )


/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
static void prvSetupHardware( void );
/*-----------------------------------------------------------*/


QueueHandle_t xQueue = NULL;

/*Tasks Handlers*/
TaskHandle_t Button_1_Monitor_Handler = NULL;
TaskHandle_t Button_2_Monitor_Handler = NULL;
TaskHandle_t Periodic_Transmitter_Handler = NULL;
TaskHandle_t UART_Receiver_Handler = NULL;
TaskHandle_t Load_1_Simulation_Handler = NULL;
TaskHandle_t Load_2_Simulation_Handler = NULL;

/*-----------------------------------------------------------*/
/*from queue API*/
typedef struct AMessage
 {
    char *msg;
 } xMessage;
/*-----------------------------------------------------------*/
void Button_1_Monitor( void * pvParameters )
{
	TickType_t xLastWakeTime = xTaskGetTickCount();	
    uint8_t  last_state = GPIO_read(PORT_1 , PIN0);
	xMessage b1_msg; //msg array
	xMessage *p_b1_msg=&b1_msg; //pointer to pointer to array
    for( ;; )
    {
		GPIO_write(PORT_0, PIN1, PIN_IS_HIGH);
		
		if( last_state != GPIO_read(PORT_1 , PIN0)){
			last_state = GPIO_read(PORT_1 , PIN0);
			
			if(last_state == 1){
				b1_msg.msg = "Button 1 Monitor: Rising Edge\n";
					xQueueSend(xQueue, (void*) &p_b1_msg, 0);
			}
			else {
				b1_msg.msg = "Button 1 Monitor: Falling Edge\n";
					xQueueSend(xQueue, (void*) &p_b1_msg , 0);
			}
		}
		
		GPIO_write(PORT_0, PIN1, PIN_IS_LOW);
		vTaskDelayUntil(&xLastWakeTime, 50);
    }
}

/*-----------------------------------------------------------*/
void Button_2_Monitor( void * pvParameters )
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint8_t  last_state = GPIO_read(PORT_1 , PIN1);
		xMessage b2_msg; //msg array
		xMessage *p_b2_msg=&b2_msg; //pointer to pointer to array
    for( ;; )
    {
		GPIO_write(PORT_0, PIN2, PIN_IS_HIGH);
		if( last_state != GPIO_read(PORT_1 , PIN1)){
			last_state = GPIO_read(PORT_1 , PIN1);
			
			if(last_state == 1){
				b2_msg.msg = "Button 2 Monitor: Rising Edge\n";
					xQueueSend(xQueue, (void*)&p_b2_msg , 0);
			}
			else {
				b2_msg.msg = "Button 2 Monitor: Falling Edge\n";
					xQueueSend(xQueue, (void*)&p_b2_msg , 0);
			}
		}
		
		GPIO_write(PORT_0, PIN2, PIN_IS_LOW);
		vTaskDelayUntil(&xLastWakeTime, 50);
    }
}

/*-----------------------------------------------------------*/
void Periodic_Transmitter( void * pvParameters )
{
	TickType_t xLastWakeTime = xTaskGetTickCount();	
		xMessage periodic_msg; //msg array
		xMessage *p_periodic_msg=&periodic_msg; //pointer to pointer to array
    for( ;; )
    {
		GPIO_write(PORT_0, PIN3, PIN_IS_HIGH);		
		periodic_msg.msg = "100ms Periodic message\n";
		xQueueSend(xQueue, (void*) &p_periodic_msg, 0);

		GPIO_write(PORT_0, PIN3, PIN_IS_LOW);
		vTaskDelayUntil(&xLastWakeTime, 100);
    }
}
/*-----------------------------------------------------------*/
void Uart_Receiver( void * pvParameters )
{
	TickType_t xLastWakeTime = xTaskGetTickCount();	
	
	xMessage *UART_msg; //msg array
		//xMessage *p_UART_msg=&UART_msg; //pointer to pointer to array

    for( ;; )
    {
		GPIO_write(PORT_0, PIN4, PIN_IS_HIGH);			
		while(xQueueReceive( xQueue,( void * ) &UART_msg,0 )== pdTRUE){
			vSerialPutString((const signed char *) UART_msg->msg, strlen(UART_msg->msg)); //display the uart rx msgs
		}
		GPIO_write(PORT_0, PIN4, PIN_IS_LOW);
		vTaskDelayUntil(&xLastWakeTime, 20);
    }
}
/*-----------------------------------------------------------*/
void Load_1_Simulation( void * pvParameters )
{
	TickType_t xLastWakeTime = xTaskGetTickCount();	
    uint32_t i = 0;

    for( ;; )
    {
		GPIO_write(PORT_0, PIN5, PIN_IS_HIGH);
		//dummy load
		for(i=0;i<37800;i++){
			i=i;
		}
		
		GPIO_write(PORT_0, PIN5, PIN_IS_LOW);
		vTaskDelayUntil(&xLastWakeTime, 10);
    }
}
/*-----------------------------------------------------------*/
void Load_2_Simulation( void * pvParameters )
{
    
	TickType_t xLastWakeTime = xTaskGetTickCount();	
    uint32_t i = 0;


    for( ;; )
    {
		GPIO_write(PORT_0, PIN6, PIN_IS_HIGH);
		//dummy load
		for(i=0;i<44000;i++){
			i=i;
		}
		GPIO_write(PORT_0, PIN6, PIN_IS_LOW);
		vTaskDelayUntil(&xLastWakeTime, 100);
    }
}


void vApplicationTickHook(void){
	GPIO_write(PORT_0, PIN0, PIN_IS_HIGH);
	GPIO_write(PORT_0, PIN0, PIN_IS_LOW);
}
void vApplicationIdleHook(void){
	GPIO_write(PORT_0, PIN7, PIN_IS_HIGH);
	GPIO_write(PORT_0, PIN7, PIN_IS_LOW);
}








/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler. 
 */
int main( void )
{
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();
	/* Create a queue capable of containing 10 pointers to AMessage
    structures.  These are to be queued by pointers as they are
    relatively large structures. */
	xQueue = xQueueCreate( 10, sizeof( xMessage * )  );

	
    /* Create Tasks here */
		 xTaskPeriodicCreate(
							Button_1_Monitor,       			//fn name
							"Button_1_Monitor",         		//task name			
							100,      							//size
							( void * ) 0,   					//param
							1,									//priority
							&Button_1_Monitor_Handler,			//handler
							50 );     							//periodicity
			xTaskPeriodicCreate(
							Button_2_Monitor,       			//fn name
							"Button_2_Monitor",          		//task name				
							100,      							//size
							( void * ) 0,    					//param
							1,									//priority
							&Button_2_Monitor_Handler,			//handler
							50);     							//periodicity
			xTaskPeriodicCreate(
							Periodic_Transmitter,       		//fn name	
							"Periodic_Transmitter",          	//task name					
							100,      							//size	
							( void * ) 0,    					//param	
							1,									//priority	
							&Periodic_Transmitter_Handler,		//handler
							100);    							//periodicity
			xTaskPeriodicCreate(
							Uart_Receiver,      				//fn name
							"Uart_Receiver",          			//task name			
							100,      							//size
							( void * ) 0,    					//param
							1,									//priority
							&UART_Receiver_Handler,				//handler
							20);								//periodicity
			xTaskPeriodicCreate(
							Load_1_Simulation,     				//fn name
							"Load_1_Simulation",   				//task name	     
							100,      							//size
							( void * ) 0,    					//param
							1,									//priority
							&Load_1_Simulation_Handler,			//handler
							10 );								//periodicity
			xTaskPeriodicCreate(
							Load_2_Simulation,      			//fn name	
							"Load_2_Simulation",          		//task name	     	
							100,      							//size	
							( void * ) 0,    					//param	
							1,									//priority	
							&Load_2_Simulation_Handler,			//handler
							100 );								//periodicity
							
			


	/* Now all the tasks have been started - start the scheduler.

	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */
	vTaskStartScheduler();

	/* Should never reach here!  If you do then there was not enough heap
	available for the idle task to be created. */
	for( ;; );
}

/*-----------------------------------------------------------*/

/* Function to reset timer 1 */
void timer1Reset(void)
{
	T1TCR |= 0x2;
	T1TCR &= ~0x2;
}

/* Function to initialize and start timer 1 */
static void configTimer1(void)
{
	T1PR = 1000;
	T1TCR |= 0x1;
}

static void prvSetupHardware( void )
{
	/* Perform the hardware setup required.  This is minimal as most of the
	setup is managed by the settings in the project file. */

	/* Configure UART */
	xSerialPortInitMinimal(mainCOM_TEST_BAUD_RATE);

	/* Configure GPIO */
	GPIO_init();
	
	/* Config trace timer 1 and read T1TC to get current tick */
	configTimer1();

	/* Setup the peripheral bus to be the same as the PLL output. */
	VPBDIV = mainBUS_CLK_FULL;
}
