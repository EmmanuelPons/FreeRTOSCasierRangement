#include <stdlib.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"

#include "stm32f10x.h"

#define ledSTACK_SIZE		configMINIMAL_STACK_SIZE
#define ledNUMBER_OF_LEDS	( 7 )

/* The task that is created three times. */
static portTASK_FUNCTION_PROTO( vLed1Task, pvParameters );
static portTASK_FUNCTION_PROTO( vLed2Task, pvParameters );
static portTASK_FUNCTION_PROTO( vLed3Task, pvParameters );
static portTASK_FUNCTION_PROTO( vLed4Task, pvParameters );
/*-----------------------------------------------------------*/
unsigned char argument =3;
void vInit_myTasks( UBaseType_t uxPriority )
{
//BaseType_t xLEDTask;
	  xTaskCreate( vLed1Task, "LED1", ledSTACK_SIZE, &argument, uxPriority, ( TaskHandle_t * ) NULL );
	  xTaskCreate( vLed2Task, "LED2", ledSTACK_SIZE, NULL, uxPriority, ( TaskHandle_t * ) NULL );
	  xTaskCreate( vLed3Task, "LED3", ledSTACK_SIZE, NULL, uxPriority, ( TaskHandle_t * ) NULL );
	  xTaskCreate( vLed4Task, "LED4", ledSTACK_SIZE, NULL, uxPriority, ( TaskHandle_t * ) NULL );
}
/*-----------------------------------------------------------*/

//static portTASK_FUNCTION( vLed1Task, pvParameters )
void vLed1Task( void *pvParameters )
{ static unsigned char var_static_fonction =0;
	unsigned char var_fonction =  *( (unsigned char*)pvParameters);

	for(;;)
	{ GPIOA->BSRR = 1<<4;
	vTaskDelay((TickType_t)var_fonction);
	GPIOA->BSRR = 1<<(4+16);
	vTaskDelay((TickType_t)var_fonction);	
	}
} /*lint !e715 !e818 !e830 Function definition must be standard for task creation. */


static portTASK_FUNCTION( vLed2Task, pvParameters )
{
	/* Les parametres ne sont pas utilisés. */
	( void ) pvParameters;
 
	for(;;)
	{ GPIOA->BSRR = 1<<5;
	vTaskDelay(1);
	GPIOA->BSRR = 1<<(5+16);
	vTaskDelay(8);	
	}
} /*lint !e715 !e818 !e830 Function definition must be standard for task creation. */
static portTASK_FUNCTION( vLed3Task, pvParameters )
{
	/* Les parametres ne sont pas utilisés. */
	( void ) pvParameters;

	for(;;)
	{ GPIOA->BSRR = 1<<6;
	vTaskDelay(4);
	GPIOA->BSRR = 1<<(6+16);
	vTaskDelay(3);	
	}
} /*lint !e715 !e818 !e830 Function definition must be standard for task creation. */

static portTASK_FUNCTION( vLed4Task, pvParameters )
{
	/* Les parametres ne sont pas utilisés. */
	( void ) pvParameters;

	for(;;)
	{ GPIOA->BSRR = 1<<7;
	vTaskDelay(7);
	GPIOA->BSRR = 1<<(7+16);
	vTaskDelay(6);	
	}
} /*lint !e715 !e818 !e830 Function definition must be standard for task creation. */

