#include <stdlib.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"


/* le fichier .h du TP  */
#include "myTasks.h"

// #define ledSTACK_SIZE		128




/* d�claration des diverses taches avec la fonction prototype */
//static portTASK_FUNCTION_PROTO( vLed1Task, pvParameters );//exemple de d�claration

/*-------------------variables globales propres aux taches------------*/
unsigned char argument_tache1 =3;

void vInit_myTasks( UBaseType_t uxPriority )
{//ici on cree toutes les taches et tous les s�maphores, mutex ....
//xTaskCreate( vLed1Task, "LED1", ledSTACK_SIZE, &argument_tache1, uxPriority, ( TaskHandle_t * ) NULL );

}
//
