#include <stdlib.h>
#include <stdint.h>
#include <string.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"


/* le fichier .h du TP  */
#include "myTasks.h"

// #define ledSTACK_SIZE		128

#define BP_DEMANDE_TEST_PIN   12
#define BP_DEMANDE_APPRO_PIN  13

typedef enum {
    MODE_EFFACE,
    MODE_TEST,
    MODE_APPRO
} Mode_t;

//Var global

volatile Mode_t current_mode = MODE_EFFACE;
volatile int num_case_resistance = 0; // MODE TEST
volatile uint8_t  mesure_finie = 0; // MODE TEST

volatile uint8_t etat_reed[60] = {0};  
volatile uint8_t casiers_ouverts[60] = {0};

// **Lire l'état d'un bouton**
uint8_t is_button_pressed(uint8_t pin) {
    return (GPIOC->IDR & (1 << pin)) == 0;  // Hypothèse : logique inversée (appui = 0)
}

void clear_all_leds(){

}

void vTaskModeManagement(void *pvParameters) {

    TickType_t last_mode_change_time;
    TickType_t last_state_change_time = xTaskGetTickCount();

    while (1) {
        switch (current_mode) {
					
            case MODE_EFFACE:
							
                //clear_all_leds
								memset((void *)etat_reed, 0, sizeof(etat_reed));
						
                if (is_button_pressed(BP_DEMANDE_APPRO_PIN)) {
                    current_mode = MODE_APPRO;
                    last_mode_change_time = xTaskGetTickCount();
                }
								else if (is_button_pressed(BP_DEMANDE_TEST_PIN)) {
                    current_mode = MODE_TEST;
                    last_mode_change_time = xTaskGetTickCount();
                }
                break;

						 case MODE_TEST:
							 
                if (mesure_finie) {
									if (num_case_resistance > 0){
										for (uint8_t i = 0; i < 60; i++) etat_reed[i] = (i == num_case_resistance) ? 1 : 0; // Seul le bon casier passe à 1, les autres à 0
									}
                
								else if (num_case_resistance < 0) {
									
							// Faire clignoter le casier correspondant (valeur absolue)
							uint8_t casier_a_clignoter = (uint8_t)(-num_case_resistance);
            
            // Vérifier si le numéro de casier est valide
            if (casier_a_clignoter < 60) {
                for (uint8_t i = 0; i < 60; i++) {
                    etat_reed[i] = 0; // Éteint tous les casiers
                }

								
								//!!!!!!!!!!!!\ BLOQUANT à CHANGER /!!!!!!!!!!!!\
								
                // Clignotement en FreeRTOS : allume et éteint à intervalle régulier
                for (uint8_t j = 0; j < 5; j++) {  // Fait clignoter 5 fois
                    etat_reed[casier_a_clignoter] = 1;  // Allume
                    vTaskDelay(pdMS_TO_TICKS(500));     // Attend 500ms
                    etat_reed[casier_a_clignoter] = 0;  // Éteint
                    vTaskDelay(pdMS_TO_TICKS(500));     // Attend 500ms
												}
										}
								} else {
										// Si la valeur est nulle, éteint toutes les LEDs
										for (uint8_t i = 0; i < 60; i++) {
												etat_reed[i] = 0;
										}
								}
							}		
                if ((xTaskGetTickCount() - last_mode_change_time) >= pdMS_TO_TICKS(5000)) {
                    current_mode = MODE_EFFACE;
                }
								
								if (is_button_pressed(BP_DEMANDE_APPRO_PIN)) {
                    current_mode = MODE_APPRO;
                    last_mode_change_time = xTaskGetTickCount();
                }
                break;

            case MODE_APPRO:
							
                //update_appro_mode
						    for (uint8_t i = 0; i < 60; i++) {
									if (casiers_ouverts[i]) {
											etat_reed[i] = 1;  // Casier ferme LED rouge
									} else {
											etat_reed[i] = 0;  // Casier ouvert LED verte
									}
							}

                if ((xTaskGetTickCount() - last_state_change_time) >= pdMS_TO_TICKS(5000)) {
                    current_mode = MODE_EFFACE;
                }
								
								if (is_button_pressed(BP_DEMANDE_TEST_PIN)) {
                    current_mode = MODE_TEST;
                    last_mode_change_time = xTaskGetTickCount();
                }
                break;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}



/* déclaration des diverses taches avec la fonction prototype */
//static portTASK_FUNCTION_PROTO( vLed1Task, pvParameters );//exemple de déclaration

/*-------------------variables globales propres aux taches------------*/
unsigned char argument_tache1 =3;

void vInit_myTasks( UBaseType_t uxPriority )
{//ici on cree toutes les taches et tous les sémaphores, mutex ....
//xTaskCreate( vLed1Task, "LED1", ledSTACK_SIZE, &argument_tache1, uxPriority, ( TaskHandle_t * ) NULL );

}



