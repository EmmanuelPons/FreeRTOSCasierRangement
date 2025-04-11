#include <stdlib.h>
#include <stdint.h>
#include <string.h>

/* FreeRTOS include files */
#include "FreeRTOS.h"
#include "task.h"

/* Custom header */
#include "myTasks.h"

#include "stm32f10x.h"  // STM32F103RB library

// ** Pin Definitions **
#define BP_DEMANDE_TEST_PIN   12
#define BP_DEMANDE_APPRO_PIN  13
#define LED_OUTPUT_PIN        5  // PA5 for WS2812 LED control

#define NUM_LEDS              60  // Total LEDs controlled

#define NB_COLONNES 5
#define PREMIERE_COLONNE_PIN 6  // GPIOA.6
#define PREMIERE_LIGNE_PIN 1    // GPIOB.1

// Colonnes : GPIOA, pins A6 à A10
const uint8_t colonnes_pins[NB_COLONNES] = {6, 7, 8, 9, 10};


// ** Modes Enumeration **
typedef enum {
    MODE_EFFACE,
    MODE_TEST,
    MODE_APPRO
} Mode_t;

// ** Global Variables **
volatile Mode_t current_mode = MODE_EFFACE;
volatile int num_case_resistance = 0;  // MODE_TEST
volatile uint8_t mesure_finie = 0;     // MODE_TEST
volatile uint8_t etat_led[NUM_LEDS] = {0};  
volatile uint8_t casiers_ouverts[NUM_LEDS] = {0};

// ** Timer Configuration (TIM2 for precise delays) **
void TIM2_Config(void);
void GPIO_Config(void);
void delay_exact_us(float us);
void send_one(void);
void send_zero(void);
void send_pwm(uint8_t tram[NUM_LEDS * 3]);
void generate_tram(uint8_t mode, uint8_t leds[NUM_LEDS], uint8_t tram[NUM_LEDS * 3]);

// ** Read button state **
uint8_t is_button_pressed(uint8_t pin) {
    return (GPIOC->IDR & (1 << pin)) == 0;  // Assume active-low buttons
}

// ** Set output state **
void set_gpio_pin_state(GPIO_TypeDef *port, uint8_t pin, uint8_t state) {
    if (state) {
        port->BSRR = (1 << pin);    // Set pin high
    } else {
        port->BRR  = (1 << pin);    // Set pin low
    }
}

// Lit les 5 colonnes pour une ligne GPIOB donnée
void lire_ligne_reed(uint8_t ligne_pin, uint8_t resultat[NB_COLONNES]) {
		int i = 0;
	
    // 1. Mettre la ligne à LOW pour activer la lecture
    set_gpio_pin_state(GPIOB, ligne_pin, 0);

    // 2. Lire l’état des colonnes (GPIOA)
    for (i = 0; i < NB_COLONNES; i++) {
        uint8_t pin = colonnes_pins[i];
        // Reed fermé => entrée à 0 => ON
        resultat[i] = ((GPIOA->IDR & (1 << pin)) == 0) ? 1 : 0;
    }

    //	 3. Remettre la ligne à HIGH (repos)
    set_gpio_pin_state(GPIOB, ligne_pin, 1);
}

void scanner_tous_reed(void){
	int i = PREMIERE_LIGNE_PIN ;
	int j = 0;
	uint8_t resultat[NB_COLONNES];
	uint8_t pin_ligne = 0;
	
	
	for (i = 0; i < 12 ; i++){
		
			pin_ligne = PREMIERE_LIGNE_PIN + i;
		lire_ligne_reed(pin_ligne, resultat);

		
		for (j = 0; j < NB_COLONNES; j++){
			casiers_ouverts[i + j * 12] = resultat[j];
		}
				
	}

}


// ** Configure TIM2 for microsecond timing **
void TIM2_Config(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Enable TIM2 clock
    TIM2->PSC = 72 - 1;                 // Set prescaler (1 tick = 1µs)
    TIM2->ARR = 0xFFFF;                 // Max ARR value
    TIM2->CR1 |= TIM_CR1_CEN;           // Enable TIM2
}

// ** Microsecond delay using TIM2 **
void delay_exact_us(float us) {
    uint16_t target; // Moved to top
    TIM2->CNT = 0;  // Reset counter
    target = (uint16_t)(us * 72.0f); // Convert µs to clock cycles
    while (TIM2->CNT < target);
}

// ** Configure PA5 as output for WS2812 control **
void GPIO_Config(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; // Enable GPIOA clock
    GPIOA->CRL &= ~(GPIO_CRL_MODE5 | GPIO_CRL_CNF5);
    GPIOA->CRL |= (GPIO_CRL_MODE5_0 | GPIO_CRL_MODE5_1); // Output mode 50MHz
}

// ** Send a WS2812-compatible "1" signal on PA5 **
void send_one(void) {
    GPIOA->BSRR = GPIO_BSRR_BS5; // Set PA5 HIGH
    delay_exact_us(0.98f);
    GPIOA->BSRR = GPIO_BSRR_BR5; // Set PA5 LOW
    delay_exact_us(0.42f);
}

// ** Send a WS2812-compatible "0" signal on PA5 **
void send_zero(void) {
    GPIOA->BSRR = GPIO_BSRR_BS5; // Set PA5 HIGH
    delay_exact_us(0.42f);
    GPIOA->BSRR = GPIO_BSRR_BR5; // Set PA5 LOW
    delay_exact_us(0.98f);
}

// ** Generate LED data for WS2812 **
void generate_tram(uint8_t mode, uint8_t leds[NUM_LEDS], uint8_t tram[NUM_LEDS * 3]) {
    int i;
    for (i = 0; i < NUM_LEDS; i++) {
        int led_index = NUM_LEDS - 1 - i; // Reverse order
        if (mode == MODE_APPRO) {
            tram[i * 3]     = leds[led_index] ? 0x00 : 0xFC;     // Red
            tram[i * 3 + 1] = leds[led_index] ? 0xFC : 0x00;     // Green
            tram[i * 3 + 2] = 0x00;                              // Blue
        } else if (mode == MODE_TEST) {
            tram[i * 3]     = 0x00;                              // Red
            tram[i * 3 + 1] = 0x00;                              // Green
            tram[i * 3 + 2] = leds[led_index] ? 0xFC : 0x00;     // Blue
        }
    }
}

// ** Send WS2812 PWM signal **
void send_pwm(uint8_t tram[NUM_LEDS * 3]) {
    int i, bit;
    for (i = 0; i < NUM_LEDS * 3; i++) {
        for (bit = 7; bit >= 0; bit--) {
            if (tram[i] & (1 << bit)) {
                send_one();
            } else {
                send_zero();
            }
        }
    }
}


// FAIRE TACHE SwitchMode() qui va gérer le changement des taches timeout + appui bouton

// ** FreeRTOS Task: Mode Management **
void vTaskModeManagement(void *pvParameters) {
    uint8_t j;
    uint8_t i;
    TickType_t last_mode_change_time;
    TickType_t last_state_change_time = xTaskGetTickCount();
    uint8_t tram[NUM_LEDS * 3];

    while (1) {
        switch (current_mode) {
            case MODE_EFFACE:
                memset((void *)etat_led, 0, sizeof(etat_led));
                if (is_button_pressed(BP_DEMANDE_APPRO_PIN)) {
                    current_mode = MODE_APPRO;
                    last_mode_change_time = xTaskGetTickCount();
                } else if (is_button_pressed(BP_DEMANDE_TEST_PIN)) {
                    current_mode = MODE_TEST;
                    last_mode_change_time = xTaskGetTickCount();
                }
                break;

            case MODE_TEST:
                if (mesure_finie) {
                    if (num_case_resistance > 0) {
                        for (i = 0; i < NUM_LEDS; i++) {
                            etat_led[i] = (i == num_case_resistance) ? 1 : 0;
                        }
                    } else if (num_case_resistance < 0) {
                        uint8_t casier_a_clignoter = (uint8_t)(-num_case_resistance);
                        if (casier_a_clignoter < NUM_LEDS) {
                            for (j = 0; j < 5; j++) {
                                etat_led[casier_a_clignoter] = 1;
                                vTaskDelay(pdMS_TO_TICKS(500));
                                etat_led[casier_a_clignoter] = 0;
                                vTaskDelay(pdMS_TO_TICKS(500));
                            }
                        }
                    } else {
                        memset((void *)etat_led, 0, sizeof(etat_led));
                    }
                }

                if ((xTaskGetTickCount() - last_mode_change_time) >= pdMS_TO_TICKS(5000)) {
                    current_mode = MODE_EFFACE;
                }
                break;

            case MODE_APPRO:
                for (i = 0; i < NUM_LEDS; i++) {
                    etat_led[i] = casiers_ouverts[i] ? 1 : 0;
                }

                if ((xTaskGetTickCount() - last_state_change_time) >= pdMS_TO_TICKS(5000)) {
                    current_mode = MODE_EFFACE;
                }
                break;
        }

        generate_tram(current_mode, etat_led, tram);
        send_pwm(tram);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* Task Initialization */
void vInit_myTasks(UBaseType_t uxPriority) {
    xTaskCreate(vTaskModeManagement, "ModeTask", configMINIMAL_STACK_SIZE, NULL, uxPriority, NULL);
}
