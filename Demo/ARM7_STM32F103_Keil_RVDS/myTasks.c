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

// 30, 31, ..., 59, hors casier
const uint16_t SEUILS1[] = {
    3850,3804,3752,3691,3618,3532,3432,3320,3183,3033,
    2875,2695,2504,2322,2143,1959,1772,1583,1401,1231,
    1065,913,785,664,559,476,406,345,291,244,
    203
};

// hors casier, 0 , 1, ..., 29
const uint16_t SEUILS2[] = {
    3898,3859,3810,3752,3686,3605,3511,3414,3307,3185,
    3048,2893,2723,2545,2346,2141,1946,1741,1542,1369,
    1214,1065,927,797,681,580,487,406,341,283,
    234
};

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

void ADC1_Init(void) {
    // Enable GPIOA and ADC1 clocks
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_ADC1EN;

    // Set PA0 (ADC1_IN0) to analog mode (CNF=00, MODE=00)
    GPIOA->CRL &= ~(GPIO_CRL_CNF0 | GPIO_CRL_MODE0);

    // Turn on ADC1
    ADC1->CR2 |= ADC_CR2_ADON;
    for (volatile int i = 0; i < 1000; i++); // Short delay

    // Start calibration
    ADC1->CR2 |= ADC_CR2_CAL;
    while (ADC1->CR2 & ADC_CR2_CAL);  // Wait for calibration to finish
}

uint16_t ADC1_Read(void) {
    // Select channel 0 (PA0)
    ADC1->SQR3 = 0;

    // Start conversion
    ADC1->CR2 |= ADC_CR2_ADON;

    // Wait for conversion to complete
    while (!(ADC1->SR & ADC_SR_EOC));

    // Read 12-bit result
    return ADC1->DR;
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

void SwitchCurrentMode(void) {
    if (is_button_pressed(BP_DEMANDE_APPRO_PIN)) {
        current_mode = MODE_APPRO;
    } else if (is_button_pressed(BP_DEMANDE_TEST_PIN)) {
        current_mode = MODE_TEST;
    }
}

void transistor_on(void)
{
    set_gpio_pin_state(GPIOB, 13, 1);  // Set PB13 HIGH
}

void transistor_off(void)
{
    set_gpio_pin_state(GPIOB, 13, 0);  // Set PB13 LOW
}

void check_seuil(uint16_t x) {
    int i;

    // Check against SEUILS1 (for casiers 30–59)
    if (x <= SEUILS1[0]) {
        for (i = 0; i < 30; i++) {
            if (x <= SEUILS1[i] && x > SEUILS1[i + 1]) {
                int casier = 30 + i;
                casiers_ouverts[casier] = 1;
                return;
            }
        }

        // Out of range ("hors casier"), do nothing
        return;
    }

    // Value is higher than SEUILS1[0] → transistor + second read
    transistor_on();
    uint16_t new_x = ADC1_Read();
		transistor_off();

    // Check against SEUILS2 (for casiers 0–29)
    if (new_x >= SEUILS2[30]) {
        for (i = 0; i < 30; i++) {
            if (new_x <= SEUILS2[i] && new_x > SEUILS2[i + 1]) {
                int casier = i;
                casiers_ouverts[casier] = 1;
                return;
            }
        }
    }

    // Still hors casier → do nothing
}


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
