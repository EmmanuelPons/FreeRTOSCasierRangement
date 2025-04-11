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

#define NB_COLONNES 5
#define PREMIERE_COLONNE_PIN 6  // GPIOA.6
#define PREMIERE_LIGNE_PIN 1    // GPIOB.1

// Colonnes : GPIOA, pins A6 to A10 are used for reed columns.
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

// Forward declarations for timing & LED functions
void TIM2_Config(void);
void GPIO_Config(void);
void delay_exact_us(float us);
void send_one(void);
void send_zero(void);
void send_pwm(uint8_t tram[NUM_LEDS * 3]);
void generate_tram(uint8_t mode, uint8_t leds[NUM_LEDS], uint8_t tram[NUM_LEDS * 3]);

// ** Read button state **
uint8_t is_button_pressed(uint8_t pin) {
    return (GPIOC->IDR & (1 << pin)) == 0;  // Active-low assumption
}

// ** Set GPIO state function **
void set_gpio_pin_state(GPIO_TypeDef *port, uint8_t pin, uint8_t state) {
    if (state)
        port->BSRR = (1 << pin);    // Set pin high
    else
        port->BRR = (1 << pin);     // Set pin low
}

// ---------------------------
// SwitchCurrentMode: Reads buttons and updates current_mode.
// ---------------------------
void SwitchCurrentMode(void) {
    if (is_button_pressed(BP_DEMANDE_APPRO_PIN))
        current_mode = MODE_APPRO;
    else if (is_button_pressed(BP_DEMANDE_TEST_PIN))
        current_mode = MODE_TEST;
}

// Reads the 5 column states for a given row line (on GPIOB) and saves them in resultat[]
void lire_ligne_reed(uint8_t ligne_pin, uint8_t resultat[NB_COLONNES]) {
    int i;
    // Activate the row by setting it LOW.
    set_gpio_pin_state(GPIOB, ligne_pin, 0);
    for (i = 0; i < NB_COLONNES; i++) {
        uint8_t pin = colonnes_pins[i];
        // When a reed is closed, the corresponding column reads 0.
        resultat[i] = ((GPIOA->IDR & (1 << pin)) == 0) ? 1 : 0;
    }
    // Deactivate the row (set HIGH).
    set_gpio_pin_state(GPIOB, ligne_pin, 1);
}

void scanner_tous_reed(void){
    int i, j;
    uint8_t resultat[NB_COLONNES];
    uint8_t pin_ligne;
    
    // Scan 12 rows starting from PREMIERE_LIGNE_PIN (e.g., PB1 to PB12)
    for (i = 0; i < 12; i++){
        pin_ligne = PREMIERE_LIGNE_PIN + i;
        lire_ligne_reed(pin_ligne, resultat);
        for (j = 0; j < NB_COLONNES; j++){
            casiers_ouverts[i + j * 12] = resultat[j];
        }
    }
}

// ---------------------------
// Timing & LED Functions
// ---------------------------
void TIM2_Config(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Enable TIM2 clock
    TIM2->PSC = 72 - 1;                 // 1 tick = 1µs
    TIM2->ARR = 0xFFFF;                 // Max ARR
    TIM2->CR1 |= TIM_CR1_CEN;           // Start timer
}

void delay_exact_us(float us) {
    uint16_t target;
    TIM2->CNT = 0;
    target = (uint16_t)(us * 72.0f);
    while (TIM2->CNT < target);
}

void GPIO_Config(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; // Enable clock for GPIOA
    GPIOA->CRL &= ~(GPIO_CRL_MODE5 | GPIO_CRL_CNF5);
    GPIOA->CRL |= (GPIO_CRL_MODE5_0 | GPIO_CRL_MODE5_1); // Set PA5 as output at 50MHz
}

void send_one(void) {
    GPIOA->BSRR = GPIO_BSRR_BS5;
    delay_exact_us(0.98f);
    GPIOA->BSRR = GPIO_BSRR_BR5;
    delay_exact_us(0.42f);
}

void send_zero(void) {
    GPIOA->BSRR = GPIO_BSRR_BS5;
    delay_exact_us(0.42f);
    GPIOA->BSRR = GPIO_BSRR_BR5;
    delay_exact_us(0.98f);
}

void generate_tram(uint8_t mode, uint8_t leds[NUM_LEDS], uint8_t tram[NUM_LEDS * 3]) {
    int i;
    for (i = 0; i < NUM_LEDS; i++) {
        int led_index = NUM_LEDS - 1 - i;  // Reverse order mapping
        if (mode == MODE_APPRO) {
            tram[i * 3]     = leds[led_index] ? 0x00 : 0xFC; // Red
            tram[i * 3 + 1] = leds[led_index] ? 0xFC : 0x00; // Green
            tram[i * 3 + 2] = 0x00;                         // Blue
        } else if (mode == MODE_TEST) {
            tram[i * 3]     = 0x00;
            tram[i * 3 + 1] = 0x00;
            tram[i * 3 + 2] = leds[led_index] ? 0xFC : 0x00;
        }
    }
}

void send_pwm(uint8_t tram[NUM_LEDS * 3]) {
    int i, bit;
    for (i = 0; i < NUM_LEDS * 3; i++){
        for (bit = 7; bit >= 0; bit--){
            if (tram[i] & (1 << bit))
                send_one();
            else
                send_zero();
        }
    }
}

// ---------------------------
// ADC & Transistor Functions
// ---------------------------
void ADC1_Init(void) {
    int i;
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_ADC1EN;
    GPIOA->CRL &= ~(GPIO_CRL_CNF0 | GPIO_CRL_MODE0);  // PA0 as analog input
    ADC1->CR2 |= ADC_CR2_ADON;  // Turn on ADC1
    for (i = 0; i < 1000; i++);  // Wait for stabilization
    ADC1->CR2 |= ADC_CR2_CAL;  // Start calibration
    while (ADC1->CR2 & ADC_CR2_CAL);
}

uint16_t ADC1_Read(void) {
    ADC1->SQR3 = 0;          // Channel 0 (PA0)
    ADC1->CR2 |= ADC_CR2_ADON; // Start conversion
    while (!(ADC1->SR & ADC_SR_EOC));
    return ADC1->DR;
}

void transistor_on(void) {
    set_gpio_pin_state(GPIOB, 13, 1); // PB13 HIGH
}

void transistor_off(void) {
    set_gpio_pin_state(GPIOB, 13, 0); // PB13 LOW
}

// ---------------------------
// ADC Value Checking function
// ---------------------------
void check_seuil(uint16_t x) {
    int i;
    uint16_t new_x;
    // Check against SEUILS1 (mapped to casiers 30–59)
    if (x <= SEUILS1[0]) {
        for (i = 0; i < 30; i++) {
            if (x <= SEUILS1[i] && x > SEUILS1[i + 1]) {
                int casier = 30 + i;
                casiers_ouverts[casier] = 1;
                return;
            }
        }
        return;  // Out of range—do nothing
    }
    // x exceeds highest threshold → use transistor action and re-read ADC
    transistor_on();
    new_x = ADC1_Read();
    transistor_off();
    // Check against SEUILS2 (mapped to casiers 0–29)
    if (new_x >= SEUILS2[30]) {
        for (i = 0; i < 30; i++) {
            if (new_x <= SEUILS2[i] && new_x > SEUILS2[i + 1]) {
                int casier = i;
                casiers_ouverts[casier] = 1;
                return;
            }
        }
    }
}

// ---------------------------
// FreeRTOS Task Definitions
// ---------------------------

// Task handles for notifications
static TaskHandle_t xHandleCheckSeuil = NULL;
static TaskHandle_t xHandleScannerReed = NULL;

// --- Mode Management Task ---
// This task continuously polls buttons, updates the global mode,
// and sends notifications to the appropriate task based on mode.
void vTaskModeManagement(void *pvParameters) {
    uint8_t i;
    TickType_t last_mode_change_time = xTaskGetTickCount();
    TickType_t last_state_change_time = xTaskGetTickCount();
    uint8_t tram[NUM_LEDS * 3];

    for (;;) {
        // Update global mode from button state
        SwitchCurrentMode();

        switch (current_mode) {
            case MODE_EFFACE:
                memset((void *)etat_led, 0, sizeof(etat_led));
                break;

            case MODE_TEST:
                // Notify the ADC check_seuil task to process ADC reading
                if (xHandleCheckSeuil != NULL)
                    xTaskNotifyGive(xHandleCheckSeuil);

                if ((xTaskGetTickCount() - last_mode_change_time) >= pdMS_TO_TICKS(5000)) {
                    current_mode = MODE_EFFACE;
                    last_mode_change_time = xTaskGetTickCount();
                }
                break;

            case MODE_APPRO:
                // Notify the reed scanner task
                if (xHandleScannerReed != NULL)
                    xTaskNotifyGive(xHandleScannerReed);

                // Update etat_led based on casiers_ouverts (for WS2812 output)
                for (i = 0; i < NUM_LEDS; i++) {
                    etat_led[i] = casiers_ouverts[i] ? 1 : 0;
                }
                if ((xTaskGetTickCount() - last_state_change_time) >= pdMS_TO_TICKS(5000)) {
                    current_mode = MODE_EFFACE;
                    last_state_change_time = xTaskGetTickCount();
                }
                break;
        }

        // Generate WS2812 data and send via PWM regardless of mode
        generate_tram(current_mode, etat_led, tram);
        send_pwm(tram);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// --- ADC Check Seuil Task ---
// Waits for a notification (from mode management) to perform an ADC read
// and then processes the value with check_seuil.
void vTaskCheckSeuil(void *pvParameters) {
    int i;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        {
            uint16_t adc_val = ADC1_Read();
            check_seuil(adc_val);
            // Optionally update etat_led based on casiers_ouverts
            for (i = 0; i < NUM_LEDS; i++) {
                etat_led[i] = casiers_ouverts[i] ? 1 : 0;
            }
        }
    }
}

// --- Reed Scanner Task ---
// Waits for notification then scans all reed switches.
void vTaskScannerReed(void *pvParameters) {
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        scanner_tous_reed();
    }
}

// ---------------------------
// Task Initialization Function
// ---------------------------
void vInit_myTasks(UBaseType_t uxPriority) {
    xTaskCreate(vTaskModeManagement, "ModeTask", configMINIMAL_STACK_SIZE, NULL, uxPriority, NULL);
    xTaskCreate(vTaskCheckSeuil, "CheckSeuil", 128, NULL, uxPriority, &xHandleCheckSeuil);
    xTaskCreate(vTaskScannerReed, "ScannerReed", 128, NULL, uxPriority, &xHandleScannerReed);
}
