/* Standard includes. */
#include <stdlib.h>
#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "def_type_gpio.h"
#include "myTasks.h"  // Declares vInit_myTasks()

/* Priorities for the tasks. */
#define mainLED_TASK_PRIORITY      ( tskIDLE_PRIORITY + 2 )

// Button definitions
#define BP_DEMANDE_TEST_PIN   12
#define BP_DEMANDE_APPRO_PIN  13

// Global parameter instances for Clignoteur tasks (optional, uncomment if used)
// PARAM_CLIGNOTEUR ParamClignoteurA = {GPIOC, 9, 5, 4};
// PARAM_CLIGNOTEUR ParamClignoteurB = {GPIOC, 10, 3, 2};
// PARAM_CLIGNOTEUR ParamClignoteurC = {GPIOC, 11, 1, 2};

static void prvSetupHardware(void);
void init_gpio(void);

int main(void)
{
    static uint32_t oscille_plantage = 1 << 13; // Avoid main stack issues (use static)
    
    // Minimal hardware initialization
    prvSetupHardware();

    // Initialize all tasks (this now creates:
    //    - the Mode Management task (vTaskModeManagement),
    //    - the ADC check_seuil task (vTaskCheckSeuil),
    //    - the reed scanning task (vTaskScannerReed))
    vInit_myTasks(mainLED_TASK_PRIORITY);

    // Uncomment these if you want to run the Clignoteur tasks concurrently
    // xTaskCreate(ClignoteurTask, "ClignoteurA", 128, &ParamClignoteurA, 1, NULL);
    // xTaskCreate(ClignoteurTask, "ClignoteurB", 128, &ParamClignoteurB, 1, NULL);
    // xTaskCreate(ClignoteurTask, "ClignoteurC", 128, &ParamClignoteurC, 1, NULL);

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    // Should never reach here; in case of failure, blink PB13 as error indication.
    for (;;) {
         GPIOB->BSRR = oscille_plantage;
         oscille_plantage ^= (1 << 13);
    }
}

void init_gpio(void)
{
		int pin;
    /*
     * We must initialize all pins that are used in the application.
     * 
     * The following pins are used:
     * 
     * 1. GPIOA:
     *    - PA0 is used by the ADC (configured in ADC1_Init).
     *    - PA5 is used as the output for the WS2812 LED control.
     *    - PA6, PA7, PA8, PA9, PA10 are used as reed “column” inputs.
     *      They must be configured as inputs with pull-up/pull-down.
     *
     * 2. GPIOB:
     *    - Reed “row” lines: defined by PREMIERE_LIGNE_PIN and used for scanning;
     *      here, we need to initialize PB1 to PB12 as outputs.
     *    - PB13 is used for transistor control and error blinking.
     *
     * 3. GPIOC:
     *    - PC12 and PC13 (pins 12 and 13 per our definitions) are used for buttons.
     *
     * Other pins (like those used by Clignoteur tasks on GPIOC) are commented.
     */

    // ---- GPIOA Initialization ----
    // PA5: WS2812 LED output.
    init_GPIOx(GPIOA, 5, GPIO_MODE_OUTPUT_PP_50MHz);

    // PA6 to PA10: Reed column inputs.
    init_GPIOx(GPIOA, 6, GPIO_MODE_INPUT_PULL_UP_DOWN);
    init_GPIOx(GPIOA, 7, GPIO_MODE_INPUT_PULL_UP_DOWN);
    init_GPIOx(GPIOA, 8, GPIO_MODE_INPUT_PULL_UP_DOWN);
    init_GPIOx(GPIOA, 9, GPIO_MODE_INPUT_PULL_UP_DOWN);
    init_GPIOx(GPIOA, 10, GPIO_MODE_INPUT_PULL_UP_DOWN);

    // ---- GPIOB Initialization ----
    // Reed row outputs: initialize PB1 to PB12 as outputs.
    for (pin = 1; pin <= 12; pin++) {
        init_GPIOx(GPIOB, pin, GPIO_MODE_OUTPUT_PP_50MHz);
    }
    // PB13: Transistor control and error indicator.
    init_GPIOx(GPIOB, 13, GPIO_MODE_OUTPUT_PP_50MHz);

    // ---- GPIOC Initialization ----
    // Buttons on PC12 and PC13.
    init_GPIOx(GPIOC, BP_DEMANDE_TEST_PIN, GPIO_MODE_INPUT_PULL_UP_DOWN);
    init_GPIOx(GPIOC, BP_DEMANDE_APPRO_PIN, GPIO_MODE_INPUT_PULL_UP_DOWN);

    // (Optional) Clignoteur outputs on GPIOC if used:
    // init_GPIOx(GPIOC, 9, GPIO_MODE_OUTPUT_PP_50MHz);
    // init_GPIOx(GPIOC, 10, GPIO_MODE_OUTPUT_PP_50MHz);
    // init_GPIOx(GPIOC, 11, GPIO_MODE_OUTPUT_PP_50MHz);
}

// USART2 initialization
void init_USART2(void) {
    RCC->APB1ENR |= (1 << 17);
    init_GPIOx(GPIOA, 2, 0x09); //TODO change UART pins
    init_GPIOx(GPIOA, 3, 0x08); //TODO change UART pins

    USART2->BRR = 36000000 / 9600;
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
    USART2->CR2 = 0x0000;
    USART2->CR3 = 0x0000;
}

static void prvSetupHardware(void)
{
    init_gpio();
		TIM2_Config();
    // Additional hardware initialization (e.g., clocks, timers) can be added here.
}
