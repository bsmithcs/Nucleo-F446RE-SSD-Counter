/****************************************************************
 *    TEAM 07: B. Smith and J. Zawatsky 
 *    CPEG222 Proj2A, 9/26/25
 *    NucleoF466RE CMSIS STM32F4xx 
 *    Counter w/ SSD 
 ****************************************************************/

#include "stm32f4xx.h"
#include <stdbool.h>

// SSDs
// !Insert port-pins for SSDs
#define SSD_PORT_A GPIOA
#define SSD_PORT_B GPIOB
#define SSD_PORT_C GPIOC

#define SSD_AA_C 7
#define SSD_AB_A 9
#define SSD_AC_A 8
#define SSD_AD_B 10
#define SSD_AE_B 4
#define SSD_AF_B 5
#define SSD_AG_B 3
#define SSD_CAT_A 10

// Digit Definitions
const unsigned int digitSegments[] = {
    0b0111111, // 0
    0b0000110, // 1
    0b1011011, // 2
    0b1001111, // 3
    0b1100110, // 4
    0b1101101, // 5
    0b1111101, // 6
    0b0000111, // 7
    0b1111111, // 8
    0b1101111, // 9
    0b0000000 // blank/off
};

volatile int count = 0;
volatile bool digitSelect = true;

/// @brief This function sets the given port:pin as an output and clears its data
/// @param port: GPIO port to be set as output 
/// @param pin: GPIO pin to be set as output 
void set_output(GPIO_TypeDef *port, int pin) {
    port->MODER &= ~(0x3 << (pin * 2));
    port->MODER |= (0x1 << (pin * 2));

    port->ODR &= ~(0x1 << pin);
}

/// @brief This function displays the given digit w/ the current CAT selection
/// @param digit: Digit to be displayed (0 to 10, 10 being Blank) 
void display_digit(int digit) {

    SSD_PORT_A->ODR &= ~((1 << SSD_AB_A) | (1 << SSD_AC_A));
    SSD_PORT_B->ODR &= ~((1 << SSD_AD_B) | (1 << SSD_AE_B) 
                        | (1 << SSD_AF_B) | (1 << SSD_AG_B));
    SSD_PORT_C->ODR &= ~(1 << SSD_AA_C);

    if (digitSegments[digit] & 0b0000001) SSD_PORT_C->ODR |= (1 << SSD_AA_C); 
    if (digitSegments[digit] & 0b0000010) SSD_PORT_A->ODR |= (1 << SSD_AB_A); 
    if (digitSegments[digit] & 0b0000100) SSD_PORT_A->ODR |= (1 << SSD_AC_A); 
    if (digitSegments[digit] & 0b0001000) SSD_PORT_B->ODR |= (1 << SSD_AD_B); 
    if (digitSegments[digit] & 0b0010000) SSD_PORT_B->ODR |= (1 << SSD_AE_B); 
    if (digitSegments[digit] & 0b0100000) SSD_PORT_B->ODR |= (1 << SSD_AF_B); 
    if (digitSegments[digit] & 0b1000000) SSD_PORT_B->ODR |= (1 << SSD_AG_B); 
}

/// @brief This function configures TIM2 with interrupts (assuming 16MHz HSI clock)
/// @param period: Period between interrupts in microseconds 
/// @param priority: Priority of interrupt (0 to 255)
void Timer2_Config(int period, int priority) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Enable TIM2 clock
    TIM2->PSC = 15; // Prescaler: (16MHz/(15+1) = 1MHz, 1usec period)
    TIM2->ARR = period - 1; // Auto-reload when CNT = XX: (period = XX usec)
    TIM2->DIER |= TIM_DIER_UIE; // Enable update interrupt
    TIM2->SR &= ~TIM_SR_UIF; // Clear any pending interrupt
    NVIC_EnableIRQ(TIM2_IRQn); // Enable TIM2 interrupt in NVIC
    NVIC_SetPriority(TIM2_IRQn, priority); // Set priority for TIM2
    TIM2->CR1 = TIM_CR1_CEN; // Enable TIM2
}

void SysTick_Handler(void) {
    // Update current count
    count = (count + 1) % 100; 
}

void TIM2_IRQHandler(void){
    if(TIM2->SR & TIM_SR_UIF){ // Check if the update interrupt flag is set
        if (digitSelect) { // If digitSelect is true, update the first digit
            SSD_PORT_A->ODR |= (1 << SSD_CAT_A); // Turn on CAT pin for first digit
            int firstDigit = count / 10; // Get the first digit
            if (firstDigit == 0) { // If first digit is 0 turn off all segments
                display_digit(10);
            }
            else {
                display_digit(firstDigit);
            }
        }
        else {
            SSD_PORT_A->ODR &= ~(1 << SSD_CAT_A); // Turn off CAT pin for second digit
            int secondDigit = count % 10; // Get the second digit
            display_digit(secondDigit);
        }

        digitSelect = !digitSelect; // Toggle digitSelect for next interrupt
        TIM2->SR &= ~TIM_SR_UIF; // Clear the update interrupt flag
    }
}



int main(void) {
    // Enable Port A
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN_Msk; 
    // Enable Port B
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN_Msk; 
    // Enable Port C
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN_Msk;

    // Set up SSD pins as outputs
    set_output(SSD_PORT_C, SSD_AA_C);
    set_output(SSD_PORT_A, SSD_AB_A);
    set_output(SSD_PORT_A, SSD_AC_A);
    set_output(SSD_PORT_B, SSD_AD_B);
    set_output(SSD_PORT_B, SSD_AE_B);
    set_output(SSD_PORT_B, SSD_AF_B);
    set_output(SSD_PORT_B, SSD_AG_B);
    set_output(SSD_PORT_A, SSD_CAT_A);

    // Initialization

    // Set up SysTick for 1 sec interrupts
    SysTick_Config(16000000 * 1 - 1);

    // Enable Systick Interrupts
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN_Msk;

    // Timer Config with 500 usec interrupts at priority level 1
    Timer2_Config(500, 1);

    while (1) {
        // Contiuous Execution
    }

    return 0;
}
