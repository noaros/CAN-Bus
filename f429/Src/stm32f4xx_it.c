/* Interrupt handlers for NUCLEO-F429ZI */
#include "stm32f4xx_hal.h"

void NMI_Handler(void)
{
    while (1) {
    }
}

void HardFault_Handler(void)
{
    while (1) {
    }
}

/* 1 ms tick used by HAL_Delay() and HAL timeouts */
void SysTick_Handler(void)
{
    HAL_IncTick();
}
