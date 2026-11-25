/* Interrupt handlers for NUCLEO-H723ZG */
#include "stm32h7xx_hal.h"

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

/* FDCAN1 interrupt line 0 (all FDCAN1 interrupts by default)
 * -> HAL_FDCAN_RxFifo0Callback() / HAL_FDCAN_ErrorStatusCallback() / ... */
extern FDCAN_HandleTypeDef hfdcan1;

void FDCAN1_IT0_IRQHandler(void)
{
    HAL_FDCAN_IRQHandler(&hfdcan1);
}
