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

/* CAN1 FIFO0 message pending -> HAL_CAN_RxFifo0MsgPendingCallback() */
extern CAN_HandleTypeDef hcan1;

void CAN1_RX0_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan1);
}

/* CAN1 status change / error -> HAL_CAN_ErrorCallback() */
void CAN1_SCE_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan1);
}
