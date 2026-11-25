/*
 * NUCLEO-F429ZI: bxCAN loopback test (bring-up step 3).
 *
 * CAN1 runs at 500 kbps in loopback mode: every transmitted frame is fed
 * back to the controller's own receiver, so the CAN setup (clock, bit
 * timing, filter, RX interrupt) is tested without the other board.
 *
 * Every 500 ms a frame with ID 0x123 and an incrementing counter is sent.
 * The RX interrupt checks that the received frame matches what was sent.
 *
 *   LD2 (blue, PB7)   toggles on every frame sent
 *   LD1 (green, PB0)  toggles on every frame received that matches
 *   LD3 (red, PB14)   on for good after a mismatch, a CAN error or a failed
 *                     HAL call
 *
 * Working: blue and green both blink at 1 Hz, red stays off.
 */
#include "stm32f4xx_hal.h"

#define LED_GREEN   GPIO_PIN_0
#define LED_BLUE    GPIO_PIN_7
#define LED_RED     GPIO_PIN_14

#define TEST_ID     0x123U
#define TEST_DLC    8U

CAN_HandleTypeDef hcan1;

/* Last frame sent, compared against what comes back. Written in main,
 * read in the RX interrupt. */
static volatile uint32_t tx_count;

static void SystemClock_Config(void);
static void LED_Init(void);
static void CAN1_Init(void);
static void CAN1_FilterAcceptAll(void);
static void fill_payload(uint8_t data[8], uint32_t count);
void Error_Handler(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    LED_Init();
    CAN1_Init();
    CAN1_FilterAcceptAll();

    if (HAL_CAN_ActivateNotification(&hcan1,
            CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR | CAN_IT_BUSOFF |
            CAN_IT_LAST_ERROR_CODE) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        Error_Handler();
    }

    CAN_TxHeaderTypeDef tx = {
        .StdId              = TEST_ID,
        .IDE                = CAN_ID_STD,
        .RTR                = CAN_RTR_DATA,
        .DLC                = TEST_DLC,
        .TransmitGlobalTime = DISABLE,
    };
    uint8_t data[8];
    uint32_t mailbox;

    while (1) {
        HAL_Delay(500);

        /* With one frame every 500 ms a mailbox is always free; if none is,
         * the previous frames never left, which is itself the fault. */
        if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
            Error_Handler();
        }

        tx_count++;
        fill_payload(data, tx_count);
        if (HAL_CAN_AddTxMessage(&hcan1, &tx, data, &mailbox) != HAL_OK) {
            Error_Handler();
        }
        HAL_GPIO_TogglePin(GPIOB, LED_BLUE);
    }
}

/* Payload: counter in bytes 0-3 (little-endian), fixed pattern in 4-7. */
static void fill_payload(uint8_t data[8], uint32_t count)
{
    data[0] = (uint8_t)(count);
    data[1] = (uint8_t)(count >> 8);
    data[2] = (uint8_t)(count >> 16);
    data[3] = (uint8_t)(count >> 24);
    data[4] = 0xCA;
    data[5] = 0xFE;
    data[6] = 0xBA;
    data[7] = 0xBE;
}

/* Called from HAL_CAN_IRQHandler when FIFO0 holds a frame */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx;
    uint8_t data[8];
    uint8_t expected[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx, data) != HAL_OK) {
        Error_Handler();
    }

    fill_payload(expected, tx_count);
    int ok = rx.IDE == CAN_ID_STD && rx.StdId == TEST_ID &&
             rx.RTR == CAN_RTR_DATA && rx.DLC == TEST_DLC;
    for (uint32_t i = 0; ok && i < TEST_DLC; i++) {
        ok = data[i] == expected[i];
    }

    if (ok) {
        HAL_GPIO_TogglePin(GPIOB, LED_GREEN);
    } else {
        Error_Handler();
    }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    (void)hcan;
    Error_Handler();
}

/*
 * 500 kbps from the 45 MHz APB1 clock (see README, Bit Timing):
 * tq = 6 / 45 MHz = 133.3 ns, bit = (1 + 12 + 2) tq = 2 us, sample point 86.7%.
 */
static void CAN1_Init(void)
{
    hcan1.Instance                  = CAN1;
    hcan1.Init.Prescaler            = 6;
    hcan1.Init.Mode                 = CAN_MODE_LOOPBACK;
    hcan1.Init.SyncJumpWidth        = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1             = CAN_BS1_12TQ;
    hcan1.Init.TimeSeg2             = CAN_BS2_2TQ;
    hcan1.Init.TimeTriggeredMode    = DISABLE;
    hcan1.Init.AutoBusOff           = ENABLE;
    hcan1.Init.AutoWakeUp           = DISABLE;
    hcan1.Init.AutoRetransmission   = ENABLE;
    hcan1.Init.ReceiveFifoLocked    = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;
    if (HAL_CAN_Init(&hcan1) != HAL_OK) {
        Error_Handler();
    }
}

/*
 * Without an active filter bxCAN discards every received frame, including
 * its own in loopback. Mask 0 means no ID bits are compared, so every frame
 * is accepted into FIFO0. Narrowed to Project 1's IDs once the bus works.
 */
static void CAN1_FilterAcceptAll(void)
{
    CAN_FilterTypeDef f = {
        .FilterBank           = 0,
        .FilterMode           = CAN_FILTERMODE_IDMASK,
        .FilterScale          = CAN_FILTERSCALE_32BIT,
        .FilterIdHigh         = 0,
        .FilterIdLow          = 0,
        .FilterMaskIdHigh     = 0,
        .FilterMaskIdLow      = 0,
        .FilterFIFOAssignment = CAN_RX_FIFO0,
        .FilterActivation     = CAN_FILTER_ENABLE,
        .SlaveStartFilterBank = 14,  /* banks 0-13 CAN1, 14-27 CAN2 */
    };
    if (HAL_CAN_ConfigFilter(&hcan1, &f) != HAL_OK) {
        Error_Handler();
    }
}

/* Called by HAL_CAN_Init: clocks, pins and interrupts for CAN1 */
void HAL_CAN_MspInit(CAN_HandleTypeDef *hcan)
{
    GPIO_InitTypeDef gpio = {0};

    if (hcan->Instance != CAN1) {
        return;
    }

    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* PD0 = CAN1_RX, PD1 = CAN1_TX (AF9) */
    gpio.Pin       = GPIO_PIN_0 | GPIO_PIN_1;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOD, &gpio);

    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
}

/*
 * HSE 8 MHz (ST-LINK MCO, bypass) -> PLL: /M=4 = 2 MHz x N=180 = 360 MHz VCO
 *   P /2 = 180 MHz SYSCLK.
 * AHB = 180 MHz, APB1 = /4 = 45 MHz (bxCAN clock), APB2 = /2 = 90 MHz.
 */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState       = RCC_HSE_BYPASS;
    osc.PLL.PLLState   = RCC_PLL_ON;
    osc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM       = 4;
    osc.PLL.PLLN       = 180;
    osc.PLL.PLLP       = RCC_PLLP_DIV2;
    osc.PLL.PLLQ       = 8;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        Error_Handler();
    }

    /* Over-drive is required to run above 168 MHz */
    if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
        Error_Handler();
    }

    clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                         RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV4;
    clk.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

/* LD1 green PB0, LD2 blue PB7, LD3 red PB14 */
static void LED_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio.Pin   = LED_GREEN | LED_BLUE | LED_RED;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);
}

/* Red LED on, everything stopped. Inspect hcan1.ErrorCode in a debugger. */
void Error_Handler(void)
{
    __disable_irq();
    HAL_GPIO_WritePin(GPIOB, LED_GREEN | LED_BLUE, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, LED_RED, GPIO_PIN_SET);
    while (1) {
    }
}
