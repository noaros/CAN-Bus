/*
 * NUCLEO-H723ZG: FDCAN loopback test (bring-up step 5).
 *
 * FDCAN1 runs classic CAN at 500 kbps in external loopback mode: every
 * transmitted frame is fed back to the controller's own receiver, so the
 * FDCAN setup (kernel clock, bit timing, message RAM, filter, RX interrupt)
 * is tested without the other board. Same test as the F429's step 3.
 *
 * Every 500 ms a frame with ID 0x123 and an incrementing counter is sent.
 * The RX interrupt checks that the received frame matches what was sent.
 *
 *   LD2 (yellow, PE1)  toggles on every frame sent
 *   LD1 (green, PB0)   toggles on every frame received that matches
 *   LD3 (red, PB14)    on for good after a mismatch, a CAN error or a
 *                      failed HAL call
 *
 * Working: yellow and green both blink at 1 Hz, red stays off.
 *
 * For checking over SWD: clk_sys_hz and clk_fdcan_hz hold the clock
 * frequencies the HAL computes from the RCC registers; tx_count and
 * rx_ok_count count frames sent and frames received that matched.
 */
#include "stm32h7xx_hal.h"

#define LED_GREEN   GPIO_PIN_0    /* LD1, PB0 */
#define LED_RED     GPIO_PIN_14   /* LD3, PB14 */
#define LED_YELLOW  GPIO_PIN_1    /* LD2, PE1 */

#define TEST_ID     0x123U
#define TEST_DLC    8U

FDCAN_HandleTypeDef hfdcan1;

volatile uint32_t clk_sys_hz;
volatile uint32_t clk_fdcan_hz;

/* Last frame sent, compared against what comes back. Written in main,
 * read in the RX interrupt. */
volatile uint32_t tx_count;
volatile uint32_t rx_ok_count;

static void SystemClock_Config(void);
static void LED_Init(void);
static void FDCAN1_Init(void);
static void FDCAN1_FilterAcceptAll(void);
static void fill_payload(uint8_t data[8], uint32_t count);
void Error_Handler(void);

int main(void)
{
    HAL_Init();
    LED_Init();             /* first, so Error_Handler can light LD3 */
    SystemClock_Config();

    clk_sys_hz   = HAL_RCC_GetSysClockFreq();
    clk_fdcan_hz = (uint32_t)HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN);

    FDCAN1_Init();
    FDCAN1_FilterAcceptAll();

    if (HAL_FDCAN_ActivateNotification(&hfdcan1,
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_MESSAGE_LOST |
            FDCAN_IT_ARB_PROTOCOL_ERROR | FDCAN_IT_ERROR_WARNING |
            FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_BUS_OFF, 0) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        Error_Handler();
    }

    FDCAN_TxHeaderTypeDef tx = {
        .Identifier          = TEST_ID,
        .IdType              = FDCAN_STANDARD_ID,
        .TxFrameType         = FDCAN_DATA_FRAME,
        .DataLength          = FDCAN_DLC_BYTES_8,
        .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
        .BitRateSwitch       = FDCAN_BRS_OFF,
        .FDFormat            = FDCAN_CLASSIC_CAN,
        .TxEventFifoControl  = FDCAN_NO_TX_EVENTS,
        .MessageMarker       = 0,
    };
    uint8_t data[8];

    while (1) {
        HAL_Delay(500);

        /* With one frame every 500 ms the TX FIFO is never full; if it is,
         * the previous frames never left, which is itself the fault. */
        if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0) {
            Error_Handler();
        }

        tx_count++;
        fill_payload(data, tx_count);
        if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx, data) != HAL_OK) {
            Error_Handler();
        }
        HAL_GPIO_TogglePin(GPIOE, LED_YELLOW);
    }
}

/* Payload: counter in bytes 0-3 (little-endian), fixed pattern in 4-7.
 * Same as the F429. */
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

/* Called from HAL_FDCAN_IRQHandler for Rx FIFO 0 events */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) {
        Error_Handler();
    }
    if (!(RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)) {
        return;
    }

    FDCAN_RxHeaderTypeDef rx;
    uint8_t data[8];
    uint8_t expected[8];

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx, data) != HAL_OK) {
        Error_Handler();
    }

    fill_payload(expected, tx_count);
    int ok = rx.IdType == FDCAN_STANDARD_ID && rx.Identifier == TEST_ID &&
             rx.RxFrameType == FDCAN_DATA_FRAME &&
             rx.FDFormat == FDCAN_CLASSIC_CAN && rx.DataLength == TEST_DLC;
    for (uint32_t i = 0; ok && i < TEST_DLC; i++) {
        ok = data[i] == expected[i];
    }

    if (ok) {
        rx_ok_count++;
        HAL_GPIO_TogglePin(GPIOB, LED_GREEN);
    } else {
        Error_Handler();
    }
}

/* Error warning, error passive or bus-off status changed */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
    (void)hfdcan;
    (void)ErrorStatusITs;
    Error_Handler();
}

/* Protocol error, message RAM access failure, etc. (hfdcan1.ErrorCode) */
void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
    (void)hfdcan;
    Error_Handler();
}

/*
 * 500 kbps classic CAN from the 80 MHz FDCAN kernel clock (see README,
 * Bit Timing): tq = 10 / 80 MHz = 125 ns, bit = (1 + 13 + 2) tq = 2 us,
 * sample point 87.5%. Message RAM sizes as in the README's FDCAN message
 * RAM table.
 */
static void FDCAN1_Init(void)
{
    hfdcan1.Instance                  = FDCAN1;
    hfdcan1.Init.FrameFormat          = FDCAN_FRAME_CLASSIC;
    hfdcan1.Init.Mode                 = FDCAN_MODE_EXTERNAL_LOOPBACK;
    hfdcan1.Init.AutoRetransmission   = ENABLE;
    hfdcan1.Init.TransmitPause        = DISABLE;
    hfdcan1.Init.ProtocolException    = DISABLE;

    hfdcan1.Init.NominalPrescaler     = 10;
    hfdcan1.Init.NominalSyncJumpWidth = 1;
    hfdcan1.Init.NominalTimeSeg1      = 13;
    hfdcan1.Init.NominalTimeSeg2      = 2;
    /* Data phase is only used by CAN FD with bit rate switching; set to
     * the minimum valid values */
    hfdcan1.Init.DataPrescaler        = 1;
    hfdcan1.Init.DataSyncJumpWidth    = 1;
    hfdcan1.Init.DataTimeSeg1         = 1;
    hfdcan1.Init.DataTimeSeg2         = 1;

    hfdcan1.Init.MessageRAMOffset     = 0;
    hfdcan1.Init.StdFiltersNbr        = 1;
    hfdcan1.Init.ExtFiltersNbr        = 0;
    hfdcan1.Init.RxFifo0ElmtsNbr      = 8;
    hfdcan1.Init.RxFifo0ElmtSize      = FDCAN_DATA_BYTES_8;
    hfdcan1.Init.RxFifo1ElmtsNbr      = 0;
    hfdcan1.Init.RxFifo1ElmtSize      = FDCAN_DATA_BYTES_8;
    hfdcan1.Init.RxBuffersNbr         = 0;
    hfdcan1.Init.RxBufferSize         = FDCAN_DATA_BYTES_8;
    hfdcan1.Init.TxEventsNbr          = 0;
    hfdcan1.Init.TxBuffersNbr         = 0;
    hfdcan1.Init.TxFifoQueueElmtsNbr  = 8;
    hfdcan1.Init.TxFifoQueueMode      = FDCAN_TX_FIFO_OPERATION;
    hfdcan1.Init.TxElmtSize           = FDCAN_DATA_BYTES_8;
    if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) {
        Error_Handler();
    }
}

/*
 * Unlike bxCAN, FDCAN's global filter accepts frames that match no filter
 * into Rx FIFO 0 by default. Set it to reject them, so frames only get in
 * through the configured filter, as on the F429. That filter is a classic
 * ID/mask filter with mask 0: no ID bits compared, every standard ID
 * accepted. Narrowed to Project 1's IDs once the bus works.
 */
static void FDCAN1_FilterAcceptAll(void)
{
    FDCAN_FilterTypeDef f = {
        .IdType       = FDCAN_STANDARD_ID,
        .FilterIndex  = 0,
        .FilterType   = FDCAN_FILTER_MASK,
        .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
        .FilterID1    = 0x000,    /* ID */
        .FilterID2    = 0x000,    /* mask */
    };
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &f) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
            FDCAN_REJECT, FDCAN_REJECT,                       /* non-matching std, ext */
            FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK) {
        Error_Handler();
    }
}

/* Called by HAL_FDCAN_Init: clocks, pins and interrupt for FDCAN1.
 * The kernel clock source (PLL1Q) is selected in SystemClock_Config. */
void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hfdcan)
{
    GPIO_InitTypeDef gpio = {0};

    if (hfdcan->Instance != FDCAN1) {
        return;
    }

    __HAL_RCC_FDCAN_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* PD0 = FDCAN1_RX, PD1 = FDCAN1_TX (AF9) */
    gpio.Pin       = GPIO_PIN_0 | GPIO_PIN_1;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOD, &gpio);

    /* All FDCAN1 interrupts are on line 0 by default */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
}

/*
 * HSE 8 MHz (ST-LINK MCO, bypass) -> PLL1: /M=2 = 4 MHz x N=200 = 800 MHz VCO
 *   P /2 = 400 MHz SYSCLK (CPU)
 *   Q /10 = 80 MHz -> FDCAN kernel clock (sets the CAN bit timing)
 *   R /2 = 400 MHz (unused)
 * AHB = /2 = 200 MHz, APB1-4 = /2 = 100 MHz.
 * Voltage scale 0 allows up to 550 MHz; flash needs 2 wait states at 200 MHz AXI.
 */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};
    RCC_PeriphCLKInitTypeDef periph = {0};

    /* The H723 has no SMPS: the core supply comes from the internal LDO */
    if (HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY) != HAL_OK) {
        Error_Handler();
    }
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
    }

    osc.OscillatorType  = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState        = RCC_HSE_BYPASS;
    osc.PLL.PLLState    = RCC_PLL_ON;
    osc.PLL.PLLSource   = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM        = 2;
    osc.PLL.PLLN        = 200;
    osc.PLL.PLLP        = 2;
    osc.PLL.PLLQ        = 10;
    osc.PLL.PLLR        = 2;
    osc.PLL.PLLRGE      = RCC_PLL1VCIRANGE_2;   /* 4 MHz PLL input: 4-8 MHz range */
    osc.PLL.PLLVCOSEL   = RCC_PLL1VCOWIDE;      /* 800 MHz VCO: wide range */
    osc.PLL.PLLFRACN    = 0;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        Error_Handler();
    }

    clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                         RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                         RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.SYSCLKDivider  = RCC_SYSCLK_DIV1;
    clk.AHBCLKDivider  = RCC_HCLK_DIV2;
    clk.APB3CLKDivider = RCC_APB3_DIV2;
    clk.APB1CLKDivider = RCC_APB1_DIV2;
    clk.APB2CLKDivider = RCC_APB2_DIV2;
    clk.APB4CLKDivider = RCC_APB4_DIV2;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }

    /* FDCANSEL mux: PLL1Q (80 MHz) */
    periph.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    periph.FdcanClockSelection  = RCC_FDCANCLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&periph) != HAL_OK) {
        Error_Handler();
    }
}

/* LD1 green PB0, LD3 red PB14, LD2 yellow PE1 */
static void LED_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Pin   = LED_GREEN | LED_RED;
    HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Pin   = LED_YELLOW;
    HAL_GPIO_Init(GPIOE, &gpio);
}

/* Red LED on, everything stopped. Inspect hfdcan1.ErrorCode in a debugger. */
void Error_Handler(void)
{
    __disable_irq();
    HAL_GPIO_WritePin(GPIOB, LED_GREEN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, LED_YELLOW, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, LED_RED, GPIO_PIN_SET);
    while (1) {
    }
}
