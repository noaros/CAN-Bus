/*
 * NUCLEO-F429ZI: exchange with the H723 over the real bus, with hardware
 * acceptance filtering (bring-up step 7).
 *
 * CAN1 runs at 500 kbps in normal mode. Every 500 ms it sends ID 0x100
 * (P1_ID_F429) with an incrementing counter, and it receives the H723's
 * ID 0x200 frames, checking the ID, DLC, pattern and that the counter goes
 * up by exactly 1. Frame layout is shared with the H723 in
 * common/project1_frames.h.
 *
 * The filter accepts only ID 0x200. To test it, each board also sends a
 * decoy (here 0x101) that the other board's filter must drop in hardware;
 * a decoy reaching software would be counted in rx.bad. Build with
 * `make FILTER=all` for the accept-all control, where decoys do get through.
 *
 *   LD2 (blue, PB7)   toggles on every frame queued for sending
 *   LD1 (green, PB0)  toggles on every good frame from the H723
 *   LD3 (red, PB14)   on while there is a problem: error passive or bus-off,
 *                     or no good frame from the H723 in the last 1.5 s.
 *                     Goes off again by itself once the bus recovers.
 *
 * Working: blue and green both blink at 1 Hz, red stays off.
 *
 * Errors don't stop the program here, unlike step 3: on a real bus they are
 * expected while the other board is missing (no one acknowledges, so the
 * error counters rise to error passive), and CAN recovers by itself once it
 * returns. Only a failed HAL setup call stops everything (red on, others off).
 *
 * For checking over SWD: tx_count, tx_skipped, decoy_count, rx
 * (p1_rx_stats_t), rx_overruns and bus (error counters and state,
 * refreshed every 500 ms).
 */
#include "stm32f4xx_hal.h"
#include "project1_frames.h"

#define LED_GREEN   GPIO_PIN_0
#define LED_BLUE    GPIO_PIN_7
#define LED_RED     GPIO_PIN_14

#define RX_TIMEOUT_MS  1500U

CAN_HandleTypeDef hcan1;

volatile uint32_t tx_count;     /* frames queued for sending */
volatile uint32_t tx_skipped;   /* sends skipped because fewer than 2 mailboxes were free */
volatile uint32_t decoy_count;  /* decoy frames queued for sending */
volatile p1_rx_stats_t rx;      /* frames received from the H723 */
volatile uint32_t rx_overruns;  /* FIFO0 overruns: frames lost because it was full */
volatile p1_err_counts_t errs;  /* bus errors by type (see project1_frames.h) */

/* Snapshot of CAN1's error status register (ESR) */
volatile struct {
    uint32_t tec;           /* transmit error counter */
    uint32_t rec;           /* receive error counter */
    uint32_t lec;           /* last error code: 0 none, 3 ACK, ... (see README) */
    uint32_t error_passive;
    uint32_t bus_off;
} bus;

static void SystemClock_Config(void);
static void LED_Init(void);
static void CAN1_Init(void);
static void CAN1_Filter(void);
static void update_bus_status(void);
void Error_Handler(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    LED_Init();
    CAN1_Init();
    CAN1_Filter();

    if (HAL_CAN_ActivateNotification(&hcan1,
            CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_OVERRUN |
            CAN_IT_ERROR | CAN_IT_LAST_ERROR_CODE) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        Error_Handler();
    }

    CAN_TxHeaderTypeDef tx = {
        .StdId              = P1_ID_F429,
        .IDE                = CAN_ID_STD,
        .RTR                = CAN_RTR_DATA,
        .DLC                = P1_DLC,
        .TransmitGlobalTime = DISABLE,
    };
    CAN_TxHeaderTypeDef decoy = tx;
    decoy.StdId = P1_ID_F429_DECOY;
    uint8_t data[8];
    uint32_t mailbox;

    while (1) {
        HAL_Delay(500);
        update_bus_status();

        /* Mailboxes stay busy while no node acknowledges (the controller
         * keeps retrying). Skip rather than stop; sending resumes once the
         * H723 is back. Two are needed: the frame and the decoy. */
        if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) < 2) {
            tx_skipped++;
            continue;
        }

        tx_count++;
        p1_fill_payload(data, tx_count);
        if (HAL_CAN_AddTxMessage(&hcan1, &tx, data, &mailbox) != HAL_OK) {
            Error_Handler();
        }
        /* Same payload under the decoy ID: the H723 must never see it */
        decoy_count++;
        if (HAL_CAN_AddTxMessage(&hcan1, &decoy, data, &mailbox) != HAL_OK) {
            Error_Handler();
        }
        HAL_GPIO_TogglePin(GPIOB, LED_BLUE);
    }
}

/* Refresh the bus snapshot and set the red LED from it */
static void update_bus_status(void)
{
    uint32_t esr = hcan1.Instance->ESR;

    bus.tec           = (esr & CAN_ESR_TEC) >> CAN_ESR_TEC_Pos;
    bus.rec           = (esr & CAN_ESR_REC) >> CAN_ESR_REC_Pos;
    bus.lec           = (esr & CAN_ESR_LEC) >> CAN_ESR_LEC_Pos;
    bus.error_passive = (esr & CAN_ESR_EPVF) != 0;
    bus.bus_off       = (esr & CAN_ESR_BOFF) != 0;

    int quiet = rx.ok == 0 || (HAL_GetTick() - rx.last_ok_tick) > RX_TIMEOUT_MS;
    int problem = bus.error_passive || bus.bus_off || quiet;
    HAL_GPIO_WritePin(GPIOB, LED_RED, problem ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* Called from HAL_CAN_IRQHandler when FIFO0 holds a frame */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef h;
    uint8_t data[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &h, data) != HAL_OK) {
        Error_Handler();
    }

    int ok = h.IDE == CAN_ID_STD && h.StdId == P1_ID_H723 &&
             h.RTR == CAN_RTR_DATA && h.DLC == P1_DLC &&
             p1_payload_pattern_ok(data);
    if (!ok) {
        rx.bad++;
        return;
    }
    p1_rx_count(&rx, p1_payload_count(data), HAL_GetTick());
    HAL_GPIO_TogglePin(GPIOB, LED_GREEN);
}

/* FIFO0 overrun and bus errors (by last error code): count and carry on */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    uint32_t e = hcan->ErrorCode;

    if (e & HAL_CAN_ERROR_RX_FOV0) rx_overruns++;
    if (e & HAL_CAN_ERROR_STF)     errs.lec[1]++;
    if (e & HAL_CAN_ERROR_FOR)     errs.lec[2]++;
    if (e & HAL_CAN_ERROR_ACK)     errs.lec[3]++;
    if (e & HAL_CAN_ERROR_BR)      errs.lec[4]++;
    if (e & HAL_CAN_ERROR_BD)      errs.lec[5]++;
    if (e & HAL_CAN_ERROR_CRC)     errs.lec[6]++;
    HAL_CAN_ResetError(hcan);
}

/*
 * 500 kbps from the 45 MHz APB1 clock (see README, Bit Timing):
 * tq = 6 / 45 MHz = 133.3 ns, bit = (1 + 12 + 2) tq = 2 us, sample point 86.7%.
 */
static void CAN1_Init(void)
{
    hcan1.Instance                  = CAN1;
    hcan1.Init.Prescaler            = 6;
    hcan1.Init.Mode                 = CAN_MODE_NORMAL;
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
 * Bank 0, 32-bit ID/mask mode. In 32-bit scale the registers hold the
 * frame's identifier field as bxCAN stores it, not the plain ID:
 *
 *   bits 31:21  standard ID (11 bits)    -> ID << 21, or << 5 in the high half
 *   bits 20:3   extended ID bits 17:0    (unused for standard frames)
 *   bit  2      IDE: 0 standard, 1 extended
 *   bit  1      RTR: 0 data, 1 remote
 *
 * Mask bit 1 = must match, 0 = don't care. This mask compares all 11 ID
 * bits plus IDE and RTR, so only standard data frames with ID 0x200 pass.
 *
 * With -DP1_FILTER_ACCEPT_ALL (make FILTER=all) the mask is 0 instead, so
 * every frame passes: the control for step 7's decoy test.
 */
static void CAN1_Filter(void)
{
#ifdef P1_FILTER_ACCEPT_ALL
    const uint32_t id = 0, mask = 0;
#else
    const uint32_t id   = (P1_ID_H723 << 21);         /* IDE = 0, RTR = 0 */
    const uint32_t mask = (0x7FFU << 21) | CAN_ID_EXT | CAN_RTR_REMOTE;
#endif
    CAN_FilterTypeDef f = {
        .FilterBank           = 0,
        .FilterMode           = CAN_FILTERMODE_IDMASK,
        .FilterScale          = CAN_FILTERSCALE_32BIT,
        .FilterIdHigh         = id >> 16,
        .FilterIdLow          = id & 0xFFFF,
        .FilterMaskIdHigh     = mask >> 16,
        .FilterMaskIdLow      = mask & 0xFFFF,
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

    /* RX0 carries all FIFO0 interrupts: message pending and overrun */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    /* SCE (status change / error): bus errors, counted by type in errs */
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

/* A HAL setup call failed: red LED on, everything stopped. */
void Error_Handler(void)
{
    __disable_irq();
    HAL_GPIO_WritePin(GPIOB, LED_GREEN | LED_BLUE, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, LED_RED, GPIO_PIN_SET);
    while (1) {
    }
}
