/*
 * NUCLEO-H723ZG: periodic sensor frames to the F429 (bring-up step 8).
 *
 * FDCAN1 runs classic CAN at 500 kbps in normal mode. Every 500 ms it reads
 * the chip's internal temperature sensor and VREFINT with ADC3, and sends
 * ID 0x200 (NET_ID_H723) with a counter, the temperature and VDDA. It
 * receives the F429's ID 0x100 frames, checking the ID, DLC, that the
 * sensor values are plausible and that the counter goes up by exactly 1.
 * The hardware filter accepts only ID 0x100 (`make FILTER=all` for the
 * accept-all control). Frame layout and sensor conversion are shared with
 * the F429 in common/net_frames.h.
 *
 *   LD2 (yellow, PE1)  toggles on every frame queued for sending
 *   LD1 (green, PB0)   toggles on every good frame from the F429
 *   LD3 (red, PB14)    on while there is a problem: error passive or
 *                      bus-off, or no good frame from the F429 in the last
 *                      1.5 s. Goes off again by itself once the bus recovers.
 *
 * Working: yellow and green both blink at 1 Hz, red stays off.
 *
 * Bus errors don't stop the program: they are expected while the other
 * board is missing, and CAN recovers by itself (after bus-off, FDCAN needs
 * software to restart it, which update_bus_status() does). Only a failed
 * HAL setup call stops everything.
 *
 * For checking over SWD: clk_sys_hz, clk_fdcan_hz, local (own sensor
 * readings), tx_count, tx_skipped, rx (net_rx_stats_t, including the F429's
 * last sensor values), rx_lost_fifo, bus_off_recoveries, errs (bus errors
 * by type) and bus (error counters and state).
 */
#include "stm32h7xx_hal.h"
#include "stm32h7xx_ll_adc.h"
#include "net_frames.h"

#define LED_GREEN   GPIO_PIN_0    /* LD1, PB0 */
#define LED_RED     GPIO_PIN_14   /* LD3, PB14 */
#define LED_YELLOW  GPIO_PIN_1    /* LD2, PE1 */

#define RX_TIMEOUT_MS  1500U
#define ADC_SAMPLES    16U        /* readings averaged per sensor */

FDCAN_HandleTypeDef hfdcan1;
ADC_HandleTypeDef hadc3;

volatile uint32_t clk_sys_hz;
volatile uint32_t clk_fdcan_hz;

volatile uint32_t tx_count;           /* frames queued for sending */
volatile uint32_t tx_skipped;         /* sends skipped because the TX queue was full */
volatile net_rx_stats_t rx;            /* frames received from the F429 */
volatile uint32_t rx_lost_fifo;       /* Rx FIFO 0 message lost: it was full */
volatile uint32_t bus_off_recoveries; /* restarts after bus-off */
volatile net_err_counts_t errs;        /* bus errors by type (see net_frames.h) */

/* Own sensor readings: raw ADC averages and converted values */
volatile struct {
    uint32_t vrefint_raw;
    uint32_t ts_raw;
    int32_t  temp_c100;
    uint32_t vdda_mv;
} local;

/* Snapshot of FDCAN1's error counters (ECR) and protocol status (PSR) */
volatile struct {
    uint32_t tec;           /* transmit error counter */
    uint32_t rec;           /* receive error counter */
    uint32_t lec;           /* last error code: 0 none, 3 ACK, 7 no change (see README) */
    uint32_t error_passive;
    uint32_t bus_off;
} bus;

static void SystemClock_Config(void);
static void LED_Init(void);
static void ADC3_Init(void);
static void read_sensors(net_payload_t *p);
static void FDCAN1_Init(void);
static void FDCAN1_Filter(void);
static void update_bus_status(void);
void Error_Handler(void);

int main(void)
{
    HAL_Init();
    LED_Init();             /* first, so Error_Handler can light LD3 */
    SystemClock_Config();

    clk_sys_hz   = HAL_RCC_GetSysClockFreq();
    clk_fdcan_hz = (uint32_t)HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN);

    ADC3_Init();
    FDCAN1_Init();
    FDCAN1_Filter();

    if (HAL_FDCAN_ActivateNotification(&hfdcan1,
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_MESSAGE_LOST |
            FDCAN_IT_ARB_PROTOCOL_ERROR, 0) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        Error_Handler();
    }

    FDCAN_TxHeaderTypeDef tx = {
        .Identifier          = NET_ID_H723,
        .IdType              = FDCAN_STANDARD_ID,
        .TxFrameType         = FDCAN_DATA_FRAME,
        .DataLength          = FDCAN_DLC_BYTES_8,
        .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
        .BitRateSwitch       = FDCAN_BRS_OFF,
        .FDFormat            = FDCAN_CLASSIC_CAN,
        .TxEventFifoControl  = FDCAN_NO_TX_EVENTS,
        .MessageMarker       = 0,
    };
    net_payload_t payload;
    uint8_t data[8];

    while (1) {
        HAL_Delay(500);
        update_bus_status();

        /* The queue fills while no node acknowledges (the controller keeps
         * retrying). Skip rather than stop; sending resumes once the F429
         * is back. */
        if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0) {
            tx_skipped++;
            continue;
        }

        read_sensors(&payload);
        tx_count++;
        payload.count = tx_count;
        net_pack(data, &payload);
        if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx, data) != HAL_OK) {
            Error_Handler();
        }
        HAL_GPIO_TogglePin(GPIOE, LED_YELLOW);
    }
}

/*
 * ADC3 (the H723's 12-bit ADC, the only one wired to the internal sensors),
 * single conversions started by software. Its kernel clock is per_ck, which
 * defaults to HSI (64 MHz), divided by 4 = 16 MHz; selected in
 * SystemClock_Config. HSI's accuracy doesn't matter for the ADC.
 */
static void ADC3_Init(void)
{
    hadc3.Instance                      = ADC3;
    hadc3.Init.ClockPrescaler           = ADC_CLOCK_ASYNC_DIV4;
    hadc3.Init.Resolution               = ADC_RESOLUTION_12B;
    hadc3.Init.DataAlign                = ADC3_DATAALIGN_RIGHT;
    hadc3.Init.ScanConvMode             = ADC_SCAN_DISABLE;
    hadc3.Init.EOCSelection             = ADC_EOC_SINGLE_CONV;
    hadc3.Init.LowPowerAutoWait         = DISABLE;
    hadc3.Init.ContinuousConvMode       = DISABLE;
    hadc3.Init.NbrOfConversion          = 1;
    hadc3.Init.DiscontinuousConvMode    = DISABLE;
    hadc3.Init.ExternalTrigConv         = ADC_SOFTWARE_START;
    hadc3.Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
    hadc3.Init.SamplingMode             = ADC_SAMPLING_MODE_NORMAL;
    hadc3.Init.DMAContinuousRequests    = DISABLE;
    hadc3.Init.Overrun                  = ADC_OVR_DATA_OVERWRITTEN;
    hadc3.Init.LeftBitShift             = ADC_LEFTBITSHIFT_NONE;
    hadc3.Init.OversamplingMode         = DISABLE;
    if (HAL_ADC_Init(&hadc3) != HAL_OK) {
        Error_Handler();
    }
    /* Measure and cancel the ADC's own offset error before first use */
    if (HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK) {
        Error_Handler();
    }
}

/* Average ADC_SAMPLES conversions of one channel */
static uint32_t adc_read(uint32_t channel)
{
    ADC_ChannelConfTypeDef ch = {
        .Channel      = channel,
        .Rank         = ADC_REGULAR_RANK_1,
        /* 640.5 cycles at 16 MHz = 40 us; the temperature sensor needs at
         * least 9 us to charge the ADC's sampling capacitor */
        .SamplingTime = ADC3_SAMPLETIME_640CYCLES_5,
        .SingleDiff   = ADC_SINGLE_ENDED,
        .OffsetNumber = ADC_OFFSET_NONE,
        .Offset       = 0,
    };
    uint32_t sum = 0;

    if (HAL_ADC_ConfigChannel(&hadc3, &ch) != HAL_OK) {
        Error_Handler();
    }
    for (uint32_t i = 0; i < ADC_SAMPLES; i++) {
        if (HAL_ADC_Start(&hadc3) != HAL_OK ||
            HAL_ADC_PollForConversion(&hadc3, 10) != HAL_OK) {
            Error_Handler();
        }
        sum += HAL_ADC_GetValue(&hadc3);
    }
    HAL_ADC_Stop(&hadc3);
    return sum / ADC_SAMPLES;
}

/* Read VREFINT and the temperature sensor; fill in temperature and VDDA */
static void read_sensors(net_payload_t *p)
{
    /* Factory calibration in system memory (see net_frames.h). On the
     * H723 these are 12-bit readings, and TS_CAL2 is taken at 130 degC. */
    const net_sensor_cal_t cal = {
        .vrefint_cal = *VREFINT_CAL_ADDR,
        .ts_cal1     = *TEMPSENSOR_CAL1_ADDR,
        .ts_cal2     = *TEMPSENSOR_CAL2_ADDR,
        .ts_cal1_c   = TEMPSENSOR_CAL1_TEMP,    /* 30 degC */
        .ts_cal2_c   = TEMPSENSOR_CAL2_TEMP,    /* 130 degC */
        .cal_vdda_mv = VREFINT_CAL_VREF,        /* 3300 mV */
    };

    uint32_t vref = adc_read(ADC_CHANNEL_VREFINT);
    uint32_t ts   = adc_read(ADC_CHANNEL_TEMPSENSOR);
    net_sensor_convert(&cal, vref, ts, &p->temp_c100, &p->vdda_mv);

    local.vrefint_raw = vref;
    local.ts_raw      = ts;
    local.temp_c100   = p->temp_c100;
    local.vdda_mv     = p->vdda_mv;
}

/* Called by HAL_ADC_Init */
void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC3) {
        __HAL_RCC_ADC3_CLK_ENABLE();
    }
}

/* Refresh the bus snapshot, restart after bus-off, and set the red LED */
static void update_bus_status(void)
{
    FDCAN_ProtocolStatusTypeDef ps;
    FDCAN_ErrorCountersTypeDef ec;

    if (HAL_FDCAN_GetProtocolStatus(&hfdcan1, &ps) != HAL_OK ||
        HAL_FDCAN_GetErrorCounters(&hfdcan1, &ec) != HAL_OK) {
        Error_Handler();
    }
    bus.tec           = ec.TxErrorCnt;
    bus.rec           = ec.RxErrorCnt;
    bus.lec           = ps.LastErrorCode;
    bus.error_passive = ps.ErrorPassive;
    bus.bus_off       = ps.BusOff;

    /* Bus-off sets CCCR.INIT, which stops FDCAN. Clearing it starts the
     * standard recovery (wait for 128 x 11 recessive bits), then the node
     * rejoins the bus. bxCAN does this itself with AutoBusOff. */
    if (ps.BusOff && (hfdcan1.Instance->CCCR & FDCAN_CCCR_INIT)) {
        CLEAR_BIT(hfdcan1.Instance->CCCR, FDCAN_CCCR_INIT);
        bus_off_recoveries++;
    }

    int quiet = rx.ok == 0 || (HAL_GetTick() - rx.last_ok_tick) > RX_TIMEOUT_MS;
    int problem = bus.error_passive || bus.bus_off || quiet;
    HAL_GPIO_WritePin(GPIOB, LED_RED, problem ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* Called from HAL_FDCAN_IRQHandler for Rx FIFO 0 events */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) {
        rx_lost_fifo++;
    }
    if (!(RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)) {
        return;
    }

    FDCAN_RxHeaderTypeDef h;
    uint8_t data[8];

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &h, data) != HAL_OK) {
        Error_Handler();
    }

    net_payload_t p = net_unpack(data);
    int ok = h.IdType == FDCAN_STANDARD_ID && h.Identifier == NET_ID_F429 &&
             h.RxFrameType == FDCAN_DATA_FRAME &&
             h.FDFormat == FDCAN_CLASSIC_CAN && h.DataLength == NET_DLC &&
             net_payload_plausible(&p);
    if (!ok) {
        rx.bad++;
        return;
    }
    net_rx_count(&rx, &p, HAL_GetTick());
    HAL_GPIO_TogglePin(GPIOB, LED_GREEN);
}

/* Protocol error (classic CAN only has the nominal "arbitration" phase):
 * count it by the last error code in PSR, then carry on */
void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
    uint32_t lec = hfdcan->Instance->PSR & FDCAN_PSR_LEC;

    if (lec >= 1 && lec <= 6) {
        errs.lec[lec]++;
    }
    hfdcan->ErrorCode = HAL_FDCAN_ERROR_NONE;
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
    hfdcan1.Init.Mode                 = FDCAN_MODE_NORMAL;
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
 * Standard filter element 0 in classic ID/mask mode: FilterID1 is the ID
 * and FilterID2 the mask, both as plain 11-bit values (no shifting, unlike
 * bxCAN). Mask bit 1 = must match, 0 = don't care; 0x7FF compares every ID
 * bit, so only ID 0x100 passes.
 *
 * Unlike bxCAN, FDCAN's global filter accepts frames that match no filter
 * into Rx FIFO 0 by default. It is set to reject them, along with all
 * remote frames, so frames only get in through filter element 0.
 *
 * With -DFILTER_ACCEPT_ALL (make FILTER=all) the mask is 0 instead, so
 * every standard data frame passes: the control for step 7's filter test.
 */
static void FDCAN1_Filter(void)
{
#ifdef FILTER_ACCEPT_ALL
    const uint32_t id = 0x000, mask = 0x000;
#else
    const uint32_t id = NET_ID_F429, mask = 0x7FF;
#endif
    FDCAN_FilterTypeDef f = {
        .IdType       = FDCAN_STANDARD_ID,
        .FilterIndex  = 0,
        .FilterType   = FDCAN_FILTER_MASK,
        .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
        .FilterID1    = id,
        .FilterID2    = mask,
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
 * HSE (ST-LINK MCO, bypass), measured 8.098123 MHz instead of the nominal 8
 * (see HSE_VALUE) -> PLL1: /M=2 = 4.04906 MHz x (N=197 + FRACN 4724/8192
 * = 197.5767) = 800.000 MHz VCO. The fractional part cancels the HSE error;
 * with an exact 8 MHz HSE it would be N=200, FRACN=0.
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
    osc.PLL.PLLN        = 197;
    osc.PLL.PLLP        = 2;
    osc.PLL.PLLQ        = 10;
    osc.PLL.PLLR        = 2;
    osc.PLL.PLLRGE      = RCC_PLL1VCIRANGE_2;   /* 4 MHz PLL input: 4-8 MHz range */
    osc.PLL.PLLVCOSEL   = RCC_PLL1VCOWIDE;      /* 800 MHz VCO: wide range */
    osc.PLL.PLLFRACN    = 4724;               /* trims the fast HSE: see above */
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

    /* FDCANSEL mux: PLL1Q (80 MHz). ADCSEL mux: per_ck (HSI 64 MHz by default) */
    periph.PeriphClockSelection = RCC_PERIPHCLK_FDCAN | RCC_PERIPHCLK_ADC;
    periph.FdcanClockSelection  = RCC_FDCANCLKSOURCE_PLL;
    periph.AdcClockSelection    = RCC_ADCCLKSOURCE_CLKP;
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

/* A HAL setup or read call failed: red LED on, everything stopped. */
void Error_Handler(void)
{
    __disable_irq();
    HAL_GPIO_WritePin(GPIOB, LED_GREEN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, LED_YELLOW, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, LED_RED, GPIO_PIN_SET);
    while (1) {
    }
}
