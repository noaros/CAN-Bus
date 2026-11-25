/*
 * NUCLEO-H723ZG: build check for the HAL setup (bring-up step 4).
 *
 * Configures the clocks used for CAN (400 MHz system clock, 80 MHz FDCAN
 * kernel clock from PLL1Q) and blinks LD1 (green, PB0) so a flashed board
 * shows it is running. FDCAN code is added in step 5.
 *
 * The frequencies the HAL computes from the RCC registers are stored in
 * clk_sys_hz and clk_fdcan_hz, so they can be read over SWD.
 */
#include "stm32h7xx_hal.h"

#define LED_GREEN   GPIO_PIN_0    /* LD1, PB0 */
#define LED_RED     GPIO_PIN_14   /* LD3, PB14 */

volatile uint32_t clk_sys_hz;
volatile uint32_t clk_fdcan_hz;

static void SystemClock_Config(void);
static void LED_Init(void);
void Error_Handler(void);

int main(void)
{
    HAL_Init();
    LED_Init();             /* first, so Error_Handler can light LD3 */
    SystemClock_Config();

    clk_sys_hz   = HAL_RCC_GetSysClockFreq();
    clk_fdcan_hz = (uint32_t)HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN);

    while (1) {
        HAL_GPIO_TogglePin(GPIOB, LED_GREEN);
        HAL_Delay(500);
    }
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

/* LD1 green PB0, LD3 red PB14 */
static void LED_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio.Pin   = LED_GREEN | LED_RED;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);
}

/* Red LED on, everything stopped */
void Error_Handler(void)
{
    __disable_irq();
    HAL_GPIO_WritePin(GPIOB, LED_GREEN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, LED_RED, GPIO_PIN_SET);
    while (1) {
    }
}
