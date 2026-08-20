/*
 * STM32F407G-DISC1 - Send periodic sensor readings to an ESP32 Wi-Fi module over UART
 *
 * UART connection to ESP32:
 *   STM32 PA9  = TX  -> ESP32 RX (GPIO16/UART2 RX)
 *   STM32 PA10 = RX  -> ESP32 TX (GPIO17/UART2 TX)
 *   STM32 GND  -> ESP32 GND
 *   STM32 3.3V -> ESP32 3.3V
 *   STM32 PA0  -> ESP32 EN (optional reset/control pin)
 *
 * The ESP32 is expected to connect to Wi-Fi and send data to the internet.
 * The STM32 only reads the sensor and transmits a simple value over UART.
 */

#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>

UART_HandleTypeDef huart1;
ADC_HandleTypeDef hadc1;

static void SystemClock_Config(void);
static void GPIO_Init(void);
static void USART1_UART_Init(void);
static void ADC1_Init(void);
static float read_sensor(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    GPIO_Init();
    USART1_UART_Init();
    ADC1_Init();

    /* Keep ESP32 active at startup. */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_Delay(1000);

    char msg[64];
    char status[128];
    uint32_t tick = 0;

    const char *banner = "\r\n=== STM32 -> ESP32 TX Started ===\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t *)banner, strlen(banner), HAL_MAX_DELAY);
    HAL_Delay(200);

    while (1)
    {
        float temperature = read_sensor();
        tick += 1000;

        int len = snprintf(msg, sizeof(msg), "[%lu ms] TEMP=%.2f\r\n", (unsigned long)tick, temperature);
        HAL_UART_Transmit(&huart1, (uint8_t *)msg, len, HAL_MAX_DELAY);

        snprintf(status, sizeof(status), "[%lu ms] UART data sent to ESP32: %d bytes\r\n", (unsigned long)tick, len);
        HAL_UART_Transmit(&huart1, (uint8_t *)status, strlen(status), HAL_MAX_DELAY);

        HAL_Delay(1000);
    }
}

static float read_sensor(void)
{
    // HAL_ADC_Start(&hadc1);
    // HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    // uint32_t raw = HAL_ADC_GetValue(&hadc1);
    // HAL_ADC_Stop(&hadc1);

    // /* Internal temp sensor conversion (STM32F407 datasheet, VREFINT = 3.3V, 12-bit ADC) */
    // float v_sense = (raw * 3.3f) / 4095.0f;
    // float temperature = ((v_sense - 0.76f) / 0.0025f) + 25.0f;
    return rand() % 100;  // Placeholder: return a random temperature for demonstration
}

static void USART1_UART_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 9600;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
}

static void ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USART1 GPIO Configuration: PA9 = TX, PA10 = RX (AF7) */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (huart->Instance == USART1)
    {
        __HAL_RCC_USART1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
}

static void GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PA0 = EN/KEY (output, optional) */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);  /* Keep EN/KEY high for normal mode */

    /* PA8 is intentionally unused for the ESP32 setup. */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* System Clock Configuration: HSE 8MHz -> PLL -> 168MHz SYSCLK (standard F407-Disco setup) */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}
