/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "IIC_OLED.h"
#include "math.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MCU_FREQ 72000000
#define TIM_FREQ 1
#define CAPACITY 58.5
#define TIM_TICK (1.0 / TIM_FREQ)
#define SAMPLE_RESISTANCE 0.1
#define TIMES 100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint32_t adc_value[256] = {0};
uint8_t aRxBuffer;                // 接收中断缓冲
uint8_t Uart1_RxBuff[256] = {0};  // 接收缓冲
uint8_t Uart1_Rx_Cnt = 0;         // 接收缓冲计数
uint8_t Uart1_RxFlag = 0;
uint8_t cAlmStr[] = "数据溢出(大于256)\r\n";
uint8_t Uart1_TxBuff[256] = {0};  // 发�?�缓�????????????????????????
char msg[256] = {0};
uint8_t BLEConnected = 0, started = 0;
uint8_t ADC_DMAFlag = 0;
float u1 = 0, u2 = 0, uSample = 0, remainEnergy = 0, power = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (Uart1_Rx_Cnt >= 255)  // 溢出判断
    {
        Uart1_Rx_Cnt = 0;
        memset(Uart1_RxBuff, 0x00, sizeof(Uart1_RxBuff));
        HAL_UART_Transmit(&huart1, (uint8_t *)&cAlmStr, sizeof(cAlmStr), 0xFFFF);
    } else {
        Uart1_RxBuff[Uart1_Rx_Cnt++] = aRxBuffer;
        if ((Uart1_RxBuff[Uart1_Rx_Cnt - 1] == 0x0A) && (Uart1_RxBuff[Uart1_Rx_Cnt - 2] == 0x0D)) {
            Uart1_RxFlag = 1;
        }
    }

    HAL_UART_Receive_IT(&huart1, (uint8_t *)&aRxBuffer, 1);
}

void cal_arr_and_psc(float SamplingRate, float MCUFreq, TIM_TypeDef *TIMx) {
    uint32_t totalDashed = MCUFreq / SamplingRate;
    uint32_t sqrtedTotal = (uint32_t)sqrt(totalDashed);

    for (uint32_t Prescaler = 2; Prescaler <= sqrtedTotal || Prescaler < 0xffff;
         Prescaler++) {
        if (totalDashed % Prescaler == 0 && Prescaler > 1 &&
            totalDashed / Prescaler > 1) {
            if (totalDashed / Prescaler <= 0xffff) {
                TIMx->ARR = totalDashed / Prescaler - 1;
                TIMx->PSC = Prescaler - 1;
            }
        }
    }
    totalDashed += 1;
    for (uint32_t Prescaler = 2; Prescaler <= sqrtedTotal || Prescaler < 0xffff;
         Prescaler++) {
        if (totalDashed % Prescaler == 0 && Prescaler > 1 &&
            totalDashed / Prescaler > 1) {
            if (totalDashed / Prescaler <= 0xffff) {
                TIMx->ARR = totalDashed / Prescaler - 1;
                TIMx->PSC = Prescaler - 1;
            }
        }
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (u1 == 0 && u2 == 0) {
        u1 = adc_value[0] * 3.3 / 4096;
        return;
    }
    ADC_DMAFlag = 1;
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_TIM3_Init();
    MX_USART1_UART_Init();
    /* USER CODE BEGIN 2 */
    HAL_UART_Receive_IT(&huart1, &aRxBuffer, 1);
    HAL_ADCEx_Calibration_Start(&hadc1);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_TIM_Base_Start(&htim3);
    HAL_ADC_Start_DMA(&hadc1, adc_value, 256);
    HAL_Delay(1000);
    OLED_Init();
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        if (Uart1_RxFlag != 0) {
            // 接收串口蓝牙信息, 判断是否连接, 若已连接则开始广播模式
            if (strstr((char *)Uart1_RxBuff, "LINK_ID") != NULL) {
                HAL_UART_Transmit(&huart1, (uint8_t *)"AT+BT_TRANS=1\r\n", 15, 100);
                BLEConnected = 1;
            }
            if (strstr((char *)Uart1_RxBuff, "DISCONN") != NULL) {
                BLEConnected = 0;
            }

            // 若蓝牙收到开始信息, 拉高电平, 开始放电
            if (strstr((char *)Uart1_RxBuff, "start") != NULL) {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
                started = 1;
            }

            // 重置串口信息
            Uart1_RxFlag = 0;
            Uart1_Rx_Cnt = 0;
            memset(Uart1_RxBuff, 0x00, 256);
        }

        // DMA完成标志位
        if (ADC_DMAFlag) {
            // adc_value[2n]: ADC_CH1, 电容电压, 128次采样取平均值
            for (int i = 0; i < 256; i += 2)
                u2 += adc_value[i] * 3.3 / 4096;
            u2 /= 128; 

            remainEnergy = CAPACITY * u2 * u2 / 2; //利用E = 1/2 * C * U^2的公式计算剩余能量
            // 若剩余能量小于阈值, 则拉低电平, 停止放电
            if (u2 <= 1.3) {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
                started = 0;
            }
            
            // adc_value[2n-1]: ADC_CH2, 采样电阻电压, 128次采样取平均值
            for (int i = 1; i < 256; i += 2)
                uSample += adc_value[i] * 3.3 / 4096;
            uSample /= 128;

            power = 0.1597 * uSample * uSample - 0.01121 * uSample + 0.1495; // 计算功率, 功率参数为线性拟合后的参数
            if (started == 0) power = 0; // 若未开始放电, 功率置零

            // power = (float)CAPACITY * (u2 * u2 - u1 * u1) / (2 * TIM_TICK);
            // u1 = u2;

            // 储存功率/能量信息
            sprintf(msg, "Energy: %.2fJ  Power: %.2fW", remainEnergy, power);
            // OLED打印信息
            OLED_Clear();
            OLED_ShowString(0, 0, (uint8_t *)msg, 16, 1);
            // 如果已链接蓝牙, 则发送功率/能量信息
            if (BLEConnected) {
								// sprintf((char *)Uart1_TxBuff, "%s\r\nVs: %.2f", msg, uSample);
								sprintf((char *)Uart1_TxBuff, "%s\r\n", msg);
                HAL_UART_Transmit(&huart1, (uint8_t *)Uart1_TxBuff, strlen((char *)Uart1_TxBuff), 0xFFFF);
            }

            // 重置标志位
            ADC_DMAFlag = 0;
        }
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {
    /* USER CODE BEGIN ADC1_Init 0 */

    /* USER CODE END ADC1_Init 0 */

    ADC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN ADC1_Init 1 */

    /* USER CODE END ADC1_Init 1 */

    /** Common config
     */
    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 2;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }

    /** Configure Regular Channel
     */
    sConfig.Channel = ADC_CHANNEL_0;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    /** Configure Regular Channel
     */
    sConfig.Channel = ADC_CHANNEL_1;
    sConfig.Rank = ADC_REGULAR_RANK_2;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN ADC1_Init 2 */

    /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {
    /* USER CODE BEGIN TIM3_Init 0 */

    /* USER CODE END TIM3_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM3_Init 1 */

    /* USER CODE END TIM3_Init 1 */
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 599;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 59;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM3_Init 2 */

    /* USER CODE END TIM3_Init 2 */
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {
    /* USER CODE BEGIN USART1_Init 0 */

    /* USER CODE END USART1_Init 0 */

    /* USER CODE BEGIN USART1_Init 1 */

    /* USER CODE END USART1_Init 1 */
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN USART1_Init 2 */

    /* USER CODE END USART1_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {
    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA1_Channel1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET);

    /*Configure GPIO pin : PC13 */
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins : PB6 PB7 */
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
