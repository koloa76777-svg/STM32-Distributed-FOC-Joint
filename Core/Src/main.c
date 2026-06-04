/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <stdio.h>
#include <math.h>   // 加这一行（sin、PI）
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SQRT3_2   0.86602540378f   // √3/2,逆Clark用
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint32_t isr_counter = 0;
volatile float theta_isr = 0.0f;
volatile uint16_t angle_raw_isr = 0;     // 中断里读的编码器原始值
volatile float angle_deg_isr = 0.0f;     // 中断里算的编码器角度
volatile float Uq_cmd = 0.0f;   // 交轴电压指令(转矩),先给0,跑通后串口调
volatile float Ud_cmd = 0.0f;   // 直轴电压指令,开环恒为0
volatile float Ua_dbg = 0.0f;   // 调试用:三相电压,中断写,主循环读
volatile float Ub_dbg = 0.0f;
volatile float Uc_dbg = 0.0f;
volatile uint8_t sector_dbg = 0;
volatile float theta_offset = 0.0f;   // 编码器零位偏移(校准得到)
volatile float theta_e_global = 0.0f;   // 主循环更新,中断使用
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// printf 重定向：覆盖 newlib 默认的 _write 弱定义
int _write(int fd, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}
/**
 * @brief 读取 AS5047P 角度寄存器
 * @retval 14位原始角度值 (0~16383)
 *
 * 注意：第一次调用返回的是"前一次"的数据（SPI 延迟响应特性）
 *      连续调用时，第 N 次返回第 N-1 次请求的数据
 */
#define AS5047_REG_ANGLE       0x3FFF
#define AS5047_CMD_READ        0x4000
#define AS5047_PARITY_BIT      0x8000
#define AS5047_CMD_READ_ANGLE  (AS5047_REG_ANGLE | AS5047_CMD_READ | AS5047_PARITY_BIT)
#define AS5047_ANGLE_MASK      0x3FFFU

uint16_t AS5047_ReadAngle(void)
{
    uint8_t tx_buf[2] = { (AS5047_CMD_READ_ANGLE >> 8) & 0xFF,
                          (AS5047_CMD_READ_ANGLE)      & 0xFF };
    uint8_t rx_buf[2] = {0};


    HAL_GPIO_WritePin(CSn_GPIO_Port, CSn_Pin, GPIO_PIN_RESET);

    HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, 2, 100);

    HAL_GPIO_WritePin(CSn_GPIO_Port, CSn_Pin, GPIO_PIN_SET);


    uint16_t raw = (rx_buf[0] << 8) | rx_buf[1];
    return raw & AS5047_ANGLE_MASK;
}

// ============== 开环 SPWM ==============

#define PWM_ARR             1800.0f         // ARR 值（M2 设定的）
#define PWM_CENTER          900.0f          // 50% 占空比对应的 CCR
#define VBUS                12.0f            // 输出正弦电压幅值 (V)，5010 限流值 // 电源电压 (V)
#define U_AMPLITUDE         1.0f
#define POLE_PAIRS          7               // 5010 极对数
#define TWO_PI              6.28318530718f
#define TWO_PI_OVER_3       2.09439510239f  // 120°

/**
 * @brief 设置三相 PWM 占空比
 * @param Ua, Ub, Uc 三相电压指令 (-VBUS/2 ~ +VBUS/2)
 */
void SetThreePhasePWM(float Ua, float Ub, float Uc)
{
    // 把电压转换成 CCR 值：50% + (U/Vbus) × ARR
    int32_t ccr_a = (int32_t)(PWM_CENTER + (Ua / VBUS) * PWM_ARR);
    int32_t ccr_b = (int32_t)(PWM_CENTER + (Ub / VBUS) * PWM_ARR);
    int32_t ccr_c = (int32_t)(PWM_CENTER + (Uc / VBUS) * PWM_ARR);

    // 限幅保护：不能超出 [0, ARR] 范围
    if (ccr_a < 0) ccr_a = 0; else if (ccr_a > (int32_t)PWM_ARR) ccr_a = (int32_t)PWM_ARR;
    if (ccr_b < 0) ccr_b = 0; else if (ccr_b > (int32_t)PWM_ARR) ccr_b = (int32_t)PWM_ARR;
    if (ccr_c < 0) ccr_c = 0; else if (ccr_c > (int32_t)PWM_ARR) ccr_c = (int32_t)PWM_ARR;

    // 写入对应 CCR 寄存器（M2 的引脚映射）
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ccr_a);  // PC7 → Phase A
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ccr_b);  // PB4 → Phase B
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, ccr_c);  // PB10 → Phase C
}
// ============== SVPWM 七段式 ==============
#define SVPWM_TS   PWM_ARR     // 一个PWM周期 = ARR(1800)
//#define SVPWM_TS   (PWM_ARR * 2.0f)   // 中心对齐,周期是2倍
void SVPWM(float U_alpha, float U_beta)
{
    // 反Clark: αβ → 三相参考电压
    float Ua = U_alpha;
    float Ub = -0.5f * U_alpha + SQRT3_2 * U_beta;
    float Uc = -0.5f * U_alpha - SQRT3_2 * U_beta;

    // 注入法核心: min-max共模(七段式的数学等价实现)
    float Vmax = fmaxf(Ua, fmaxf(Ub, Uc));
    float Vmin = fminf(Ua, fminf(Ub, Uc));
    float Vcom = 0.5f * (Vmax + Vmin);

    // 减去共模 + 归一化到占空比[0,1],再×ARR
    // (Ux - Vcom)/Vdc 得到[-0.5,0.5],+0.5居中,×ARR得CCR
    float duty_a = ((Ua - Vcom) / VBUS + 0.5f) * PWM_ARR;
    float duty_b = ((Ub - Vcom) / VBUS + 0.5f) * PWM_ARR;
    float duty_c = ((Uc - Vcom) / VBUS + 0.5f) * PWM_ARR;

    // 限幅
    if (duty_a < 0) duty_a = 0; else if (duty_a > PWM_ARR) duty_a = PWM_ARR;
    if (duty_b < 0) duty_b = 0; else if (duty_b > PWM_ARR) duty_b = PWM_ARR;
    if (duty_c < 0) duty_c = 0; else if (duty_c > PWM_ARR) duty_c = PWM_ARR;

    Ua_dbg = duty_a; Ub_dbg = duty_b; Uc_dbg = duty_c;

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)duty_a);  // PC7 → A
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)duty_b);  // PB4 → B
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (uint32_t)duty_c);  // PB10 → C
}
/**
 * @brief 编码器零位校准(强制定向法)
 * 强行给d轴电压,把转子吸到电气零点,记下编码器读数作为offset
 */
void Encoder_Calibrate(void)
{
    // 强制电角度=0方向输出电压(Ud方向),把转子吸到电气零点
    // 用SVPWM直接给一个指向α轴(θe=0)的电压
    float Ud_align = 3.0f;   // 对齐电压,够把转子吸过去即可
    // θe=0时: Uα=Ud, Uβ=0
    SVPWM(Ud_align, 0.0f);
    printf("aligning...\n");   // ← 加这行,确认校准在跑
    HAL_Delay(1000);   // 等转子稳定对齐(听到"咔")

    // 读此刻编码器机械角(弧度),就是offset
    uint16_t raw = AS5047_ReadAngle();
    theta_offset = (float)raw / 16384.0f * TWO_PI;

    // 松开,电压归零
    SVPWM(0.0f, 0.0f);
    HAL_Delay(500);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  // 使能 L6234 驱动芯片（拉高 EN_GATE）
  HAL_GPIO_WritePin(EN_GATE_GPIO_Port, EN_GATE_Pin, GPIO_PIN_SET);
  HAL_Delay(10);  // 等 L6234 启动稳定
  // PWM 启动顺序：先启 Slave，再启 Master
  // 原因：Slave 等触发才走，先启 Slave 让它"待命"，再启 Master 同步开闸
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  // ★Step6: 编码器零位校准(必须在开中断前,避免虚拟theta干扰)
  Uq_cmd = 0.0f;        // 确保校准时无杂散指令
  Ud_cmd = 0.0f;
  Encoder_Calibrate();  // 强制对齐,测offset

//  printf("offset = %.4f rad\n", theta_offset);  // 打印确认
  // ★临时:卡在这里反复打印offset,看清了再继续(验证完删掉)

  // 启动 TIM3 Update 中断
  __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
  // 占空比测试（50% = 1.65V 平均电压）
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 900);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 900);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 900);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  float theta = 0.0f;                  // 虚拟电角度
  float theta_increment = 0.0001f;     // 每个循环增加的角度（控制转速）
  uint32_t print_counter = 0;
  while (1)
  {
	    Uq_cmd = 2.0f;   // 先0,验证

	    // 主循环读编码器,算电角度,存全局给中断用
	    uint16_t raw = AS5047_ReadAngle();
	    float theta_m = (float)raw / 16384.0f * TWO_PI;
//	    float theta_e = (theta_offset - theta_m) * POLE_PAIRS;
	    float theta_e = (theta_m - theta_offset) * POLE_PAIRS;  // 反过来试
	    theta_e = fmodf(theta_e, TWO_PI);
	    if (theta_e < 0) theta_e += TWO_PI;
	    theta_e_global = theta_e;   // 更新给中断

//	    print_counter++;
//	    if (print_counter >= 5000)
//	    {
//	        print_counter = 0;
//	        printf("theta_m=%.1f theta_e=%.1f Uq=%.2f\n",
//	               theta_m*57.3f, theta_e*57.3f, Uq_cmd);
//	        //printf("Ualpha=%.2f Ubeta=%.2f Uq=%.2f\n", Ua_dbg, Ub_dbg, Uq_cmd);
//	        //	        printf("%.2f,%.2f\n", theta_m*57.3f, theta_e*57.3f);
//	    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
  htim2.Init.Period = 1800;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 900;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
  htim3.Init.Period = 1800;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 900;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
 // HAL_GPIO_WritePin(EN_GATE_GPIO_Port, EN_GATE_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(EN_GATE_GPIO_Port, EN_GATE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CSn_GPIO_Port, CSn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : EN_GATE_Pin */
  GPIO_InitStruct.Pin = EN_GATE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(EN_GATE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CSn_Pin */
  GPIO_InitStruct.Pin = CSn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CSn_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
#define THETA_INCREMENT_ISR  0.00176f


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        float theta_e = theta_e_global;   // 用主循环算好的角度,中断不碰SPI
        float s = sinf(theta_e);
        float c = cosf(theta_e);
        float Ualpha = Ud_cmd * c - Uq_cmd * s;
        float Ubeta  = Ud_cmd * s + Uq_cmd * c;
        Ua_dbg = Ualpha;   // 临时:看逆Park输出
        Ub_dbg = Ubeta;
        SVPWM(Ualpha, Ubeta);
    }
}
/* USER CODE END 4 */


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
