/* USER CODE BEGIN Header */
/**串口用不了，我在debug里面改theta_target，可以实现控制。希望能优化一下，然后在theta_multi不断增大过程中，电机出现了抖动，在theta_multi不断减小的过程中，电机抖动明显减弱非常多。
 * 这是一个有意思的点。把这个弄完之后，我们就停下来了，准备弄CAN通信了，然后开始上传github，当成制作简历的一个项目。
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
#include <stdlib.h>   // atof
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// ===== 回零状态机 =====
typedef enum {
    HOMING_IDLE = 0,    // 未回零(正常工作)
    HOMING_SEARCH,      // 搜索静摩擦突破点
    HOMING_CRUISE,      // 降力矩巡航向挡块
    HOMING_DONE         // 回零完成
} HomingState_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SQRT3_2   0.86602540378f   // √3/2,逆Clark用
#define ADC_VREF      3.3f
#define ADC_RES       4096.0f
#define INA_GAIN      50.0f
#define R_SHUNT       0.01f


#define I_BASE   1.5f
#define U_BASE   6.9282f               // = 12 / √3，SVPWM线性区相电压上限
#define I_BASE_INV   (1.0f / I_BASE)


// 码值差 → 安培: (raw - offset) * Vref / 4096 / (Gain * Rshunt)
#define ADC_TO_AMP(raw, off)  (((float)(raw) - (off)) * ADC_VREF / ADC_RES / (INA_GAIN * R_SHUNT))
#define POS_DEADBAND  0.05f   // 位置环死区(rad),略大于编码器噪声峰峰

// ===== 堵转检测参数 =====
#define STALL_IQ_THRESH    0.7f     // 电流阈值(标幺): |i_q标幺|超过算"在使劲"
#define STALL_MOVE_THRESH  0.01f    // 位置变化阈值(rad): N拍内变化小于此算"没动"
#define STALL_COUNT_MAX    50       // 持续拍数: 连续满足这么多次才判堵转(50×~0.5ms≈25ms)

#define HOMING_TORQUE_RAMP   0.002f   // 力矩每拍爬升步长(标幺), 朝负方向
#define HOMING_TORQUE_MAX    1.5f     // 力矩上限(标幺), 爬到这还没突破=异常
#define HOMING_BREAK_MOVE    0.5f     // 突破判据: 累积位移超此值(rad)算动了
#define HOMING_HOLD_MAX   5000    // 力矩到上限后, 顶住的拍数(400×0.5ms≈200ms)

// ===== 软件位置限位(回零后坐标: 挡块=0, 向左为正) =====
#define POS_LIMIT_MIN   0.0f     // 下限(rad), 挡块内侧留~4mm余量
#define POS_LIMIT_MAX   185.0f   // 上限(rad), 左端留余量(左端开放无挡块!)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

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
uint16_t adc_buf[2];        // DMA 接收缓冲: [0]=A相(IN0/PA0), [1]=B相(IN4/PA4)
float offset_A = 0, offset_B = 0;   // 提全局,FOC中断里换算电流要用
float i_a = 0, i_b = 0, i_c = 0;    // 三相电流(安培),给VOFA看
volatile float i_alpha = 0, i_beta = 0;   // Clark输出
volatile float i_d = 0, i_q = 0;          // Park输出(直流量)
typedef struct {
    float Kp, Ki, integral, out_max, out_min;
} PI_Controller;
PI_Controller pi_id = {0};
PI_Controller pi_iq = {0};
volatile float iq_ref = 0.0f;   // q轴电流指令
// ===== SPI DMA 流水线 =====
uint8_t spi_tx_buf[2] = {0};
uint8_t spi_rx_buf[2] = {0};
volatile uint16_t angle_raw_dma = 0;     // DMA读回的原始角度
volatile uint8_t  spi_busy = 0;          // 1=DMA进行中,防重入
volatile uint32_t spi_cb_count = 0;
volatile HAL_StatusTypeDef spi_kick_ret = HAL_OK;   // Kick时HAL返回值
volatile uint32_t spi_state_dbg = 0;                // 踢之前的HAL State
volatile float theta_comp = -1.3f;   // 对齐补偿角,调试时手动改
volatile float delta_theta_dbg = 0.0f;
volatile float theta_force = 0.0f;     // 开环强制电角度
volatile float open_loop_speed = 0.0f; // 每拍递增量(电角速度)
volatile float open_loop_volt = 0.8f;  // 开环电压幅值(V)
// ===== 速度估计 =====
volatile float omega_e = 0.0f;        // 电角速度估计(rad/s),滤波后
volatile float theta_e_prev = 0.0f;   // 上一次电角度(算微分用)
volatile float omega_raw = 0.0f;      // 微分原始值(滤波前,调试看)
PI_Controller pi_speed = {0};         // 速度环PI
volatile float omega_ref = 0.0f;      // 目标电角速度(rad/s)

// ===== PLL 速度观测器 =====
typedef struct {
    float theta_hat;   // 估计机械角(rad)
    float omega_hat;   // 估计机械角速度(rad/s)
    float Kp;          // PLL比例增益
    float Ki;          // PLL积分增益
    float Ts;          // 采样周期(s)
} PLL_Observer;

PLL_Observer pll = {0};
// ===== M法测速: 环形缓冲存历史电角度 =====
#define SPEED_WIN  10              // 测速窗口(拍数),先10试
float theta_hist[SPEED_WIN] = {0};   // 历史电角度环形缓冲
volatile uint8_t hist_idx = 0;       // 环形缓冲写指针
// ===== 位置环参数 =====
float    theta_target   = 0.0f;    // 目标机械角(rad), 调试时手动改这个看"指哪打哪"
float    omega_max_mech = 3.0f;    // 机械角ω_ref限幅(rad/s), 防误差大时飞车
// 标幺化: 电流量纲增益 ÷I_base (×0.6667), 输出从安培变标幺
float    Kp_pos         = 0.2f;     // 0.8 / 1.5
float    Kd_pos         = 0.005f;   // 0.01 / 1.5
float    Ki_pos         = 0.0002f;  // 0.001 / 1.5
// ===== 多圈累加角 =====
float theta_multi = 0.0f;        // 多圈连续机械角(rad), 可超出0~2π
float theta_m_last = 0.0f;       // 上一拍单圈机械角(检测过零用)
volatile int32_t turns = 0;      // 圈数计数(调试看)
volatile float theta_target_final = 0.0f;   // 最终目标(你真正想去的位置)
float theta_ramp_rate = 0.0001f;    // 每拍爬升步长(rad/拍), 决定转速; 先小

// ===== 串口接收指令 =====
uint8_t rx_byte;                 // 单字节接收
char rx_cmd_buf[32];             // 命令字符串缓冲
volatile uint8_t rx_cmd_idx = 0; // 缓冲写指针

// float ↔ 4字节 互转, 用于把CAN数据段还原成角度
typedef union {
    float    f;
    uint8_t  b[4];
} float_bytes_t;

CAN_RxHeaderTypeDef RxHeader;   // 接收帧头
uint8_t RxData[8];              // 接收数据缓冲

// ===== 控制模式 =====
typedef enum { MODE_POSITION = 0, MODE_TORQUE = 1 } ControlMode_t;
volatile ControlMode_t control_mode = MODE_TORQUE;   // 改: 上电默认力矩模式
volatile float torque_ref = 0.0f;   // 外部力矩指令(标幺, 范围-1.0~1.0, 对应±1.5A的iq)

volatile float pos_integral = 0.0f;   // 位置环积分(提全局, 供模式切换清零)

volatile uint32_t rx_irq_count = 0;   // RX中断进入次数(调试)
volatile char last_cmd_char = 0;     // 加到 PV 区
volatile uint8_t last_cmd_len = 0;
volatile uint8_t rx_byte_dbg = 0;   // 抓每个进中断的原始字节
volatile float Kff = 0.0003f;   // 反电势前馈增益, 先0(基准), debug里调
volatile float Uq_ff_dbg = 0.0f;  // 前馈量, 调试看

// ===== 堵转检测状态 =====
volatile uint8_t  stall_flag = 0;        // 堵转标志: 1=检测到堵转
volatile uint16_t stall_counter = 0;     // 连续满足条件的计数
float    theta_multi_last_stall = 0.0f;  // 上次检测时的多圈角(算变化用)

// ===== 回零状态机 =====
volatile HomingState_t homing_state = HOMING_IDLE;   // 当前回零状态
volatile float homing_torque = 0.0f;                 // 回零过程的力矩输出
float homing_t_break = 0.0f;                          // 记录的突破力矩
float homing_start_pos = 0.0f;   // SEARCH起点位置(算突破位移用)
volatile uint16_t homing_hold_counter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
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
/**
 * @brief 非阻塞踢一帧 SPI DMA 读角度
 *        CS拉低 → 启动DMA收发 → 立即返回(不等待)
 *        传输完成由 HAL_SPI_TxRxCpltCallback 收尾
 */
//void AS5047_ReadAngle_DMA_Kick(void)
//{
//    if (spi_busy) return;            // 上一帧还没传完,跳过(防重入)
//    spi_state_dbg = HAL_SPI_GetState(&hspi1);   // ★记录踢之前的State
//    if (spi_state_dbg != HAL_SPI_STATE_READY) return;
//    spi_busy = 1;
//    HAL_GPIO_WritePin(CSn_GPIO_Port, CSn_Pin, GPIO_PIN_RESET);
////    spi_kick_ret = HAL_SPI_Receive_DMA(&hspi1, spi_rx_buf, 2);  // ★只收不发
//    spi_kick_ret = HAL_SPI_TransmitReceive_DMA(&hspi1, spi_tx_buf, spi_rx_buf, 2);  // ★记录返回值
//    if (spi_kick_ret != HAL_OK)
//    {
//        spi_busy = 0;
//        HAL_GPIO_WritePin(CSn_GPIO_Port, CSn_Pin, GPIO_PIN_SET);
//    }
//}
void AS5047_ReadAngle_DMA_Kick(void)
{
    if (spi_busy) return;

    HAL_SPI_StateTypeDef st = HAL_SPI_GetState(&hspi1);
    if (st != HAL_SPI_STATE_READY)
    {
        // ★HAL状态机锁死(BUSY/ERROR),强制复位恢复
        HAL_SPI_Abort(&hspi1);              // 中止当前传输
        __HAL_SPI_DISABLE(&hspi1);
        hspi1.ErrorCode = HAL_SPI_ERROR_NONE;
        hspi1.State = HAL_SPI_STATE_READY;  // 强制拉回READY
        __HAL_SPI_ENABLE(&hspi1);
        HAL_GPIO_WritePin(CSn_GPIO_Port, CSn_Pin, GPIO_PIN_SET);  // CS归位
        spi_busy = 0;
        return;   // 这一拍先恢复,下一拍正常踢
    }

    spi_busy = 1;
    HAL_GPIO_WritePin(CSn_GPIO_Port, CSn_Pin, GPIO_PIN_RESET);
    spi_kick_ret = HAL_SPI_TransmitReceive_DMA(&hspi1, spi_tx_buf, spi_rx_buf, 2);
    if (spi_kick_ret != HAL_OK)
    {
        spi_busy = 0;
        HAL_GPIO_WritePin(CSn_GPIO_Port, CSn_Pin, GPIO_PIN_SET);
    }
}
/**
 * @brief SPI DMA 传输完成回调(HAL硬件自动调用)
 *        CS拉高 → 解析角度 → 清busy
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
//void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)   // ★TxRx → Rx
{
    if (hspi->Instance == SPI1)
    {
        HAL_GPIO_WritePin(CSn_GPIO_Port, CSn_Pin, GPIO_PIN_SET);  // CS↑
        uint16_t raw = (spi_rx_buf[0] << 8) | spi_rx_buf[1];
        angle_raw_dma = raw & AS5047_ANGLE_MASK;
        spi_busy = 0;          // ★清忙标志,这是关键!
        spi_cb_count++;
    }
}

// ============== 开环 SPWM ==============

#define PWM_ARR             1800.0f         // ARR 值（M2 设定的）
#define PWM_CENTER          900.0f          // 50% 占空比对应的 CCR
#define VBUS                12.0f            // 输出正弦电压幅值 (V)，5010 限流值 // 电源电压 (V)
#define U_AMPLITUDE         1.0f
#define POLE_PAIRS          7               // 5010 极对数
#define TWO_PI              6.28318530718f
#define TWO_PI_OVER_3       2.09439510239f  // 120°
#define PI 					3.14159265f
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
//    uint16_t raw = AS5047_ReadAngle();
//    theta_offset = (float)raw / 16384.0f * TWO_PI;
    AS5047_ReadAngle();              // 第一次:丢弃(延迟响应,返回上一帧)
    HAL_Delay(1);
    uint16_t raw = AS5047_ReadAngle();   // 第二次:当前真实角度
    theta_offset = (float)raw / 16384.0f * TWO_PI;
    // 松开,电压归零
    SVPWM(0.0f, 0.0f);
    HAL_Delay(500);
}

float PI_Update(PI_Controller *pi, float error)
{
    float p_out = pi->Kp * error;
    pi->integral += pi->Ki * error;
    if (pi->integral > pi->out_max) pi->integral = pi->out_max;
    else if (pi->integral < pi->out_min) pi->integral = pi->out_min;
    float output = p_out + pi->integral;
    if (output > pi->out_max) output = pi->out_max;
    else if (output < pi->out_min) output = pi->out_min;
    return output;
}
void PLL_Update(PLL_Observer *p, float theta_m)
{
    float err = theta_m - p->theta_hat;
    while (err >  PI) err -= TWO_PI;
    while (err < -PI) err += TWO_PI;

    p->omega_hat += p->Ki * err * p->Ts;          // 积分项累加(这就是平滑速度)
    p->theta_hat += (p->omega_hat + p->Kp * err) * p->Ts;  // 角度更新(含比例修正)

    if (p->theta_hat >= TWO_PI) p->theta_hat -= TWO_PI;
    if (p->theta_hat <  0)      p->theta_hat += TWO_PI;
}
// 平滑速度取 p->omega_hat (机械角速度), ×POLE_PAIRS 得电角速度


/**
 * @brief 切换控制模式, 并清理切换瞬间的隐患
 * @param new_mode  MODE_POSITION 或 MODE_TORQUE
 *
 * 切换动作(无论往哪切都做):
 *   1. iq_ref 清零 → 切换瞬间不出力, 避免旧指令残留导致跳变
 *   2. pos_integral 清零 → 防止位置环积分windup(力矩模式期间冻结的旧积分)
 *   3. torque_ref 清零 → 切到力矩模式时从0开始, 防上一次的力矩值突然加上
 */
void SetControlMode(ControlMode_t new_mode)
{
    iq_ref       = 0.0f;
    pos_integral = 0.0f;
    torque_ref   = 0.0f;
    pi_speed.integral = 0.0f;   // ★加这行:清速度环积分,防切入瞬间跳变
    if (new_mode == MODE_POSITION)
    {
        // 切回位置模式: 把目标设成当前实际位置, 避免"指哪打哪"瞬间猛冲
        theta_target_final = theta_multi;
        theta_target       = theta_multi;
    }

    control_mode = new_mode;   // 最后才改模式, 前面的清零先生效
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
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(200);          // 等 200ms,让 3.3V/VREF/INA 供电稳定

  spi_tx_buf[0] = (AS5047_CMD_READ_ANGLE >> 8) & 0xFF;  // 0xFF
    spi_tx_buf[1] = (AS5047_CMD_READ_ANGLE)      & 0xFF;  // 0xFF

  // ======== 第一步:电流零偏校准(必须在任何通电动作之前)========
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, 2);

  HAL_GPIO_WritePin(EN_GATE_GPIO_Port, EN_GATE_Pin, GPIO_PIN_RESET);  // EN 拉低,无电流
  HAL_Delay(50);
  // PWM 启动顺序：先启 Slave，再启 Master
  // 原因：Slave 等触发才走，先启 Slave 让它"待命"，再启 Master 同步开闸
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  offset_A = 0;        // ← 删掉 float,否则新建局部变量遮蔽全局
  offset_B = 0;
  float sum_A = 0, sum_B = 0;
//  for (int i = 0; i < 1000; i++) {
//      HAL_ADC_Start(&hadc1);
//      HAL_Delay(1);
//      sum_A += adc_buf[0];
//      sum_B += adc_buf[1];
//  }
  // 校准循环改成同样的轮询读法
  for (int i = 0; i < 1000; i++) {
      HAL_Delay(1);              // 等DMA刷新(TRGO每PWM周期都在触发)
      sum_A += adc_buf[0];
      sum_B += adc_buf[1];
  }
  offset_A = sum_A / 1000.0f;
  offset_B = sum_B / 1000.0f;
  printf("offset: %.1f, %.1f\n", offset_A, offset_B);
  // 使能 L6234 驱动芯片（拉高 EN_GATE）
  HAL_GPIO_WritePin(EN_GATE_GPIO_Port, EN_GATE_Pin, GPIO_PIN_SET);
  HAL_Delay(10);  // 等 L6234 启动稳定

  // ★Step6: 编码器零位校准(必须在开中断前,避免虚拟theta干扰)
  Uq_cmd = 0.0f;        // 确保校准时无杂散指令
  Ud_cmd = 0.0f;
  Encoder_Calibrate();  // 强制对齐,测offset

  // 标幺化后等价缩放: Kp_new = Kp_old × (I_base/U_base) = ×0.2165
  pi_id.Kp = 0.108f;  pi_id.Ki = 0.00217f;
  pi_iq.Kp = 0.108f;  pi_iq.Ki = 0.00217f;

  pi_id.out_max = 0.95f;  pi_id.out_min = -0.95f;   // 标幺电压限幅，留5%余量防过调制
  pi_iq.out_max = 0.95f;  pi_iq.out_min = -0.95f;
  pi_id.integral = 0;  pi_iq.integral = 0;
  // ★Step2: 开控制中断前,先踢一帧DMA"预热"
  //   否则第一拍中断读 angle_raw_dma 还是0(没人踢过),θe会是无效值
  // ===== 速度环参数(初始化跑一次,别放while里) =====
  pi_speed.Kp = 0.001f;
  pi_speed.Ki = 0.0001f;
  pi_speed.out_max =  1.0f;
  pi_speed.out_min = -1.0f;
  pi_speed.integral = 0;
  //   预热后第一拍就有真实角度可取,流水线立即满载

  // ===== PLL观测器初始化 (fn=80Hz, ζ=0.707) =====
    pll.Ts = 0.0001f;   // 采样周期=测速节奏=1ms(和原M法同节奏)
    float pll_wn = 2.0f * PI * 80.0f;     // ωn, 带宽80Hz
    pll.Kp = 2.0f * 0.707f * pll_wn;      // ≈710
    pll.Ki = pll_wn * pll_wn;             // ≈252000
    pll.theta_hat  = theta_target;        // 初值=当前机械角(你前面已读), 避免启动猛冲
    pll.omega_hat  = 0.0f;

  AS5047_ReadAngle_DMA_Kick();
  HAL_Delay(1);   // 等这帧DMA传回(后台~3µs,给足余量)

  {
//      uint16_t raw0 = angle_raw_dma;
//      theta_target = (float)raw0 / 16384.0f * TWO_PI;   // 当前机械角
      theta_target = 0.0f;   // 与theta_multi初值(0)对齐, 上电原地不动
  }
  // 启动UART接收中断(单字节, 收到后进回调)
  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
  // 启动 TIM3 Update 中断
  __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
//  // 占空比测试（50% = 1.65V 平均电压）
//  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 900);
//  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 900);
//  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 900);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
//	  pi_speed.Kp = 0.002f;   // 速度环Kp(先小,稳了再加)
//	    pi_speed.Ki = 0.0002f;  // 速度环Ki
//	    pi_speed.out_max =  1.5f;   // iq_ref上限 = 电流限幅
//	    pi_speed.out_min = -1.5f;
//	    pi_speed.integral = 0;
//	    omega_ref = 0.0f;   // ★目标电角速度50rad/s(慢速起步)
//	    // 不要再写 iq_ref!
//	    HAL_Delay(10);
//	    omega_ref = 0.0f;   // 先不驱动
//	    printf("%.2f,%.2f\n", omega_raw, omega_e);
//	    HAL_Delay(10);
//	    printf("%.2f,%.2f,%.3f\n", omega_ref, omega_e, iq_ref);
//	    HAL_Delay(1);   // 1kHz发送,够看抖动
//	  printf("%.3f,%.3f,%.3f,%.4f\n",
//	            i_d,            // ch0: d轴电流(卡住时应该≈0, 若不为0=对齐残差)
//	            i_q,            // ch1: q轴电流
//	            theta_e_global, // ch2: 当前电角度
//	            theta_target - (float)angle_raw_dma/16384.0f*TWO_PI);  // ch3: 位置误差(rad)

//	  printf("%.3f,%.3f,%.2f,%.2f,%.2f\n",
//	         (float)angle_raw_dma / 16384.0f * TWO_PI,
//	         theta_multi,
//	         (float)turns, i_d, i_q);   // ★turns强转float, 对上%.2f
////	   printf("%.3f,%.3f,%.3f\n", i_d, i_q, theta_e_global);  // d轴电流, q轴电流, 电角度
////	    printf("%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.3f\n",
////	               (float)angle_raw_dma / 16384.0f * TWO_PI,  // ch0: 真实机械角theta_m
////	               pll.theta_hat,                              // ch1: PLL估计角theta_hat
////	               pll.theta_hat - (float)angle_raw_dma / 16384.0f * TWO_PI,  // ch2: 滞后误差(rad)
////	               pll.omega_hat,omega_ref, omega_e, iq_ref);                             // ch3: PLL机械角速度
//	        HAL_Delay(1);
//      printf("%.1f,%.1f,%.3f,%.3f\n",
//             omega_ref,   // ch0: 目标电角速度
//             omega_e,     // ch1: 实际电角速度(PLL)
//             i_d,         // ch2: d轴电流
//             i_q);        // ch3: q轴电流
//    		  printf("%.2f,%.1f,%.3f,%.3f\n",
//    		               theta_multi,                    // ch0: 多圈累加角(rad)
//    		               theta_multi * 0.7958f,          // ch1: 换算位移(mm), =theta_multi/(2π)×5
//    		               (float)turns,                   // ch2: 圈数(整数, 强转float对齐格式)
//    		               i_q);                           // ch3: q轴电流(A), 看推力够不够
//	  printf("%.2f,%.1f,%.3f,%.1f\n",
//	               theta_multi,                    // ch0: 多圈角(rad)
//	               i_q,                            // ch1: q轴电流(A), 看使劲没
//	               theta_multi * 0.7958f,          // ch2: 位移(mm)
//	               (float)stall_flag);             // ch3: 堵转标志(0/1), 撞挡块应跳1
//	  printf("%.2f,%.1f,%.1f,%.1f\n",
//	  	               theta_multi,                    // ch0: 多圈角(rad)
//	  	               i_q,                            // ch1: q轴电流(A)
//	  	               (float)stall_flag,              // ch2: 堵转标志
//	  	               (float)homing_state);           // ch3: 回零状态(0=IDLE,1=SEARCH,2=CRUISE,3=DONE)
	  printf("%.2f,%.1f,%.1f,%.1f,%.1f\n",
	  	  	               theta_multi,                    //实际角度
						   theta_target_final,             //目标角度
						   theta_target_final * 0.7958f,// 目标位置(mm)
						   theta_multi * 0.7958f,//  实际位置(mm)
	  	  	               iq_ref);           // 力矩
//	        printf("%.3f,%.3f,%.3f,%.3f,%.3f\n",
//	               theta_target_final,                          // ch0: CAN收到的目标角(主板发来的)
//	               (float)angle_raw_dma / 16384.0f * TWO_PI,    // ch1: 从板当前实际角
//	               theta_multi,i_d,i_q);                                // ch2: 从板多圈角
	        HAL_Delay(10);
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  CAN_FilterTypeDef sFilterConfig;   // 滤波器配置结构体

  sFilterConfig.FilterBank           = 0;                       // 用第0组滤波器(单CAN, 0~13都行)
  sFilterConfig.FilterMode           = CAN_FILTERMODE_IDMASK;   // 掩码模式
  sFilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT;   // 32位单滤波器(标准帧也用32位最简单)

  sFilterConfig.FilterIdHigh         = 0x101 << 5;   // 放行ID=0x101, 标准ID左移5位到高16位字段
  sFilterConfig.FilterIdLow          = 0x0000;       // 低16位不用
  sFilterConfig.FilterMaskIdHigh     = 0x7FF << 5;   // 掩码全1: 11位ID必须完全匹配
  sFilterConfig.FilterMaskIdLow      = 0x0000;

  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;  // 命中→塞进FIFO0(对应你勾的RX0中断)
  sFilterConfig.FilterActivation     = ENABLE;        // 激活本滤波器
  sFilterConfig.SlaveStartFilterBank = 14;            // CAN1/CAN2分界(单用CAN1, 填14即可)

  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
  {
      Error_Handler();   // 滤波器配置失败→卡死, 方便调试时发现
  }

  if (HAL_CAN_Start(&hcan1) != HAL_OK)   // 启动CAN, 进入正常通信态
  {
      Error_Handler();
  }

  // 打开"收到报文进FIFO0"的中断通知, 之后报文一到就触发回调
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
      Error_Handler();
  }
  /* USER CODE END CAN1_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 2);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 2);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

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

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//    if (htim->Instance == TIM3)
//    {
//        // ===== 开环: 自己生成匀速旋转的电角度 =====
//        theta_force += open_loop_speed;
//        if (theta_force >= TWO_PI) theta_force -= TWO_PI;
//        if (theta_force < 0)       theta_force += TWO_PI;
//
//        float s = sinf(theta_force);
//        float c = cosf(theta_force);
//
//        // ===== 同时读编码器算真实电角度(只为观察,不参与驱动) =====
//        uint16_t raw = angle_raw_dma;
//        float theta_m = (float)raw / 16384.0f * TWO_PI;
//        float theta_e_enc = (theta_offset - theta_m) * POLE_PAIRS + theta_comp;
//        theta_e_enc = fmodf(theta_e_enc, TWO_PI);
//        if (theta_e_enc < 0) theta_e_enc += TWO_PI;
//        theta_e_global = theta_e_enc;   // 存编码器算的,给主循环对比
//
//        // ===== 电流采样(观察用) =====
//        float ia = ADC_TO_AMP(adc_buf[0], offset_A);
//        float ib = ADC_TO_AMP(adc_buf[1], offset_B);
//        i_alpha = ia;
//        i_beta  = (ia + 2.0f * ib) * 0.57735027f;
//        // 用强制角度做Park(看开环下的id/iq)
//        i_d =  i_alpha * c + i_beta * s;
//        i_q = -i_alpha * s + i_beta * c;
//
//        // ===== 开环电压: 纯q轴给电压,直接驱动 =====
//        float Ualpha = -open_loop_volt * s;   // Uq沿q轴: Uα=-Uq·sin, Uβ=Uq·cos
//        float Ubeta  =  open_loop_volt * c;
//        SVPWM(Ualpha, Ubeta);
//
//        // 踢DMA
//        static uint16_t kick_div = 0;
//        if (++kick_div >= 5) { kick_div = 0; AS5047_ReadAngle_DMA_Kick(); }
//    }
//}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        // ===== ① 读上一拍DMA传回的角度 → 算电角度 =====
        uint16_t raw = angle_raw_dma;                       // 上一拍踢的DMA,已传回
        float theta_m = (float)raw / 16384.0f * TWO_PI;
//       float theta_e = (theta_offset - theta_m) * POLE_PAIRS;  // ★方向:你验证过的那个
//        float theta_e = (theta_offset - theta_m) * POLE_PAIRS + 1.5708f;  // +90°电角度补偿
        float theta_e = (theta_offset - theta_m) * POLE_PAIRS - theta_comp;
//        float theta_e = (theta_m - theta_offset) * POLE_PAIRS;
        theta_e = fmodf(theta_e, TWO_PI);
        if (theta_e < 0) theta_e += TWO_PI;
        theta_e_global = theta_e;   // 存全局,留给调试看
        float s = sinf(theta_e);
        float c = cosf(theta_e);

        // ===== Step2: 电流采样 → Clark → Park =====★新增
        float ia = ADC_TO_AMP(adc_buf[0], offset_A);
        float ib = ADC_TO_AMP(adc_buf[1], offset_B);
        i_alpha = ia;
        i_beta  = (ia + 2.0f * ib) * 0.57735027f;   // 1/√3
        i_d =  i_alpha * c + i_beta * s;
        i_q = -i_alpha * s + i_beta * c;
        delta_theta_dbg = atan2f(i_d, i_q);   // 反电动势法: 这个值=Δθ,对齐好应恒定
        // 电流环PI

        // ===== 转换点①：物理电流(A) → 标幺电流(pu) =====
        float id_pu = i_d * I_BASE_INV;
        float iq_pu = i_q * I_BASE_INV;
        float Ud = PI_Update(&pi_id, 0.0f    - id_pu);   // 新:标幺 − 标幺
        float Uq = PI_Update(&pi_iq, iq_ref  - iq_pu);   // 新:标幺 − 标幺
        float Uq_ff = Kff * omega_e;     // 反电势前馈(标幺)
        Uq_ff_dbg = Uq_ff;
        Uq = Uq + Uq_ff;
        if (Uq >  0.95f) Uq =  0.95f;
        if (Uq < -0.95f) Uq = -0.95f;
        // ===== 转换点②：标幺电压(pu) → 物理电压(V) =====
        float Ud_v = Ud * U_BASE;
        float Uq_v = Uq * U_BASE;

        float Ualpha = Ud_v * c - Uq_v * s;
        float Ubeta  = Ud_v * s + Uq_v * c;
        Ua_dbg = Ualpha;   // 临时:看逆Park输出
        Ub_dbg = Ubeta;
        SVPWM(Ualpha, Ubeta);
        // ===== ⑥ 踢下一拍DMA(降频:每50拍踢一次,验证重入假设) =====
        // ===== 速度估计(跟随θe更新节奏,分频5 → 10kHz) =====
        static uint16_t kick_div = 0;
                if (++kick_div >= 5)
                {
                    kick_div = 0;

//                    // ===== M法测速: 当前角度 vs N拍前角度 =====
//                    float theta_now = theta_e;
//                    float theta_old = theta_hist[hist_idx];   // 环形缓冲里最旧的(N拍前)
//
//                    // 角度差(过零处理)
//                    float dtheta = theta_now - theta_old;
//                    if (dtheta >  3.14159265f) dtheta -= TWO_PI;
//                    if (dtheta < -3.14159265f) dtheta += TWO_PI;
//
//                    // 总时间 = N拍 × (5中断×20µs) = SPEED_WIN × 0.0001s
//                    omega_raw = dtheta / (SPEED_WIN * 0.0001f);
//
//                    // 轻滤波即可(噪声已从源头降低)
//                    omega_e = 0.03f * omega_raw + 0.97f * omega_e;
//
//                    // 当前角度写入环形缓冲,指针前进
//                    theta_hist[hist_idx] = theta_now;
//                    hist_idx = (hist_idx + 1) % SPEED_WIN;
                    // ===== PLL观测器测速(替换M法) =====
//                     PLL_Update(&pll, theta_m);          // 喂机械角
//                     omega_e = -pll.omega_hat * POLE_PAIRS;  // 机械角速度→电角速度
//
//                    // 位置环用PLL的平滑角度(可选,先用编码器原始theta_m也行)
//                    // float theta_for_pos = pll.theta_hat;
//                    omega_ref=500;
//                    iq_ref = 0.0f;   // ★直接给恒定转矩电流, 不经速度环
//                    if(0)
//                    {       // ===== ★位置环 P (套在速度环外) ===== 新增
//                    // 误差用机械角(单圈最短路径), 输出乘POLE_PAIRS转成电角度ω_ref
//                    float theta_err = theta_target - theta_m;            // 机械角误差(rad)
//                    while (theta_err >  PI) theta_err -= TWO_PI;         // wrap到±π,走最短路
//                    while (theta_err < -PI) theta_err += TWO_PI;
////                    // ===== ★位置环死区: 误差在±ε内当作0,屏蔽编码器噪声 =====
////                    if (theta_err < POS_DEADBAND && theta_err > -POS_DEADBAND)
////                        theta_err = 0.0f;
//                    float omega_ref_mech = Kp_pos * theta_err;           // 机械角ω_ref(rad/s)
//                    if (omega_ref_mech >  omega_max_mech) omega_ref_mech =  omega_max_mech;
//                    if (omega_ref_mech < -omega_max_mech) omega_ref_mech = -omega_max_mech;
//
//                    omega_ref = omega_ref_mech * POLE_PAIRS;             // ★转电角度,喂速度环
//                    }
                    // 速度环PI
//                    iq_ref = PI_Update(&pi_speed, -(omega_ref - omega_e));

                    // ===== 单圈→多圈累加 =====
                    float dtheta_m = theta_m - theta_m_last;
                    if (dtheta_m >  PI) { dtheta_m -= TWO_PI; turns--; }   // 过0→2π边界(倒转)
                    if (dtheta_m < -PI) { dtheta_m += TWO_PI; turns++; }   // 过2π→0边界(正转)
                    theta_multi += dtheta_m;        // 累加真实增量
                    theta_m_last = theta_m;
                    // ===== 堵转检测(纯观测, 不动作) =====
                    // 判据: i_q在使劲(电流大) 且 theta_multi几乎没动 → 累加计数
                    float iq_pu_abs = i_q * I_BASE_INV;       // i_q转标幺
                    if (iq_pu_abs < 0) iq_pu_abs = -iq_pu_abs; // 取绝对值

                    float dmulti = theta_multi - theta_multi_last_stall;
                    if (dmulti < 0) dmulti = -dmulti;          // 位置变化绝对值

                    if (iq_pu_abs > STALL_IQ_THRESH && dmulti < STALL_MOVE_THRESH)
                    {
                        // 使劲了但没动 → 计数++
                        if (stall_counter < STALL_COUNT_MAX) stall_counter++;
                        if (stall_counter >= STALL_COUNT_MAX) stall_flag = 1;   // 持续够久 → 判堵转
                    }
                    else
                    {
                        // 任一条件不满足(没使劲 或 动了) → 计数清零, 解除标志
                        stall_counter = 0;
                        stall_flag = 0;
                        theta_multi_last_stall = theta_multi;   // ★加这行: 没堵转时, 基准跟着当前位置走
                    }

                    // ===== PLL测速(只为D项阻尼用, 可以重滤波) =====
                    PLL_Update(&pll, theta_m);

                    omega_e = pll.omega_hat * POLE_PAIRS;
                    // 给omega_e再加一层重滤波, 专门给D项用(慢没关系)
                    static float omega_filt = 0;
                    omega_filt = 0.9f * omega_filt + 0.1f * omega_e;
                    // ===== 指令缓动: 中间目标以恒定速率爬向最终目标(匀速运动) =====
                    float target_err = theta_target_final - theta_target;
                    if (target_err >  theta_ramp_rate)      theta_target += theta_ramp_rate;
                    else if (target_err < -theta_ramp_rate) theta_target -= theta_ramp_rate;
                    else                                    theta_target = theta_target_final;
                    // ===== 位置环: 多圈角, 误差限幅实现匀速 =====
                    // ===== 回零状态机(骨架, 逻辑后填) =====
                                        switch (homing_state)
                                        {
                                        case HOMING_SEARCH:
                                            // ① 力矩缓慢爬升(朝负方向=挡块方向)
                                            homing_torque -= HOMING_TORQUE_RAMP;
                                            if (homing_torque < -HOMING_TORQUE_MAX)
                                                homing_torque = -HOMING_TORQUE_MAX;   // 限幅

                                            // ② 检测突破: 累积位移超阈值 = 滑块动了
                                            float move = theta_multi - homing_start_pos;
                                            if (move < 0) move = -move;
                                            if (move > HOMING_BREAK_MOVE)
                                            {
                                                homing_t_break = homing_torque;   // 记录突破力矩
                                                homing_state = HOMING_CRUISE;     // 转巡航
                                            }

                                            // ③ 力矩到上限: 不立刻退出, 先顶住一段时间给机会突破
                                            if (homing_torque <= -HOMING_TORQUE_MAX)
                                            {
                                                homing_hold_counter++;
                                                if (homing_hold_counter >= HOMING_HOLD_MAX)
                                                {
                                                    // 顶住够久还没突破 → 真的推不动, 退出
                                                    homing_torque = 0.0f;
                                                    homing_hold_counter = 0;
                                                    homing_state = HOMING_IDLE;
                                                }
                                            }


                                            // ★把回零力矩输出给 torque_ref(让电机真的动)
                                            torque_ref = homing_torque;
                                            break;

                                            case HOMING_CRUISE:
                                                // TODO: 降力矩巡航, 撞挡块(stall_flag) → 转DONE
                                                // ① 降力矩: 突破后用突破力矩的60%, 防止加速狂奔
                                                torque_ref = homing_t_break * 0.6f;

                                                // ② 撞挡块检测: 堵转标志触发 → 回零完成
                                                if (stall_flag)
                                                {
                                                    homing_state = HOMING_DONE;
                                                }
                                                break;

                                            case HOMING_DONE:
                                                // 回零完成: 设零点, 切位置模式
                                                theta_multi = 0.0f;
                                                theta_multi_last_stall = 0.0f;
                                                theta_target_final = 0.0f;
                                                homing_torque = 0.0f;
                                                torque_ref = 0.0f;            // ★加: 停力矩, 别再顶挡块
                                                stall_flag = 0;               // ★加: 清堵转标志
                                                stall_counter = 0;
                                                homing_state = HOMING_IDLE;   // 回到IDLE
                                                // 注: 这里先不切位置模式, 框架阶段保持力矩模式观察
                                                break;

                                            case HOMING_IDLE:
                                            default:
                                                break;
                                        }
                    if (control_mode == MODE_TORQUE)
                    {
                        // ===== 力矩模式: 外部指令直接给 iq_ref(标幺) =====
                        iq_ref = torque_ref;
                        // ===== 力矩模式软件限位: 到边界时禁止继续往界外使力 =====
                        // (回零SEARCH/CRUISE期间不限, 否则回零撞不了挡块)
                        if (homing_state == HOMING_IDLE)
                        {
                            // 已到上限(左端) 且 还想往左(正iq) → 禁止
                            if (theta_multi > POS_LIMIT_MAX && iq_ref > 0.0f)
                                iq_ref = 0.0f;
                            // 已到下限(挡块) 且 还想往右(负iq) → 禁止
                            if (theta_multi < POS_LIMIT_MIN && iq_ref < 0.0f)
                                iq_ref = 0.0f;
                        }
                    }
                    else
                    {
                        // ===== 位置模式软件限位: clamp目标在安全行程内 =====
                          if (theta_target_final > POS_LIMIT_MAX) theta_target_final = POS_LIMIT_MAX;
                          if (theta_target_final < POS_LIMIT_MIN) theta_target_final = POS_LIMIT_MIN;
                    // ===== 缓动: 中间目标target以恒定速率爬向final(决定回正转速) =====
                                theta_target = theta_target_final;

                                        // ===== 位置环: 用完整误差跟踪target(不限幅, 力矩充足) =====
                                float theta_err = theta_target_final - theta_multi;

                                float err_max = 5.0f;
                                float theta_err_sat = theta_err;
                                if (theta_err_sat >  err_max) theta_err_sat =  err_max;
                                if (theta_err_sat < -err_max) theta_err_sat = -err_max;



                                if (theta_err < 0.6f && theta_err > -0.6f) {
                                    // 死区: 冻结, 保持力矩
                                    iq_ref = 0;

                                } else {
                                    pos_integral += Ki_pos * theta_err_sat;   // ★用限幅误差积分, 防windup
                                    if (pos_integral >  1.0f) pos_integral =  1.0f;   // 0.2/1.5
                                    if (pos_integral < -1.0f) pos_integral = -1.0f;

                                iq_ref = Kp_pos * theta_err_sat + pos_integral
                                         - Kd_pos * (omega_filt / POLE_PAIRS);
                            }
                    }
                            if (iq_ref >  1.5f) iq_ref =  1.5f;   // 1.5/1.5 = 标幺满量程
                            if (iq_ref < -1.5f) iq_ref = -1.5f;
                    // 踢DMA
                    AS5047_ReadAngle_DMA_Kick();
                }
    }
}
// ===== UART接收完成回调: 逐字符收, 遇\n解析角度 =====
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {rx_irq_count++;
    rx_byte_dbg = rx_byte;   // ★抓原始字节值
        if (rx_byte == '\n' || rx_byte == '\r')
        {
        	if (rx_cmd_idx > 0)
        	{
        	    rx_cmd_buf[rx_cmd_idx] = '\0';
        	    last_cmd_char = rx_cmd_buf[0];   // ★抓首字符
        	    last_cmd_len  = rx_cmd_idx;      // ★抓长度
        	    // ===== 命令解析: 单字符命令 vs 数字 =====
        	    if (rx_cmd_buf[0] == 'P' || rx_cmd_buf[0] == 'p')
        	    {
        	        SetControlMode(MODE_POSITION);     // 发 "P" → 位置模式
        	    }
        	    else if (rx_cmd_buf[0] == 'T' || rx_cmd_buf[0] == 't')
        	    {
        	        SetControlMode(MODE_TORQUE);       // 发 "T" → 力矩模式
        	    }
        	    else if (rx_cmd_buf[0] == 'H' || rx_cmd_buf[0] == 'h')   // ★新增
        	    {
        	        SetControlMode(MODE_TORQUE);       // 回零在力矩模式下做
        	        homing_state = HOMING_SEARCH;      // 启动回零状态机
        	        homing_torque = 0.0f;              // 力矩从0开始爬
        	        homing_start_pos = theta_multi;   // 记下回零起点
        	    }
        	    else
        	    {
        	        // 纯数字: 按当前模式分发
        	        float val = atof(rx_cmd_buf);
        	        if (control_mode == MODE_TORQUE)
        	            torque_ref = val;              // 力矩模式: 数字 = 力矩指令(标幺)
//        	            omega_ref = val;   // 临时: 数字=目标转速

        	        else
        	            theta_target_final = val;      // 位置模式: 数字 = 目标角(rad)
        	    }

        	    rx_cmd_idx = 0;
        	}
        }
        else   // ★加回这个分支:普通字符存进buffer
        {
            if (rx_cmd_idx < sizeof(rx_cmd_buf) - 1)
                rx_cmd_buf[rx_cmd_idx++] = rx_byte;
        }
        // ★关键: 重新启动下一字节接收(否则只收一次)
        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    }
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
    {
        return;
    }

    if (RxHeader.StdId == 0x101)
    {
        float_bytes_t conv;
        conv.b[0] = RxData[0];
        conv.b[1] = RxData[1];
        conv.b[2] = RxData[2];
        conv.b[3] = RxData[3];

        theta_target_final = conv.f;
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
