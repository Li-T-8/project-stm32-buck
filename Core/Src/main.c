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
#include "adc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// 1. PID ?????
typedef struct {
    float Kp;          // ????
    float Ki;          // ????
    float Kd;          // ????
    float Target;      // ???
    float Actual;      // ???
    float Error;       // ????
    float Last_Error;  // ????
    float Integral;    // ????
    float Output_Max;  // PWM???? (??????)
    float Output_Min;  // PWM???? (??????)
} PID_HandleTypeDef;

// 2. ??????
PID_HandleTypeDef Buck_PID;
uint16_t adc_raw = 0;
float v_out_filt = 0.0f;
volatile uint8_t adc_ready = 0; // ADC??????
float duty_pulse = 0.0f; 

// 3. ???????? (ADC????)
#define ADC_SAMPLES 16
uint16_t adc_buffer[ADC_SAMPLES];
uint8_t buffer_idx = 0;
uint32_t adc_sum = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void PID_Init(PID_HandleTypeDef *pid, float p, float i, float d, float max, float min);
float PID_Calc(PID_HandleTypeDef *pid, float current_val);

void PID_Init(PID_HandleTypeDef *pid, float p, float i, float d, float max, float min) {
    pid->Kp = p;
    pid->Ki = i;
    pid->Kd = d;
    pid->Target = 12.0f; // ??? 12V
    pid->Error = 0.0f;
    pid->Last_Error = 0.0f;
    pid->Integral = 0.0f;
    pid->Output_Max = max;
    pid->Output_Min = min;
}

float PID_Calc(PID_HandleTypeDef *pid, float current_val) {
    pid->Actual = current_val;
    pid->Error = pid->Target - pid->Actual;
    
    // ????
    pid->Integral += pid->Error;
    // ???????
    if(pid->Integral > pid->Output_Max) pid->Integral = pid->Output_Max;
    if(pid->Integral < pid->Output_Min) pid->Integral = pid->Output_Min;
    
    // ??PID??
    float output = pid->Kp * pid->Error + pid->Ki * pid->Integral + pid->Kd * (pid->Error - pid->Last_Error);
    pid->Last_Error = pid->Error;
    
    // ????(????????)
    if(output > pid->Output_Max) output = pid->Output_Max;
    if(output < pid->Output_Min) output = pid->Output_Min;
    
    return output;
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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  // 1. PID ????? (????????????!)
PID_Init(&Buck_PID, 1.5f, 0.2f, 0.0f, 650.0f, 100.0f); 
  // 2. ???? PWM ??
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);

  // 3. ?? ?????????????????
  // ??????? 2ms,? IR2103 ? C2 ???????
  // 3. ??????????(??? C2 ????)
  // ????:????????,????? TIM_CHANNEL_1N?
  // ?????(CH1)?? 0 ?,??????????(CH1N)?? 100% ????
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0); 
  HAL_Delay(2); // ?????? 2ms,? IR2103 ? C2 ?????
  
  // ??????? PWM ????(????? 50%,? 360)
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 360);     // ???????50%

  // 4. ?? TIM3 ????? (10kHz ????)
  HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// 1. TIM3 ????? (???? 100us)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
        // ?? ADC ????
        HAL_ADC_Start_IT(&hadc1); 
        
        // ?? ADC ?????,?? PID ??
        if (adc_ready) {
            adc_ready = 0; // ????
            
            // ??????? (3.3V??4095,?????5?)
            v_out_filt = (adc_raw / 4095.0f) * 13.3f; 
            
            // ?? PID ??,???? PWM ???
            duty_pulse = PID_Calc(&Buck_PID, v_out_filt); // ?????????
            
            // ??????????? TIM1 ? CH1
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)duty_pulse);
            // ???????????,????0??
        }
    }
}

// 2. ADC ??????
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {
        uint16_t temp_val = HAL_ADC_GetValue(&hadc1);
        
        // ???????? (????????????)
        adc_sum -= adc_buffer[buffer_idx];
        adc_buffer[buffer_idx] = temp_val;
        adc_sum += temp_val;
        buffer_idx++;
        if (buffer_idx >= ADC_SAMPLES) buffer_idx = 0;
        
        adc_raw = adc_sum / ADC_SAMPLES; // ?????
        adc_ready = 1; // ?? TIM3 ???????
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
