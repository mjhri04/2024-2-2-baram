/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#include <stdio.h>
#include <stdbool.h>
#include <usbd_cdc_if.h>
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_SENSORS 4
#define IMU_DATA_SIZE 50
#define PACKET_SIZE 154
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;

/* USER CODE BEGIN PV */
uint16_t sensorValues[NUM_SENSORS]; //압력센서값 저장
uint8_t packetBuffer[PACKET_SIZE];
//nt16_t pressureSensors[4];

//각 IMU값 저장
unsigned char IMU1array[IMU_DATA_SIZE] = {0};
unsigned char IMU2array[IMU_DATA_SIZE] = {0};

double alpha_value = 0.2;
double prevSensorValues[4] = {0};

double roll1 = 0.0, pitch1 = 0.0, yaw1 = 0.0;
double gyro_x1 = 0.0, gyro_y1 = 0.0, gyro_z1 = 0.0;
double accel_x1 = 0.0, accel_y1 = 0.0, accel_z1 = 0.0;
char alpha1[10] = {0}, beta1[10] = {0}, gamm1[10] = {0};
char gyro_x1_str[10] = {0}, gyro_y1_str[10] = {0}, gyro_z1_str[10] = {0};
char accel_x1_str[10] = {0}, accel_y1_str[10] = {0}, accel_z1_str[10] = {0};

double roll2 = 0.0, pitch2 = 0.0, yaw2 = 0.0;
double gyro_x2 = 0.0, gyro_y2 = 0.0, gyro_z2 = 0.0;
double accel_x2 = 0.0, accel_y2 = 0.0, accel_z2 = 0.0;
char alpha2[10] = {0}, beta2[10] = {0}, gamm2[10] = {0};
char gyro_x2_str[10] = {0}, gyro_y2_str[10] = {0}, gyro_z2_str[10] = {0};
char accel_x2_str[10] = {0}, accel_y2_str[10] = {0}, accel_z2_str[10] = {0};


double roll1_prev = 0, pitch1_prev = 0, yaw1_prev = 0;
double roll2_prev = 0, pitch2_prev = 0, yaw2_prev = 0;
double roll3_prev = 0, pitch3_prev = 0, yaw3_prev = 0;


double prevAccelX1 = 0, prevAccelX2 = 0, prevAccelX3 = 0;
double prevAccelY1 = 0, prevAccelY2 = 0, prevAccelY3 = 0;
double prevAccelZ1 = 0, prevAccelZ2 = 0, prevAccelZ3 = 0;


double prevGyroX1 = 0, prevGyroX2 = 0, prevGyroX3 = 0;
double prevGyroY1 = 0, prevGyroY2 = 0, prevGyroY3 = 0;
double prevGyroZ1 = 0, prevGyroZ2 = 0, prevGyroZ3 = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM9_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void get_IMU1(void);
void get_IMU2(void);
void Read_Pressure_Data(void);
double ApplyLowPassFilter(int16_t currentValue, double *prevValue);
void BuildPacket(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_USB_DEVICE_Init();
  MX_TIM9_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim9);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  Read_Pressure_Data();
	  get_IMU1();
	  get_IMU2();

	  //패킷화 하기
	  packetBuffer[0] = 0xAA;

      /* IMU1 Data */
      memcpy(packetBuffer + 1, &roll1, sizeof(roll1));
      memcpy(packetBuffer + 9, &pitch1, sizeof(pitch1));
      memcpy(packetBuffer + 17, &yaw1, sizeof(yaw1));
      memcpy(packetBuffer + 25, &gyro_x1, sizeof(gyro_x1));
      memcpy(packetBuffer + 33, &gyro_y1, sizeof(gyro_y1));
      memcpy(packetBuffer + 41, &gyro_z1, sizeof(gyro_z1));
      memcpy(packetBuffer + 49, &accel_x1, sizeof(accel_x1));
      memcpy(packetBuffer + 57, &accel_y1, sizeof(accel_y1));
      memcpy(packetBuffer + 65, &accel_z1, sizeof(accel_z1));

      /* IMU2 Data */
      memcpy(packetBuffer + 73, &roll2, sizeof(roll2));
      memcpy(packetBuffer + 81, &pitch2, sizeof(pitch2));
      memcpy(packetBuffer + 89, &yaw2, sizeof(yaw2));
      memcpy(packetBuffer + 97, &gyro_x2, sizeof(gyro_x2));
      memcpy(packetBuffer + 105, &gyro_y2, sizeof(gyro_y2));
      memcpy(packetBuffer + 113, &gyro_z2, sizeof(gyro_z2));
      memcpy(packetBuffer + 121, &accel_x2, sizeof(accel_x2));
      memcpy(packetBuffer + 129, &accel_y2, sizeof(accel_y2));
      memcpy(packetBuffer + 137, &accel_z2, sizeof(accel_z2));

      /* Pressure Sensor Data */
      for (int i = 0; i < NUM_SENSORS; i++) {
          packetBuffer[145 + i * 2] = (sensorValues[i] >> 8) & 0xFF; // 상위 바이트
          packetBuffer[146 + i * 2] = sensorValues[i] & 0xFF;        // 하위 바이트
      }


      /* End frame */
      packetBuffer[153] = 0xBB;

      /* Transmit data via USB CDC */
      if (CDC_Transmit_FS(packetBuffer, sizeof(packetBuffer)) == USBD_OK) {
         HAL_Delay(40);
      } else {
          HAL_Delay(10);
      }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* ADC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ADC_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* TIM1_BRK_TIM9_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
  /* USART6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART6_IRQn);
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
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

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  for(int i=0;i < NUM_SENSORS; i++){
	  sConfig.Channel = ADC_CHANNEL_0 + i;
	  sConfig.Rank = i + 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
	  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
  }
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 839;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 100-1;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Read_Pressure_Data(void) {
    ADC_ChannelConfTypeDef sConfig = {0};

    for (int i = 0; i < NUM_SENSORS; i++) {
        // ADC 채널을 각 센서 핀에 맞게 설정 (IN0, IN1, IN4, IN5)
        switch(i) {
            case 0: sConfig.Channel = ADC_CHANNEL_0; break; // IN0
            case 1: sConfig.Channel = ADC_CHANNEL_1; break; // IN1
            case 2: sConfig.Channel = ADC_CHANNEL_4; break; // IN4
            case 3: sConfig.Channel = ADC_CHANNEL_5; break; // IN5
        }

        sConfig.Rank = 1;
        sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
            Error_Handler();
        }

        if (HAL_ADC_Start(&hadc1) == HAL_OK) {
            if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {

            	sensorValues[i] = HAL_ADC_GetValue(&hadc1);
            } else {
                Error_Handler();
            }
            HAL_ADC_Stop(&hadc1);
        } else {
            Error_Handler();
        }
    }
}

void get_IMU1()
{
    HAL_UART_Receive_DMA(&huart1, IMU1array, 50);  // 수신 데이터 크기 확장
    int cnt0 = 0, pos = 0;
    for (int i = 0; i < 50; i++) {
        if (IMU1array[i] == ',') {
            switch (cnt0++) {
                case 0: strncpy(alpha1, (char *)IMU1array + pos, i - pos); alpha1[i - pos] = '\0'; break;
                case 1: strncpy(beta1, (char *)IMU1array + pos, i - pos); beta1[i - pos] = '\0'; break;
                case 2: strncpy(gamm1, (char *)IMU1array + pos, i - pos); gamm1[i - pos] = '\0'; break;
                case 3: strncpy(gyro_x1_str, (char *)IMU1array + pos, i - pos); gyro_x1_str[i - pos] = '\0'; break;
                case 4: strncpy(gyro_y1_str, (char *)IMU1array + pos, i - pos); gyro_y1_str[i - pos] = '\0'; break;
                case 5: strncpy(gyro_z1_str, (char *)IMU1array + pos, i - pos); gyro_z1_str[i - pos] = '\0'; break;
                case 6: strncpy(accel_x1_str, (char *)IMU1array + pos, i - pos); accel_x1_str[i - pos] = '\0'; break;
                case 7: strncpy(accel_y1_str, (char *)IMU1array + pos, i - pos); accel_y1_str[i - pos] = '\0'; break;
                case 8: strncpy(accel_z1_str, (char *)IMU1array + pos, i - pos); accel_z1_str[i - pos] = '\0'; break;
            }
            pos = i + 1;
        }
    }
    roll1 = strtod(alpha1, NULL);
    pitch1 = strtod(beta1, NULL);
    yaw1 = strtod(gamm1, NULL);
    gyro_x1 = strtod(gyro_x1_str, NULL);
    gyro_y1 = strtod(gyro_y1_str, NULL);
    gyro_z1 = strtod(gyro_z1_str, NULL);
    accel_x1 = strtod(accel_x1_str, NULL);
    accel_y1 = strtod(accel_y1_str, NULL);
    accel_z1 = strtod(accel_z1_str, NULL);
}

void get_IMU2()
{
    HAL_UART_Receive_DMA(&huart6, IMU2array, 50);
    int cnt0 = 0, pos = 0;
    for (int i = 0; i < 50; i++) {
        if (IMU2array[i] == ',') {
            switch (cnt0++) {
                case 0: strncpy(alpha2, (char *)IMU2array + pos, i - pos); alpha2[i - pos] = '\0'; break;
                case 1: strncpy(beta2, (char *)IMU2array + pos, i - pos); beta2[i - pos] = '\0'; break;
                case 2: strncpy(gamm2, (char *)IMU2array + pos, i - pos); gamm2[i - pos] = '\0'; break;
                case 3: strncpy(gyro_x2_str, (char *)IMU2array + pos, i - pos); gyro_x2_str[i - pos] = '\0'; break;
                case 4: strncpy(gyro_y2_str, (char *)IMU2array + pos, i - pos); gyro_y2_str[i - pos] = '\0'; break;
                case 5: strncpy(gyro_z2_str, (char *)(char *)IMU2array + pos, i - pos); gyro_z2_str[i - pos] = '\0'; break;
                case 6: strncpy(accel_x2_str, (char *)IMU2array + pos, i - pos); accel_x2_str[i - pos] = '\0'; break;
                case 7: strncpy(accel_y2_str, (char *)IMU2array + pos, i - pos); accel_y2_str[i - pos] = '\0'; break;
                case 8: strncpy(accel_z2_str, (char *)IMU2array + pos, i - pos); accel_z2_str[i - pos] = '\0'; break;
            }
            pos = i + 1;
        }
    }
    roll2 = strtod(alpha2, NULL);
    pitch2 = strtod(beta2, NULL);
    yaw2 = strtod(gamm2, NULL);
    gyro_x2 = strtod(gyro_x2_str, NULL);
    gyro_y2 = strtod(gyro_y2_str, NULL);
    gyro_z2 = strtod(gyro_z2_str, NULL);
    accel_x2 = strtod(accel_x2_str, NULL);
    accel_y2 = strtod(accel_y2_str, NULL);
    accel_z2 = strtod(accel_z2_str, NULL);
}
double ApplyLowPassFilter(int16_t currentValue, double *prevValue){
	*prevValue = alpha_value * currentValue + (1.0 - alpha_value) * (*prevValue);
	return *prevValue;
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

#ifdef  USE_FULL_ASSERT
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
