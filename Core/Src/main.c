/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @author         : Kaushik Mehta
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "math.h"
#include "temp.h"
#include "CAN.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum
{
	DISCHARGE_STATE = 1,
	CHARGING_STATE = 2

} ECU_StateType;

// Define boolean variable
typedef enum
{
	true = 1,
	false = 0

} bool;

ECU_StateType ecuState;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SEGMENT_ID (0x01U)

#define DISCHARGE_TEMP_MAX_LIMIT 60
#define DISCHARGE_TEMP_MIN_LIMIT -20
#define CHARGE_TEMP_MAX_LIMIT 45
#define CHARGE_TEMP_MIN_LIMIT 0
#define ERROR_THRESHOLD 120

#define OPEN_CIRCUIT_THRESHOLD -4000
#define SHORT_TO_GROUND_THRESHOLD 14000

#define CELL_COUNT 24

#define SYNC_TIME_SHIFT_ECU1 25
#define SYNC_TIME_SHIFT_ECU2 50
#define SYNC_TIME_SHIFT_ECU3 75
#define SYNC_TIME_SHIFT_ECU4 100
#define SYNC_TIME_SHIFT_ECU5 125

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan;

IWDG_HandleTypeDef hiwdg;

/* USER CODE BEGIN PV */

// Temp Sense RCU Fault
const uint16_t FAULT_IN_PIN = GPIO_PIN_9;
const uint16_t FAULT_OUT_PIN = GPIO_PIN_8;
GPIO_TypeDef* FAULT_PIN_PORT = GPIOB;

// Watchdog
const uint16_t WATCHDOG_LED_PIN = GPIO_PIN_13;
GPIO_TypeDef* WATCHDOG_LED_PIN_PORT = GPIOC;

// Temperature Collection and Processing
int highestTemp = DISCHARGE_TEMP_MIN_LIMIT, lowestTemp = DISCHARGE_TEMP_MAX_LIMIT, tempArray[24] = { 0 };
int counterCH1 = 0, counterCH2 = 12, faultCounter = 0;
int cellTempSum = 0, averageSegmentTemp;
unsigned int highestTempCellCount = 0;

// CAN RX headers
CAN_RxHeaderTypeDef rxHeaderFIFO0;
uint8_t dataFIFO0[8] = { (0x00U) };
TS_ECU_SYNC_RX1_t diagnosticSync_t;
TS_ECU_SYNC_RX2_t temperatureSync_t;
TS_ECU_ChargingStateTrigger_t stateTrigger_t;

// CAN TX header
TS_ECU1_TX1_t tx1_t;
volatile bool msgPendingFlag = false;

// Timing Variables
unsigned int currentChargingTriggerTime = 0, syncOneTime = 0, syncTwoTime = 0;

// Charging State
bool chargingTriggerFlag = false;

// Sync
bool syncOneFlag = false;
bool syncTwoFlag = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */

// ADC to Temperature conversion
static int getTempCH1(ADC_HandleTypeDef* hadc);
static int getTempCH2(ADC_HandleTypeDef* hadc);

// CAN ISR
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  /* Prevent unused argument(s) compilation warning */
  //UNUSED(hcan);

	if (hcan->Instance == CAN1)
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeaderFIFO0, dataFIFO0);

		msgPendingFlag = true;
	}

  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_CAN_RxFifo0MsgPendingCallback could be implemented in the
            user file
   */
}

// Fault ISR
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  //UNUSED(GPIO_Pin);

	if (GPIO_Pin == FAULT_IN_PIN)
	{
		tx1_t.TS_ECU_FaultInState = HAL_GPIO_ReadPin(FAULT_PIN_PORT, FAULT_IN_PIN);

		HAL_GPIO_WritePin(FAULT_PIN_PORT, FAULT_OUT_PIN, GPIO_PIN_RESET);
	}

  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
}

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
  MX_ADC1_Init();
  MX_CAN_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

  // Initial State
  ecuState = DISCHARGE_STATE;

  HAL_GPIO_WritePin(FAULT_PIN_PORT, FAULT_OUT_PIN, GPIO_PIN_SET);

  tx1_t.TS_ECU_FaultOutState = GPIO_PIN_SET;

  tx1_t.TS_ECU_FaultInState = HAL_GPIO_ReadPin(FAULT_PIN_PORT, FAULT_IN_PIN);

  // Initialize CAN
  HAL_CAN_Start(&hcan);

  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

  // CAN Filter Configuration
  TS_ECU_ChargingStateTrigFilterConfig();

  TS_ECU_SYNC_RX1_FilterConfig();

  TS_ECU_SYNC_RX2_FilterConfig();

  tx1_t.TS_ECU_OpenCircuitFault = false;
  tx1_t.TS_ECU_TempThresholdFault = false;

  HAL_Delay(100);

  HAL_GPIO_WritePin(WATCHDOG_LED_PIN_PORT, WATCHDOG_LED_PIN, GPIO_PIN_RESET);

  // Enable Watchdog
  HAL_IWDG_Init(&hiwdg);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // CAN Message Processing
	  if (msgPendingFlag == true)
	  {
		  if (rxHeaderFIFO0.StdId == TS_ECU_SYNC_RX1_CANID && (((dataFIFO0[0] & (0xFFU)) == (0x00U)) || ((dataFIFO0[0] & (0xFFU)) == SEGMENT_ID)))
		  {
			  syncOneFlag = true;

			  //syncOneTime = HAL_GetTick();
		  }
		  else if (rxHeaderFIFO0.StdId == TS_ECU_SYNC_RX2_CANID && (((dataFIFO0[0] & (0xFFU)) == (0x00U)) || ((dataFIFO0[0] & (0xFFU)) == SEGMENT_ID)))
		  {
			  syncTwoFlag = true;

			  syncTwoTime = HAL_GetTick();
		  }
		  else if (rxHeaderFIFO0.StdId == TS_ECU_ChargingStateTrigger_CANID)
		  {
			  Unpack_TS_ECU_ChargingStateTrigger_Temp(&stateTrigger_t, dataFIFO0, TS_ECU_ChargingStateTrigger_DLC);

			  if (stateTrigger_t.Orion_2_ChargePowerState == (0x01U) && stateTrigger_t.Orion_2_ChargeSafetyState == (0x01U))
			  {
				  ecuState = CHARGING_STATE;

				  chargingTriggerFlag = true;

				  currentChargingTriggerTime = HAL_GetTick();
			  }
		  }

		  msgPendingFlag = false;
	  }

	  // Process Temperatures
	  for (int s3 = 0; s3 <= 1; s3++)
	  {
		  for (int s2 = 0; s2 <= 1; s2++)
		  {
			  for (int s1 = 0; s1 <= 1; s1++)
			  {
				  for (int s0 = 0; s0 <= 1; s0++)
				  {
					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, s3);
					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, s2);
					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, s1);
					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, s0);

					  if ((s3 == 1 && s2 == 1 && s1 == 1) || (s0 == 1 && s2 == 1 && s3 == 1) || (s3 == 1 && s2 == 1 && s1 == 0 && s0 == 0))
					  {
						  continue;
					  }

					  /*******************************  Cell 1-12 *******************************/

					  tempArray[counterCH1] = getTempCH1(&hadc1);

					  if (tempArray[counterCH1] < OPEN_CIRCUIT_THRESHOLD)
					  {
						  HAL_GPIO_WritePin(FAULT_PIN_PORT, FAULT_OUT_PIN, GPIO_PIN_RESET);

						  tempArray[counterCH1] = 200;

						  tx1_t.TS_ECU_OpenCircuitFault = true;

						  tx1_t.TS_ECU_FaultOutState = GPIO_PIN_RESET;
					  }

					  if (tempArray[counterCH1] > SHORT_TO_GROUND_THRESHOLD)
					  {
						  HAL_GPIO_WritePin(FAULT_PIN_PORT, FAULT_OUT_PIN, GPIO_PIN_RESET);

						  tempArray[counterCH1] = 200;

						  tx1_t.FAN_ECU_ShortToGroundFault = true;

						  tx1_t.TS_ECU_FaultOutState = GPIO_PIN_RESET;
					  }

					  if (tempArray[counterCH1] == 200)
					  {
						  cellTempSum += 0;
					  }
					  else
					  {
						  cellTempSum += tempArray[counterCH1];
					  }

					  if (tempArray[counterCH1] > highestTemp && tempArray[counterCH1] < ERROR_THRESHOLD)
					  {
						  highestTemp = tempArray[counterCH1];

						  highestTempCellCount = 0;
					  }

					  if (tempArray[counterCH1] < lowestTemp)
					  {
						  lowestTemp = tempArray[counterCH1];
					  }

					  if (tempArray[counterCH1] == highestTemp)
					  {
						  highestTempCellCount++;
					  }

					  /*******************************  Cell 13-24 *******************************/

					  tempArray[counterCH2] = getTempCH2(&hadc1);

					  if (tempArray[counterCH2] < OPEN_CIRCUIT_THRESHOLD)
					  {
						  HAL_GPIO_WritePin(FAULT_PIN_PORT, FAULT_OUT_PIN, GPIO_PIN_RESET);

						  tempArray[counterCH2] = 200;

						  tx1_t.TS_ECU_OpenCircuitFault = true;

						  tx1_t.TS_ECU_FaultOutState = GPIO_PIN_RESET;
					  }

					  if (tempArray[counterCH2] > SHORT_TO_GROUND_THRESHOLD)
					  {
						  HAL_GPIO_WritePin(FAULT_PIN_PORT, FAULT_OUT_PIN, GPIO_PIN_RESET);

						  tempArray[counterCH2] = 200;

						  tx1_t.FAN_ECU_ShortToGroundFault = true;

						  tx1_t.TS_ECU_FaultOutState = GPIO_PIN_RESET;
					  }

					  if (tempArray[counterCH2] == 200)
					  {
						  cellTempSum += 0;
					  }
					  else
					  {
						  cellTempSum += tempArray[counterCH2];
					  }

					  if (tempArray[counterCH2] > highestTemp && tempArray[counterCH2] < ERROR_THRESHOLD)
					  {
						  highestTemp = tempArray[counterCH2];

						  highestTempCellCount = 0;
					  }

					  if (tempArray[counterCH2] < lowestTemp)
					  {
						  lowestTemp = tempArray[counterCH2];
					  }

					  if (tempArray[counterCH2] == highestTemp)
					  {
						  highestTempCellCount++;
					  }

					  counterCH1++;

					  counterCH2++;
				  }
			  }
		  }
	  }

	  averageSegmentTemp = (cellTempSum / CELL_COUNT);

	  // Charging State Watchdog
	  if (HAL_GetTick() - currentChargingTriggerTime > 5000)
	  {
		  ecuState = DISCHARGE_STATE;

		  chargingTriggerFlag = false;
	  }

	  // Temperature Threshold Check
	  switch (ecuState)
	  {
	  case DISCHARGE_STATE:

		  if ((highestTemp > DISCHARGE_TEMP_MAX_LIMIT && highestTemp < ERROR_THRESHOLD)  || lowestTemp < DISCHARGE_TEMP_MIN_LIMIT)
		  {
			  if (faultCounter == 4)
			  {
				  HAL_GPIO_WritePin(FAULT_PIN_PORT, FAULT_OUT_PIN, GPIO_PIN_RESET);

				  tx1_t.TS_ECU_TempThresholdFault = true;

				  tx1_t.TS_ECU_FaultOutState = GPIO_PIN_RESET;

				  faultCounter = 0;
			  }
			  else
			  {
				  faultCounter++;
			  }
		  }

		  break;
	  case CHARGING_STATE:

		  if ((highestTemp > CHARGE_TEMP_MAX_LIMIT && highestTemp < ERROR_THRESHOLD) || lowestTemp < CHARGE_TEMP_MIN_LIMIT)
		  {
			  if (faultCounter == 4)
			  {
				  HAL_GPIO_WritePin(FAULT_PIN_PORT, FAULT_OUT_PIN, GPIO_PIN_RESET);

				  tx1_t.TS_ECU_TempThresholdFault = true;

				  tx1_t.TS_ECU_FaultOutState = GPIO_PIN_RESET;

				  faultCounter = 0;
			  }
			  else
			  {
				  faultCounter++;
			  }
		  }

		  break;
	  }

	  // Sync One Response
	  if (syncOneFlag == true)
	  {
		  tx1_t.TS_ECU_AverageSegmentTemp = TS_ECU1_TX1_TS_ECU_AverageSegmentTemp_toS(averageSegmentTemp);
		  tx1_t.TS_ECU_MaxSegmentTemperature = TS_ECU1_TX1_TS_ECU_MaxSegmentTemperature_toS(highestTemp);
		  tx1_t.TS_ECU_MinSegmentTemperature = TS_ECU1_TX1_TS_ECU_MinSegmentTemperature_toS(lowestTemp);
		  tx1_t.TS_ECU_MaxTemperatureCellCount = highestTempCellCount;
		  tx1_t.TS_ECU_FaultInState = HAL_GPIO_ReadPin(FAULT_PIN_PORT, FAULT_IN_PIN);

		  if (ecuState == DISCHARGE_STATE)
		  {
			  tx1_t.TS_ECU_CurrentState = 0;
		  }
		  else
		  {
			  tx1_t.TS_ECU_CurrentState = 1;
		  }

		  if (SEGMENT_ID == (0x01U))
		  {
			  TS_ECU1_SendDiagnosticData(&tx1_t);
		  }
		  else if (SEGMENT_ID == (0x02U))
		  {

			  TS_ECU2_SendDiagnosticData(&tx1_t);
		  }
		  else if (SEGMENT_ID == (0x03U))
		  {

			  TS_ECU3_SendDiagnosticData(&tx1_t);
		  }
		  else if (SEGMENT_ID == (0x04U))
		  {

			  TS_ECU4_SendDiagnosticData(&tx1_t);
		  }
		  else
		  {
			  TS_ECU4_SendDiagnosticData(&tx1_t);
		  }

		  syncOneFlag = false;
	  }

	  // Sync Two Response
	  if (syncTwoFlag == true)
	  {
		  if (SEGMENT_ID == (0x01U))
		  {
			  if (HAL_GetTick() - syncTwoTime > SYNC_TIME_SHIFT_ECU1)
			  {
				  TS_ECU1_SendTemperatures(tempArray);

				  syncTwoFlag = false;
			  }
		  }
		  else if (SEGMENT_ID == (0x02U))
		  {
			  if (HAL_GetTick() - syncTwoTime > SYNC_TIME_SHIFT_ECU2)
			  {
				  TS_ECU2_SendTemperatures(tempArray);

				  syncTwoFlag = false;
			  }
		  }
		  else if (SEGMENT_ID == (0x03U))
		  {
			  if (HAL_GetTick() - syncTwoTime > SYNC_TIME_SHIFT_ECU3)
			  {
				  TS_ECU3_SendTemperatures(tempArray);

				  syncTwoFlag = false;
			  }
		  }
		  else if (SEGMENT_ID == (0x04U))
		  {
			  if (HAL_GetTick() - syncTwoTime > SYNC_TIME_SHIFT_ECU4)
			  {
				  TS_ECU4_SendTemperatures(tempArray);

				  syncTwoFlag = false;
			  }
		  }
		  else
		  {
			  if (HAL_GetTick() - syncTwoTime > SYNC_TIME_SHIFT_ECU5)
			  {
				  TS_ECU5_SendTemperatures(tempArray);

				  syncTwoFlag = false;
			  }
		  }
	  }

	  // Reset Counters
	  highestTemp = DISCHARGE_TEMP_MIN_LIMIT;
	  lowestTemp = DISCHARGE_TEMP_MAX_LIMIT;
	  highestTempCellCount = 0;
	  cellTempSum = 0;
	  counterCH1 = 0;
	  counterCH2 = 12;

	  // Reset Watchdog Counter
	  HAL_IWDG_Refresh(&hiwdg);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Reload = 300;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA4 PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static int getTempCH1(ADC_HandleTypeDef* hadc)
{
	uint32_t adcVal = 0;
	float voltage = 0.0;
	int temp = 0;
	ADC_ChannelConfTypeDef sConfig = {0};

	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;

	if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_ADC_Start(hadc);

	HAL_ADC_PollForConversion(hadc, 1);

	adcVal = HAL_ADC_GetValue(hadc);

	HAL_ADC_Stop(hadc);

	adcVal += 105;

	voltage = ((adcVal * 3.3) / 4096);

	temp = (int)(18212.8 - (47967.26*voltage) + (50732.41*pow(voltage, 2)) - (26799.56*pow(voltage, 3)) + (7056.825*pow(voltage, 4))
		      - (740.8519*pow(voltage, 5)));

	return temp;
}

static int getTempCH2(ADC_HandleTypeDef* hadc)
{
	uint32_t adcVal = 0;
	float voltage = 0.0;
	int temp = 0;
	ADC_ChannelConfTypeDef sConfig = {0};

	sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;

	if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_ADC_Start(hadc);

	HAL_ADC_PollForConversion(hadc, 1);

	adcVal = HAL_ADC_GetValue(hadc);

	HAL_ADC_Stop(hadc);

	adcVal += 105;

	voltage = ((adcVal * 3.3) / 4096);

	temp = (int)(18212.8 - (47967.26*voltage) + (50732.41*pow(voltage, 2)) - (26799.56*pow(voltage, 3)) + (7056.825*pow(voltage, 4))
	        - (740.8519*pow(voltage, 5)));

	return temp;
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
