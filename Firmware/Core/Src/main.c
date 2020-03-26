/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fatfs_sd.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BUF_LEN 1024
#define uart_rx_buffer_len 256
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
FATFS fs;
FIL fil;
FRESULT fresult;

UINT br, bw;

FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;

char timestamp[20]; //Char array used to store a timestamp from the RTC
char datalogName[32]; //The name of the datalog .csv file
char headers[1024]; //The headers to be applied to the .csv
char latitude[20]; //The latitude received from the GPS
char longitude[20]; //The longitude received from the GPS
char altitude[20]; //The latitude received from the GPS
float scaling_factors[20]; //The scaling factors to be applied to the various data, 0-15 for ADC, 16-19 for PWM
uint8_t CANresponse[50]; //The response received from the CAN
CAN_RxHeaderTypeDef RxCanHeader; //The header associated with the CAN data

uint32_t raw_ADC[16]; //The raw ADC data before any modification

uint32_t total_ADC[16]; //the ADC data totaled
float ADC_Array[16]; //The useful ADC data
uint16_t adc_buf[ADC_BUF_LEN]; //Circular buffer collecting ADC data
char uart_rx_buffer[uart_rx_buffer_len]; //Circular buffer collecting UART data
char gps_msg[uart_rx_buffer_len]; //The uart message received from the GPS
int initialized = 0; //Boolean to check if initialization is done, lets the interrupts run
int counter = 5; //Counter used to create a 10Hz function
float ADC_MAX = 3755; //The max value the ADC can read

//PWM sample arrays
uint32_t IC_Value1[4];
uint32_t IC_Value2[4];
uint32_t difference[4];
uint32_t frequency[4];
uint8_t is_first[4];

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) { //input capture callback. gets raised by an interrupt during a rising edge
	if (initialized) {
		uint32_t  channel = 0;
		uint32_t actual_channel=0;

		//set up the TIM_CHANNEL_X macro to make the HAL Happy
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
			channel = 0;
			actual_channel = TIM_CHANNEL_1;
		} else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2){
			channel = 1;
			actual_channel = TIM_CHANNEL_2;
		} else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3){
			channel = 2;
			actual_channel = TIM_CHANNEL_3;
		} else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4){
			channel = 3;
			actual_channel = TIM_CHANNEL_4;
		} else {
			Error_Handler(); //otherwise, raise an error
		}

		if (is_first[channel] == 0) { //if this is the first rising edge of a pair
			IC_Value1[channel] = HAL_TIM_ReadCapturedValue(htim, actual_channel); //store the current time
			is_first[channel] = 1; //flag that the next value will be the second rising edge
		} else if (is_first[channel] == 1) { //if second rising edge of a pair
			IC_Value2[channel] = HAL_TIM_ReadCapturedValue(htim, actual_channel); //store the current time
			if (IC_Value2[channel] > IC_Value1[channel]) { //if the second time is after the first time
				difference[channel] = IC_Value2[channel] - IC_Value1[channel]; //calculate period
			} else if (IC_Value2[channel] < IC_Value1[channel]) { //if the first time is after the second time (can be caused due to interrupts)
				difference[channel] = ((0xffff - IC_Value1[channel]) + IC_Value2[channel]) + 1; //take absolute value and calculate period
			} else {
				Error_Handler(); //otherwise, raise an error
			}
			//frequency = apb2 tim clock/difference/prescaler
			frequency[channel] = 72000000 / difference[channel] / 1000;
			is_first[channel] = 0; //start flag again
		}
	}
}

void float_to_string(char *buffer, float input) {
	sprintf(buffer, "%.3f", input);
}
//Function called by 50Hz timer interrupt
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (initialized) { //if (initialized && HAL_GPIO_ReadPin(Log_Switch_GPIO_Port, Log_Switch_Pin)) { //Check if initialization is done
		GPIO_PinState SD_Connected = !HAL_GPIO_ReadPin(SD_Detect_GPIO_Port,
				SD_Detect_Pin); //active low
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
		char sADC_Array[16][128]; //temp array used to store the scaled ADC data
		char sADC_msg[128] = { }; //Char array for the ADC data
		for (int i = 0; i < 16; i++) {
			ADC_Array[i] = raw_ADC[i] / ADC_MAX * 3.3 * scaling_factors[i]; //scale the array
			float_to_string(sADC_Array[i], ADC_Array[i]); //convert to string
			if (i == 0) { //Check if this is the first data value to ensure formating in the .csv file
				sprintf(sADC_msg, "%s", sADC_Array[i]); //append string to adc value string
			} else {
				sprintf(sADC_msg, "%s,%s", sADC_msg, sADC_Array[i]); //append string to adc value string
			}
		}

		//Apply the scaling factors to the PWM data
		uint32_t f1_scaled = frequency[0] * scaling_factors[16];
		uint32_t f2_scaled = frequency[1] * scaling_factors[17];
		uint32_t f3_scaled = frequency[2] * scaling_factors[18];
		uint32_t f4_scaled = frequency[3] * scaling_factors[19];

		char *msg[4096]; //msg used to write data to the .csv
		get_time(); //get an updated timestamp for the data
		sprintf(msg, "%s,%s,%s,%s,%s,%d,%d,%d,%d,%d\n", timestamp, latitude,
				longitude, altitude, sADC_msg, f1_scaled, f2_scaled, f3_scaled,
				f4_scaled, CANresponse[0]); //compile the data into the msg
		fresult = f_write(&fil, msg, strlen(msg), &bw); //write to the .csv
		counter--; //Decrement the counter, used to create a 10Hz function
		if (counter == 0) { //10hz function
			f_close(&fil); //close the sd card
			fresult = f_open(&fil, datalogName,
			FA_OPEN_APPEND | FA_READ | FA_WRITE); //re-open the sd card so new data can be written
			counter = 5; //reset the counter
		}
		if (SD_Connected) { //check if there is an SD card in the slot
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
		}
	}
}

//Function to get time from RTC
void get_time(void) {
	RTC_DateTypeDef gDate;
	RTC_TimeTypeDef gTime;
	/* Get the RTC current Time */
	HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);
	/* Get the RTC current Date */
	HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);
	/* Display time Format: hh:mm:ss */
	sprintf((char*) timestamp, "%02d-%02d-%02d_%02d-%02d-%2d", gTime.Hours,
			gTime.Minutes, gTime.Seconds, gDate.Date, gDate.Month,
			2000 + gDate.Year);
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
	for (int j = 0; j < 16; j++) {
		total_ADC[j] = 0;
	}
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM6_Init();
  MX_FATFS_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

	//Define the CAN Filter configuration
	CAN_FilterTypeDef filterConfig;

	filterConfig.FilterBank = 14;
	filterConfig.FilterActivation = ENABLE;
	filterConfig.FilterFIFOAssignment = 0;
	filterConfig.FilterMaskIdLow = 0x0000;
	filterConfig.FilterMaskIdHigh = 0x0000;
	filterConfig.FilterIdHigh = 0x0000;
	filterConfig.FilterIdLow = 0x0000;
	filterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	filterConfig.FilterMode = CAN_FILTERMODE_IDMASK;

	HAL_CAN_ConfigFilter(&hcan1, &filterConfig); //Assign the filter

	HAL_CAN_Start(&hcan1); //Activate CAN
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); //Activate CAN interrupts

	//Enable timer interrupts
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_buf, ADC_BUF_LEN);
	HAL_UART_Receive_DMA(&huart2, uart_rx_buffer, uart_rx_buffer_len);

	//ENable PWM Interrupt
	HAL_StatusTypeDef PWM_Response1 = HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);
	HAL_StatusTypeDef PWM_Response2 = HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
	HAL_StatusTypeDef PWM_Response3 = HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
	HAL_StatusTypeDef PWM_Response4 = HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);

	//Build the name of the datalog from the time
	get_time();
	sprintf(datalogName, "datalog_%s.csv", timestamp);

	//Mount SD Card
	fresult = f_mount(&fs, "", 0);

	//Get total and free space
	f_getfree("", &fre_clust, &pfs);
	total = (uint32_t) ((pfs->n_fatent - 2) * pfs->csize * 0.5);
	free_space = (uint32_t) (fre_clust * pfs->csize * 0.5);

	//Open and read the config file from the SD card
	fresult = f_open(&fil, "config.txt",
	FA_OPEN_EXISTING | FA_READ); //Try to open an existing config file
	if (fresult == FR_OK) { //if the file exists, read from it
		int scaling_Index = 0; //index for the scaling_factors array
		char fileRead[512]; //temp buffer for the input from the config file
		while (f_gets(fileRead, 512, &fil)) { //Loop while there is more to read from the config
			if (fileRead[0] != '/' && fileRead[1] != '/'
					&& fileRead[0] != '\n') { //Ignore comment lines in the config
				char *temp;
				temp = strtok(fileRead, ":"); //Split the string along :
				temp = strtok(NULL, ":"); //Ignore the first :
				if (temp[strlen(temp) - 1] == '\n') { //Check for and remove unwanted \n
					temp[strlen(temp) - 1] = '\0';
				}
				if (strlen(headers) == 0) { //Check for the first header being written
					sprintf(headers, "%s", temp); //Write the header to the headers array
				} else {
					sprintf(headers, "%s,%s", headers, temp); //Append the header to the headers array
				}
				temp = strtok(NULL, ":"); //Get the final token
				if (temp != NULL) { //Check for a scaling factor
					scaling_factors[scaling_Index] = atof(temp); //Add the scaling factor to the list of scaling factor
					scaling_Index++;
				}
			}
		}
		//Write the headers into the file
		fresult = f_open(&fil, datalogName,
		FA_OPEN_APPEND | FA_READ | FA_WRITE);
		sprintf(headers, "%s\n", headers);
		fresult = f_write(&fil, headers, strlen(headers), &bw);
		f_close(&fil);
	} else { //Config file not found
		fresult = f_open(&fil, datalogName,
		FA_OPEN_APPEND | FA_READ | FA_WRITE);
		fresult =
				f_write(&fil,
						"config.txt not found it has been rebuilt. The data here has not been scaled and falls under default headers,\n",
						109, &bw); //Inform the user in the datalog that the config wasn't found
		fresult =
				f_write(&fil,
						"Timestamp,Latitude,Longitude,Altitude,A0,A1,A2,A3,A4,A5,A6,A7,A8,A9,A10,A11,A12,A13,A14,A15,PWM0,PWM1,PWM2,PWM3,\n",
						113, &bw); //Apply basic headers to the datalog

		f_close(&fil);
		fresult = f_open(&fil, "config.txt", //Reconstruct the config file
		FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
		fresult =
				f_write(&fil,
						"//The Order of the headers must be maintained (GPS,Analog,PWM,CAN)\n//Lines with " "//" " "
								"at the beginning will be ignored\n//Do NOT include any commas in the headers\n\n//GPS Headers (Input:Header)\nTime:Timestamp\nLat:Latitude\nLong:Longitude\nAlt:Altitude\n\n//Analog Inputs (Input:Header:Scaling Factor)\nA0:N/A:1\nA1:N/A:1\nA2:N/A:1\nA3:N/A:1\nA4:N/A:1\nA5:N/A:1\nA6:N/A:1\nA7:N/A:1\nA8:N/A:1\nA9:N/A:1\nA10:N/A:1\nA11:N/A:1\n"
								"A12:N/A:1\nA13:N/A:1\nA14:N/A:1\nA15:N/A:1\n\n//PWM Inputs (Input:Header:Scaling Factor)\nPWM0:N/A:1\nPWM1:N/A:1\n"
								"PWM2:N/A:1\nPWM3:N/A:1", 529, &bw);
		f_close(&fil);

		for (int i = 0; i < 20; i++) { //set the scaling factors to 1 to retain raw data
			scaling_factors[i] = 1;
		}
	}

	//Assign values to lat, long, and alt
	sprintf(latitude, "0.00");
	sprintf(longitude, "0.00");
	sprintf(altitude, "0.00");

	//Open the datalog file in preparation for new data to be written
	fresult = f_open(&fil, datalogName,
	FA_OPEN_APPEND | FA_READ | FA_WRITE);

	//Initialization complete, allow interrupts to progress
	initialized = 1;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
	HAL_TIM_Base_Stop_IT(&htim6);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 16;
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
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 10;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 11;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 12;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 13;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 14;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 15;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 16;
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
  hcan1.Init.Prescaler = 9;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
	if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) != 0x32F2) {
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date 
  */
  sTime.Hours = 0x16;
  sTime.Minutes = 0x48;
  sTime.Seconds = 0x30;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_THURSDAY;
  sDate.Month = RTC_MONTH_MARCH;
  sDate.Date = 0x5;
  sDate.Year = 0x20;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
		HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2);
	}
  /* USER CODE END RTC_Init 2 */

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
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffff;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xffff;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 224;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 7199;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  huart2.Init.BaudRate = 460800;
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
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED3_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, SD_CS_Pin|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPS_Ready_GPIO_Port, GPS_Ready_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : Log_Switch_Pin */
  GPIO_InitStruct.Pin = Log_Switch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Log_Switch_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED3_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED3_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SD_CS_Pin PG7 GPS_Ready_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin|GPIO_PIN_7|GPS_Ready_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : GPS_nReset_Pin */
  GPIO_InitStruct.Pin = GPS_nReset_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPS_nReset_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_Detect_Pin */
  GPIO_InitStruct.Pin = SD_Detect_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SD_Detect_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
}

// Called when buffer is completely filled
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	//averaging code

	for (int i = 0; i < ADC_BUF_LEN; i += 16) { //Increment through the dma buffer
		for (int j = 0; j < 16; j++) { //for each adc channel
			total_ADC[j] += adc_buf[i + j]; //add to an individual sum
		}
	}
	for (int j = 0; j < 16; j++) {//for each adc channel
		raw_ADC[j] = total_ADC[j] / (ADC_BUF_LEN / 16); //divide sum by number of samples to get an averaged value
		total_ADC[j] = 0; //reset sum value
	}
}

//called when UART finished receiving a msg
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	char temp_msg[uart_rx_buffer_len];
	sprintf(temp_msg, strtok(uart_rx_buffer, "#"));
	if (memcmp("BESTPOSA", temp_msg,8) == 0){
		sprintf(gps_msg, temp_msg);
		char *token = strtok(gps_msg, ","); //split the message on ',' char
		// loop through the string to extract all other tokens
		int index = 0;
		while (token != NULL) {
			token = strtok(NULL, ","); //split again
			if (index == 10)
				strncpy(latitude, token, 20); //extract latitude value from 10th index
			if (index == 11)
				strncpy(longitude, token, 20); //extract longitude value from 11th index
			if (index == 12) {
				strncpy(altitude, token, 20); //extract altitude value from 12th index
				break;
			}
			index++; //increment
		}
	}
	uart_rx_buffer[0]='\0';
	HAL_UART_Receive_DMA(&huart2, uart_rx_buffer, uart_rx_buffer_len);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
