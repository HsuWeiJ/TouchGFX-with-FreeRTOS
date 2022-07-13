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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "app_touchgfx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Components/ili9341/ili9341.h"
#include "SHT31.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <SD_SPI_FATFS.h>
#include "File_Handling_RTOS.h"
/**********Flash***********/
#include "FLASH_SECTOR_F4.h"
/**********CPU USAGE***********/
#include "cpu_utils.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define REFRESH_COUNT           ((uint32_t)1386)   /* SDRAM refresh counter */
#define SDRAM_TIMEOUT           ((uint32_t)0xFFFF)

/**
  * @brief  FMC SDRAM Mode definition register defines
  */
#define SDRAM_MODEREG_BURST_LENGTH_1             ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_LENGTH_2             ((uint16_t)0x0001)
#define SDRAM_MODEREG_BURST_LENGTH_4             ((uint16_t)0x0002)
#define SDRAM_MODEREG_BURST_LENGTH_8             ((uint16_t)0x0004)
#define SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL      ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_TYPE_INTERLEAVED     ((uint16_t)0x0008)
#define SDRAM_MODEREG_CAS_LATENCY_2              ((uint16_t)0x0020)
#define SDRAM_MODEREG_CAS_LATENCY_3              ((uint16_t)0x0030)
#define SDRAM_MODEREG_OPERATING_MODE_STANDARD    ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_PROGRAMMED ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_SINGLE     ((uint16_t)0x0200)

#define I2C3_TIMEOUT_MAX                    0x3000 /*<! The value of the maximal timeout for I2C waiting loops */
#define SPI5_TIMEOUT_MAX                    0x1000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;

CRC_HandleTypeDef hcrc;

DMA2D_HandleTypeDef hdma2d;

I2C_HandleTypeDef hi2c3;
DMA_HandleTypeDef hdma_i2c3_rx;
DMA_HandleTypeDef hdma_i2c3_tx;

LTDC_HandleTypeDef hltdc;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi5;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;

SDRAM_HandleTypeDef hsdram1;

/* Definitions for GUI_Task */
osThreadId_t GUI_TaskHandle;
const osThreadAttr_t GUI_Task_attributes = {
  .name = "GUI_Task",
  .stack_size = 8192 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SHT */
osThreadId_t SHTHandle;
const osThreadAttr_t SHT_attributes = {
  .name = "SHT",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for ADC */
osThreadId_t ADCHandle;
const osThreadAttr_t ADC_attributes = {
  .name = "ADC",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for PWM */
osThreadId_t PWMHandle;
const osThreadAttr_t PWM_attributes = {
  .name = "PWM",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Log */
osThreadId_t LogHandle;
const osThreadAttr_t Log_attributes = {
  .name = "Log",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for SD */
osThreadId_t SDHandle;
const osThreadAttr_t SD_attributes = {
  .name = "SD",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for Duty_Queue */
osMessageQueueId_t Duty_QueueHandle;
const osMessageQueueAttr_t Duty_Queue_attributes = {
  .name = "Duty_Queue"
};
/* Definitions for RTC_Timer */
osTimerId_t RTC_TimerHandle;
const osTimerAttr_t RTC_Timer_attributes = {
  .name = "RTC_Timer"
};
/* Definitions for RFID_Timer */
osTimerId_t RFID_TimerHandle;
const osTimerAttr_t RFID_Timer_attributes = {
  .name = "RFID_Timer"
};
/* Definitions for Voltage_SEM */
osMutexId_t Voltage_SEMHandle;
const osMutexAttr_t Voltage_SEM_attributes = {
  .name = "Voltage_SEM"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_I2C3_Init(void);
static void MX_SPI5_Init(void);
static void MX_FMC_Init(void);
static void MX_LTDC_Init(void);
static void MX_DMA2D_Init(void);
static void MX_RTC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_DMA_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_SPI1_Init(void);
void TouchGFX_Task(void *argument);
void SHT_Task(void *argument);
void ADC_Task(void *argument);
void PWM_Task(void *argument);
void Log_Task(void *argument);
void SD_Task(void *argument);
void RTC_Timer_Callback(void *argument);
void RFID_Timer_Callback(void *argument);

/* USER CODE BEGIN PFP */
static void BSP_SDRAM_Initialization_Sequence(SDRAM_HandleTypeDef *hsdram, FMC_SDRAM_CommandTypeDef *Command);



static uint8_t            I2C3_ReadData(uint8_t Addr, uint8_t Reg);
static void               I2C3_WriteData(uint8_t Addr, uint8_t Reg, uint8_t Value);
static uint8_t            I2C3_ReadBuffer(uint8_t Addr, uint8_t Reg, uint8_t *pBuffer, uint16_t Length);

/* SPIx bus function */
static void               SPI5_Write(uint16_t Value);
static uint32_t           SPI5_Read(uint8_t ReadSize);
static void               SPI5_Error(void);

/* Link function for LCD peripheral */
void                      LCD_IO_Init(void);
void                      LCD_IO_WriteData(uint16_t RegValue);
void                      LCD_IO_WriteReg(uint8_t Reg);
uint32_t                  LCD_IO_ReadData(uint16_t RegValue, uint8_t ReadSize);
void                      LCD_Delay(uint32_t delay);

/* IOExpander IO functions */
void                      IOE_Init(void);
void                      IOE_ITConfig(void);
void                      IOE_Delay(uint32_t Delay);
void                      IOE_Write(uint8_t Addr, uint8_t Reg, uint8_t Value);
uint8_t                   IOE_Read(uint8_t Addr, uint8_t Reg);
uint16_t                  IOE_ReadMultiple(uint8_t Addr, uint8_t Reg, uint8_t *pBuffer, uint16_t Length);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static LCD_DrvTypeDef* LcdDrv;

uint32_t I2c3Timeout = I2C3_TIMEOUT_MAX; /*<! Value of Timeout when I2C communication fails */  
uint32_t Spi5Timeout = SPI5_TIMEOUT_MAX; /*<! Value of Timeout when SPI communication fails */



/**************** QUEUE HANDLERS ***********************/
xQueueHandle DutyQueue;
xQueueHandle RFID_TO_GUIQueue;
xQueueHandle RFID_ResetQueue;
xQueueHandle RFID_TO_LogQueue;
xQueueHandle CPU_Usage_Queue;
xQueueHandle TEM_TO_GUI_Queue;
xQueueHandle ADC_TO_GUI_Queue;
/*************Semaphore*********************/

SemaphoreHandle_t SHT31_SEM;
SemaphoreHandle_t I2C_SEM;
SemaphoreHandle_t Voltage_SEM;
/**************** EVENTGROUP HANDLERS ******************/
EventGroupHandle_t Log_Out_Handler;
/***********Record the screen before log out***************/
EventBits_t Screen_Switch_Event = 1;
/*******************CPU Usage***************************/
xTaskHandle    xIdleHandle = NULL;
__IO uint32_t  osCPU_Usage = 0;
uint32_t       osCPU_IdleStartTime = 0;
uint32_t       osCPU_IdleSpentTime = 0;
uint32_t       osCPU_TotalIdleTime = 0;
uint16_t 	   Cpu_Usage = 0 ;
/********** Flag related (send to Model.c to display the change) *********/
int SHT_Flag = 0;
int ADC_Flag = 0;
int RFID_Reset_Flag = 0;
int RTC_Flag = 0;
int RFID_confirmed_Flag = 0;
/********** RTC related *********/
RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;

char date[50];
char time[50];

typedef struct{
	RTC_TimeTypeDef Time;
	RTC_DateTypeDef Date;
} RTC_Data;
RTC_Data RTC_Data_;
/********** SHT related *********/
float temperature = 0;
float humidity = 0;
sht3x_handle_t SHT31 = {
	.i2c_handle = &hi2c3,   // The i2c port we are using
	.device_address = SHT3X_I2C_DEVICE_ADDRESS_ADDR_PIN_LOW  // Use the low address but i don't look up the datasheet
};
//for Queue (send information from SHT_Task to SD_Task)
typedef struct{
	float temperature;
	float humidity;
} SHT_Data;

/********** ADC PWM related *********/
uint16_t ADC_VAL;
//FLASH
uint32_t RX_buf [10];
char translation[32];
char string[50];

//PWM
int pwm_cnt=0;
int pwm_output=90;
uint16_t pwm_data[6]={90,80,70,60,50,40};
bool TOCHANGE_PWMCNT=false;
//extern int duty;
uint8_t duty=10;

/********** The wanted RFID ID *********/
char *RFID_confirmed = "1B002694F75E";

/********** The RFID ID card buffer to store the RFID information *********/
char RFID[13] = "";

typedef struct{
	char confirm;
	uint8_t ID[13];
} RFID_Data;
RFID_Data RFID_Data_;
/*************switch screen*************/
char switch_data = 0;
/**********Initialize SD*********/
FATFS fs;  // file system
FIL fil; // File
FILINFO fno;
FRESULT fresult;  // result
UINT br, bw;  // File read/write count

/**** capacity related *****/
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;

/***********SD Read***************/
#define BUFFER_SIZE 200
char buffer_sd[BUFFER_SIZE];  // to store strings..
uint32_t sd_f_size;
void clear_buffer (void)
{
	for (int i=0; i<BUFFER_SIZE; i++) buffer_sd[i] = '\0';
}

void send_uart (char *string)
{
	uint16_t len = strlen (string);
	HAL_UART_Transmit(&huart1, (uint8_t *) string, len, HAL_MAX_DELAY);  // transmit in blocking mode
}
/***********Flash***************/
float test_flash = 0;
float Flash_buf[4];

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
  __HAL_RCC_I2C3_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CRC_Init();
  MX_I2C3_Init();
  MX_SPI5_Init();
  MX_FMC_Init();
  MX_LTDC_Init();
  MX_DMA2D_Init();
  MX_RTC_Init();
  MX_USART1_UART_Init();
  MX_FATFS_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_DMA_Init();
  MX_USART6_UART_Init();
  MX_SPI1_Init();
  MX_TouchGFX_Init();
  /* USER CODE BEGIN 2 */
  /*********** Initialize the SHT sensor ***************/
  sht3x_init(&SHT31);

  /*********** Receive the information of RFID card : the id is 12 byte long ***************/
  /*********** and I make the RFID char as 13 byte long with end of '\0' to be a string ***************/
  /*********** The Uart DMA callback function is void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)***************/
 // HAL_UART_Receive_DMA(&huart5, (uint8_t *) RFID, 12);
  HAL_UART_Receive_DMA(&huart6, (uint8_t *) RFID_Data_.ID, 12);

  /**********Start Timer***********/
    TIM2->CCR1=90;
    HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);

	/**********Create Semaphore ***********/
//	SHT31_SEM = xSemaphoreCreateMutex();
//	Voltage_SEM = xSemaphoreCreateMutex();
	I2C_SEM = xSemaphoreCreateBinary();
	//xSemaphoreGive(I2C_SEM);

	/**********Create EventGroup ***********/
	Log_Out_Handler = xEventGroupCreate();

  /********** Initialize SDCARD *********/
	Mount_SD("/");
	Format_SD();
	Create_File("record.TXT");
	Unmount_SD("/");
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of Voltage_SEM */
  Voltage_SEMHandle = osMutexNew(&Voltage_SEM_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of RTC_Timer */
  RTC_TimerHandle = osTimerNew(RTC_Timer_Callback, osTimerPeriodic, NULL, &RTC_Timer_attributes);

  /* creation of RFID_Timer */
  RFID_TimerHandle = osTimerNew(RFID_Timer_Callback, osTimerOnce, NULL, &RFID_Timer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */

  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of Duty_Queue */
  Duty_QueueHandle = osMessageQueueNew (1, sizeof(uint8_t), &Duty_Queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /************************* Create Integer Queue ****************************/
  	DutyQueue = xQueueCreate(1, sizeof (uint8_t));
  	RFID_TO_GUIQueue = xQueueCreate(1, sizeof (RFID_Data_));
  	RFID_TO_LogQueue = xQueueCreate(1, sizeof (char));
  	RFID_ResetQueue = xQueueCreate(1, sizeof (char));
  	CPU_Usage_Queue = xQueueCreate(1, sizeof (uint16_t));
  	TEM_TO_GUI_Queue = xQueueCreate(1, sizeof (SHT_Data));
  	ADC_TO_GUI_Queue = xQueueCreate(1, sizeof (uint16_t));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of GUI_Task */
  GUI_TaskHandle = osThreadNew(TouchGFX_Task, NULL, &GUI_Task_attributes);

  /* creation of SHT */
  SHTHandle = osThreadNew(SHT_Task, NULL, &SHT_attributes);

  /* creation of ADC */
  ADCHandle = osThreadNew(ADC_Task, NULL, &ADC_attributes);

  /* creation of PWM */
  PWMHandle = osThreadNew(PWM_Task, NULL, &PWM_attributes);

  /* creation of Log */
  LogHandle = osThreadNew(Log_Task, NULL, &Log_attributes);

  /* creation of SD */
  SDHandle = osThreadNew(SD_Task, NULL, &SD_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* This software timer is used to update the RTC, so set the trigger time as 1 sec */
  osTimerStart(RTC_TimerHandle, 1000);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  sDate.WeekDay=7;
  HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
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
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
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
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief DMA2D Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMA2D_Init(void)
{

  /* USER CODE BEGIN DMA2D_Init 0 */

  /* USER CODE END DMA2D_Init 0 */

  /* USER CODE BEGIN DMA2D_Init 1 */

  /* USER CODE END DMA2D_Init 1 */
  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_RGB565;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_RGB565;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DMA2D_Init 2 */

  /* USER CODE END DMA2D_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_DISABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
static void MX_LTDC_Init(void)
{

  /* USER CODE BEGIN LTDC_Init 0 */

  /* USER CODE END LTDC_Init 0 */

  LTDC_LayerCfgTypeDef pLayerCfg = {0};

  /* USER CODE BEGIN LTDC_Init 1 */

  /* USER CODE END LTDC_Init 1 */
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 9;
  hltdc.Init.VerticalSync = 1;
  hltdc.Init.AccumulatedHBP = 29;
  hltdc.Init.AccumulatedVBP = 3;
  hltdc.Init.AccumulatedActiveW = 269;
  hltdc.Init.AccumulatedActiveH = 323;
  hltdc.Init.TotalWidth = 279;
  hltdc.Init.TotalHeigh = 327;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 240;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 320;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
  pLayerCfg.Alpha = 255;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg.FBStartAdress = 0;
  pLayerCfg.ImageWidth = 240;
  pLayerCfg.ImageHeight = 320;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LTDC_Init 2 */
    /*Select the device */
  LcdDrv = &ili9341_drv;
  /* LCD Init */
  LcdDrv->Init();
  
  LcdDrv->DisplayOff();
  /* USER CODE END LTDC_Init 2 */

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
  /* USER CODE BEGIN RTC_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */
  
  

  /* USER CODE END SPI5_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 840-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart6.Init.BaudRate = 9600;
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
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_SDRAM_TimingTypeDef SdramTiming = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK2;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_3;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_DISABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_1;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 2;
  SdramTiming.ExitSelfRefreshDelay = 7;
  SdramTiming.SelfRefreshTime = 4;
  SdramTiming.RowCycleDelay = 7;
  SdramTiming.WriteRecoveryTime = 3;
  SdramTiming.RPDelay = 2;
  SdramTiming.RCDDelay = 2;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */
  
  FMC_SDRAM_CommandTypeDef command;
  
  /* Program the SDRAM external device */
  BSP_SDRAM_Initialization_Sequence(&hsdram1, &command);
  /* USER CODE END FMC_Init 2 */
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**
  * @brief  Perform the SDRAM external memory initialization sequence
  * @param  hsdram: SDRAM handle
  * @param  Command: Pointer to SDRAM command structure
  * @retval None
  */
static void BSP_SDRAM_Initialization_Sequence(SDRAM_HandleTypeDef *hsdram, FMC_SDRAM_CommandTypeDef *Command)
{
 __IO uint32_t tmpmrd =0;
  
  /* Step 1:  Configure a clock configuration enable command */
  Command->CommandMode             = FMC_SDRAM_CMD_CLK_ENABLE;
  Command->CommandTarget           = FMC_SDRAM_CMD_TARGET_BANK2;
  Command->AutoRefreshNumber       = 1;
  Command->ModeRegisterDefinition  = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand(hsdram, Command, SDRAM_TIMEOUT);

  /* Step 2: Insert 100 us minimum delay */ 
  /* Inserted delay is equal to 1 ms due to systick time base unit (ms) */
  HAL_Delay(1);

  /* Step 3: Configure a PALL (precharge all) command */ 
  Command->CommandMode             = FMC_SDRAM_CMD_PALL;
  Command->CommandTarget           = FMC_SDRAM_CMD_TARGET_BANK2;
  Command->AutoRefreshNumber       = 1;
  Command->ModeRegisterDefinition  = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand(hsdram, Command, SDRAM_TIMEOUT);  
  
  /* Step 4: Configure an Auto Refresh command */ 
  Command->CommandMode             = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
  Command->CommandTarget           = FMC_SDRAM_CMD_TARGET_BANK2;
  Command->AutoRefreshNumber       = 4;
  Command->ModeRegisterDefinition  = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand(hsdram, Command, SDRAM_TIMEOUT);
  
  /* Step 5: Program the external memory mode register */
  tmpmrd = (uint32_t)SDRAM_MODEREG_BURST_LENGTH_1          |
                     SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL   |
                     SDRAM_MODEREG_CAS_LATENCY_3           |
                     SDRAM_MODEREG_OPERATING_MODE_STANDARD |
                     SDRAM_MODEREG_WRITEBURST_MODE_SINGLE;
  
  Command->CommandMode             = FMC_SDRAM_CMD_LOAD_MODE;
  Command->CommandTarget           = FMC_SDRAM_CMD_TARGET_BANK2;
  Command->AutoRefreshNumber       = 1;
  Command->ModeRegisterDefinition  = tmpmrd;

  /* Send the command */
  HAL_SDRAM_SendCommand(hsdram, Command, SDRAM_TIMEOUT);
  
  /* Step 6: Set the refresh rate counter */
  /* Set the device refresh rate */
  HAL_SDRAM_ProgramRefreshRate(hsdram, REFRESH_COUNT); 
}

/**
  * @brief  IOE Low Level Initialization.
  */
void IOE_Init(void) 
{
  //Dummy function called when initializing to stmpe811 to setup the i2c.
  //This is done with cubmx and is therfore not done here.
}

/**
  * @brief  IOE Low Level Interrupt configuration.
  */
void IOE_ITConfig(void)
{
  //Dummy function called when initializing to stmpe811 to setup interupt for the i2c.
  //The interupt is not used in our case, therefore nothing is done here.
}

/**
  * @brief  IOE Writes single data operation.
  * @param  Addr: I2C Address
  * @param  Reg: Reg Address 
  * @param  Value: Data to be written
  */
void IOE_Write(uint8_t Addr, uint8_t Reg, uint8_t Value)
{
  I2C3_WriteData(Addr, Reg, Value);
}

/**
  * @brief  IOE Reads single data.
  * @param  Addr: I2C Address
  * @param  Reg: Reg Address 
  * @retval The read data
  */
uint8_t IOE_Read(uint8_t Addr, uint8_t Reg)
{
  return I2C3_ReadData(Addr, Reg);
}

/**
  * @brief  IOE Reads multiple data.
  * @param  Addr: I2C Address
  * @param  Reg: Reg Address 
  * @param  pBuffer: pointer to data buffer
  * @param  Length: length of the data
  * @retval 0 if no problems to read multiple data
  */
uint16_t IOE_ReadMultiple(uint8_t Addr, uint8_t Reg, uint8_t *pBuffer, uint16_t Length)
{
 return I2C3_ReadBuffer(Addr, Reg, pBuffer, Length);
}

/**
  * @brief  IOE Delay.
  * @param  Delay in ms
  */
void IOE_Delay(uint32_t Delay)
{
  HAL_Delay(Delay);
}

/**
  * @brief  Writes a value in a register of the device through BUS.
  * @param  Addr: Device address on BUS Bus.  
  * @param  Reg: The target register address to write
  * @param  Value: The target register value to be written 
  */
static void I2C3_WriteData(uint8_t Addr, uint8_t Reg, uint8_t Value)
{
  HAL_StatusTypeDef status = HAL_OK;
  
  status = HAL_I2C_Mem_Write(&hi2c3, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, &Value, 1, I2c3Timeout); 
  
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Re-Initialize the BUS */
    //I2Cx_Error();
  }        
}

/**
  * @brief  Reads a register of the device through BUS.
  * @param  Addr: Device address on BUS Bus.  
  * @param  Reg: The target register address to write
  * @retval Data read at register address
  */
static uint8_t I2C3_ReadData(uint8_t Addr, uint8_t Reg)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint8_t value = 0;
  
  status = HAL_I2C_Mem_Read(&hi2c3, Addr, Reg, I2C_MEMADD_SIZE_8BIT, &value, 1, I2c3Timeout);
 
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Re-Initialize the BUS */
    //I2Cx_Error();
  
  }
  return value;
}

/**
  * @brief  Reads multiple data on the BUS.
  * @param  Addr: I2C Address
  * @param  Reg: Reg Address 
  * @param  pBuffer: pointer to read data buffer
  * @param  Length: length of the data
  * @retval 0 if no problems to read multiple data
  */
static uint8_t I2C3_ReadBuffer(uint8_t Addr, uint8_t Reg, uint8_t *pBuffer, uint16_t Length)
{
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Mem_Read(&hi2c3, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, pBuffer, Length, I2c3Timeout);
  
  /* Check the communication status */
  if(status == HAL_OK)
  {
    return 0;
  }
  else
  {
    /* Re-Initialize the BUS */
    //I2Cx_Error();

    return 1;
  }
}

/**
  * @brief  Reads 4 bytes from device.
  * @param  ReadSize: Number of bytes to read (max 4 bytes)
  * @retval Value read on the SPI
  */
static uint32_t SPI5_Read(uint8_t ReadSize)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint32_t readvalue;
  
  status = HAL_SPI_Receive(&hspi5, (uint8_t*) &readvalue, ReadSize, Spi5Timeout);
  
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Re-Initialize the BUS */
    SPI5_Error();
  }
  
  return readvalue;
}

/**
  * @brief  Writes a byte to device.
  * @param  Value: value to be written
  */
static void SPI5_Write(uint16_t Value)
{
  HAL_StatusTypeDef status = HAL_OK;
  
  status = HAL_SPI_Transmit(&hspi5, (uint8_t*) &Value, 1, Spi5Timeout);
  
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Re-Initialize the BUS */
    SPI5_Error();
  }
}

/**
  * @brief  SPI5 error treatment function.
  */
static void SPI5_Error(void)
{
  /* De-initialize the SPI communication BUS */
  //HAL_SPI_DeInit(&SpiHandle);
  
  /* Re- Initialize the SPI communication BUS */
  //SPIx_Init();
}

void LCD_IO_Init(void)
{
  /* Set or Reset the control line */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
}

/**
  * @brief  Writes register value.
  */
void LCD_IO_WriteData(uint16_t RegValue) 
{
  /* Set WRX to send data */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
  
  /* Reset LCD control line(/CS) and Send data */  
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
  SPI5_Write(RegValue);
  
  /* Deselect: Chip Select high */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
}

/**
  * @brief  Writes register address.
  */
void LCD_IO_WriteReg(uint8_t Reg) 
{
  /* Reset WRX to send command */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
  
  /* Reset LCD control line(/CS) and Send command */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
  SPI5_Write(Reg);
  
  /* Deselect: Chip Select high */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
}

/**
  * @brief  Reads register value.
  * @param  RegValue Address of the register to read
  * @param  ReadSize Number of bytes to read
  * @retval Content of the register value
  */
uint32_t LCD_IO_ReadData(uint16_t RegValue, uint8_t ReadSize) 
{
  uint32_t readvalue = 0;

  /* Select: Chip Select low */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);

  /* Reset WRX to send command */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
  
  SPI5_Write(RegValue);
  
  readvalue = SPI5_Read(ReadSize);

  /* Set WRX to send data */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);

  /* Deselect: Chip Select high */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
  
  return readvalue;
}

/**
  * @brief  Wait for loop in ms.
  * @param  Delay in ms.
  */
void LCD_Delay(uint32_t Delay)
{
  HAL_Delay(Delay);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	static uint8_t RFID_Confirm_Data;
	if (huart->Instance == USART6)
	{
		/* Let TouchGFX Model make change of display */
		//RFID_Flag = 1;

		if(strcmp(RFID_confirmed, RFID_Data_.ID) == 0){
			RFID_Data_.confirm = 1;
			BaseType_t pxHigherPriorityTaskWoken = 0;
			BaseType_t pxHigherPriorityTaskWoken_log = 0;
			xQueueOverwriteFromISR(RFID_TO_GUIQueue, &RFID_Data_, pxHigherPriorityTaskWoken);
			xQueueOverwriteFromISR(RFID_TO_LogQueue, &RFID_Data_.confirm, pxHigherPriorityTaskWoken_log);

			portYIELD_FROM_ISR(pxHigherPriorityTaskWoken_log | pxHigherPriorityTaskWoken);
		}
		else {
			RFID_Data_.confirm = 0;
			BaseType_t pxHigherPriorityTaskWoken = 0;
			BaseType_t pxHigherPriorityTaskWoken_log = 0;
			xQueueOverwriteFromISR(RFID_TO_GUIQueue, &RFID_Data_, pxHigherPriorityTaskWoken);
			xQueueOverwriteFromISR(RFID_TO_LogQueue, &RFID_Data_.confirm, pxHigherPriorityTaskWoken_log);
			portYIELD_FROM_ISR(pxHigherPriorityTaskWoken_log | pxHigherPriorityTaskWoken);
		}

		//portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);

//		/* if the incoming ID of RFID card is the wanted one, notify the RFID task to turn on the pwm to control motor */
//		if(strcmp(RFID_confirmed, RFID) == 0){
//			RFID_confirmed_Flag = 1;
//			BaseType_t pxHigherPriorityTaskWoken = 0;
//			xTaskNotifyFromISR((osThreadId_t) RFID_TaskHandle, 1, (eNotifyAction)eSetBits, &pxHigherPriorityTaskWoken);
//			portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
//		}
//		else {
//			RFID_confirmed_Flag = 0;
//			BaseType_t pxHigherPriorityTaskWoken = 0;
//			xTaskNotifyFromISR((osThreadId_t) RFID_TaskHandle, 2, (eNotifyAction)eSetBits, &pxHigherPriorityTaskWoken);
//			portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
//		}

		/* start the DMA again */
 		HAL_UART_Receive_DMA(&huart6, (uint8_t *) RFID_Data_.ID, 12);

		/********** queue send to sd (haven't checked the correctness) **************/
//		BaseType_t pxHigherPriorityTaskWoken = 0;
//		xQueueOverwriteFromISR(RFID_Queue_Handler, &RFID_Data_, pxHigherPriorityTaskWoken);
//		portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
	}
}
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(I2C_SEM, &xHigherPriorityTaskWoken);  // ISR SAFE VERSION
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(I2C_SEM, &xHigherPriorityTaskWoken);  // ISR SAFE VERSION
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_TouchGFX_Task */
/**
  * @brief  Function implementing the GUI_Task thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_TouchGFX_Task */
__weak void TouchGFX_Task(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_SHT_Task */
/**
* @brief Function implementing the SHT thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SHT_Task */
void SHT_Task(void *argument)
{
  /* USER CODE BEGIN SHT_Task */
  /* Infinite loop */
//	osEvent event;
//	event=osSignalWait(0x01,portMAX_DELAY);
	SHT_Data SHT_Data_;
	osThreadSuspend(SHTHandle);
  for(;;)
  {
	  /* Start sensing the temperature again */
	  	sht3x_read_temperature_and_humidity(&SHT31, &temperature, &humidity);
	  	SHT_Data_.temperature=temperature;
	  	SHT_Data_.humidity=humidity;
	  	xQueueOverwrite(TEM_TO_GUI_Queue,&SHT_Data_);
	  	/* Uart port to computer for debug */
	  //	sprintf (buffer, "temperature : %d,  humidity : %d\n", (int) temperature, (int) humidity);
	  //	HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);

	  	/* Let TouchGFX Model make change of display */
	  	//SHT_Flag = 1;

	  	/********** queue send information to SD_Task **************/
//	  	SHT_Data_ptr->humidity = humidity;
//	  	SHT_Data_ptr->temperature = temperature;
//	  	xQueueOverwrite(SHT_Queue_Handler, &SHT_Data_ptr);

	  	osDelay(300);
  }
  /* USER CODE END SHT_Task */
}

/* USER CODE BEGIN Header_ADC_Task */
/**
* @brief Function implementing the ADC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ADC_Task */
void ADC_Task(void *argument)
{
  /* USER CODE BEGIN ADC_Task */
	osThreadSuspend(ADCHandle);
  /* Infinite loop */

  for(;;)
  {
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 10);
	if (osMutexAcquire(Voltage_SEMHandle, 100) != osOK)
	{
		//HAL_UART_Transmit(&huart2, (uint8_t *) "Unable to acquire Voltage_semaphore\n", 36, 100);
		osDelay(100);
	}
	else
	{
		ADC_VAL = HAL_ADC_GetValue(&hadc1) *3300 / 4096 ;
		//ADC_Flag = 1;
		osMutexRelease(Voltage_SEMHandle);
		xQueueOverwrite(ADC_TO_GUI_Queue,&ADC_VAL);
		HAL_ADC_Stop(&hadc1);
		//xEventGroupSetBits(TFT_Display_Handler, VOL_DUTY_EVENT);
		osDelay(200);
	}
  }
  /* USER CODE END ADC_Task */
}

/* USER CODE BEGIN Header_PWM_Task */
/**
* @brief Function implementing the PWM thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PWM_Task */
void PWM_Task(void *argument)
{
  /* USER CODE BEGIN PWM_Task */
	int received;
	uint8_t Duty_buf=0;
	TIM2->CCR1=100-10;
	osThreadSuspend(PWMHandle);
  /* Infinite loop */
  for(;;)
  {
//	if (xQueueReceive(PwmQueue, &received, portMAX_DELAY) == pdTRUE)
//	{
//		pwm_output=90-received*10;
//		TIM3->CCR1=pwm_output;
//		xEventGroupSetBits(TFT_Display_Handler, VOL_DUTY_EVENT);
//		vTaskDelay(1500);
//	}
	//pwm_output=90-received*10;
	xQueueReceive(DutyQueue, &Duty_buf, portMAX_DELAY);
	duty=Duty_buf;
	TIM2->CCR1=100-Duty_buf;
	//xEventGroupSetBits(TFT_Display_Handler, VOL_DUTY_EVENT);
	osDelay(50);


  }
  /* USER CODE END PWM_Task */
}

/* USER CODE BEGIN Header_Log_Task */
/**
* @brief Function implementing the Log thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Log_Task */
void Log_Task(void *argument)
{
  /* USER CODE BEGIN Log_Task */
  /* Infinite loop */

	char switch_buf=0;

  for(;;)
  {
	  xQueueReceive(RFID_TO_LogQueue, &switch_buf, portMAX_DELAY);
	  switch_data=switch_buf;
	  osTimerStart(RFID_TimerHandle, 2000);
	  if(switch_buf>0)
	  {
		  temperature = Flash_Read_NUM(0x0810C000);
		  humidity = Flash_Read_NUM(0x0810C004);
		  ADC_VAL = (uint16_t)Flash_Read_NUM(0x0810C008);
		  duty = (uint8_t)Flash_Read_NUM(0x0810C00C);
//		  LogOut_Flash_Read(0x08110000,Flash_buf);
		  //test_flash=Flash_Read_NUM(0x08110000);
		  Screen_Switch_Event=xEventGroupWaitBits(Log_Out_Handler, 0x07, pdTRUE, pdFALSE, portMAX_DELAY);
		  osThreadSuspend(SHTHandle);
		  osThreadSuspend(ADCHandle);
		  osThreadSuspend(PWMHandle);
		  TIM2->CCR1=100-0;
		  osThreadSuspend(SDHandle);
		  Unmount_SD("/");
//		  Flash_Write_NUM(0x08110000,temperature);
//		  Flash_Write_NUM(0x08110004,humidity);
//		  Flash_Write_NUM(0x08110008,(float)ADC_VAL);
//		  Flash_Write_NUM(0x0811000C,(float)duty);
		  Flash_buf[0]=temperature;
		  Flash_buf[1]=humidity;
		  Flash_buf[2]=(float)ADC_VAL;
		  Flash_buf[3]=(float)duty;
		  LogOut_Flash_Write(0x0810C000,Flash_buf);
	  }
	  osDelay(10);
  }
  /* USER CODE END Log_Task */
}

/* USER CODE BEGIN Header_SD_Task */
/**
* @brief Function implementing the SD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SD_Task */
void SD_Task(void *argument)
{
  /* USER CODE BEGIN SD_Task */
	Mount_SD("/");
	int test = 0;

	/* SHT_Queue can work, but I don't know why RTC_Queue cannot work, so I just use the global variable to receive the RTC data*/
	/* And I don't verify the RFID_Queue, I directly use the global variable to receive the RFID's data */
	SHT_Data *SHT_Data_prt;
	RFID_Data *RFID_Data_ptr;
	RTC_Data *RTC_Data_ptr;
	osThreadSuspend(SDHandle);
  /* Infinite loop */
  for(;;)
  {
	  /* Receive the SHT's data */
	  //xQueueReceive(SHT_Queue_Handler, &SHT_Data_prt, portMAX_DELAY);

	  /* Store the information into the SD card */
	  char *buffer = pvPortMalloc(100*sizeof(char));
	  sprintf (buffer, "Date : 20%02d.%02d.%02d., Time : %02d.%02d.%02d., Tem = %dC,Hum = %d%%RH,Vol : %umV, Duty : %d\n", sDate.Year, sDate.Month, sDate.Date, sTime.Hours, sTime.Minutes, sTime.Seconds,(int) temperature,(int) humidity,ADC_VAL, duty);
	  //sprintf (buffer, "Date : 20%02d.%02d.%02d., Time : %02d.%02d.%02d., Temerature = %d, PWM Duty : %d/255\n", sDate.Year, sDate.Month, sDate.Date, sTime.Hours, sTime.Minutes, sTime.Seconds,(int) temperature, 10);
	  HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
	  Update_File("record.TXT", buffer);
	  vPortFree(buffer);
	  osDelay(1000);
  }
  /* USER CODE END SD_Task */
}

/* RTC_Timer_Callback function */
void RTC_Timer_Callback(void *argument)
{
  /* USER CODE BEGIN RTC_Timer_Callback */

	Cpu_Usage = osGetCPUUsage();
	xQueueOverwrite(CPU_Usage_Queue,&Cpu_Usage);
	/* This software timer interrupt is triggered by 1 sec to update the RTC information */
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

	/* Let TouchGFX Model make change of display */
	RTC_Flag = 1;


	/* I don't know why RTC_Queue cannot work, so I just use the global variable to receive the RTC data*/
//	BaseType_t pxHigherPriorityTaskWoken = 0;
//	xQueueOverwriteFromISR(RTC_Queue_Handler, &RTC_Data_, pxHigherPriorityTaskWoken);
//	portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);

	/* Uart port to computer for debug */
//	sprintf (date, "Date : %2d : %2d : %2d\n", sDate.Date, sDate.Month, sDate.Year);
//	sprintf (time, "Time : %2d : %2d : %2d\n", sTime.Hours, sTime.Minutes, sTime.Seconds);
//	HAL_UART_Transmit(&huart1, (uint8_t *)date, strlen(date), HAL_MAX_DELAY);
//	HAL_UART_Transmit(&huart1, (uint8_t *)time, strlen(time), HAL_MAX_DELAY);

  /* USER CODE END RTC_Timer_Callback */
}

/* RFID_Timer_Callback function */
void RFID_Timer_Callback(void *argument)
{
  /* USER CODE BEGIN RFID_Timer_Callback */

	if(switch_data>0)
		xQueueOverwrite(RFID_ResetQueue,&Screen_Switch_Event);
	else
		xQueueOverwrite(RFID_ResetQueue,&switch_data);
	//osSignalSet(SHTHandle,0x01);
	if(switch_data>0)
	{
	  osThreadResume(SHTHandle);
	  osThreadResume(ADCHandle);
	  osThreadResume(PWMHandle);
	  TIM2->CCR1=100-duty;
	  osThreadResume(SDHandle);
	  Mount_SD("/");
	}

	//switch_data=0;
  /* USER CODE END RFID_Timer_Callback */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
