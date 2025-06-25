/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c-lcd.h"
#include "stdio.h"
#include "string.h"
#include "stdbool.h"
#include "stdlib.h"
#include "flash_storage.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Struktur untuk menyimpan informasi setiap timer
// typedef struct
//{
//  uint32_t remaining_seconds; // Waktu tersisa dalam detik
//  bool is_running;            // Status: true jika berjalan, false jika berhenti
//  bool is_finished;           // Status: true jika timer sudah selesai (mencapai 0)
//} TimerInfo_t;

// Mode operasi program
typedef enum
{
  MODE_RUN, // Mode normal, timer berjalan dan menampilkan waktu
  MODE_SET  // Mode pengaturan, untuk mengubah nilai timer
} OperatingMode_t;

// Enum untuk menentukan bagian mana yang sedang diedit di MODE_SET
typedef enum
{
  EDIT_FOCUS_HOUR,
  EDIT_FOCUS_MINUTE,
  EDIT_FOCUS_SECOND,
  EDIT_FOCUS_CURRENT,
  EDIT_FOCUS_START_STOP,
  EDIT_FOCUS_EXIT // Status virtual untuk keluar dari mode SET
} EditFocus_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_TIMERS 8   // Jumlah total timer
#define NUM_MATRICES 8 // Jumlah modul matriks yang di-cascade
#define JENIS_FONT FONT_TIPIS // Jenis font yang digunakan untuk tampilan FONT_TIPIS atau FONT_TEBAL

// Definisi untuk driver MAX7219
#define MAX7219_CS_PORT MAX7219_CS_GPIO_Port
#define MAX7219_CS_PIN MAX7219_CS_Pin

#define MAX7219_REG_NO_OP 0x00
#define MAX7219_REG_DECODE_MODE 0x09
#define MAX7219_REG_INTENSITY 0x0A
#define MAX7219_REG_SCAN_LIMIT 0x0B
#define MAX7219_REG_SHUTDOWN 0x0C
#define MAX7219_REG_DISPLAY_TEST 0x0F

// === Definisi Driver 74HC595 (via SPI2) ===
#define HC595_LATCH_PORT HC595_LATCH_GPIO_Port // Pin STCP / RCLK
#define HC595_LATCH_PIN HC595_LATCH_Pin        // Pin RCLK
// Pin DATA (MOSI) dan CLK (SCK) dikontrol oleh periferal SPI2

// Definisi pin tombol (sesuaikan dengan konfigurasi CubeMX Anda)
#define BTN_SET_PORT BTN_ENC_GPIO_Port
#define BTN_SET_PIN BTN_ENC_Pin

// Definisi pin buzzer
#define BUZZER_PORT BUZZER_GPIO_Port
#define BUZZER_PIN BUZZER_Pin

// --- Definisi Modbus ---
#define MODBUS_SLAVE_ID 1
#define MODBUS_WRITE_MULTIPLE_REGS 0x10
#define MODBUS_START_ADDRESS 0
#define MODBUS_NUM_REGISTERS (NUM_TIMERS * 3)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* Definitions for taskTimerManager */
osThreadId_t taskTimerManagerHandle;
const osThreadAttr_t taskTimerManager_attributes = {
    .name = "taskTimerManager",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal,
};
/* Definitions for taskDisplayManager */
osThreadId_t taskDisplayManagerHandle;
const osThreadAttr_t taskDisplayManager_attributes = {
    .name = "taskDisplayManager",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityHigh,
};
/* Definitions for taskInputHandler */
osThreadId_t taskInputHandlerHandle;
const osThreadAttr_t taskInputHandler_attributes = {
    .name = "taskInputHandler",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for taskBuzzer */
osThreadId_t taskBuzzerHandle;
const osThreadAttr_t taskBuzzer_attributes = {
    .name = "taskBuzzer",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for taskModbusMaster */
osThreadId_t taskModbusMasterHandle;
const osThreadAttr_t taskModbusMaster_attributes = {
    .name = "taskModbusMaster",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for timerDataMutex */
osMutexId_t timerDataMutexHandle;
const osMutexAttr_t timerDataMutex_attributes = {
    .name = "timerDataMutex"};
/* Definitions for buzzerSemaphore */
osSemaphoreId_t buzzerSemaphoreHandle;
const osSemaphoreAttr_t buzzerSemaphore_attributes = {
    .name = "buzzerSemaphore"};
/* USER CODE BEGIN PV */
// Variabel Global (Shared State) - dilindungi oleh Mutex
TimerInfo_t timers[NUM_TIMERS];
OperatingMode_t current_mode = MODE_RUN;
uint8_t selected_timer_index = 0;         // Timer yang sedang dipilih (untuk diatur/dilihat)
uint8_t run_mode_display_index = 0;       // Timer yang sedang ditampilkan di mode RUN
EditFocus_t edit_focus = EDIT_FOCUS_HOUR; // Fokus edit saat berada di MODE_SET
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
void StartTaskTimerManager(void *argument);
void StartTaskDisplayManager(void *argument);
void StartTaskInputHandler(void *argument);
void StartTaskBuzzer(void *argument);
void StartTaskModbusMaster(void *argument);

/* USER CODE BEGIN PFP */
// Prototipe fungsi helper
void InitializeTimers(void);
void format_time(char *buffer, uint32_t total_seconds);
void format_time_split(uint32_t total_seconds, uint32_t *h, uint32_t *m, uint32_t *s);

// Prototipe Driver MAX7219
// void MAX7219_Send(uint8_t reg, uint8_t data);
// void MAX7219_Init(void);
// void MAX7219_Clear(void);
// void MAX7219_DisplayNumber(uint32_t number);

// ===================================================================
// === PROTOTIPE DRIVER MAX7219 UNTUK DOT MATRIX CASCADE (BARU) ====
// ===================================================================
void MAX7219_Init(void);
void MAX7219_Send_Cascade(uint8_t reg, const uint8_t *data_array);
void MAX7219_Clear_Cascade(void);
void DrawCharacterOnMatrix(uint8_t matrix_buffer[8], char c);

// Prototipe Driver 74HC595
void HC595_Write(uint8_t data);

// --- Prototipe Fungsi Modbus ---
uint16_t Modbus_CRC16(const uint8_t *nData, uint16_t wLength);
void ModbusMaster_WriteMultipleRegisters(uint8_t slave_id, uint16_t start_addr, uint16_t num_regs, uint16_t *reg_data);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ===================================================================
// ===         FONT DEFINITION UNTUK DOT MATRIX 8x8 (BARU)         ===
// ===================================================================
// Definisi font untuk karakter 0-9 dan ':'
// Setiap karakter direpresentasikan oleh 8 byte, masing-masing byte untuk satu baris
// Lebar karakter 5 piksel, tinggi 8 piksel
const uint8_t font_tipis[11][8] = {
    {0x3E, 0x42, 0x46, 0x4A, 0x52, 0x62, 0x42, 0x3E}, // 0
    {0x10, 0x30, 0x10, 0x10, 0x10, 0x10, 0x10, 0x38}, // 1
    {0x3E, 0x42, 0x02, 0x04, 0x08, 0x10, 0x40, 0x7E}, // 2
    {0x3E, 0x42, 0x02, 0x1C, 0x02, 0x02, 0x42, 0x3E}, // 3
    {0x04, 0x0C, 0x14, 0x24, 0x7E, 0x04, 0x04, 0x04}, // 4
    {0x7E, 0x40, 0x40, 0x7C, 0x02, 0x02, 0x42, 0x3E}, // 5
    {0x3E, 0x42, 0x40, 0x7C, 0x42, 0x42, 0x42, 0x3E}, // 6
    {0x7E, 0x42, 0x04, 0x08, 0x10, 0x20, 0x20, 0x20}, // 7
    {0x3E, 0x42, 0x42, 0x3E, 0x42, 0x42, 0x42, 0x3E}, // 8
    {0x3E, 0x42, 0x42, 0x42, 0x3E, 0x02, 0x42, 0x3E}, // 9
    {0x00, 0x00, 0x30, 0x30, 0x00, 0x30, 0x30, 0x00}  // :
};

const uint8_t font_tebal[11][8] = {
    {0x7C, 0xC6, 0xCE, 0xDE, 0xF6, 0xE6, 0x7C, 0x00}, // U+0030 (0)
    {0x30, 0x70, 0x30, 0x30, 0x30, 0x30, 0xFC, 0x00}, // U+0031 (1)
    {0x78, 0xCC, 0x0C, 0x38, 0x60, 0xCC, 0xFC, 0x00}, // U+0032 (2)
    {0x78, 0xCC, 0x0C, 0x38, 0x0C, 0xCC, 0x78, 0x00}, // U+0033 (3)
    {0x1C, 0x3C, 0x6C, 0xCC, 0xFE, 0x0C, 0x1E, 0x00}, // U+0034 (4)
    {0xFC, 0xC0, 0xF8, 0x0C, 0x0C, 0xCC, 0x78, 0x00}, // U+0035 (5)
    {0x38, 0x60, 0xC0, 0xF8, 0xCC, 0xCC, 0x78, 0x00}, // U+0036 (6)
    {0xFC, 0xCC, 0x0C, 0x30, 0x60, 0x60, 0x60, 0x00}, // U+0037 (7)
    {0x78, 0xCC, 0xCC, 0x78, 0xCC, 0xCC, 0x78, 0x00}, // U+0038 (8)
    {0x78, 0xCC, 0xCC, 0x7C, 0x0C, 0x30, 0x70, 0x00}, // U+0039 (9)
    {0x00, 0x30, 0x30, 0x00, 0x00, 0x30, 0x30, 0x00}  // U+003A (:)
};
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  /* Initialize custom peripherals and data */
  InitializeTimers();
  lcd_init();
  MAX7219_Init();
  HC595_Write(0x00); // Matikan semua LED

  // Mulai timer encoder
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of timerDataMutex */
  timerDataMutexHandle = osMutexNew(&timerDataMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of buzzerSemaphore */
  buzzerSemaphoreHandle = osSemaphoreNew(1, 1, &buzzerSemaphore_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of taskTimerManager */
  taskTimerManagerHandle = osThreadNew(StartTaskTimerManager, NULL, &taskTimerManager_attributes);

  /* creation of taskDisplayManager */
  taskDisplayManagerHandle = osThreadNew(StartTaskDisplayManager, NULL, &taskDisplayManager_attributes);

  /* creation of taskInputHandler */
  taskInputHandlerHandle = osThreadNew(StartTaskInputHandler, NULL, &taskInputHandler_attributes);

  /* creation of taskBuzzer */
  taskBuzzerHandle = osThreadNew(StartTaskBuzzer, NULL, &taskBuzzer_attributes);

  /* creation of taskModbusMaster */
  taskModbusMasterHandle = osThreadNew(StartTaskModbusMaster, NULL, &taskModbusMaster_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  //  cek apakah thread sudah berhasil atau belum
  if (taskTimerManagerHandle == NULL ||
      taskDisplayManagerHandle == NULL ||
      taskInputHandlerHandle == NULL ||
      taskBuzzerHandle == NULL ||
      taskModbusMasterHandle == NULL)
  {
    /* Thread creation failed, handle error */
    //    lcd_put_cur(0, 0);
    //    lcd_send_string("Dtech Eng");
    //    lcd_put_cur(1, 0);
    //    lcd_send_string("Failed");
    Error_Handler();
  }
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
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
 * @brief SPI3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
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
  huart1.Init.WordLength = UART_WORDLENGTH_9B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_EVEN;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MAX7219_CS_Pin | HC595_LATCH_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MODBUS_SEL_GPIO_Port, MODBUS_SEL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_ENC_Pin */
  GPIO_InitStruct.Pin = BTN_ENC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN_ENC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MAX7219_CS_Pin HC595_LATCH_Pin */
  GPIO_InitStruct.Pin = MAX7219_CS_Pin | HC595_LATCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MODBUS_SEL_Pin */
  GPIO_InitStruct.Pin = MODBUS_SEL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MODBUS_SEL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUZZER_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// ============== FUNGSI HELPER & DRIVER MAX7219 =================

/**
 * @brief Menginisialisasi semua timer ke nilai default.
 */
void InitializeTimers(void)
{
  if (osMutexAcquire(timerDataMutexHandle, osWaitForever) == osOK)
  {
    for (int i = 0; i < NUM_TIMERS; i++)
    {
      timers[i].remaining_seconds = 0; // Default 0 detik
      timers[i].is_running = false;
      timers[i].is_finished = false;
    }
    osMutexRelease(timerDataMutexHandle);
  }
}

/**
 * @brief Mengubah total detik menjadi format HH:MM:SS
 */
void format_time(char *buffer, uint32_t total_seconds)
{
  uint32_t hours = total_seconds / 3600;
  uint32_t minutes = (total_seconds % 3600) / 60;
  uint32_t seconds = total_seconds % 60;
  sprintf(buffer, "%02lu:%02lu:%02lu", hours, minutes, seconds);
}

void format_time_split(uint32_t total_seconds, uint32_t *h, uint32_t *m, uint32_t *s)
{
  *h = total_seconds / 3600;
  *m = (total_seconds % 3600) / 60;
  *s = total_seconds % 60;
}

// ============== DRIVER 74HC595 =================

/**
 * @brief Mengirim 1 byte data ke 74HC595 menggunakan SPI2.
 */
void HC595_Write(uint8_t data)
{
  // Set Latch pin ke LOW sebelum mengirim data
  HAL_GPIO_WritePin(HC595_LATCH_PORT, HC595_LATCH_PIN, GPIO_PIN_RESET);

  // Kirim data melalui SPI. Ini adalah operasi blocking.
  // Timeout diatur ke 100ms.
  if (HAL_SPI_Transmit(&hspi1, &data, 1, 100) != HAL_OK)
  {
    Error_Handler();
  }

  // Set Latch pin ke HIGH untuk memindahkan data dari shift register ke storage register
  HAL_GPIO_WritePin(HC595_LATCH_PORT, HC595_LATCH_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(HC595_LATCH_PORT, HC595_LATCH_PIN, GPIO_PIN_RESET);
}

// ===================================================================
// ===     DRIVER MAX7219 UNTUK DOT MATRIX CASCADE (DIMODIFIKASI)    ===
// ===================================================================

/**
 * @brief Mengirim data ke semua modul MAX7219 yang di-cascade.
 * @param reg: Alamat register MAX7219 yang dituju (misal, baris 1-8).
 * @param data_array: Array 8 byte. data_array[0] untuk modul 1, ..., data_array[7] untuk modul 8.
 */
void MAX7219_Send_Cascade(uint8_t reg, const uint8_t *data_array)
{
  uint8_t spi_data[2];

  HAL_GPIO_WritePin(MAX7219_CS_PORT, MAX7219_CS_PIN, GPIO_PIN_RESET);

  // Kirim data ke semua modul dalam satu siklus CS
  // Data dikirim secara terbalik, karena sifat shift register.
  // Data untuk modul terakhir (paling jauh dari MCU) dikirim pertama kali.
  for (int i = NUM_MATRICES - 1; i >= 0; i--)
  {
    spi_data[0] = reg;
    spi_data[1] = data_array[i];
    HAL_SPI_Transmit(&hspi3, spi_data, 2, 100);
  }

  HAL_GPIO_WritePin(MAX7219_CS_PORT, MAX7219_CS_PIN, GPIO_PIN_SET);
}

/**
 * @brief Inisialisasi modul MAX7219 untuk mode dot matrix.
 */
void MAX7219_Init(void)
{
  uint8_t init_data[NUM_MATRICES] = {0}; // Buffer dummy untuk data

  // Matikan display untuk sementara
  memset(init_data, 0, NUM_MATRICES);
  MAX7219_Send_Cascade(MAX7219_REG_SHUTDOWN, init_data);

  // Matikan display test
  memset(init_data, 0, NUM_MATRICES);
  MAX7219_Send_Cascade(MAX7219_REG_DISPLAY_TEST, init_data);

  // Set scan limit ke 8 baris (digit 0-7)
  memset(init_data, 7, NUM_MATRICES);
  MAX7219_Send_Cascade(MAX7219_REG_SCAN_LIMIT, init_data);

  // Set mode ke No Decode untuk dot matrix
  memset(init_data, 0x00, NUM_MATRICES); // <-- PERUBAHAN PENTING
  MAX7219_Send_Cascade(MAX7219_REG_DECODE_MODE, init_data);

  // Set kecerahan (0-15)
  memset(init_data, 7, NUM_MATRICES);
  MAX7219_Send_Cascade(MAX7219_REG_INTENSITY, init_data);

  // Nyalakan kembali display
  memset(init_data, 1, NUM_MATRICES);
  MAX7219_Send_Cascade(MAX7219_REG_SHUTDOWN, init_data);

  // Bersihkan layar
  MAX7219_Clear_Cascade();
}

/**
 * @brief Membersihkan semua layar dot matrix.
 */
void MAX7219_Clear_Cascade(void)
{
  uint8_t clear_data[NUM_MATRICES];
  memset(clear_data, 0x00, NUM_MATRICES); // 0x00 untuk mematikan semua LED
  for (int i = 1; i <= 8; i++)
  {
    MAX7219_Send_Cascade(i, clear_data);
  }
}

/**
 * @brief Menggambar satu karakter ke buffer 8x8.
 * @param matrix_buffer: Buffer 8 byte untuk satu matriks.
 * @param c: Karakter yang akan digambar ('0'-'9' atau ':').
 */
void DrawCharacterOnMatrix(uint8_t matrix_buffer[8], char c)
{
  int font_index;
  if (c >= '0' && c <= '9')
  {
    font_index = c - '0';
  }
  else if (c == ':')
  {
    font_index = 10;
  }
  else
  {
    // Karakter tidak dikenal, gambar spasi (kosong)
    memset(matrix_buffer, 0x00, 8);
    return;
  }

// Salin data font ke buffer matriks
#if JENIS_FONT == FONT_TIPIS
  memcpy(matrix_buffer, font_tipis[font_index], 8);
#elif JENIS_FONT == FONT_TEBAL
  memcpy(matrix_buffer, font_tebal[font_index], 8);
#endif
}

// ============== FUNGSI MODBUS =================

/**
 * @brief  Menghitung CRC16 untuk frame Modbus.
 * @param  nData: Pointer ke buffer data.
 * @param  wLength: Panjang data dalam byte.
 * @retval Nilai CRC16.
 */
uint16_t Modbus_CRC16(const uint8_t *nData, uint16_t wLength)
{
  static const uint16_t wCRCTable[] = {
      0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
      0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
      0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
      0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XCD01, 0X0DC0, 0X0C80, 0XCC41,
      0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
      0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
      0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
      0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
      0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
      0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
      0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
      0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
      0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
      0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
      0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
      0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
      0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
      0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
      0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
      0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
      0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
      0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
      0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
      0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
      0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
      0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
      0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
      0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9D01, 0X5DC0, 0X5C80, 0X9C41,
      0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
      0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
      0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
      0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040};

  uint8_t nTemp;
  uint16_t wCRCWord = 0xFFFF;

  while (wLength--)
  {
    nTemp = *nData++ ^ wCRCWord;
    wCRCWord >>= 8;
    wCRCWord ^= wCRCTable[nTemp];
  }
  return wCRCWord;
}

/**
 * @brief  Mengirim frame Modbus "Write Multiple Registers" (FC 16).
 * @param  slave_id: Alamat slave (1-247).
 * @param  start_addr: Alamat register awal.
 * @param  num_regs: Jumlah register yang akan ditulis.
 * @param  reg_data: Pointer ke array data register (uint16_t).
 * @retval None
 */
void ModbusMaster_WriteMultipleRegisters(uint8_t slave_id, uint16_t start_addr, uint16_t num_regs, uint16_t *reg_data)
{
  // Ukuran frame: 1(ID) + 1(FC) + 2(Addr) + 2(NumRegs) + 1(ByteCount) + N*2(Data) + 2(CRC)
  uint8_t frame[256];
  uint8_t frame_len = 0;

  // 1. Isi header Modbus
  frame[frame_len++] = slave_id;
  frame[frame_len++] = MODBUS_WRITE_MULTIPLE_REGS;
  frame[frame_len++] = (start_addr >> 8) & 0xFF; // Alamat (MSB)
  frame[frame_len++] = start_addr & 0xFF;        // Alamat (LSB)
  frame[frame_len++] = (num_regs >> 8) & 0xFF;   // Jumlah Reg (MSB)
  frame[frame_len++] = num_regs & 0xFF;          // Jumlah Reg (LSB)
  frame[frame_len++] = num_regs * 2;             // Jumlah Byte

  // 2. Isi data payload
  for (int i = 0; i < num_regs; i++)
  {
    frame[frame_len++] = (reg_data[i] >> 8) & 0xFF; // Data (MSB)
    frame[frame_len++] = reg_data[i] & 0xFF;        // Data (LSB)
  }

  // 3. Hitung dan tambahkan CRC
  uint16_t crc = Modbus_CRC16(frame, frame_len);
  frame[frame_len++] = crc & 0xFF;        // CRC (LSB)
  frame[frame_len++] = (crc >> 8) & 0xFF; // CRC (MSB)

  // 4. Kirim frame melalui UART
  // Atur pin DE/RE ke mode Transmit
  HAL_GPIO_WritePin(MODBUS_SEL_GPIO_Port, MODBUS_SEL_Pin, GPIO_PIN_SET);

  // Beri sedikit jeda agar pin DE/RE stabil sebelum mengirim data
  osDelay(1);

  HAL_UART_Transmit(&huart1, frame, frame_len, 1000); // Timeout 1 detik

  // Beri jeda untuk memastikan semua data terkirim sebelum menonaktifkan DE
  osDelay(10); // Sesuaikan delay ini jika perlu (tergantung baudrate)

  // Kembalikan pin DE/RE ke mode Receive
  HAL_GPIO_WritePin(MODBUS_SEL_GPIO_Port, MODBUS_SEL_Pin, GPIO_PIN_RESET);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTaskTimerManager */
/**
 * @brief  Function implementing the taskTimerManager thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTaskTimerManager */
void StartTaskTimerManager(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1000); // Tunggu 1 detik

    // Kunci mutex sebelum mengakses data timer
    if (osMutexAcquire(timerDataMutexHandle, osWaitForever) == osOK)
    {
      if (current_mode == MODE_RUN)
      {
        for (int i = 0; i < NUM_TIMERS; i++)
        {
          if (timers[i].is_running && timers[i].remaining_seconds > 0)
          {
            timers[i].remaining_seconds--;
            if (timers[i].remaining_seconds == 0)
            {
              timers[i].is_running = false;
              timers[i].is_finished = true; // Tandai sebagai selesai
              // Beri sinyal ke task buzzer bahwa timer telah selesai
              osSemaphoreRelease(buzzerSemaphoreHandle);
            }
          }
        }
      }
      // Lepaskan mutex setelah selesai
      osMutexRelease(timerDataMutexHandle);
    }
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTaskDisplayManager */
/**
 * @brief Function implementing the taskDisplayManager thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTaskDisplayManager */
void StartTaskDisplayManager(void *argument)
{
  /* USER CODE BEGIN StartTaskDisplayManager */
  char lcd_buffer_line1[17];
  char lcd_buffer_line2[17];
  char time_format_buffer[10];

  // Buffer untuk menampung data piksel dari 8 modul matriks
  uint8_t display_buffer[NUM_MATRICES][8]; // [indeks_matriks][baris]

  uint8_t display_index = 0;

  /* Infinite loop */
  for (;;)
  {
    if (osMutexAcquire(timerDataMutexHandle, osWaitForever) == osOK)
    {
      if (current_mode == MODE_RUN)
      {
        // Di mode RUN, tampilkan timer yang dipilih oleh rotary encoder
        display_index = run_mode_display_index;
        sprintf(lcd_buffer_line1, "RUN MODE   T:%d", display_index + 1);

        format_time(time_format_buffer, timers[display_index].remaining_seconds);
        sprintf(lcd_buffer_line2, "%s %s", time_format_buffer,
                timers[display_index].is_running ? "RUN" : "STOP");
      }
      else // MODE_SET
      {
        // Di mode SET, tampilkan timer yang sedang di-edit
        display_index = selected_timer_index;
        sprintf(lcd_buffer_line1, "SET MODE   T:%d", display_index + 1);

        // Tampilkan kursor `^` untuk menunjukkan apa yang sedang diedit
        format_time(time_format_buffer, timers[display_index].remaining_seconds);

        // Tulis waktu di atasnya
        sprintf(lcd_buffer_line2, "%s %s", time_format_buffer,
                edit_focus == EDIT_FOCUS_HOUR      ? "HOUR"
                : edit_focus == EDIT_FOCUS_MINUTE  ? "MINUTE"
                : edit_focus == EDIT_FOCUS_SECOND  ? "SECOND"
                : timers[display_index].is_running ? "RUN?"
                                                   : "STOP?");
      }

      // Update LED Indikator via 74HC595
      HC595_Write(1 << display_index);

      // --- LOGIKA BARU UNTUK DOT MATRIX ---

      // 1. Bersihkan buffer virtual
      memset(display_buffer, 0x00, sizeof(display_buffer));

      // 2. Siapkan string yang akan ditampilkan
      uint32_t h, m, s;
      format_time_split(timers[display_index].remaining_seconds, &h, &m, &s);

      char display_string[10];
      // Format: "T:HHMMSS" -> T=Timer #, H=Jam, M=Menit, S=Detik
      sprintf(display_string, "%1d:%02lu%02lu%02lu", (int)(display_index + 1), h, m, s);
      // Contoh hasil string: "1:012345"

      // 3. Gambar string ke buffer virtual, karakter per karakter
      // Matriks 0: Detik satuan
      DrawCharacterOnMatrix(display_buffer[0], display_string[7]);
      // Matriks 1: Detik puluhan
      DrawCharacterOnMatrix(display_buffer[1], display_string[6]);
      // Matriks 2: Menit satuan
      DrawCharacterOnMatrix(display_buffer[2], display_string[5]);
      // Matriks 3: Menit puluhan
      DrawCharacterOnMatrix(display_buffer[3], display_string[4]);
      // Matriks 4: Jam satuan
      DrawCharacterOnMatrix(display_buffer[4], display_string[3]);
      // Matriks 5: Jam puluhan
      DrawCharacterOnMatrix(display_buffer[5], display_string[2]);
      // Matriks 6: ':'
      DrawCharacterOnMatrix(display_buffer[6], display_string[1]);
      // Matriks 7: Timer #
      DrawCharacterOnMatrix(display_buffer[7], display_string[0]);

      osMutexRelease(timerDataMutexHandle);

      // 4. Kirim buffer virtual ke semua matriks, baris per baris
      uint8_t row_data[NUM_MATRICES];
      for (int row = 0; row < 8; row++)
      {
        // Kumpulkan data untuk baris 'row' dari semua 8 matriks
        for (int matrix = 0; matrix < NUM_MATRICES; matrix++)
        {
          row_data[matrix] = display_buffer[matrix][row];
        }
        // Kirim data untuk satu baris ke semua modul sekaligus
        // Register 1 untuk baris 1, 2 untuk baris 2, dst.
        MAX7219_Send_Cascade(row + 1, row_data);
      }

      // Update LCD
      lcd_put_cur(0, 0);
      lcd_send_string(lcd_buffer_line1);
      lcd_put_cur(1, 0);
      lcd_send_string(lcd_buffer_line2);

    } // End mutex acquire

    osDelay(150); // Refresh display setiap 150ms
  }
  /* USER CODE END StartTaskDisplayManager */
}

/* USER CODE BEGIN Header_StartTaskInputHandler */
/**
 * @brief Function implementing the taskInputHandler thread.
 * @param argument: Not used
 * @retval None
 *
 * Logika Baru untuk Rotary Encoder:
 * - Putaran:
 * - Mode RUN: Mengganti timer yang ditampilkan (run_mode_display_index).
 * - Mode SET: Mengubah nilai (jam/menit/detik) sesuai fokus.
 * - Tombol Tekan (B1_Pin):
 * - Tekan Singkat:
 * - Mode RUN: Masuk ke Mode SET untuk timer yang sedang ditampilkan.
 * - Mode SET: Memindahkan fokus edit (Jam -> Menit -> Detik -> Start/Stop -> Keluar).
 * - Tekan Lama:
 * - Mode SET: Keluar dari Mode SET dan kembali ke Mode RUN.
 */
/* USER CODE END Header_StartTaskInputHandler */
void StartTaskInputHandler(void *argument)
{
  /* USER CODE BEGIN StartTaskInputHandler */
  int16_t last_encoder_val = 0;
  uint32_t btn_press_start_time = 0;
  bool btn_is_pressed = false;
  bool long_press_triggered = false;
  const uint32_t long_press_duration = 1000; // 1 detik

  last_encoder_val = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);

  for (;;)
  {
    // --- 1. Membaca Putaran Encoder ---
    int16_t current_encoder_val = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    int16_t diff = current_encoder_val - last_encoder_val;

    if (abs(diff) >= 4) // Encoder biasanya 4 pulsa per 'klik'
    {
      int8_t clicks = diff / 4;
      last_encoder_val = current_encoder_val;

      if (osMutexAcquire(timerDataMutexHandle, 20) == osOK)
      {
        if (current_mode == MODE_RUN)
        {
          // Ganti timer yang ditampilkan
          int temp_idx = run_mode_display_index + clicks;
          if (temp_idx < 0)
            temp_idx = (temp_idx % NUM_TIMERS + NUM_TIMERS) % NUM_TIMERS;
          run_mode_display_index = temp_idx % NUM_TIMERS;
        }
        else
        { // MODE_SET
          uint32_t *p_seconds = &timers[selected_timer_index].remaining_seconds;
          uint32_t h, m, s;
          format_time_split(*p_seconds, &h, &m, &s);

          switch (edit_focus)
          {
          case EDIT_FOCUS_HOUR:
            h = (h + clicks + 24) % 24; // Batasi 0-23
            break;
          case EDIT_FOCUS_MINUTE:
            m = (m + clicks + 60) % 60; // Batasi 0-59
            break;
          case EDIT_FOCUS_SECOND:
            s = (s + clicks + 60) % 60; // Batasi 0-59
            break;
          case EDIT_FOCUS_START_STOP:
            if (clicks != 0)
            { // Cukup satu klik untuk toggle
              timers[selected_timer_index].is_running = !timers[selected_timer_index].is_running;
              if (timers[selected_timer_index].is_running)
              {
                timers[selected_timer_index].is_finished = false;
              }
            }
            break;
          default:
            break;
          }
          *p_seconds = h * 3600 + m * 60 + s;
        }
        osMutexRelease(timerDataMutexHandle);
      }
    }

    // --- 2. Membaca Tombol Tekan Encoder ---
    // Menggunakan pull-up, jadi pin RESET (LOW) saat ditekan
    bool btn_current_state = (HAL_GPIO_ReadPin(BTN_ENC_GPIO_Port, BTN_ENC_Pin) == GPIO_PIN_RESET);

    // Deteksi tombol baru ditekan (transisi dari tidak ditekan ke ditekan)
    if (btn_current_state && !btn_is_pressed)
    {
      btn_is_pressed = true;
      long_press_triggered = false;
      btn_press_start_time = osKernelGetTickCount();
    }
    // Deteksi tombol dilepas (transisi dari ditekan ke tidak ditekan)
    else if (!btn_current_state && btn_is_pressed)
    {
      btn_is_pressed = false;
      if (!long_press_triggered)
      {
        // --- AKSI TEKAN SINGKAT ---
        if (osMutexAcquire(timerDataMutexHandle, 20) == osOK)
        {
          if (current_mode == MODE_RUN)
          {
            current_mode = MODE_SET;
            selected_timer_index = run_mode_display_index; // Edit timer yang ditampilkan
            edit_focus = EDIT_FOCUS_HOUR;                  // Mulai dari edit jam
          }
          else
          { // MODE_SET
            edit_focus++;
            if (edit_focus > EDIT_FOCUS_EXIT)
            {
              edit_focus = EDIT_FOCUS_HOUR;
            }
            if (edit_focus == EDIT_FOCUS_EXIT)
            {
              current_mode = MODE_RUN; // Simpan dan keluar
            }
          }
          osMutexRelease(timerDataMutexHandle);
        }
      }
    }

    // Deteksi tombol ditekan lama
    if (btn_is_pressed && !long_press_triggered && (osKernelGetTickCount() - btn_press_start_time > long_press_duration))
    {
      long_press_triggered = true;
      // --- AKSI TEKAN LAMA ---
      if (osMutexAcquire(timerDataMutexHandle, 20) == osOK)
      {
        if (current_mode == MODE_SET)
        {
          current_mode = MODE_RUN; // Batalkan dan keluar
                                   // Opsional: bisa ditambahkan logika untuk mengembalikan nilai timer seperti sebelum diedit
        }
        osMutexRelease(timerDataMutexHandle);
      }
    }

    osDelay(20); // Poll input setiap 20ms
  }
  /* USER CODE END StartTaskInputHandler */
}

/* USER CODE BEGIN Header_StartTaskBuzzer */
/**
 * @brief Function implementing the taskBuzzer thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTaskBuzzer */
void StartTaskBuzzer(void *argument)
{
  /* USER CODE BEGIN StartTaskBuzzer */
  /* Infinite loop */
  for (;;)
  {
    // Tunggu sinyal dari semaphore (akan block sampai ada sinyal)
    if (osSemaphoreAcquire(buzzerSemaphoreHandle, osWaitForever) == osOK)
    {
      // Buzzer berbunyi
      HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
      osDelay(500);
      HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
      osDelay(100);
      HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
      osDelay(500);
      HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
    }
  }
  /* USER CODE END StartTaskBuzzer */
}

/* USER CODE BEGIN Header_StartTaskModbusMaster */
/**
 * @brief Function implementing the taskModbusMaster thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTaskModbusMaster */
void StartTaskModbusMaster(void *argument)
{
  /* USER CODE BEGIN StartTaskModbusMaster */
  // Buffer untuk menampung data yang akan dikirim
  uint16_t modbus_data_buffer[MODBUS_NUM_REGISTERS];

  /* Infinite loop */
  for (;;)
  {
    // Kirim data setiap 2 detik
    osDelay(2000);

    // Ambil data timer dengan aman menggunakan mutex
    if (osMutexAcquire(timerDataMutexHandle, osWaitForever) == osOK)
    {
      // Susun data ke dalam buffer Modbus
      for (int i = 0; i < NUM_TIMERS; i++)
      {
        uint16_t status = 0; // Default: STOP

        if (timers[i].is_finished)
        {
          status = 2; // FINISHED
        }
        else if (timers[i].is_running)
        {
          status = 1; // RUNNING
        }

        uint32_t val = timers[i].remaining_seconds;
        uint16_t val_low = val & 0xFFFF;
        uint16_t val_high = (val >> 16) & 0xFFFF;

        modbus_data_buffer[i * 3 + 0] = val_low;
        modbus_data_buffer[i * 3 + 1] = val_high;
        modbus_data_buffer[i * 3 + 2] = status;
      }
      osMutexRelease(timerDataMutexHandle);

      // Kirim data ke slave
      ModbusMaster_WriteMultipleRegisters(MODBUS_SLAVE_ID, MODBUS_START_ADDRESS, MODBUS_NUM_REGISTERS, modbus_data_buffer);
    }
  }
  /* USER CODE END StartTaskModbusMaster */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM7 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7)
  {
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
  __disable_irq();
  while (1)
  {
    static uint32_t lastTick = 0;
    if (HAL_GetTick() - lastTick >= 500)
    {
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // Toggle an LED or GPIO pin to indicate an error
      lastTick = HAL_GetTick();
    }
    osDelay(10); // Prevents tight loop, allows RTOS to run other tasks
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
