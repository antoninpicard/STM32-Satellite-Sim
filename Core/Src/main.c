/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "ssd1306.h"
#include "ssd1306_fonts.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ---- I2C addresses ---- */
#define MPU6050_ADDR       (0x68 << 1)
#define BMP280_ADDR        (0x76 << 1)

/* ---- MPU-6050 registers ---- */
#define MPU_REG_PWR_MGMT1  0x6B
#define MPU_REG_ACCEL_CFG  0x1C
#define MPU_REG_GYRO_CFG   0x1B
#define MPU_REG_DATA_START 0x3B

/* ---- BMP280 registers ---- */
#define BMP_REG_ID         0xD0
#define BMP_REG_CTRL_MEAS  0xF4
#define BMP_REG_CONFIG     0xF5
#define BMP_REG_CALIB      0x88
#define BMP_REG_DATA_START 0xF7

/* ---- Joystick thresholds ---- */
#define JOY_LEFT_THRESH    1000
#define JOY_RIGHT_THRESH   3000
#define JOY_UP_THRESH      3000
#define JOY_DOWN_THRESH    1000

/* ---- Alarm thresholds ---- */
#define ALARM_TILT_G       0.8f     /* accel X or Y > 0.8g        */
#define ALARM_TEMP_HIGH    50.0f    /* BMP temperature > 50 C     */
#define ALARM_PRES_LOW     950.0f   /* pressure < 950 hPa         */
#define ALARM_PRES_HIGH    1050.0f  /* pressure > 1050 hPa        */
#define ALARM_ALT_HIGH     2000.0f  /* altitude > 2000 m          */
#define ALARM_ALT_LOW      -100.0f  /* altitude < -100 m          */

/* ---- SD card SPI commands ---- */
#define SD_CMD0    0    /* GO_IDLE_STATE           */
#define SD_CMD1    1    /* SEND_OP_COND            */
#define SD_CMD8    8    /* SEND_IF_COND            */
#define SD_CMD16   16   /* SET_BLOCKLEN            */
#define SD_CMD17   17   /* READ_SINGLE_BLOCK       */
#define SD_CMD24   24   /* WRITE_BLOCK             */
#define SD_CMD55   55   /* APP_CMD                 */
#define SD_CMD58   58   /* READ_OCR                */
#define SD_ACMD41  41   /* SD_SEND_OP_COND         */

/* ---- SD log data packet ---- */
#define SD_BLOCK_SIZE      512
#define LOG_PACKET_SIZE    40   /* bytes per log entry     */
#define LOG_SYNC_MARKER    0xAA55AA55

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* ---- Shared IMU data ---- */
typedef struct {
  float accel_x, accel_y, accel_z;   /* in g          */
  float gyro_x, gyro_y, gyro_z;     /* in deg/s      */
  float temp;                        /* in degrees C  */
} IMU_Data_t;

IMU_Data_t imu_data = {0};

osMutexId_t mutexIMU_Handle;
const osMutexAttr_t mutexIMU_attr = {
  .name = "mutexIMU"
};

/* ---- Shared BMP280 data ---- */
typedef struct {
  float temperature;    /* in degrees C  */
  float pressure;       /* in hPa        */
  float altitude;       /* in meters     */
} BMP_Data_t;

BMP_Data_t bmp_data = {0};

osMutexId_t mutexBMP_Handle;
const osMutexAttr_t mutexBMP_attr = {
  .name = "mutexBMP"
};

/* ---- Shared alarm flags ---- */
typedef struct {
  uint8_t tilt;          /* 1 = tilt alert active    */
  uint8_t temp_high;     /* 1 = temp alert active    */
  uint8_t pres_abnormal; /* 1 = pressure alert       */
  uint8_t alt_critical;  /* 1 = altitude alert       */
} Alarm_Flags_t;

Alarm_Flags_t alarm_flags = {0};

osMutexId_t mutexAlarm_Handle;
const osMutexAttr_t mutexAlarm_attr = {
  .name = "mutexAlarm"
};

/* ---- BMP280 calibration (read once at init) ---- */
static uint16_t bmp_dig_T1;
static int16_t  bmp_dig_T2, bmp_dig_T3;
static uint16_t bmp_dig_P1;
static int16_t  bmp_dig_P2, bmp_dig_P3, bmp_dig_P4, bmp_dig_P5;
static int16_t  bmp_dig_P6, bmp_dig_P7, bmp_dig_P8, bmp_dig_P9;
static int32_t  bmp_t_fine;

/* ---- SD card state ---- */
static volatile uint8_t sd_initialized = 0;

/* ---- Task handles ---- */
osThreadId_t taskIMU_Handle;
osThreadId_t taskBMP_Handle;
osThreadId_t taskDisplay_Handle;
osThreadId_t taskTelemetry_Handle;
osThreadId_t taskAlarm_Handle;
osThreadId_t taskSDLog_Handle;

/* ---- Task attributes ---- */
const osThreadAttr_t taskIMU_attr = {
  .name = "Task_IMU",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

const osThreadAttr_t taskBMP_attr = {
  .name = "Task_BMP",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t taskDisplay_attr = {
  .name = "Task_Display",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t taskTelemetry_attr = {
  .name = "Task_Telemetry",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

const osThreadAttr_t taskAlarm_attr = {
  .name = "Task_Alarm",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

const osThreadAttr_t taskSDLog_attr = {
  .name = "Task_SDLog",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

/* ---- I2C mutex (shared between IMU, BMP, OLED) ---- */
osMutexId_t mutexI2C_Handle;
const osMutexAttr_t mutexI2C_attr = {
  .name = "mutexI2C"
};

/* ---- SPI mutex (SD card) ---- */
osMutexId_t mutexSPI_Handle;
const osMutexAttr_t mutexSPI_attr = {
  .name = "mutexSPI"
};

/* ---- Screen navigation ---- */
static volatile uint8_t current_screen = 0;
#define SCREEN_COUNT 4  /* IMU / BMP / ALARM / STATUS */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void Task_IMU(void *argument);
void Task_BMP(void *argument);
void Task_Display(void *argument);
void Task_Telemetry(void *argument);
void Task_Alarm(void *argument);
void Task_SDLog(void *argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ---- BMP280 compensation (datasheet formulas) ---- */
static float BMP280_CompensateTemp(int32_t adc_T)
{
  float var1 = (((float)adc_T) / 16384.0f - ((float)bmp_dig_T1) / 1024.0f) * ((float)bmp_dig_T2);
  float var2 = ((((float)adc_T) / 131072.0f - ((float)bmp_dig_T1) / 8192.0f)
              * (((float)adc_T) / 131072.0f - ((float)bmp_dig_T1) / 8192.0f)) * ((float)bmp_dig_T3);
  bmp_t_fine = (int32_t)(var1 + var2);
  return (var1 + var2) / 5120.0f;
}

static float BMP280_CompensatePressure(int32_t adc_P)
{
  float var1 = ((float)bmp_t_fine / 2.0f) - 64000.0f;
  float var2 = var1 * var1 * ((float)bmp_dig_P6) / 32768.0f;
  var2 = var2 + var1 * ((float)bmp_dig_P5) * 2.0f;
  var2 = (var2 / 4.0f) + (((float)bmp_dig_P4) * 65536.0f);
  var1 = (((float)bmp_dig_P3) * var1 * var1 / 524288.0f + ((float)bmp_dig_P2) * var1) / 524288.0f;
  var1 = (1.0f + var1 / 32768.0f) * ((float)bmp_dig_P1);
  if (var1 == 0.0f) return 0.0f;
  float p = 1048576.0f - (float)adc_P;
  p = (p - (var2 / 4096.0f)) * 6250.0f / var1;
  var1 = ((float)bmp_dig_P9) * p * p / 2147483648.0f;
  var2 = p * ((float)bmp_dig_P8) / 32768.0f;
  p = p + (var1 + var2 + ((float)bmp_dig_P7)) / 16.0f;
  return p / 100.0f;
}

/* ---- Read a specific ADC channel ---- */
static uint16_t ADC_ReadChannel(uint32_t channel)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = channel;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 10);
  uint16_t val = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);
  return val;
}

/* ---- Buzzer helpers ---- */
static void Buzzer_Beep(uint16_t on_ms, uint16_t off_ms, uint8_t count)
{
  for (uint8_t i = 0; i < count; i++)
  {
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
    osDelay(on_ms);
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
    if (i < count - 1) osDelay(off_ms);
  }
}

/* ---- SD card SPI helpers ---- */
static void SD_CS_Low(void)
{
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);
}

static void SD_CS_High(void)
{
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
}

static uint8_t SD_SPI_Byte(uint8_t data)
{
  uint8_t rx;
  HAL_SPI_TransmitReceive(&hspi1, &data, &rx, 1, HAL_MAX_DELAY);
  return rx;
}

static uint8_t SD_SendCmd(uint8_t cmd, uint32_t arg)
{
  uint8_t crc = 0xFF;
  if (cmd == SD_CMD0) crc = 0x95;  /* Valid CRC for CMD0 */
  if (cmd == SD_CMD8) crc = 0x87;  /* Valid CRC for CMD8 */

  SD_SPI_Byte(0xFF);
  SD_SPI_Byte(0x40 | cmd);
  SD_SPI_Byte((arg >> 24) & 0xFF);
  SD_SPI_Byte((arg >> 16) & 0xFF);
  SD_SPI_Byte((arg >> 8) & 0xFF);
  SD_SPI_Byte(arg & 0xFF);
  SD_SPI_Byte(crc);

  /* Wait for response (up to 8 bytes) */
  uint8_t res;
  for (uint8_t i = 0; i < 8; i++)
  {
    res = SD_SPI_Byte(0xFF);
    if (res != 0xFF) break;
  }
  return res;
}

static uint8_t SD_Init(void)
{
  /* Send 80 clock pulses with CS high */
  SD_CS_High();
  for (uint8_t i = 0; i < 10; i++) SD_SPI_Byte(0xFF);

  /* CMD0: go to idle state */
  SD_CS_Low();
  uint8_t res = SD_SendCmd(SD_CMD0, 0);
  SD_CS_High();
  SD_SPI_Byte(0xFF);
  if (res != 0x01) return 1;

  /* CMD8: check voltage range */
  SD_CS_Low();
  res = SD_SendCmd(SD_CMD8, 0x000001AA);
  if (res == 0x01)
  {
    /* Read 4 bytes of R7 response */
    for (uint8_t i = 0; i < 4; i++) SD_SPI_Byte(0xFF);
  }
  SD_CS_High();
  SD_SPI_Byte(0xFF);

  /* ACMD41: initialize card */
  uint16_t retry = 0;
  do {
    SD_CS_Low();
    SD_SendCmd(SD_CMD55, 0);
    SD_CS_High();
    SD_SPI_Byte(0xFF);

    SD_CS_Low();
    res = SD_SendCmd(SD_ACMD41, 0x40000000);
    SD_CS_High();
    SD_SPI_Byte(0xFF);

    retry++;
    if (retry > 500) return 2;
    osDelay(10);
  } while (res != 0x00);

  /* CMD16: set block length to 512 */
  SD_CS_Low();
  res = SD_SendCmd(SD_CMD16, SD_BLOCK_SIZE);
  SD_CS_High();
  SD_SPI_Byte(0xFF);
  if (res != 0x00) return 3;

  return 0;  /* Success */
}

static uint8_t SD_WriteBlock(uint32_t block_addr, const uint8_t *data)
{
  SD_CS_Low();

  /* CMD24: write single block */
  uint8_t res = SD_SendCmd(SD_CMD24, block_addr);
  if (res != 0x00)
  {
    SD_CS_High();
    return 1;
  }

  /* Send start token */
  SD_SPI_Byte(0xFF);
  SD_SPI_Byte(0xFE);

  /* Send 512 bytes of data */
  for (uint16_t i = 0; i < SD_BLOCK_SIZE; i++)
  {
    SD_SPI_Byte(data[i]);
  }

  /* Send dummy CRC */
  SD_SPI_Byte(0xFF);
  SD_SPI_Byte(0xFF);

  /* Read data response */
  res = SD_SPI_Byte(0xFF);
  if ((res & 0x1F) != 0x05)
  {
    SD_CS_High();
    return 2;
  }

  /* Wait for card to finish writing */
  uint16_t timeout = 0;
  while (SD_SPI_Byte(0xFF) == 0x00)
  {
    timeout++;
    if (timeout > 5000)
    {
      SD_CS_High();
      return 3;
    }
  }

  SD_CS_High();
  SD_SPI_Byte(0xFF);
  return 0;
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  mutexI2C_Handle   = osMutexNew(&mutexI2C_attr);
  mutexIMU_Handle   = osMutexNew(&mutexIMU_attr);
  mutexBMP_Handle   = osMutexNew(&mutexBMP_attr);
  mutexAlarm_Handle = osMutexNew(&mutexAlarm_attr);
  mutexSPI_Handle   = osMutexNew(&mutexSPI_attr);
  /* USER CODE END RTOS_MUTEX */

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  taskIMU_Handle       = osThreadNew(Task_IMU,       NULL, &taskIMU_attr);
  taskBMP_Handle       = osThreadNew(Task_BMP,       NULL, &taskBMP_attr);
  taskDisplay_Handle   = osThreadNew(Task_Display,   NULL, &taskDisplay_attr);
  taskTelemetry_Handle = osThreadNew(Task_Telemetry, NULL, &taskTelemetry_attr);
  taskAlarm_Handle     = osThreadNew(Task_Alarm,     NULL, &taskAlarm_attr);
  taskSDLog_Handle     = osThreadNew(Task_SDLog,     NULL, &taskSDLog_attr);
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
      /* USER CODE BEGIN WHILE */


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

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, BUZZER_Pin|SD_CS_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = BUZZER_Pin|SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* ===================================================================
 *  Task_IMU — Read MPU-6050 (accelerometer + gyroscope)
 *  Period: 50ms (20 Hz)
 * =================================================================== */
void Task_IMU(void *argument)
{
  uint8_t reg;

  /* ---- Init MPU-6050 with device reset ---- */
  osMutexAcquire(mutexI2C_Handle, osWaitForever);
  HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x75, 1, &reg, 1, HAL_MAX_DELAY);
  reg = 0x80;
  HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU_REG_PWR_MGMT1, 1, &reg, 1, HAL_MAX_DELAY);
  osMutexRelease(mutexI2C_Handle);

  osDelay(100);

  osMutexAcquire(mutexI2C_Handle, osWaitForever);
  reg = 0x00;
  HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU_REG_PWR_MGMT1, 1, &reg, 1, HAL_MAX_DELAY);
  HAL_Delay(50);
  reg = 0x00;
  HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU_REG_ACCEL_CFG, 1, &reg, 1, HAL_MAX_DELAY);
  reg = 0x00;
  HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU_REG_GYRO_CFG, 1, &reg, 1, HAL_MAX_DELAY);
  osMutexRelease(mutexI2C_Handle);

  osDelay(100);

  for (;;)
  {
    uint8_t buf[14];

    osMutexAcquire(mutexI2C_Handle, osWaitForever);
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU_REG_DATA_START, 1, buf, 14, HAL_MAX_DELAY);
    osMutexRelease(mutexI2C_Handle);

    int16_t raw_ax = (int16_t)((uint16_t)buf[0]  << 8 | buf[1]);
    int16_t raw_ay = (int16_t)((uint16_t)buf[2]  << 8 | buf[3]);
    int16_t raw_az = (int16_t)((uint16_t)buf[4]  << 8 | buf[5]);
    int16_t raw_t  = (int16_t)((uint16_t)buf[6]  << 8 | buf[7]);
    int16_t raw_gx = (int16_t)((uint16_t)buf[8]  << 8 | buf[9]);
    int16_t raw_gy = (int16_t)((uint16_t)buf[10] << 8 | buf[11]);
    int16_t raw_gz = (int16_t)((uint16_t)buf[12] << 8 | buf[13]);

    osMutexAcquire(mutexIMU_Handle, osWaitForever);
    imu_data.accel_x = raw_ax / 16384.0f;
    imu_data.accel_y = raw_ay / 16384.0f;
    imu_data.accel_z = raw_az / 16384.0f;
    imu_data.gyro_x  = raw_gx / 131.0f;
    imu_data.gyro_y  = raw_gy / 131.0f;
    imu_data.gyro_z  = raw_gz / 131.0f;
    imu_data.temp    = (raw_t / 340.0f) + 36.53f;
    osMutexRelease(mutexIMU_Handle);

    osDelay(50);
  }
}

/* ===================================================================
 *  Task_BMP — Read BMP280 (temperature + pressure + altitude)
 *  Period: 500ms (2 Hz)
 * =================================================================== */
void Task_BMP(void *argument)
{
  uint8_t calib[26];
  uint8_t reg;

  osMutexAcquire(mutexI2C_Handle, osWaitForever);
  HAL_I2C_Mem_Read(&hi2c1, BMP280_ADDR, BMP_REG_CALIB, 1, calib, 26, HAL_MAX_DELAY);
  osMutexRelease(mutexI2C_Handle);

  bmp_dig_T1 = (uint16_t)(calib[1]  << 8 | calib[0]);
  bmp_dig_T2 = (int16_t) (calib[3]  << 8 | calib[2]);
  bmp_dig_T3 = (int16_t) (calib[5]  << 8 | calib[4]);
  bmp_dig_P1 = (uint16_t)(calib[7]  << 8 | calib[6]);
  bmp_dig_P2 = (int16_t) (calib[9]  << 8 | calib[8]);
  bmp_dig_P3 = (int16_t) (calib[11] << 8 | calib[10]);
  bmp_dig_P4 = (int16_t) (calib[13] << 8 | calib[12]);
  bmp_dig_P5 = (int16_t) (calib[15] << 8 | calib[14]);
  bmp_dig_P6 = (int16_t) (calib[17] << 8 | calib[16]);
  bmp_dig_P7 = (int16_t) (calib[19] << 8 | calib[18]);
  bmp_dig_P8 = (int16_t) (calib[21] << 8 | calib[20]);
  bmp_dig_P9 = (int16_t) (calib[23] << 8 | calib[22]);

  osMutexAcquire(mutexI2C_Handle, osWaitForever);
  reg = 0x00;
  HAL_I2C_Mem_Write(&hi2c1, BMP280_ADDR, BMP_REG_CONFIG, 1, &reg, 1, HAL_MAX_DELAY);
  reg = 0x27;
  HAL_I2C_Mem_Write(&hi2c1, BMP280_ADDR, BMP_REG_CTRL_MEAS, 1, &reg, 1, HAL_MAX_DELAY);
  osMutexRelease(mutexI2C_Handle);

  osDelay(100);

  for (;;)
  {
    uint8_t data[6];

    osMutexAcquire(mutexI2C_Handle, osWaitForever);
    HAL_I2C_Mem_Read(&hi2c1, BMP280_ADDR, BMP_REG_DATA_START, 1, data, 6, HAL_MAX_DELAY);
    osMutexRelease(mutexI2C_Handle);

    int32_t adc_P = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | ((int32_t)data[2] >> 4);
    int32_t adc_T = ((int32_t)data[3] << 12) | ((int32_t)data[4] << 4) | ((int32_t)data[5] >> 4);

    float temp = BMP280_CompensateTemp(adc_T);
    float pres = BMP280_CompensatePressure(adc_P);
    float alt  = 44330.0f * (1.0f - powf(pres / 1013.25f, 0.1903f));

    osMutexAcquire(mutexBMP_Handle, osWaitForever);
    bmp_data.temperature = temp;
    bmp_data.pressure    = pres;
    bmp_data.altitude    = alt;
    osMutexRelease(mutexBMP_Handle);

    osDelay(500);
  }
}

/* ===================================================================
 *  Task_Alarm — Check sensor thresholds and trigger buzzer
 *  Period: 500ms (2 Hz)
 *  Short beeps = warning, long beep = critical
 * =================================================================== */
void Task_Alarm(void *argument)
{
  osDelay(2000);  /* Let sensors stabilize */

  for (;;)
  {
    /* Local copies */
    IMU_Data_t imu_local;
    osMutexAcquire(mutexIMU_Handle, osWaitForever);
    imu_local = imu_data;
    osMutexRelease(mutexIMU_Handle);

    BMP_Data_t bmp_local;
    osMutexAcquire(mutexBMP_Handle, osWaitForever);
    bmp_local = bmp_data;
    osMutexRelease(mutexBMP_Handle);

    /* Evaluate alarm conditions */
    uint8_t tilt = 0, temp_hi = 0, pres_ab = 0, alt_crit = 0;

    /* Tilt: accel X or Y exceeds threshold */
    if (fabsf(imu_local.accel_x) > ALARM_TILT_G ||
        fabsf(imu_local.accel_y) > ALARM_TILT_G)
    {
      tilt = 1;
    }

    /* High temperature */
    if (bmp_local.temperature > ALARM_TEMP_HIGH)
    {
      temp_hi = 1;
    }

    /* Abnormal pressure */
    if (bmp_local.pressure < ALARM_PRES_LOW ||
        bmp_local.pressure > ALARM_PRES_HIGH)
    {
      pres_ab = 1;
    }

    /* Critical altitude */
    if (bmp_local.altitude > ALARM_ALT_HIGH ||
        bmp_local.altitude < ALARM_ALT_LOW)
    {
      alt_crit = 1;
    }

    /* Update shared alarm flags */
    osMutexAcquire(mutexAlarm_Handle, osWaitForever);
    alarm_flags.tilt          = tilt;
    alarm_flags.temp_high     = temp_hi;
    alarm_flags.pres_abnormal = pres_ab;
    alarm_flags.alt_critical  = alt_crit;
    osMutexRelease(mutexAlarm_Handle);

    /* Sound buzzer based on severity */
    uint8_t total_alerts = tilt + temp_hi + pres_ab + alt_crit;

    if (alt_crit || pres_ab)
    {
      /* Critical: long beep */
      Buzzer_Beep(500, 200, 1);
    }
    else if (total_alerts > 0)
    {
      /* Warning: short beeps (count = number of active alerts) */
      Buzzer_Beep(100, 100, total_alerts);
    }

    osDelay(500);
  }
}

/* ===================================================================
 *  Task_Display — OLED SSD1306 + dual-axis joystick
 *  Period: 100ms (10 Hz)
 *  4 screens: [IMU] [BMP280] [ALARM] [STATUS]
 * =================================================================== */
void Task_Display(void *argument)
{
  char line[22];
  uint8_t joy_locked = 0;

  osMutexAcquire(mutexI2C_Handle, osWaitForever);
  ssd1306_Init();
  osMutexRelease(mutexI2C_Handle);

  osDelay(100);

  for (;;)
  {
    uint16_t joy_x = ADC_ReadChannel(ADC_CHANNEL_0);
    uint16_t joy_y = ADC_ReadChannel(ADC_CHANNEL_4);

    if (!joy_locked)
    {
      if (joy_x < JOY_LEFT_THRESH)
      {
        if (current_screen < SCREEN_COUNT - 1) current_screen++;
        joy_locked = 1;
      }
      else if (joy_x > JOY_RIGHT_THRESH)
      {
        if (current_screen > 0) current_screen--;
        joy_locked = 1;
      }
    }
    else
    {
      if (joy_x > JOY_LEFT_THRESH && joy_x < JOY_RIGHT_THRESH)
      {
        joy_locked = 0;
      }
    }

    /* Local copies */
    IMU_Data_t imu_local;
    osMutexAcquire(mutexIMU_Handle, osWaitForever);
    imu_local = imu_data;
    osMutexRelease(mutexIMU_Handle);

    BMP_Data_t bmp_local;
    osMutexAcquire(mutexBMP_Handle, osWaitForever);
    bmp_local = bmp_data;
    osMutexRelease(mutexBMP_Handle);

    Alarm_Flags_t alarm_local;
    osMutexAcquire(mutexAlarm_Handle, osWaitForever);
    alarm_local = alarm_flags;
    osMutexRelease(mutexAlarm_Handle);

    /* Render */
    osMutexAcquire(mutexI2C_Handle, osWaitForever);
    ssd1306_Fill(Black);

    switch (current_screen)
    {
      case 0: /* ---- IMU SCREEN ---- */
        ssd1306_SetCursor(0, 0);
        ssd1306_WriteString("[IMU]", Font_7x10, White);

        sprintf(line, "AX:%.2f", imu_local.accel_x);
        ssd1306_SetCursor(0, 14);
        ssd1306_WriteString(line, Font_7x10, White);

        sprintf(line, "AY:%.2f", imu_local.accel_y);
        ssd1306_SetCursor(0, 26);
        ssd1306_WriteString(line, Font_7x10, White);

        sprintf(line, "AZ:%.2f", imu_local.accel_z);
        ssd1306_SetCursor(0, 38);
        ssd1306_WriteString(line, Font_7x10, White);

        sprintf(line, "T:%.1fC", imu_local.temp);
        ssd1306_SetCursor(0, 50);
        ssd1306_WriteString(line, Font_7x10, White);
        break;

      case 1: /* ---- BMP280 SCREEN ---- */
        ssd1306_SetCursor(0, 0);
        ssd1306_WriteString("[BMP280]", Font_7x10, White);

        sprintf(line, "T:%.1f C", bmp_local.temperature);
        ssd1306_SetCursor(0, 14);
        ssd1306_WriteString(line, Font_7x10, White);

        sprintf(line, "P:%.1f hPa", bmp_local.pressure);
        ssd1306_SetCursor(0, 28);
        ssd1306_WriteString(line, Font_7x10, White);

        sprintf(line, "Alt:%.1f m", bmp_local.altitude);
        ssd1306_SetCursor(0, 42);
        ssd1306_WriteString(line, Font_7x10, White);
        break;

      case 2: /* ---- ALARM SCREEN ---- */
        ssd1306_SetCursor(0, 0);
        ssd1306_WriteString("[ALARM]", Font_7x10, White);

        sprintf(line, "TILT: %s", alarm_local.tilt ? "! ALERT" : "OK");
        ssd1306_SetCursor(0, 14);
        ssd1306_WriteString(line, Font_7x10, White);

        sprintf(line, "TEMP: %s", alarm_local.temp_high ? "! HIGH" : "OK");
        ssd1306_SetCursor(0, 26);
        ssd1306_WriteString(line, Font_7x10, White);

        sprintf(line, "PRES: %s", alarm_local.pres_abnormal ? "! ABNORM" : "OK");
        ssd1306_SetCursor(0, 38);
        ssd1306_WriteString(line, Font_7x10, White);

        sprintf(line, "ALT:  %s", alarm_local.alt_critical ? "! CRIT" : "OK");
        ssd1306_SetCursor(0, 50);
        ssd1306_WriteString(line, Font_7x10, White);
        break;

      case 3: /* ---- STATUS SCREEN ---- */
        ssd1306_SetCursor(0, 0);
        ssd1306_WriteString("[STATUS]", Font_7x10, White);

        ssd1306_SetCursor(0, 14);
        ssd1306_WriteString("IMU:OK  BMP:OK", Font_7x10, White);

        sprintf(line, "SD:%s", sd_initialized ? "OK" : "--");
        ssd1306_SetCursor(0, 28);
        ssd1306_WriteString(line, Font_7x10, White);

        sprintf(line, "JX:%d JY:%d", joy_x, joy_y);
        ssd1306_SetCursor(0, 42);
        ssd1306_WriteString(line, Font_7x10, White);
        break;
    }

    /* Page indicator (bottom right, all screens) */
    sprintf(line, "%d/%d", current_screen + 1, SCREEN_COUNT);
    ssd1306_SetCursor(100, 54);
    ssd1306_WriteString(line, Font_7x10, White);

    ssd1306_UpdateScreen();
    osMutexRelease(mutexI2C_Handle);

    osDelay(100);
  }
}

/* ===================================================================
 *  Task_Telemetry — Send CSV data over UART2
 *  Period: 200ms (5 Hz)
 * =================================================================== */
void Task_Telemetry(void *argument)
{
  static char tx[128];

  const char *header = "AX,AY,AZ,GX,GY,GZ,T_IMU,T_BMP,P,ALT,JX,JY,ALARMS\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)header, strlen(header), HAL_MAX_DELAY);

  for (;;)
  {
    IMU_Data_t imu_local;
    osMutexAcquire(mutexIMU_Handle, osWaitForever);
    imu_local = imu_data;
    osMutexRelease(mutexIMU_Handle);

    BMP_Data_t bmp_local;
    osMutexAcquire(mutexBMP_Handle, osWaitForever);
    bmp_local = bmp_data;
    osMutexRelease(mutexBMP_Handle);

    Alarm_Flags_t alarm_local;
    osMutexAcquire(mutexAlarm_Handle, osWaitForever);
    alarm_local = alarm_flags;
    osMutexRelease(mutexAlarm_Handle);

    uint16_t joy_x = ADC_ReadChannel(ADC_CHANNEL_0);
    uint16_t joy_y = ADC_ReadChannel(ADC_CHANNEL_4);

    /* Alarm bitfield: bit0=tilt, bit1=temp, bit2=pres, bit3=alt */
    uint8_t alarm_bits = (alarm_local.tilt)
                       | (alarm_local.temp_high << 1)
                       | (alarm_local.pres_abnormal << 2)
                       | (alarm_local.alt_critical << 3);

    sprintf(tx, "%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,",
      imu_local.accel_x, imu_local.accel_y, imu_local.accel_z,
      imu_local.gyro_x,  imu_local.gyro_y,  imu_local.gyro_z);
    HAL_UART_Transmit(&huart2, (uint8_t*)tx, strlen(tx), HAL_MAX_DELAY);

    sprintf(tx, "%.1f,%.1f,%.1f,%.1f,",
      imu_local.temp,
      bmp_local.temperature, bmp_local.pressure, bmp_local.altitude);
    HAL_UART_Transmit(&huart2, (uint8_t*)tx, strlen(tx), HAL_MAX_DELAY);

    sprintf(tx, "%d,%d,%d\r\n", joy_x, joy_y, alarm_bits);
    HAL_UART_Transmit(&huart2, (uint8_t*)tx, strlen(tx), HAL_MAX_DELAY);

    osDelay(200);
  }
}

/* ===================================================================
 *  Task_SDLog — Log raw data to SD card via SPI
 *  Period: 1000ms (1 Hz)
 *  Each 512-byte block contains multiple 40-byte log packets
 *  Packet format (40 bytes):
 *    [0-3]   sync marker (0xAA55AA55)
 *    [4-7]   timestamp (ms, uint32_t)
 *    [8-11]  accel_x (float)
 *    [12-15] accel_y (float)
 *    [16-19] accel_z (float)
 *    [20-23] gyro_x (float)
 *    [24-27] gyro_y (float)
 *    [28-31] gyro_z (float)
 *    [32-35] bmp_temp (float)
 *    [36-39] bmp_pres (float)
 *    [40-43] bmp_alt (float) -- NOTE: extends to 44 bytes
 *    Actually using 48 bytes per packet for alignment:
 *    [44-45] alarm_bits (uint16_t)
 *    [46-47] padding
 *  Adjusted: LOG_PACKET_SIZE = 48, fits 10 packets per block
 * =================================================================== */
void Task_SDLog(void *argument)
{
  static uint8_t block_buf[SD_BLOCK_SIZE];
  uint32_t block_addr = 0;
  uint16_t buf_offset = 0;

  osDelay(3000);  /* Wait for sensors to be ready */

  /* Try to init SD card */
  osMutexAcquire(mutexSPI_Handle, osWaitForever);
  uint8_t result = SD_Init();
  osMutexRelease(mutexSPI_Handle);

  if (result == 0)
  {
    sd_initialized = 1;
  }

  for (;;)
  {
    if (!sd_initialized)
    {
      /* Retry init every 5 seconds */
      osDelay(5000);
      osMutexAcquire(mutexSPI_Handle, osWaitForever);
      uint8_t res = SD_Init();
      osMutexRelease(mutexSPI_Handle);
      if (res == 0) sd_initialized = 1;
      continue;
    }

    /* Gather data */
    IMU_Data_t imu_local;
    osMutexAcquire(mutexIMU_Handle, osWaitForever);
    imu_local = imu_data;
    osMutexRelease(mutexIMU_Handle);

    BMP_Data_t bmp_local;
    osMutexAcquire(mutexBMP_Handle, osWaitForever);
    bmp_local = bmp_data;
    osMutexRelease(mutexBMP_Handle);

    Alarm_Flags_t alarm_local;
    osMutexAcquire(mutexAlarm_Handle, osWaitForever);
    alarm_local = alarm_flags;
    osMutexRelease(mutexAlarm_Handle);

    uint8_t alarm_bits = (alarm_local.tilt)
                       | (alarm_local.temp_high << 1)
                       | (alarm_local.pres_abnormal << 2)
                       | (alarm_local.alt_critical << 3);

    /* Build packet (48 bytes) */
    uint32_t timestamp = HAL_GetTick();
    uint32_t sync = LOG_SYNC_MARKER;

    memcpy(&block_buf[buf_offset +  0], &sync, 4);
    memcpy(&block_buf[buf_offset +  4], &timestamp, 4);
    memcpy(&block_buf[buf_offset +  8], &imu_local.accel_x, 4);
    memcpy(&block_buf[buf_offset + 12], &imu_local.accel_y, 4);
    memcpy(&block_buf[buf_offset + 16], &imu_local.accel_z, 4);
    memcpy(&block_buf[buf_offset + 20], &imu_local.gyro_x, 4);
    memcpy(&block_buf[buf_offset + 24], &imu_local.gyro_y, 4);
    memcpy(&block_buf[buf_offset + 28], &imu_local.gyro_z, 4);
    memcpy(&block_buf[buf_offset + 32], &bmp_local.temperature, 4);
    memcpy(&block_buf[buf_offset + 36], &bmp_local.pressure, 4);
    memcpy(&block_buf[buf_offset + 40], &bmp_local.altitude, 4);
    memcpy(&block_buf[buf_offset + 44], &alarm_bits, 1);
    block_buf[buf_offset + 45] = 0;
    block_buf[buf_offset + 46] = 0;
    block_buf[buf_offset + 47] = 0;

    buf_offset += 48;

    /* When buffer is full (10 packets = 480 bytes), pad and write */
    if (buf_offset >= 480)
    {
      /* Zero-fill remaining bytes */
      memset(&block_buf[buf_offset], 0, SD_BLOCK_SIZE - buf_offset);

      /* Write block to SD */
      osMutexAcquire(mutexSPI_Handle, osWaitForever);
      uint8_t wr = SD_WriteBlock(block_addr, block_buf);
      osMutexRelease(mutexSPI_Handle);

      if (wr != 0)
      {
        /* Write failed — try reinit next cycle */
        sd_initialized = 0;
      }

      block_addr++;
      buf_offset = 0;
    }

    osDelay(1000);
  }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Heartbeat LED — confirms scheduler is running */
  for(;;)
  {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    osDelay(500);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
