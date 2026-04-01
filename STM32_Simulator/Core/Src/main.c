/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Satellite Simulator — Main program
  * @description    : FreeRTOS-based satellite simulator on Nucleo-F446RE
  *                   7 tasks: IMU, BMP, Display, Telemetry, Alarm, SDLog, ESP32
  *                   4 operational modes: NOMINAL, SAFE, SCIENCE, ERROR
  *                   ESP32 WiFi bridge for remote monitoring and control
  ******************************************************************************
  */
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

/**
 * @brief IMU sensor data (MPU-6050)
 */
typedef struct {
    float accel_x;    /**< Accelerometer X axis (g)    */
    float accel_y;    /**< Accelerometer Y axis (g)    */
    float accel_z;    /**< Accelerometer Z axis (g)    */
    float gyro_x;     /**< Gyroscope X axis (deg/s)    */
    float gyro_y;     /**< Gyroscope Y axis (deg/s)    */
    float gyro_z;     /**< Gyroscope Z axis (deg/s)    */
    float temp;       /**< Die temperature (C)         */
} IMU_Data_t;

/**
 * @brief Barometric sensor data (BMP280)
 */
typedef struct {
    float temperature;  /**< Temperature (C)    */
    float pressure;     /**< Pressure (hPa)     */
    float altitude;     /**< Altitude (m)       */
} BMP_Data_t;

/**
 * @brief Alarm condition flags
 */
typedef struct {
    uint8_t tilt;           /**< Tilt threshold exceeded       */
    uint8_t temp_high;      /**< Temperature above limit       */
    uint8_t pres_abnormal;  /**< Pressure out of range         */
    uint8_t alt_critical;   /**< Altitude out of safe range    */
} Alarm_Flags_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ---------- I2C Addresses ---------- */
#define MPU6050_ADDR        (0x68 << 1)
#define BMP280_ADDR         (0x76 << 1)

/* ---------- MPU-6050 Registers ---------- */
#define MPU_REG_WHO_AM_I    0x75
#define MPU_REG_PWR_MGMT1   0x6B
#define MPU_REG_ACCEL_CFG   0x1C
#define MPU_REG_GYRO_CFG    0x1B
#define MPU_REG_DATA_START  0x3B

/* ---------- BMP280 Registers ---------- */
#define BMP_REG_CTRL_MEAS   0xF4
#define BMP_REG_CONFIG      0xF5
#define BMP_REG_CALIB       0x88
#define BMP_REG_DATA_START  0xF7

/* ---------- Joystick Thresholds ---------- */
#define JOY_LEFT_THRESH     1000
#define JOY_RIGHT_THRESH    3000

/* ---------- Satellite Modes ---------- */
#define MODE_NOMINAL        0
#define MODE_SAFE           1
#define MODE_SCIENCE        2
#define MODE_ERROR_LOW      3
#define MODE_ERROR_HIGH     4

/* ---------- Error Acknowledgment ---------- */
#define ERROR_ACK_SEQ_LEN   3   /* Joystick sequence: L -> R -> L */

/* ---------- SD Card ---------- */
#define SD_BLOCK_SIZE       512
#define LOG_SYNC_MARKER     0xAA55AA55

/* ---------- ESP32 UART ---------- */
#define ESP_RX_BUF_SIZE     128

/* ---------- Display ---------- */
#define SCREEN_COUNT        5   /* IMU / BMP / ALARM / MODE / STATUS */

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

/* ========== Shared Sensor Data ========== */
IMU_Data_t   imu_data   = {0};
BMP_Data_t   bmp_data   = {0};
Alarm_Flags_t alarm_flags = {0};

/* ========== Mutexes ========== */
osMutexId_t mutexIMU_Handle;
osMutexId_t mutexBMP_Handle;
osMutexId_t mutexAlarm_Handle;
osMutexId_t mutexI2C_Handle;
osMutexId_t mutexSPI_Handle;

const osMutexAttr_t mutexIMU_attr   = { .name = "mutexIMU"   };
const osMutexAttr_t mutexBMP_attr   = { .name = "mutexBMP"   };
const osMutexAttr_t mutexAlarm_attr = { .name = "mutexAlarm" };
const osMutexAttr_t mutexI2C_attr   = { .name = "mutexI2C"   };
const osMutexAttr_t mutexSPI_attr   = { .name = "mutexSPI"   };

/* ========== Configurable Alarm Thresholds ========== */
static volatile float alarm_tilt_g    = 1.5f;
static volatile float alarm_temp_high = 60.0f;
static volatile float alarm_pres_low  = 900.0f;
static volatile float alarm_pres_high = 1100.0f;
static volatile float alarm_alt_high  = 3000.0f;
static volatile float alarm_alt_low   = -200.0f;

/* ========== System State ========== */
static volatile uint8_t sat_mode       = MODE_NOMINAL;
static volatile uint8_t buzzer_enabled = 1;
static volatile uint8_t error_ack_step = 0;
static volatile uint8_t sd_initialized = 0;
static volatile uint8_t current_screen = 0;

/* ========== BMP280 Calibration Coefficients ========== */
static uint16_t bmp_dig_T1;
static int16_t  bmp_dig_T2, bmp_dig_T3;
static uint16_t bmp_dig_P1;
static int16_t  bmp_dig_P2, bmp_dig_P3, bmp_dig_P4, bmp_dig_P5;
static int16_t  bmp_dig_P6, bmp_dig_P7, bmp_dig_P8, bmp_dig_P9;
static int32_t  bmp_t_fine;

/* ========== ESP32 UART Reception ========== */
static uint8_t          esp_rx_byte;
static char             esp_rx_buf[ESP_RX_BUF_SIZE];
static volatile uint8_t esp_rx_idx    = 0;
static volatile uint8_t esp_cmd_ready = 0;
static char             esp_cmd[ESP_RX_BUF_SIZE];

/* ========== Task Handles ========== */
osThreadId_t taskIMU_Handle;
osThreadId_t taskBMP_Handle;
osThreadId_t taskDisplay_Handle;
osThreadId_t taskTelemetry_Handle;
osThreadId_t taskAlarm_Handle;
osThreadId_t taskSDLog_Handle;
osThreadId_t taskESP32_Handle;

/* ========== Task Attributes ========== */
const osThreadAttr_t taskIMU_attr = {
    .name = "Task_IMU",   .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityAboveNormal
};
const osThreadAttr_t taskBMP_attr = {
    .name = "Task_BMP",   .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityNormal
};
const osThreadAttr_t taskDisplay_attr = {
    .name = "Task_Display", .stack_size = 512 * 4,
    .priority = (osPriority_t) osPriorityNormal
};
const osThreadAttr_t taskTelemetry_attr = {
    .name = "Task_Telemetry", .stack_size = 512 * 4,
    .priority = (osPriority_t) osPriorityBelowNormal
};
const osThreadAttr_t taskAlarm_attr = {
    .name = "Task_Alarm",  .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityAboveNormal
};
const osThreadAttr_t taskSDLog_attr = {
    .name = "Task_SDLog",  .stack_size = 512 * 4,
    .priority = (osPriority_t) osPriorityBelowNormal
};
const osThreadAttr_t taskESP32_attr = {
    .name = "Task_ESP32",  .stack_size = 512 * 4,
    .priority = (osPriority_t) osPriorityNormal
};

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
void Task_ESP32(void *argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ================================================================
 *                    BMP280 Compensation
 * ================================================================ */

/**
 * @brief Compensate raw temperature reading using factory calibration
 * @param adc_T Raw 20-bit temperature value
 * @return Temperature in degrees Celsius
 */
static float BMP280_CompensateTemp(int32_t adc_T)
{
    float var1 = (((float)adc_T) / 16384.0f - ((float)bmp_dig_T1) / 1024.0f)
                 * ((float)bmp_dig_T2);
    float var2 = ((((float)adc_T) / 131072.0f - ((float)bmp_dig_T1) / 8192.0f)
                 * (((float)adc_T) / 131072.0f - ((float)bmp_dig_T1) / 8192.0f))
                 * ((float)bmp_dig_T3);

    bmp_t_fine = (int32_t)(var1 + var2);
    return (var1 + var2) / 5120.0f;
}

/**
 * @brief Compensate raw pressure reading using factory calibration
 * @param adc_P Raw 20-bit pressure value
 * @return Pressure in hPa
 * @note Must call BMP280_CompensateTemp() first to set bmp_t_fine
 */
static float BMP280_CompensatePressure(int32_t adc_P)
{
    float var1 = ((float)bmp_t_fine / 2.0f) - 64000.0f;
    float var2 = var1 * var1 * ((float)bmp_dig_P6) / 32768.0f;
    var2 = var2 + var1 * ((float)bmp_dig_P5) * 2.0f;
    var2 = (var2 / 4.0f) + (((float)bmp_dig_P4) * 65536.0f);
    var1 = (((float)bmp_dig_P3) * var1 * var1 / 524288.0f
           + ((float)bmp_dig_P2) * var1) / 524288.0f;
    var1 = (1.0f + var1 / 32768.0f) * ((float)bmp_dig_P1);

    if (var1 == 0.0f)
        return 0.0f;

    float p = 1048576.0f - (float)adc_P;
    p = (p - (var2 / 4096.0f)) * 6250.0f / var1;
    var1 = ((float)bmp_dig_P9) * p * p / 2147483648.0f;
    var2 = p * ((float)bmp_dig_P8) / 32768.0f;
    p = p + (var1 + var2 + ((float)bmp_dig_P7)) / 16.0f;

    return p / 100.0f;
}

/* ================================================================
 *                    Hardware Helpers
 * ================================================================ */

/**
 * @brief Read a single ADC channel (software triggered, polling)
 * @param channel ADC channel number (e.g. ADC_CHANNEL_0)
 * @return 12-bit ADC value (0–4095)
 */
static uint16_t ADC_ReadChannel(uint32_t channel)
{
    ADC_ChannelConfTypeDef config = {0};
    config.Channel     = channel;
    config.Rank        = 1;
    config.SamplingTime = ADC_SAMPLETIME_3CYCLES;

    HAL_ADC_ConfigChannel(&hadc1, &config);
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint16_t value = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    return value;
}

/**
 * @brief Emit buzzer beeps (respects buzzer_enabled flag)
 * @param on_ms  Duration of each beep in ms
 * @param off_ms Pause between beeps in ms
 * @param count  Number of beeps
 */
static void Buzzer_Beep(uint16_t on_ms, uint16_t off_ms, uint8_t count)
{
    if (!buzzer_enabled)
        return;

    for (uint8_t i = 0; i < count; i++)
    {
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
        osDelay(on_ms);
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
        if (i < count - 1)
            osDelay(off_ms);
    }
}

/* ================================================================
 *                    SD Card (Raw SPI)
 * ================================================================ */

static void SD_CS_Low(void)
{
    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);
}

static void SD_CS_High(void)
{
    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
}

static uint8_t SD_SPI_Byte(uint8_t tx_byte)
{
    uint8_t rx_byte;
    HAL_SPI_TransmitReceive(&hspi1, &tx_byte, &rx_byte, 1, HAL_MAX_DELAY);
    return rx_byte;
}

static uint8_t SD_SendCmd(uint8_t cmd, uint32_t arg)
{
    uint8_t crc = 0xFF;
    if (cmd == 0) crc = 0x95;
    if (cmd == 8) crc = 0x87;

    SD_SPI_Byte(0xFF);
    SD_SPI_Byte(0x40 | cmd);
    SD_SPI_Byte((arg >> 24) & 0xFF);
    SD_SPI_Byte((arg >> 16) & 0xFF);
    SD_SPI_Byte((arg >> 8)  & 0xFF);
    SD_SPI_Byte(arg & 0xFF);
    SD_SPI_Byte(crc);

    uint8_t response;
    for (uint8_t i = 0; i < 8; i++)
    {
        response = SD_SPI_Byte(0xFF);
        if (response != 0xFF)
            break;
    }
    return response;
}

/**
 * @brief Initialize SD card in SPI mode
 * @return 0 on success, non-zero on failure
 */
static uint8_t SD_Init(void)
{
    /* Send 80 clock pulses with CS high */
    SD_CS_High();
    for (uint8_t i = 0; i < 10; i++)
        SD_SPI_Byte(0xFF);

    /* CMD0: Go to idle state */
    SD_CS_Low();
    uint8_t res = SD_SendCmd(0, 0);
    SD_CS_High();
    SD_SPI_Byte(0xFF);
    if (res != 0x01)
        return 1;

    /* CMD8: Check voltage range */
    SD_CS_Low();
    res = SD_SendCmd(8, 0x000001AA);
    if (res == 0x01)
    {
        for (uint8_t i = 0; i < 4; i++)
            SD_SPI_Byte(0xFF);
    }
    SD_CS_High();
    SD_SPI_Byte(0xFF);

    /* ACMD41: Initialize card */
    uint16_t retry = 0;
    do {
        SD_CS_Low();
        SD_SendCmd(55, 0);
        SD_CS_High();
        SD_SPI_Byte(0xFF);

        SD_CS_Low();
        res = SD_SendCmd(41, 0x40000000);
        SD_CS_High();
        SD_SPI_Byte(0xFF);

        retry++;
        if (retry > 500)
            return 2;
        osDelay(10);
    } while (res != 0x00);

    /* CMD16: Set block length */
    SD_CS_Low();
    res = SD_SendCmd(16, SD_BLOCK_SIZE);
    SD_CS_High();
    SD_SPI_Byte(0xFF);
    if (res != 0x00)
        return 3;

    return 0;
}

/**
 * @brief Write a 512-byte block to SD card
 * @param block_addr Block address
 * @param data Pointer to 512 bytes of data
 * @return 0 on success, non-zero on failure
 */
static uint8_t SD_WriteBlock(uint32_t block_addr, const uint8_t *data)
{
    SD_CS_Low();
    uint8_t res = SD_SendCmd(24, block_addr);
    if (res != 0x00)
    {
        SD_CS_High();
        return 1;
    }

    SD_SPI_Byte(0xFF);
    SD_SPI_Byte(0xFE);  /* Start token */

    for (uint16_t i = 0; i < SD_BLOCK_SIZE; i++)
        SD_SPI_Byte(data[i]);

    SD_SPI_Byte(0xFF);  /* Dummy CRC */
    SD_SPI_Byte(0xFF);

    res = SD_SPI_Byte(0xFF);
    if ((res & 0x1F) != 0x05)
    {
        SD_CS_High();
        return 2;
    }

    /* Wait for write to complete */
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

/* ================================================================
 *                    Mode & Command Helpers
 * ================================================================ */

/**
 * @brief Get human-readable mode name
 */
static const char *Mode_Name(uint8_t mode)
{
    switch (mode)
    {
        case MODE_NOMINAL:    return "NOMINAL";
        case MODE_SAFE:       return "SAFE";
        case MODE_SCIENCE:    return "SCIENCE";
        case MODE_ERROR_LOW:  return "ERR_LOW";
        case MODE_ERROR_HIGH: return "ERR_HIGH";
        default:              return "???";
    }
}

/**
 * @brief Parse and execute a command received from ESP32
 *
 * Supported commands:
 *   CMD:MODE:NOMINAL / SAFE / SCIENCE / ERROR_LOW / ERROR_HIGH
 *   CMD:ALARM:TILT:0.8  (and similar for TEMP, PRES_LOW, PRES_HIGH, ALT_HIGH, ALT_LOW)
 *   CMD:DUMP
 */
static void ESP32_ProcessCmd(const char *cmd)
{
    /* ---- Mode change ---- */
    if (strncmp(cmd, "CMD:MODE:", 9) == 0)
    {
        const char *mode_str = cmd + 9;

        if (strcmp(mode_str, "NOMINAL") == 0)
        {
            sat_mode = MODE_NOMINAL;
            buzzer_enabled = 1;
        }
        else if (strcmp(mode_str, "SAFE") == 0)
        {
            sat_mode = MODE_SAFE;
            buzzer_enabled = 0;
        }
        else if (strcmp(mode_str, "SCIENCE") == 0)
        {
            sat_mode = MODE_SCIENCE;
            buzzer_enabled = 1;
        }
        else if (strcmp(mode_str, "ERROR_LOW") == 0)
        {
            sat_mode = MODE_ERROR_LOW;
            buzzer_enabled = 1;
            error_ack_step = 0;
        }
        else if (strcmp(mode_str, "ERROR_HIGH") == 0)
        {
            sat_mode = MODE_ERROR_HIGH;
            buzzer_enabled = 1;
            error_ack_step = 0;
        }
    }
    /* ---- Alarm threshold update ---- */
    else if (strncmp(cmd, "CMD:ALARM:", 10) == 0)
    {
        const char *param = cmd + 10;
        float value;

        if (sscanf(param, "TILT:%f", &value) == 1)
            alarm_tilt_g = value;
        else if (sscanf(param, "TEMP:%f", &value) == 1)
            alarm_temp_high = value;
        else if (sscanf(param, "PRES_LOW:%f", &value) == 1)
            alarm_pres_low = value;
        else if (sscanf(param, "PRES_HIGH:%f", &value) == 1)
            alarm_pres_high = value;
        else if (sscanf(param, "ALT_HIGH:%f", &value) == 1)
            alarm_alt_high = value;
        else if (sscanf(param, "ALT_LOW:%f", &value) == 1)
            alarm_alt_low = value;
    }
    /* ---- Data dump request ---- */
    else if (strcmp(cmd, "CMD:DUMP") == 0)
    {
        static char dump_buf[200];

        IMU_Data_t imu_local;
        osMutexAcquire(mutexIMU_Handle, osWaitForever);
        imu_local = imu_data;
        osMutexRelease(mutexIMU_Handle);

        BMP_Data_t bmp_local;
        osMutexAcquire(mutexBMP_Handle, osWaitForever);
        bmp_local = bmp_data;
        osMutexRelease(mutexBMP_Handle);

        sprintf(dump_buf,
            "DUMP:AX=%.2f,AY=%.2f,AZ=%.2f,T=%.1f,P=%.1f,ALT=%.1f,MODE=%s\r\n",
            imu_local.accel_x, imu_local.accel_y, imu_local.accel_z,
            bmp_local.temperature, bmp_local.pressure, bmp_local.altitude,
            Mode_Name(sat_mode));

        HAL_UART_Transmit(&huart1, (uint8_t *)dump_buf, strlen(dump_buf), HAL_MAX_DELAY);
    }
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
  /* Start listening for ESP32 commands on USART1 (interrupt-driven) */
  HAL_UART_Receive_IT(&huart1, &esp_rx_byte, 1);
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
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
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
  taskESP32_Handle     = osThreadNew(Task_ESP32,     NULL, &taskESP32_attr);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
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

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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
    Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    Error_Handler();
}

static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

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
    Error_Handler();

  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    Error_Handler();
}

static void MX_I2C1_Init(void)
{
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
    Error_Handler();
}

static void MX_SPI1_Init(void)
{
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
    Error_Handler();
}

static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
    Error_Handler();
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
    Error_Handler();
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, BUZZER_Pin | SD_CS_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin  = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin   = LED_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin   = BUZZER_Pin | SD_CS_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* ================================================================
 *              UART1 Interrupt — ESP32 Reception
 * ================================================================ */

/**
 * @brief UART RX complete callback — accumulates bytes into a command buffer
 * @note Called from ISR context. Builds a line until \n or \r is received.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        if (esp_rx_byte == '\n' || esp_rx_byte == '\r')
        {
            if (esp_rx_idx > 0 && !esp_cmd_ready)
            {
                esp_rx_buf[esp_rx_idx] = '\0';
                memcpy(esp_cmd, esp_rx_buf, esp_rx_idx + 1);
                esp_cmd_ready = 1;
                esp_rx_idx = 0;
            }
        }
        else
        {
            if (esp_rx_idx < ESP_RX_BUF_SIZE - 1)
                esp_rx_buf[esp_rx_idx++] = (char)esp_rx_byte;
        }

        /* Re-arm interrupt for next byte */
        HAL_UART_Receive_IT(&huart1, &esp_rx_byte, 1);
    }
}

/* ================================================================
 *              Task_IMU — MPU-6050 Accelerometer & Gyroscope
 *              Rate: NOMINAL=20Hz, SCIENCE=100Hz, SAFE=5Hz
 * ================================================================ */
void Task_IMU(void *argument)
{
    uint8_t reg;

    /* --- Device reset --- */
    osMutexAcquire(mutexI2C_Handle, osWaitForever);
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU_REG_WHO_AM_I, 1, &reg, 1, HAL_MAX_DELAY);
    reg = 0x80;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU_REG_PWR_MGMT1, 1, &reg, 1, HAL_MAX_DELAY);
    osMutexRelease(mutexI2C_Handle);
    osDelay(100);

    /* --- Wake up + configure --- */
    osMutexAcquire(mutexI2C_Handle, osWaitForever);
    reg = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU_REG_PWR_MGMT1, 1, &reg, 1, HAL_MAX_DELAY);
    HAL_Delay(50);
    reg = 0x00;  /* +-2g */
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU_REG_ACCEL_CFG, 1, &reg, 1, HAL_MAX_DELAY);
    reg = 0x00;  /* +-250 deg/s */
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU_REG_GYRO_CFG, 1, &reg, 1, HAL_MAX_DELAY);
    osMutexRelease(mutexI2C_Handle);
    osDelay(100);

    /* --- Read loop --- */
    for (;;)
    {
        uint8_t buf[14];

        osMutexAcquire(mutexI2C_Handle, osWaitForever);
        HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU_REG_DATA_START, 1, buf, 14, HAL_MAX_DELAY);
        osMutexRelease(mutexI2C_Handle);

        /* Assemble 16-bit values (big-endian, proper cast) */
        int16_t raw_ax = (int16_t)((uint16_t)buf[0]  << 8 | buf[1]);
        int16_t raw_ay = (int16_t)((uint16_t)buf[2]  << 8 | buf[3]);
        int16_t raw_az = (int16_t)((uint16_t)buf[4]  << 8 | buf[5]);
        int16_t raw_t  = (int16_t)((uint16_t)buf[6]  << 8 | buf[7]);
        int16_t raw_gx = (int16_t)((uint16_t)buf[8]  << 8 | buf[9]);
        int16_t raw_gy = (int16_t)((uint16_t)buf[10] << 8 | buf[11]);
        int16_t raw_gz = (int16_t)((uint16_t)buf[12] << 8 | buf[13]);

        /* Convert and store */
        osMutexAcquire(mutexIMU_Handle, osWaitForever);
        imu_data.accel_x = raw_ax / 16384.0f;   /* +-2g  = 16384 LSB/g   */
        imu_data.accel_y = raw_ay / 16384.0f;
        imu_data.accel_z = raw_az / 16384.0f;
        imu_data.gyro_x  = raw_gx / 131.0f;     /* +-250 = 131 LSB/dps   */
        imu_data.gyro_y  = raw_gy / 131.0f;
        imu_data.gyro_z  = raw_gz / 131.0f;
        imu_data.temp    = (raw_t / 340.0f) + 36.53f;
        osMutexRelease(mutexIMU_Handle);

        /* Adapt rate to current mode */
        uint32_t period = 50;
        if (sat_mode == MODE_SCIENCE)    period = 10;
        else if (sat_mode == MODE_SAFE)  period = 200;
        osDelay(period);
    }
}

/* ================================================================
 *              Task_BMP — BMP280 Temperature & Pressure
 *              Rate: NOMINAL=2Hz, SCIENCE=5Hz, SAFE=0.5Hz
 * ================================================================ */
void Task_BMP(void *argument)
{
    uint8_t calib[26];
    uint8_t reg;

    /* --- Read calibration data --- */
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

    /* --- Configure: oversampling x1, normal mode --- */
    osMutexAcquire(mutexI2C_Handle, osWaitForever);
    reg = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, BMP280_ADDR, BMP_REG_CONFIG, 1, &reg, 1, HAL_MAX_DELAY);
    reg = 0x27;  /* osrs_t=001 osrs_p=001 mode=11 */
    HAL_I2C_Mem_Write(&hi2c1, BMP280_ADDR, BMP_REG_CTRL_MEAS, 1, &reg, 1, HAL_MAX_DELAY);
    osMutexRelease(mutexI2C_Handle);
    osDelay(100);

    /* --- Read loop --- */
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

        uint32_t period = 500;
        if (sat_mode == MODE_SCIENCE)    period = 200;
        else if (sat_mode == MODE_SAFE)  period = 2000;
        osDelay(period);
    }
}

/* ================================================================
 *              Task_Alarm — Threshold Monitoring & Buzzer
 *              Rate: 2Hz (skipped in SAFE mode)
 * ================================================================ */
void Task_Alarm(void *argument)
{
    osDelay(5000);  /* Let sensors stabilize */

    for (;;)
    {
        if (sat_mode == MODE_SAFE)
        {
            osDelay(1000);
            continue;
        }

        /* Read current sensor values */
        IMU_Data_t imu_local;
        osMutexAcquire(mutexIMU_Handle, osWaitForever);
        imu_local = imu_data;
        osMutexRelease(mutexIMU_Handle);

        BMP_Data_t bmp_local;
        osMutexAcquire(mutexBMP_Handle, osWaitForever);
        bmp_local = bmp_data;
        osMutexRelease(mutexBMP_Handle);

        /* Evaluate conditions */
        uint8_t tilt    = (fabsf(imu_local.accel_x) > alarm_tilt_g
                        || fabsf(imu_local.accel_y) > alarm_tilt_g) ? 1 : 0;
        uint8_t temp_hi = (bmp_local.temperature > alarm_temp_high) ? 1 : 0;
        uint8_t pres_ab = (bmp_local.pressure < alarm_pres_low
                        || bmp_local.pressure > alarm_pres_high) ? 1 : 0;
        uint8_t alt_cr  = (bmp_local.altitude > alarm_alt_high
                        || bmp_local.altitude < alarm_alt_low) ? 1 : 0;

        /* Update shared flags */
        osMutexAcquire(mutexAlarm_Handle, osWaitForever);
        alarm_flags.tilt          = tilt;
        alarm_flags.temp_high     = temp_hi;
        alarm_flags.pres_abnormal = pres_ab;
        alarm_flags.alt_critical  = alt_cr;
        osMutexRelease(mutexAlarm_Handle);

        /* Sound buzzer (only in NOMINAL/SCIENCE — ERROR has its own logic) */
        if (sat_mode == MODE_NOMINAL || sat_mode == MODE_SCIENCE)
        {
            uint8_t total = tilt + temp_hi + pres_ab + alt_cr;
            if (alt_cr || pres_ab)
                Buzzer_Beep(500, 200, 1);       /* Critical: long beep */
            else if (total > 0)
                Buzzer_Beep(100, 100, total);   /* Warning: short beeps */
        }

        osDelay(500);
    }
}

/* ================================================================
 *              Task_Display — OLED SSD1306 + Joystick Navigation
 *              Rate: 10Hz
 *              5 screens: IMU / BMP / ALARM / MODE / STATUS
 *              In ERROR mode: override with ack screen (R-L-R)
 * ================================================================ */
void Task_Display(void *argument)
{
    char line[22];
    uint8_t joy_locked   = 0;
    uint8_t prev_joy_dir = 0;  /* 0=center, 1=left, 2=right */

    /* Initialize OLED */
    osMutexAcquire(mutexI2C_Handle, osWaitForever);
    ssd1306_Init();
    osMutexRelease(mutexI2C_Handle);
    osDelay(100);

    for (;;)
    {
        /* --- Read joystick --- */
        uint16_t joy_x = ADC_ReadChannel(ADC_CHANNEL_0);
        uint16_t joy_y = ADC_ReadChannel(ADC_CHANNEL_4);

        uint8_t joy_dir = 0;
        if (joy_x < JOY_LEFT_THRESH)       joy_dir = 1;
        else if (joy_x > JOY_RIGHT_THRESH) joy_dir = 2;

        /* --- ERROR mode: joystick used for ack sequence --- */
        if (sat_mode == MODE_ERROR_LOW || sat_mode == MODE_ERROR_HIGH)
        {
            if (joy_dir != 0 && joy_dir != prev_joy_dir)
            {
                const uint8_t expected[] = {1, 2, 1};  /* L, R, L */
                if (joy_dir == expected[error_ack_step])
                {
                    error_ack_step++;
                    if (error_ack_step >= ERROR_ACK_SEQ_LEN)
                    {
                        sat_mode = MODE_NOMINAL;
                        error_ack_step = 0;
                        Buzzer_Beep(50, 50, 3);  /* Confirmation */
                        HAL_UART_Transmit(&huart1,
                            (uint8_t *)"MODE:NOMINAL\r\n", 14, HAL_MAX_DELAY);
                    }
                }
                else
                {
                    error_ack_step = 0;
                }
            }
            prev_joy_dir = joy_dir;
        }
        /* --- Normal mode: screen navigation --- */
        else
        {
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
                    joy_locked = 0;
            }
            prev_joy_dir = joy_dir;
        }

        /* --- Grab local copies of shared data --- */
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

        /* --- Render --- */
        osMutexAcquire(mutexI2C_Handle, osWaitForever);
        ssd1306_Fill(Black);

        if (sat_mode == MODE_ERROR_LOW || sat_mode == MODE_ERROR_HIGH)
        {
            /* Error override screen */
            ssd1306_SetCursor(0, 0);
            ssd1306_WriteString("!! ERROR !!", Font_7x10, White);

            ssd1306_SetCursor(0, 14);
            ssd1306_WriteString(
                sat_mode == MODE_ERROR_HIGH ? "SEVERITY: HIGH" : "SEVERITY: LOW",
                Font_7x10, White);

            ssd1306_SetCursor(0, 28);
            ssd1306_WriteString("ACK: R-L-R", Font_7x10, White);

            sprintf(line, "Step: %d/%d", error_ack_step, ERROR_ACK_SEQ_LEN);
            ssd1306_SetCursor(0, 42);
            ssd1306_WriteString(line, Font_7x10, White);
        }
        else
        {
            switch (current_screen)
            {
                case 0: /* --- IMU --- */
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

                case 1: /* --- BMP280 --- */
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

                case 2: /* --- ALARM --- */
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

                case 3: /* --- MODE --- */
                    ssd1306_SetCursor(0, 0);
                    ssd1306_WriteString("[MODE]", Font_7x10, White);

                    sprintf(line, "Buzzer: %s", buzzer_enabled ? "ON" : "OFF");
                    ssd1306_SetCursor(0, 14);
                    ssd1306_WriteString(line, Font_7x10, White);

                    ssd1306_SetCursor(0, 28);
                    ssd1306_WriteString("ESP32: UART1", Font_7x10, White);
                    break;

                case 4: /* --- STATUS --- */
                    ssd1306_SetCursor(0, 0);
                    ssd1306_WriteString("[STATUS]", Font_7x10, White);

                    ssd1306_SetCursor(0, 14);
                    ssd1306_WriteString("IMU:OK  BMP:OK", Font_7x10, White);

                    sprintf(line, "SD:%s  GPS:--", sd_initialized ? "OK" : "--");
                    ssd1306_SetCursor(0, 28);
                    ssd1306_WriteString(line, Font_7x10, White);

                    sprintf(line, "JX:%d JY:%d", joy_x, joy_y);
                    ssd1306_SetCursor(0, 42);
                    ssd1306_WriteString(line, Font_7x10, White);
                    break;
            }

            /* Mode indicator (top right, all screens) */
            ssd1306_SetCursor(70, 0);
            ssd1306_WriteString((char *)Mode_Name(sat_mode), Font_7x10, White);

            /* Page indicator (bottom right) */
            sprintf(line, "%d/%d", current_screen + 1, SCREEN_COUNT);
            ssd1306_SetCursor(100, 54);
            ssd1306_WriteString(line, Font_7x10, White);
        }

        ssd1306_UpdateScreen();
        osMutexRelease(mutexI2C_Handle);

        osDelay(100);
    }
}

/* ================================================================
 *              Task_Telemetry — CSV output on UART2 + UART1
 *              Rate: NOMINAL=5Hz, SCIENCE=10Hz, SAFE=1Hz
 * ================================================================ */
void Task_Telemetry(void *argument)
{
    static char tx[128];

    const char *header = "AX,AY,AZ,GX,GY,GZ,T_IMU,T_BMP,P,ALT,JX,JY,ALARMS\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)header, strlen(header), HAL_MAX_DELAY);

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

        Alarm_Flags_t alarm_local;
        osMutexAcquire(mutexAlarm_Handle, osWaitForever);
        alarm_local = alarm_flags;
        osMutexRelease(mutexAlarm_Handle);

        uint16_t joy_x = ADC_ReadChannel(ADC_CHANNEL_0);
        uint16_t joy_y = ADC_ReadChannel(ADC_CHANNEL_4);

        uint8_t alarm_bits = alarm_local.tilt
                           | (alarm_local.temp_high     << 1)
                           | (alarm_local.pres_abnormal << 2)
                           | (alarm_local.alt_critical  << 3);

        /* Send in 3 chunks to avoid stack overflow */
        sprintf(tx, "%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,",
            imu_local.accel_x, imu_local.accel_y, imu_local.accel_z,
            imu_local.gyro_x,  imu_local.gyro_y,  imu_local.gyro_z);
        HAL_UART_Transmit(&huart2, (uint8_t *)tx, strlen(tx), HAL_MAX_DELAY);
        HAL_UART_Transmit(&huart1, (uint8_t *)tx, strlen(tx), HAL_MAX_DELAY);

        sprintf(tx, "%.1f,%.1f,%.1f,%.1f,",
            imu_local.temp, bmp_local.temperature,
            bmp_local.pressure, bmp_local.altitude);
        HAL_UART_Transmit(&huart2, (uint8_t *)tx, strlen(tx), HAL_MAX_DELAY);
        HAL_UART_Transmit(&huart1, (uint8_t *)tx, strlen(tx), HAL_MAX_DELAY);

        sprintf(tx, "%d,%d,%d\r\n", joy_x, joy_y, alarm_bits);
        HAL_UART_Transmit(&huart2, (uint8_t *)tx, strlen(tx), HAL_MAX_DELAY);
        HAL_UART_Transmit(&huart1, (uint8_t *)tx, strlen(tx), HAL_MAX_DELAY);

        uint32_t period = 200;
        if (sat_mode == MODE_SCIENCE)    period = 100;
        else if (sat_mode == MODE_SAFE)  period = 1000;
        osDelay(period);
    }
}

/* ================================================================
 *              Task_ESP32 — Process commands from WiFi bridge
 *              Rate: 20Hz (polling esp_cmd_ready flag)
 * ================================================================ */
void Task_ESP32(void *argument)
{
    osDelay(1000);

    /* Announce initial mode to ESP32 */
    const char *init_msg = "MODE:NOMINAL\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t *)init_msg, strlen(init_msg), HAL_MAX_DELAY);

    for (;;)
    {
        if (esp_cmd_ready)
        {
            ESP32_ProcessCmd(esp_cmd);
            esp_cmd_ready = 0;

            /* Echo current mode back to ESP32 */
            static char mode_msg[32];
            sprintf(mode_msg, "MODE:%s\r\n", Mode_Name(sat_mode));
            HAL_UART_Transmit(&huart1, (uint8_t *)mode_msg, strlen(mode_msg), HAL_MAX_DELAY);
        }
        osDelay(50);
    }
}

/* ================================================================
 *              Task_SDLog — Raw binary logging to SD card
 *              Rate: 1Hz
 *              48 bytes per entry, 10 entries per 512-byte block
 * ================================================================ */
void Task_SDLog(void *argument)
{
    static uint8_t block_buf[SD_BLOCK_SIZE];
    uint32_t block_addr = 0;
    uint16_t buf_offset = 0;

    osDelay(3000);  /* Wait for sensors */

    /* Try initial SD card init */
    osMutexAcquire(mutexSPI_Handle, osWaitForever);
    if (SD_Init() == 0)
        sd_initialized = 1;
    osMutexRelease(mutexSPI_Handle);

    for (;;)
    {
        /* Retry init if card not ready */
        if (!sd_initialized)
        {
            osDelay(5000);
            osMutexAcquire(mutexSPI_Handle, osWaitForever);
            if (SD_Init() == 0)
                sd_initialized = 1;
            osMutexRelease(mutexSPI_Handle);
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

        uint8_t alarm_bits = alarm_local.tilt
                           | (alarm_local.temp_high     << 1)
                           | (alarm_local.pres_abnormal << 2)
                           | (alarm_local.alt_critical  << 3);

        /* Build 48-byte log packet */
        uint32_t timestamp = HAL_GetTick();
        uint32_t sync      = LOG_SYNC_MARKER;

        memcpy(&block_buf[buf_offset +  0], &sync,                   4);
        memcpy(&block_buf[buf_offset +  4], &timestamp,              4);
        memcpy(&block_buf[buf_offset +  8], &imu_local.accel_x,      4);
        memcpy(&block_buf[buf_offset + 12], &imu_local.accel_y,      4);
        memcpy(&block_buf[buf_offset + 16], &imu_local.accel_z,      4);
        memcpy(&block_buf[buf_offset + 20], &imu_local.gyro_x,       4);
        memcpy(&block_buf[buf_offset + 24], &imu_local.gyro_y,       4);
        memcpy(&block_buf[buf_offset + 28], &imu_local.gyro_z,       4);
        memcpy(&block_buf[buf_offset + 32], &bmp_local.temperature,  4);
        memcpy(&block_buf[buf_offset + 36], &bmp_local.pressure,     4);
        memcpy(&block_buf[buf_offset + 40], &bmp_local.altitude,     4);
        block_buf[buf_offset + 44] = alarm_bits;
        block_buf[buf_offset + 45] = sat_mode;
        block_buf[buf_offset + 46] = 0;  /* Reserved */
        block_buf[buf_offset + 47] = 0;  /* Reserved */

        buf_offset += 48;

        /* Flush block when full (10 packets = 480 bytes) */
        if (buf_offset >= 480)
        {
            memset(&block_buf[buf_offset], 0, SD_BLOCK_SIZE - buf_offset);

            osMutexAcquire(mutexSPI_Handle, osWaitForever);
            if (SD_WriteBlock(block_addr, block_buf) != 0)
                sd_initialized = 0;  /* Mark for reinit on next cycle */
            osMutexRelease(mutexSPI_Handle);

            block_addr++;
            buf_offset = 0;
        }

        osDelay(1000);
    }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Default task — LED heartbeat + ERROR mode buzzer
  *
  * LED behavior per mode:
  *   NOMINAL:    500ms toggle (calm blink)
  *   SAFE:       2000ms toggle (slow blink)
  *   SCIENCE:    100ms toggle (fast blink)
  *   ERROR_LOW:  100ms toggle + short beeps
  *   ERROR_HIGH: LED solid ON + buzzer continuous
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
    for (;;)
    {
        switch (sat_mode)
        {
            case MODE_NOMINAL:
                HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
                osDelay(500);
                break;

            case MODE_SAFE:
                HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
                osDelay(2000);
                break;

            case MODE_SCIENCE:
                HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
                osDelay(100);
                break;

            case MODE_ERROR_LOW:
                HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
                Buzzer_Beep(50, 0, 1);
                osDelay(150);
                break;

            case MODE_ERROR_HIGH:
                HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
                osDelay(200);
                break;

            default:
                osDelay(500);
                break;
        }
    }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
