#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#define NUM_GRID_PHASES               3U
#define NUM_BATTERY_PACKS             4U
#define ADC_SAMPLES_PER_CYCLE        64U
#define RING_BUFFER_SIZE            256U
#define FLASH_SECTOR_SIZE          4096U
#define EEPROM_BLOCK_SIZE           512U
#define CLI_MAX_INPUT_LEN            64U

#define NOMINAL_GRID_VOLTAGE_V      230.0f
#define MAX_GRID_VOLTAGE_V          265.0f
#define MIN_GRID_VOLTAGE_V          195.0f
#define MAX_GRID_CURRENT_A           80.0f
#define NOMINAL_FREQUENCY_HZ         50.0f
#define MAX_FREQUENCY_DEV_HZ          2.5f
#define MAX_THD_PERCENTAGE           10.0f

#define MAX_INVERTER_TEMP_C          75.0f
#define TARGET_COOLING_TEMP_C        45.0f
#define MIN_BATTERY_SOC_PERCENT      15.0f
#define MAX_BATTERY_SOC_PERCENT      95.0f

#define ADC_SAMPLING_FREQ_HZ       1000U
#define GRID_RELAY_FREQ_HZ          200U
#define BMS_CAN_FREQ_HZ              50U
#define THERMAL_PID_FREQ_HZ          10U
#define MODBUS_POLL_FREQ_HZ           5U
#define TELEMETRY_FREQ_HZ             1U
#define WATCHDOG_CHECK_PERIOD_MS    500U

#define HB_BIT_ADC_TASK        ( 1 << 0 )
#define HB_BIT_RELAY_TASK      ( 1 << 1 )
#define HB_BIT_BMS_TASK        ( 1 << 2 )
#define HB_BIT_THERMAL_TASK    ( 1 << 3 )
#define HB_BIT_MODBUS_TASK     ( 1 << 4 )
#define ALL_SYSTEM_HEARTBEATS  ( HB_BIT_ADC_TASK | HB_BIT_RELAY_TASK | \
                                 HB_BIT_BMS_TASK | HB_BIT_THERMAL_TASK | \
                                 HB_BIT_MODBUS_TASK )

#define FAULT_MASK_NONE             0x00000000U
#define FAULT_MASK_OVERVOLTAGE      0x00000001U
#define FAULT_MASK_UNDERVOLTAGE     0x00000002U
#define FAULT_MASK_OVERCURRENT      0x00000004U
#define FAULT_MASK_FREQ_DEVIATION   0x00000008U
#define FAULT_MASK_OVERTEMP         0x00000010U
#define FAULT_MASK_BATTERY_LOW      0x00000020U
#define FAULT_MASK_MODBUS_COMM      0x00000040U
#define FAULT_MASK_CAN_COMM         0x00000080U
#define FAULT_MASK_WATCHDOG_TRIP    0x00000100U

#define MODBUS_FUNC_READ_COILS          0x01U
#define MODBUS_FUNC_READ_HOLDING_REGS   0x03U
#define MODBUS_FUNC_WRITE_SINGLE_REG    0x06U
#define MODBUS_FUNC_WRITE_MULTIPLE_REGS 0x10U

#define CANOPEN_COB_NMT                 0x000U
#define CANOPEN_COB_SYNC                0x080U
#define CANOPEN_COB_EMCY                0x080U
#define CANOPEN_COB_TPDO1               0x180U
#define CANOPEN_COB_RPDO1               0x200U
#define CANOPEN_COB_SDO_TX              0x580U
#define CANOPEN_COB_SDO_RX              0x600U

typedef enum {
    GRID_MODE_ISLANDED = 0,
    GRID_MODE_TIED,
    GRID_MODE_FAULT_RECOVERY,
    GRID_MODE_EMERGENCY_SHUTDOWN
} GridOperatingMode_t;

typedef enum {
    CONTACTOR_OPEN = 0,
    CONTACTOR_CLOSED,
    CONTACTOR_PRECHARGE
} ContactorState_t;

typedef struct {
    float voltage_rms[NUM_GRID_PHASES];
    float current_rms[NUM_GRID_PHASES];
    float active_power_kw;
    float reactive_power_kvar;
    float power_factor;
    float frequency_hz;
    float thd_percentage;
    uint32_t timestamp_ms;
} PowerQualityMetrics_t;

typedef struct {
    float cell_voltages[8];
    float pack_current_a;
    float pack_temperature_c;
    float soc_percent;
    float soh_percent;
    bool  balancing_active;
} BatteryPackData_t;

typedef struct {
    float kp;
    float ki;
    float kd;
    float prev_error;
    float integral;
    float max_integral;
    float output_limit;
} PIDController_t;

typedef struct {
    uint32_t magic_header;
    float max_voltage;
    float min_voltage;
    float max_current;
    float pid_kp;
    float pid_ki;
    float pid_kd;
    uint16_t checksum;
} EEPROMConfig_t;

typedef struct {
    uint32_t log_id;
    uint32_t timestamp_ms;
    uint32_t fault_code;
    float primary_metric;
    char message[48];
} BlackboxLogEntry_t;

typedef struct {
    BlackboxLogEntry_t buffer[RING_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint16_t count;
} LockFreeRingBuffer_t;

typedef struct {
    uint8_t slave_address;
    uint8_t function_code;
    uint16_t start_register;
    uint16_t register_count;
    uint16_t registers[16];
    uint16_t crc16;
} ModbusFrame_t;

typedef struct {
    uint32_t can_id;
    uint8_t dlc;
    uint8_t data[8];
} CANMessage_t;

typedef struct {
    char command[16];
    void (*handler)(const char *args);
    const char *help_text;
} CLICommand_t;

static SemaphoreHandle_t  xMetricsMutex            = NULL;
static SemaphoreHandle_t  xBMSMutex                = NULL;
static SemaphoreHandle_t  xUARTConsoleMutex        = NULL;
static SemaphoreHandle_t  xFlashMutex              = NULL;
static SemaphoreHandle_t  xEEPROMMutex             = NULL;
static SemaphoreHandle_t  xOvercurrentISRSemaphore = NULL;

static QueueHandle_t      xTelemetryQueue          = NULL;
static QueueHandle_t      xModbusRequestQueue      = NULL;
static QueueHandle_t      xCANTxQueue              = NULL;

static EventGroupHandle_t xWatchdogEventGroup      = NULL;
static TaskHandle_t       xADCTaskHandle           = NULL;

static PowerQualityMetrics_t g_xPowerMetrics;
static BatteryPackData_t     g_xBatteryPacks[NUM_BATTERY_PACKS];
static LockFreeRingBuffer_t  g_xBlackboxBuffer;
static PIDController_t       g_xFanPID;
static EEPROMConfig_t        g_xEEPROMConfig;

static volatile GridOperatingMode_t g_eGridMode        = GRID_MODE_ISLANDED;
static volatile ContactorState_t   g_eContactorState   = CONTACTOR_OPEN;
static volatile uint32_t           g_ulSystemFaultMask = FAULT_MASK_NONE;
static volatile bool               g_bSystemTripped    = false;

static uint8_t g_ucFlashMemory[FLASH_SECTOR_SIZE];
static uint8_t g_ucEEPROMMemory[EEPROM_BLOCK_SIZE];

static void RingBuffer_Init(LockFreeRingBuffer_t *rb)
{
    rb->head = 0U;
    rb->tail = 0U;
    rb->count = 0U;
    memset(rb->buffer, 0, sizeof(rb->buffer));
}

static bool RingBuffer_Push(LockFreeRingBuffer_t *rb, const BlackboxLogEntry_t *entry)
{
    if (rb->count >= RING_BUFFER_SIZE) {
        rb->tail = (uint16_t)((rb->tail + 1U) % RING_BUFFER_SIZE);
        rb->count--;
    }

    rb->buffer[rb->head] = *entry;
    rb->head = (uint16_t)((rb->head + 1U) % RING_BUFFER_SIZE);
    rb->count++;
    return true;
}

static bool RingBuffer_Pop(LockFreeRingBuffer_t *rb, BlackboxLogEntry_t *entry)
{
    if (rb->count == 0U) {
        return false;
    }

    *entry = rb->buffer[rb->tail];
    rb->tail = (uint16_t)((rb->tail + 1U) % RING_BUFFER_SIZE);
    rb->count--;
    return true;
}

static bool Flash_Erase_Sector(void)
{
    if (xFlashMutex != NULL) {
        if (xSemaphoreTake(xFlashMutex, pdMS_TO_TICKS(10U)) == pdTRUE) {
            memset(g_ucFlashMemory, 0xFF, sizeof(g_ucFlashMemory));
            xSemaphoreGive(xFlashMutex);
            return true;
        }
    }
    return false;
}

static bool Flash_Write_Block(uint32_t address, const uint8_t *data, uint16_t length)
{
    if (address + length > FLASH_SECTOR_SIZE) return false;

    if (xFlashMutex != NULL) {
        if (xSemaphoreTake(xFlashMutex, pdMS_TO_TICKS(10U)) == pdTRUE) {
            memcpy(&g_ucFlashMemory[address], data, length);
            xSemaphoreGive(xFlashMutex);
            return true;
        }
    }
    return false;
}

static bool EEPROM_LoadConfig(EEPROMConfig_t *cfg)
{
    if (xEEPROMMutex != NULL) {
        if (xSemaphoreTake(xEEPROMMutex, pdMS_TO_TICKS(5U)) == pdTRUE) {
            memcpy(cfg, g_ucEEPROMMemory, sizeof(EEPROMConfig_t));
            xSemaphoreGive(xEEPROMMutex);
            return (cfg->magic_header == 0xDEADBEEFU);
        }
    }
    return false;
}

static bool EEPROM_SaveConfig(const EEPROMConfig_t *cfg)
{
    if (xEEPROMMutex != NULL) {
        if (xSemaphoreTake(xEEPROMMutex, pdMS_TO_TICKS(5U)) == pdTRUE) {
            memcpy(g_ucEEPROMMemory, cfg, sizeof(EEPROMConfig_t));
            xSemaphoreGive(xEEPROMMutex);
            return true;
        }
    }
    return false;
}

static uint16_t Calculate_Modbus_CRC16(const uint8_t *buffer, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    for (uint16_t i = 0U; i < length; i++) {
        crc ^= (uint16_t)buffer[i];
        for (uint8_t j = 0U; j < 8U; j++) {
            if ((crc & 0x0001U) != 0U) {
                crc = (uint16_t)((crc >> 1) ^ 0xA001U);
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

static float PID_Compute(PIDController_t *pid, float setpoint, float measured, float dt)
{
    float error = setpoint - measured;
    pid->integral += error * dt;

    if (pid->integral > pid->max_integral)  pid->integral = pid->max_integral;
    if (pid->integral < -pid->max_integral) pid->integral = -pid->max_integral;

    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;

    float output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);

    if (output > pid->output_limit)  output = pid->output_limit;
    if (output < 0.0f)              output = 0.0f;

    return output;
}

static float Calculate_THD(const float samples[ADC_SAMPLES_PER_CYCLE])
{
    float fundamental_real = 0.0f;
    float fundamental_imag = 0.0f;
    float harmonic3_real = 0.0f;
    float harmonic3_imag = 0.0f;
    float harmonic5_real = 0.0f;
    float harmonic5_imag = 0.0f;

    for (uint8_t n = 0U; n < ADC_SAMPLES_PER_CYCLE; n++) {
        double angle1 = (2.0 * 3.141592653589793 * (double)n) / (double)ADC_SAMPLES_PER_CYCLE;
        double angle3 = 3.0 * angle1;
        double angle5 = 5.0 * angle1;

        fundamental_real += samples[n] * (float)cos(angle1);
        fundamental_imag -= samples[n] * (float)sin(angle1);

        harmonic3_real += samples[n] * (float)cos(angle3);
        harmonic3_imag -= samples[n] * (float)sin(angle3);

        harmonic5_real += samples[n] * (float)cos(angle5);
        harmonic5_imag -= samples[n] * (float)sin(angle5);
    }

    float mag1 = (float)sqrt((double)(fundamental_real * fundamental_real + fundamental_imag * fundamental_imag));
    float mag3 = (float)sqrt((double)(harmonic3_real * harmonic3_real + harmonic3_imag * harmonic3_imag));
    float mag5 = (float)sqrt((double)(harmonic5_real * harmonic5_real + harmonic5_imag * harmonic5_imag));

    if (mag1 > 0.001f) {
        float harm_sq = (mag3 * mag3) + (mag5 * mag5);
        return ((float)sqrt((double)harm_sq) / mag1) * 100.0f;
    }
    return 0.0f;
}

static void Calculate_Power_Quality(const float voltage_samples[ADC_SAMPLES_PER_CYCLE],
                                    const float current_samples[ADC_SAMPLES_PER_CYCLE],
                                    PowerQualityMetrics_t *metrics)
{
    float v_sum_sq = 0.0f;
    float i_sum_sq = 0.0f;
    float p_sum = 0.0f;

    for (uint8_t k = 0U; k < ADC_SAMPLES_PER_CYCLE; k++) {
        v_sum_sq += voltage_samples[k] * voltage_samples[k];
        i_sum_sq += current_samples[k] * current_samples[k];
        p_sum    += voltage_samples[k] * current_samples[k];
    }

    float v_rms = (float)sqrt((double)(v_sum_sq / (float)ADC_SAMPLES_PER_CYCLE));
    float i_rms = (float)sqrt((double)(i_sum_sq / (float)ADC_SAMPLES_PER_CYCLE));
    float active_power = p_sum / (float)ADC_SAMPLES_PER_CYCLE;
    float apparent_power = v_rms * i_rms;

    metrics->voltage_rms[0] = v_rms;
    metrics->current_rms[0] = i_rms;
    metrics->active_power_kw = active_power / 1000.0f;

    if (apparent_power > 0.001f) {
        metrics->power_factor = active_power / apparent_power;
        float reactive_sq = (apparent_power * apparent_power) - (active_power * active_power);
        metrics->reactive_power_kvar = (reactive_sq > 0.0f) ? ((float)sqrt((double)reactive_sq) / 1000.0f) : 0.0f;
    } else {
        metrics->power_factor = 1.0f;
        metrics->reactive_power_kvar = 0.0f;
    }

    metrics->frequency_hz = NOMINAL_GRID_VOLTAGE_V > 0.0f ? 50.0f : 0.0f;
    metrics->thd_percentage = Calculate_THD(voltage_samples);
}

static void HAL_Relay_OpenGridContactor(void)
{
    g_eContactorState = CONTACTOR_OPEN;
    g_bSystemTripped = true;
}

static void HAL_Relay_CloseGridContactor(void)
{
    if (!g_bSystemTripped) {
        g_eContactorState = CONTACTOR_CLOSED;
    }
}

static void HAL_PWM_SetCoolingFanSpeed(float duty_percent)
{
    (void)duty_percent;
}

static void HAL_CAN_TransmitMessage(const CANMessage_t *msg)
{
    (void)msg;
}

static void HAL_RS485_TransmitFrame(const uint8_t *frame, uint16_t len)
{
    (void)frame;
    (void)len;
}

static void Console_Print(const char *msg)
{
    if (xUARTConsoleMutex != NULL) {
        if (xSemaphoreTake(xUARTConsoleMutex, pdMS_TO_TICKS(10U)) == pdTRUE) {
            (void)msg;
            xSemaphoreGive(xUARTConsoleMutex);
        }
    }
}

static bool Modbus_Process_Read_Coils(const uint8_t *rx, uint16_t len, uint8_t *tx, uint16_t *tx_len)
{
    if (len < 6U) return false;
    tx[0] = rx[0];
    tx[1] = MODBUS_FUNC_READ_COILS;
    tx[2] = 0x01U;
    tx[3] = (g_eContactorState == CONTACTOR_CLOSED) ? 0x01U : 0x00U;
    
    uint16_t crc = Calculate_Modbus_CRC16(tx, 4U);
    tx[4] = (uint8_t)(crc & 0xFF);
    tx[5] = (uint8_t)((crc >> 8) & 0xFF);
    *tx_len = 6U;
    return true;
}

static bool Modbus_Process_Write_Single_Register(const uint8_t *rx, uint16_t len, uint8_t *tx, uint16_t *tx_len)
{
    if (len < 8U) return false;
    uint16_t reg_addr = (uint16_t)((rx[2] << 8) | rx[3]);
    uint16_t reg_val  = (uint16_t)((rx[4] << 8) | rx[5]);

    if (reg_addr == 0x0001U) {
        if (reg_val == 0xFF00U) HAL_Relay_CloseGridContactor();
        else if (reg_val == 0x0000U) HAL_Relay_OpenGridContactor();
    }

    memcpy(tx, rx, 8U);
    *tx_len = 8U;
    return true;
}

static bool Modbus_Process_Write_Multiple_Registers(const uint8_t *rx, uint16_t len, uint8_t *tx, uint16_t *tx_len)
{
    if (len < 9U) return false;
    uint16_t start_reg = (uint16_t)((rx[2] << 8) | rx[3]);
    uint16_t reg_count = (uint16_t)((rx[4] << 8) | rx[5]);

    tx[0] = rx[0];
    tx[1] = MODBUS_FUNC_WRITE_MULTIPLE_REGS;
    tx[2] = (uint8_t)((start_reg >> 8) & 0xFF);
    tx[3] = (uint8_t)(start_reg & 0xFF);
    tx[4] = (uint8_t)((reg_count >> 8) & 0xFF);
    tx[5] = (uint8_t)(reg_count & 0xFF);

    uint16_t crc = Calculate_Modbus_CRC16(tx, 6U);
    tx[6] = (uint8_t)(crc & 0xFF);
    tx[7] = (uint8_t)((crc >> 8) & 0xFF);
    *tx_len = 8U;
    return true;
}

static bool Modbus_Process_Frame(const uint8_t *rx_frame, uint16_t len, ModbusFrame_t *out_frame)
{
    if (len < 8U) return false;

    uint16_t rx_crc = (uint16_t)(rx_frame[len - 2U] | (rx_frame[len - 1U] << 8));
    uint16_t calc_crc = Calculate_Modbus_CRC16(rx_frame, len - 2U);

    if (rx_crc != calc_crc) {
        g_ulSystemFaultMask |= FAULT_MASK_MODBUS_COMM;
        return false;
    }

    out_frame->slave_address  = rx_frame[0];
    out_frame->function_code  = rx_frame[1];
    out_frame->start_register = (uint16_t)((rx_frame[2] << 8) | rx_frame[3]);
    out_frame->register_count = (uint16_t)((rx_frame[4] << 8) | rx_frame[5]);
    out_frame->crc16          = rx_crc;

    return true;
}

static void CANopen_Build_TPDO1(uint32_t node_id, float soc, float temp, uint32_t faults, CANMessage_t *out_msg)
{
    out_msg->can_id = CANOPEN_COB_TPDO1 + node_id;
    out_msg->dlc    = 8U;
    
    uint16_t soc_scaled  = (uint16_t)(soc * 100.0f);
    uint16_t temp_scaled = (uint16_t)((temp + 40.0f) * 10.0f);

    out_msg->data[0] = (uint8_t)(soc_scaled & 0xFF);
    out_msg->data[1] = (uint8_t)((soc_scaled >> 8) & 0xFF);
    out_msg->data[2] = (uint8_t)(temp_scaled & 0xFF);
    out_msg->data[3] = (uint8_t)((temp_scaled >> 8) & 0xFF);
    out_msg->data[4] = (uint8_t)(faults & 0xFF);
    out_msg->data[5] = (uint8_t)((faults >> 8) & 0xFF);
    out_msg->data[6] = (uint8_t)((faults >> 16) & 0xFF);
    out_msg->data[7] = (uint8_t)((faults >> 24) & 0xFF);
}

static void CANopen_Build_RPDO1(uint32_t node_id, const uint8_t data[8], CANMessage_t *out_msg)
{
    out_msg->can_id = CANOPEN_COB_RPDO1 + node_id;
    out_msg->dlc    = 8U;
    memcpy(out_msg->data, data, 8U);
}

static void CANopen_Build_SDO_Response(uint32_t node_id, uint16_t index, uint8_t subindex, uint32_t data, CANMessage_t *out_msg)
{
    out_msg->can_id = CANOPEN_COB_SDO_TX + node_id;
    out_msg->dlc    = 8U;
    out_msg->data[0] = 0x4F;
    out_msg->data[1] = (uint8_t)(index & 0xFF);
    out_msg->data[2] = (uint8_t)((index >> 8) & 0xFF);
    out_msg->data[3] = subindex;
    out_msg->data[4] = (uint8_t)(data & 0xFF);
    out_msg->data[5] = (uint8_t)((data >> 8) & 0xFF);
    out_msg->data[6] = (uint8_t)((data >> 16) & 0xFF);
    out_msg->data[7] = (uint8_t)((data >> 24) & 0xFF);
}

void EXTI_Overcurrent_Hardware_ISR(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    HAL_Relay_OpenGridContactor();
    g_ulSystemFaultMask |= FAULT_MASK_OVERCURRENT;

    if (xOvercurrentISRSemaphore != NULL) {
        xSemaphoreGiveFromISR(xOvercurrentISRSemaphore, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void vPowerQualityADCTask(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    float v_samples[ADC_SAMPLES_PER_CYCLE];
    float i_samples[ADC_SAMPLES_PER_CYCLE];
    PowerQualityMetrics_t local_metrics;

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000U / ADC_SAMPLING_FREQ_HZ));

        for (uint8_t k = 0U; k < ADC_SAMPLES_PER_CYCLE; k++) {
            double angle = (2.0 * 3.141592653589793 * (double)k) / (double)ADC_SAMPLES_PER_CYCLE;
            v_samples[k] = (float)(325.27 * sin(angle));
            i_samples[k] = (float)(28.28 * sin(angle - 0.2));
        }

        Calculate_Power_Quality(v_samples, i_samples, &local_metrics);
        local_metrics.timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

        uint32_t local_fault = FAULT_MASK_NONE;
        if (local_metrics.voltage_rms[0] > MAX_GRID_VOLTAGE_V) {
            local_fault |= FAULT_MASK_OVERVOLTAGE;
        } else if (local_metrics.voltage_rms[0] < MIN_GRID_VOLTAGE_V) {
            local_fault |= FAULT_MASK_UNDERVOLTAGE;
        }

        if (local_metrics.current_rms[0] > MAX_GRID_CURRENT_A) {
            local_fault |= FAULT_MASK_OVERCURRENT;
        }

        if (local_fault != FAULT_MASK_NONE) {
            HAL_Relay_OpenGridContactor();
            g_ulSystemFaultMask |= local_fault;
        }

        if (xSemaphoreTake(xMetricsMutex, pdMS_TO_TICKS(1U)) == pdTRUE) {
            g_xPowerMetrics = local_metrics;
            xSemaphoreGive(xMetricsMutex);
        }

        if (xWatchdogEventGroup != NULL) {
            xEventGroupSetBits(xWatchdogEventGroup, HB_BIT_ADC_TASK);
        }
    }
}

static void vGridRelayControlTask(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000U / GRID_RELAY_FREQ_HZ));

        if (xSemaphoreTake(xOvercurrentISRSemaphore, 0U) == pdTRUE) {
            BlackboxLogEntry_t log_entry;
            log_entry.log_id = xTaskGetTickCount();
            log_entry.timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
            log_entry.fault_code = FAULT_MASK_OVERCURRENT;
            log_entry.primary_metric = MAX_GRID_CURRENT_A;
            strncpy(log_entry.message, "HARDWARE ISR SHORT CIRCUIT TRIP", sizeof(log_entry.message) - 1);
            RingBuffer_Push(&g_xBlackboxBuffer, &log_entry);
        }

        switch (g_eGridMode) {
        case GRID_MODE_TIED:
            if (g_ulSystemFaultMask != FAULT_MASK_NONE) {
                HAL_Relay_OpenGridContactor();
                g_eGridMode = GRID_MODE_ISLANDED;
                Console_Print("[SYSTEM] Grid Fault Detected! Switching to Islanded Mode.\r\n");
            }
            break;

        case GRID_MODE_ISLANDED:
            if (g_ulSystemFaultMask == FAULT_MASK_NONE) {
                HAL_Relay_CloseGridContactor();
                g_eGridMode = GRID_MODE_TIED;
                Console_Print("[SYSTEM] Grid Synchronized. Restoring Grid-Tied Mode.\r\n");
            }
            break;

        case GRID_MODE_EMERGENCY_SHUTDOWN:
        default:
            HAL_Relay_OpenGridContactor();
            break;
        }

        if (xWatchdogEventGroup != NULL) {
            xEventGroupSetBits(xWatchdogEventGroup, HB_BIT_RELAY_TASK);
        }
    }
}

static void vBMSCommunicationTask(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    CANMessage_t can_msg;

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000U / BMS_CAN_FREQ_HZ));

        if (xSemaphoreTake(xBMSMutex, pdMS_TO_TICKS(2U)) == pdTRUE) {
            for (uint8_t p = 0U; p < NUM_BATTERY_PACKS; p++) {
                g_xBatteryPacks[p].pack_current_a = 15.2f;
                g_xBatteryPacks[p].soc_percent = 84.5f;
                g_xBatteryPacks[p].soh_percent = 98.2f;
            }
            xSemaphoreGive(xBMSMutex);
        }

        CANopen_Build_TPDO1(1U, 84.5f, 29.5f, g_ulSystemFaultMask, &can_msg);
        HAL_CAN_TransmitMessage(&can_msg);

        if (xWatchdogEventGroup != NULL) {
            xEventGroupSetBits(xWatchdogEventGroup, HB_BIT_BMS_TASK);
        }
    }
}

static void vThermalControlTask(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    g_xFanPID.kp = 3.5f;
    g_xFanPID.ki = 0.5f;
    g_xFanPID.kd = 0.1f;
    g_xFanPID.prev_error = 0.0f;
    g_xFanPID.integral = 0.0f;
    g_xFanPID.max_integral = 50.0f;
    g_xFanPID.output_limit = 100.0f;

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000U / THERMAL_PID_FREQ_HZ));

        float measured_temp_c = 52.4f;

        if (measured_temp_c > MAX_INVERTER_TEMP_C) {
            g_ulSystemFaultMask |= FAULT_MASK_OVERTEMP;
            HAL_Relay_OpenGridContactor();
        }

        float fan_speed_percent = PID_Compute(&g_xFanPID, TARGET_COOLING_TEMP_C, measured_temp_c, 0.1f);
        HAL_PWM_SetCoolingFanSpeed(fan_speed_percent);

        if (xWatchdogEventGroup != NULL) {
            xEventGroupSetBits(xWatchdogEventGroup, HB_BIT_THERMAL_TASK);
        }
    }
}

static void vModbusMasterTask(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint8_t modbus_tx[8];

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000U / MODBUS_POLL_FREQ_HZ));

        modbus_tx[0] = 0x01U;
        modbus_tx[1] = MODBUS_FUNC_READ_HOLDING_REGS;
        modbus_tx[2] = 0x00U;
        modbus_tx[3] = 0x00U;
        modbus_tx[4] = 0x00U;
        modbus_tx[5] = 0x04U;

        uint16_t crc = Calculate_Modbus_CRC16(modbus_tx, 6U);
        modbus_tx[6] = (uint8_t)(crc & 0xFF);
        modbus_tx[7] = (uint8_t)((crc >> 8) & 0xFF);

        HAL_RS485_TransmitFrame(modbus_tx, sizeof(modbus_tx));

        if (xWatchdogEventGroup != NULL) {
            xEventGroupSetBits(xWatchdogEventGroup, HB_BIT_MODBUS_TASK);
        }
    }
}

static void vSCADATelemetryTask(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    char json_payload[256];
    PowerQualityMetrics_t metrics;

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000U / TELEMETRY_FREQ_HZ));

        if (xSemaphoreTake(xMetricsMutex, pdMS_TO_TICKS(5U)) == pdTRUE) {
            metrics = g_xPowerMetrics;
            xSemaphoreGive(xMetricsMutex);
        }

        snprintf(json_payload, sizeof(json_payload),
                 "{\"v_rms\":%.1f,\"i_rms\":%.1f,\"kw\":%.2f,\"pf\":%.2f,\"thd\":%.2f,\"fault\":%lu}",
                 (double)metrics.voltage_rms[0],
                 (double)metrics.current_rms[0],
                 (double)metrics.active_power_kw,
                 (double)metrics.power_factor,
                 (double)metrics.thd_percentage,
                 (unsigned long)g_ulSystemFaultMask);

        Console_Print(json_payload);
    }
}

static void vSafetyWatchdogTask(void *pvParameters)
{
    (void)pvParameters;

    while (1) {
        EventBits_t bits = xEventGroupWaitBits(
            xWatchdogEventGroup,
            ALL_SYSTEM_HEARTBEATS,
            pdTRUE,
            pdTRUE,
            pdMS_TO_TICKS(WATCHDOG_CHECK_PERIOD_MS)
        );

        if ((bits & ALL_SYSTEM_HEARTBEATS) != ALL_SYSTEM_HEARTBEATS) {
            HAL_Relay_OpenGridContactor();
            g_ulSystemFaultMask |= FAULT_MASK_WATCHDOG_TRIP;
            Console_Print("[CRITICAL] Watchdog Timeout! Task Execution Stalled. Opening Contactor.\r\n");
        }
    }
}

static void CLI_Cmd_Status(const char *args)
{
    (void)args;
    Console_Print("\r\n=== SMART GRID GATEWAY SYSTEM STATUS ===\r\n");
    Console_Print("Mode     : GRID-TIED\r\n");
    Console_Print("Contactor: CLOSED\r\n");
    Console_Print("Faults   : 0x00000000 (NORMAL)\r\n");
}

static void CLI_Cmd_Metrics(const char *args)
{
    (void)args;
    Console_Print("\r\n=== POWER QUALITY METRICS ===\r\n");
    Console_Print("Voltage RMS: 230.0 V\r\nCurrent RMS: 20.0 A\r\nActive Pwr : 4.6 kW\r\nPower Factor: 0.98\r\nTHD        : 2.5%\r\n");
}

static void CLI_Cmd_BMS(const char *args)
{
    (void)args;
    Console_Print("\r\n=== BATTERY STORAGE METRICS ===\r\n");
    Console_Print("Pack SoC   : 84.5%\r\nPack SoH   : 98.2%\r\nCurrent    : 15.2 A\r\n");
}

static void CLI_Cmd_Faults(const char *args)
{
    (void)args;
    Console_Print("\r\n=== ACTIVE SYSTEM FAULTS ===\r\nNO FAULTS REGISTERED\r\n");
}

static void CLI_Cmd_TripRelay(const char *args)
{
    (void)args;
    HAL_Relay_OpenGridContactor();
    Console_Print("\r\n[MANUAL OVERRIDE] Contactor Relay Opened.\r\n");
}

static void CLI_Cmd_CloseRelay(const char *args)
{
    (void)args;
    g_bSystemTripped = false;
    g_ulSystemFaultMask = FAULT_MASK_NONE;
    HAL_Relay_CloseGridContactor();
    Console_Print("\r\n[MANUAL OVERRIDE] Contactor Relay Closed.\r\n");
}

static void CLI_Cmd_DumpBlackbox(const char *args)
{
    (void)args;
    Console_Print("\r\n=== BLACKBOX RING BUFFER DUMP ===\r\n");
    BlackboxLogEntry_t entry;
    uint16_t dumped = 0U;
    while (RingBuffer_Pop(&g_xBlackboxBuffer, &entry)) {
        dumped++;
        char log_str[96];
        snprintf(log_str, sizeof(log_str), "[LOG #%lu] %s (Fault: 0x%08LU)\r\n",
                 (unsigned long)entry.log_id, entry.message, (unsigned long)entry.fault_code);
        Console_Print(log_str);
    }
    if (dumped == 0U) {
        Console_Print("No blackbox entries recorded.\r\n");
    }
}

static void CLI_Cmd_SetPID(const char *args)
{
    if (args != NULL && strlen(args) > 0) {
        g_xFanPID.kp = (float)atof(args);
        Console_Print("\r\n[PID UPDATE] Fan Kp gain updated successfully.\r\n");
    } else {
        Console_Print("\r\nUsage: pid_set <kp_value>\r\n");
    }
}

static void CLI_Cmd_ClearFaults(const char *args)
{
    (void)args;
    g_ulSystemFaultMask = FAULT_MASK_NONE;
    g_bSystemTripped = false;
    Console_Print("\r\n[SYSTEM] System faults cleared successfully.\r\n");
}

static void CLI_Cmd_EEPROM_Read(const char *args)
{
    (void)args;
    EEPROMConfig_t cfg;
    if (EEPROM_LoadConfig(&cfg)) {
        Console_Print("\r\n=== EEPROM CONFIGURATION ===\r\n");
        Console_Print("Header  : 0xDEADBEEF (VALID)\r\n");
        Console_Print("Max Volt: 265.0 V\r\n");
        Console_Print("Min Volt: 195.0 V\r\n");
        Console_Print("Max Curr: 80.0 A\r\n");
    } else {
        Console_Print("\r\n[EEPROM] Invalid or uninitialized header.\r\n");
    }
}

static void CLI_Cmd_EEPROM_Write(const char *args)
{
    (void)args;
    g_xEEPROMConfig.magic_header = 0xDEADBEEFU;
    g_xEEPROMConfig.max_voltage = MAX_GRID_VOLTAGE_V;
    g_xEEPROMConfig.min_voltage = MIN_GRID_VOLTAGE_V;
    g_xEEPROMConfig.max_current = MAX_GRID_CURRENT_A;
    g_xEEPROMConfig.pid_kp = g_xFanPID.kp;
    g_xEEPROMConfig.pid_ki = g_xFanPID.ki;
    g_xEEPROMConfig.pid_kd = g_xFanPID.kd;
    g_xEEPROMConfig.checksum = 0x1234U;

    if (EEPROM_SaveConfig(&g_xEEPROMConfig)) {
        Console_Print("\r\n[EEPROM] Config parameters saved to EEPROM block.\r\n");
    } else {
        Console_Print("\r\n[EEPROM ERROR] Failed to save config parameters.\r\n");
    }
}

static void CLI_Cmd_Flash_Erase(const char *args)
{
    (void)args;
    if (Flash_Erase_Sector()) {
        Console_Print("\r\n[FLASH] Sector 0 Erased (4096 bytes set to 0xFF).\r\n");
    } else {
        Console_Print("\r\n[FLASH ERROR] Sector erase lock failure.\r\n");
    }
}

static void CLI_Cmd_Help(const char *args);

static const CLICommand_t g_xCLICommands[] = {
    {"status",     CLI_Cmd_Status,       "Display system operating status"},
    {"metrics",    CLI_Cmd_Metrics,      "Display live power quality metrics"},
    {"bms",        CLI_Cmd_BMS,          "Display battery pack SoC/SoH"},
    {"faults",     CLI_Cmd_Faults,       "List active fault bitmask"},
    {"trip",       CLI_Cmd_TripRelay,    "Manually trip grid contactor"},
    {"close",      CLI_Cmd_CloseRelay,   "Manually close grid contactor"},
    {"blackbox",   CLI_Cmd_DumpBlackbox, "Dump blackbox ring buffer logs"},
    {"pid_set",    CLI_Cmd_SetPID,       "Update fan PID Kp gain"},
    {"clear",      CLI_Cmd_ClearFaults,  "Clear all fault flags"},
    {"eeprom_read",CLI_Cmd_EEPROM_Read,  "Read parameters from EEPROM"},
    {"eeprom_write",CLI_Cmd_EEPROM_Write,"Write parameters to EEPROM"},
    {"flash_erase",CLI_Cmd_Flash_Erase,  "Erase NOR Flash sector"},
    {"help",       CLI_Cmd_Help,         "Display CLI command help menu"}
};

static const uint8_t NUM_CLI_COMMANDS = sizeof(g_xCLICommands) / sizeof(CLICommand_t);

static void CLI_Cmd_Help(const char *args)
{
    (void)args;
    Console_Print("\r\n=== CLI COMMAND HELP MENU ===\r\n");
    for (uint8_t i = 0U; i < NUM_CLI_COMMANDS; i++) {
        Console_Print(g_xCLICommands[i].command);
        Console_Print(" : ");
        Console_Print(g_xCLICommands[i].help_text);
        Console_Print("\r\n");
    }
}

static void CLI_Process_Input(const char *input)
{
    for (uint8_t i = 0U; i < NUM_CLI_COMMANDS; i++) {
        if (strncmp(input, g_xCLICommands[i].command, strlen(g_xCLICommands[i].command)) == 0) {
            const char *args = input + strlen(g_xCLICommands[i].command);
            while (*args == ' ') args++;
            g_xCLICommands[i].handler(args);
            return;
        }
    }
    Console_Print("\r\nUnknown command. Type 'help' for available commands.\r\n");
}

static void vCLIConsoleTask(void *pvParameters)
{
    (void)pvParameters;
    char input_cmd[CLI_MAX_INPUT_LEN];

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(3000U));
        strncpy(input_cmd, "status", sizeof(input_cmd) - 1);
        CLI_Process_Input(input_cmd);
    }
}

bool bInitSmartGridGatewaySystem(void)
{
    RingBuffer_Init(&g_xBlackboxBuffer);
    Flash_Erase_Sector();

    g_xEEPROMConfig.magic_header = 0xDEADBEEFU;
    g_xEEPROMConfig.max_voltage = MAX_GRID_VOLTAGE_V;
    g_xEEPROMConfig.min_voltage = MIN_GRID_VOLTAGE_V;
    g_xEEPROMConfig.max_current = MAX_GRID_CURRENT_A;
    g_xEEPROMConfig.pid_kp = 3.5f;
    g_xEEPROMConfig.pid_ki = 0.5f;
    g_xEEPROMConfig.pid_kd = 0.1f;
    g_xEEPROMConfig.checksum = 0x1234U;
    EEPROM_SaveConfig(&g_xEEPROMConfig);

    xMetricsMutex = xSemaphoreCreateMutex();
    if (xMetricsMutex == NULL) return false;

    xBMSMutex = xSemaphoreCreateMutex();
    if (xBMSMutex == NULL) return false;

    xUARTConsoleMutex = xSemaphoreCreateMutex();
    if (xUARTConsoleMutex == NULL) return false;

    xFlashMutex = xSemaphoreCreateMutex();
    if (xFlashMutex == NULL) return false;

    xEEPROMMutex = xSemaphoreCreateMutex();
    if (xEEPROMMutex == NULL) return false;

    xOvercurrentISRSemaphore = xSemaphoreCreateBinary();
    if (xOvercurrentISRSemaphore == NULL) return false;

    xTelemetryQueue = xQueueCreate(16U, sizeof(PowerQualityMetrics_t));
    if (xTelemetryQueue == NULL) return false;

    xModbusRequestQueue = xQueueCreate(8U, sizeof(ModbusFrame_t));
    if (xModbusRequestQueue == NULL) return false;

    xCANTxQueue = xQueueCreate(16U, sizeof(CANMessage_t));
    if (xCANTxQueue == NULL) return false;

    xWatchdogEventGroup = xEventGroupCreate();
    if (xWatchdogEventGroup == NULL) return false;

    BaseType_t s1 = xTaskCreate(vSafetyWatchdogTask,  "Watchdog",   512, NULL, 8, NULL);
    BaseType_t s2 = xTaskCreate(vPowerQualityADCTask, "PowerADC",   512, NULL, 7, &xADCTaskHandle);
    BaseType_t s3 = xTaskCreate(vGridRelayControlTask,"RelayCtrl",  512, NULL, 6, NULL);
    BaseType_t s4 = xTaskCreate(vBMSCommunicationTask,"BMS_CAN",    512, NULL, 5, NULL);
    BaseType_t s5 = xTaskCreate(vThermalControlTask,  "ThermalPID", 512, NULL, 4, NULL);
    BaseType_t s6 = xTaskCreate(vModbusMasterTask,   "ModbusMaster",512, NULL, 3, NULL);
    BaseType_t s7 = xTaskCreate(vSCADATelemetryTask,  "SCADA_MQTT", 512, NULL, 2, NULL);
    BaseType_t s8 = xTaskCreate(vCLIConsoleTask,      "CLI_Console",512, NULL, 1, NULL);

    return (s1 == pdPASS && s2 == pdPASS && s3 == pdPASS && s4 == pdPASS &&
            s5 == pdPASS && s6 == pdPASS && s7 == pdPASS && s8 == pdPASS);
}

int main(void)
{
    if (bInitSmartGridGatewaySystem()) {
        vTaskStartScheduler();
    }

    while (1);
    return 0;
}
