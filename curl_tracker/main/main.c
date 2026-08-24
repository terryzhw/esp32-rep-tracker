#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "hal/gpio_types.h"
#include "u8g2.h"
#include "mpu6500.h"
#include "model.h"

static const char *TAG = "curl_tracker";

#define I2C_SDA_PIN     8
#define I2C_SCL_PIN     9
#define LED_PIN         10
#define BUTTON_PIN      4
#define MOTOR_PIN       3

#define I2C_PORT        I2C_NUM_0
#define I2C_FREQ_HZ     400000

// 90% gyro trust — fast response but still corrects drift from accelerometer
#define ALPHA           0.90f

// SRD=9 gives 1000/(1+9) = 100Hz sample rate
#define SAMPLE_RATE_DIV 9
#define DT              ((1.0f + SAMPLE_RATE_DIV) / 1000.0f)

static mpu6500_t imu;
static u8g2_t u8g2;

static float roll = 0.0f;
static float pitch = 0.0f;
static volatile bool filterInitialized = false;
static volatile bool collecting = false;
static TaskHandle_t motorTaskHandle = NULL;
static volatile int reps = 0;
static volatile int sets = 0;
static volatile int isPressed = 1;  // pull-up means 1 = released

static uint32_t millis(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static uint8_t u8x8_gpio_and_delay_esp32(u8x8_t *u8x8, uint8_t msg,
                                          uint8_t arg_int,
                                          void *arg_ptr)
{
    switch (msg) {
        case U8X8_MSG_DELAY_MILLI:
            vTaskDelay(pdMS_TO_TICKS(arg_int));
            break;
        case U8X8_MSG_GPIO_AND_DELAY_INIT:
            break;
        default:
            break;
    }
    return 1;
}

static uint8_t u8x8_byte_esp32_i2c(u8x8_t *u8x8, uint8_t msg,
                                     uint8_t arg_int, void *arg_ptr)
{
    static uint8_t buf[128];
    static size_t buf_idx;
    uint8_t *data;

    switch (msg) {
        case U8X8_MSG_BYTE_SEND:
            data = (uint8_t *)arg_ptr;
            while (arg_int > 0) {
                buf[buf_idx++] = *data++;
                arg_int--;
            }
            break;
        case U8X8_MSG_BYTE_INIT:
            break;
        case U8X8_MSG_BYTE_SET_DC:
            break;
        case U8X8_MSG_BYTE_START_TRANSFER:
            buf_idx = 0;
            break;
        case U8X8_MSG_BYTE_END_TRANSFER: {
            uint8_t addr = u8x8_GetI2CAddress(u8x8) >> 1;
            i2c_master_write_to_device(I2C_PORT, addr, buf, buf_idx,
                                       pdMS_TO_TICKS(100));
            break;
        }
    }
    return 1;
}

static void i2c_bus_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    i2c_param_config(I2C_PORT, &conf);
    i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
}


static void task_sample_imu(void *pv)
{
    ESP_LOGI(TAG, "IMU task started");

    if (!mpu6500_init(&imu, I2C_PORT)) {
        ESP_LOGE(TAG, "Error starting IMU");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "IMU initialized OK");

    mpu6500_set_accel_range(&imu, MPU6500_ACCEL_RANGE_4G);
    mpu6500_set_gyro_range(&imu, MPU6500_GYRO_RANGE_500DPS);
    mpu6500_set_dlpf(&imu, MPU6500_DLPF_20HZ);
    mpu6500_set_srd(&imu, SAMPLE_RATE_DIV);

    printf("timestamp_ms,roll,label\n");

    for (;;) {
        if (mpu6500_read(&imu)) {
            float ax = imu.accel[0];
            float ay = imu.accel[1];
            float az = imu.accel[2];

            float gx = imu.gyro[0];
            float gy = imu.gyro[1];

            float accel_roll  = atan2f(ay, -az) * (180.0f / M_PI);
            float accel_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * (180.0f / M_PI);

            if (!filterInitialized) {
                roll = accel_roll;
                pitch = accel_pitch;
                filterInitialized = true;
            } else {
                roll  = ALPHA * (roll  + gx * (180.0f / M_PI) * DT)
                      + (1.0f - ALPHA) * accel_roll;
                pitch = ALPHA * (pitch + gy * (180.0f / M_PI) * DT)
                      + (1.0f - ALPHA) * accel_pitch;
            }

            if (collecting) {
                int label = (isPressed == 0) ? 1 : 0;
                printf("%lu,%.2f,%.2f,%d\n", (unsigned long)millis(), roll, pitch, label);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void task_display_oled(void *pv)
{
    u8g2_Setup_ssd1306_i2c_128x64_noname_f(&u8g2, U8G2_R0,
                                             u8x8_byte_esp32_i2c,
                                             u8x8_gpio_and_delay_esp32);
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);

    u8g2_SetFont(&u8g2, u8g2_font_helvR08_tr);

    for (;;) {
        u8g2_ClearBuffer(&u8g2);

        // inverted title bar — white box with black text
        u8g2_SetDrawColor(&u8g2, 1);
        u8g2_DrawBox(&u8g2, 0, 0, 128, 12);
        u8g2_SetDrawColor(&u8g2, 0);
        u8g2_DrawStr(&u8g2, 20, 10, "CURL TRACKER");
        u8g2_SetDrawColor(&u8g2, 1);

        char buf[32];
        snprintf(buf, sizeof(buf), "REPS: %d", reps);
        u8g2_DrawStr(&u8g2, 0, 28, buf);

        snprintf(buf, sizeof(buf), "SETS: %d", sets);
        u8g2_DrawStr(&u8g2, 0, 42, buf);

        u8g2_DrawStr(&u8g2, 0, 56, "MQTT:OK");

        u8g2_SendBuffer(&u8g2);

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void task_sample_button(void *pv)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUTTON_PIN) | (1ULL << LED_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io_conf);

    // gpio_config sets both pins as input; override LED back to output
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    for (;;) {
        isPressed = gpio_get_level(BUTTON_PIN);

        if (isPressed == 1) {
            gpio_set_level(LED_PIN, 0);
        } else {
            gpio_set_level(LED_PIN, 1);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void task_vibrate_motor(void *pv)
{
    gpio_config_t motor_conf = {
        .pin_bit_mask = (1ULL << MOTOR_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    gpio_config(&motor_conf);

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        gpio_set_level(MOTOR_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(200));
        gpio_set_level(MOTOR_PIN, 0);
    }
}

#define INPUT_SIZE (MODEL_WINDOW_SIZE * 2)

static float win_roll[MODEL_WINDOW_SIZE];
static float win_pitch[MODEL_WINDOW_SIZE];
static int win_idx = 0;
static bool win_full = false;

static inline float relu(float x) { return x > 0.0f ? x : 0.0f; }

static float run_model(const float *input)
{
    float h0[16];
    for (int j = 0; j < 16; j++) {
        float sum = BIAS0[j];
        for (int i = 0; i < INPUT_SIZE; i++)
            sum += input[i] * WEIGHT0[i * 16 + j];
        h0[j] = relu(sum);
    }

    float h1[8];
    for (int j = 0; j < 8; j++) {
        float sum = BIAS1[j];
        for (int i = 0; i < 16; i++)
            sum += h0[i] * WEIGHT1[i * 8 + j];
        h1[j] = relu(sum);
    }

    float sum = BIAS2[0];
    for (int i = 0; i < 8; i++)
        sum += h1[i] * WEIGHT2[i];
    return 1.0f / (1.0f + expf(-sum));
}

static void task_inference(void *pv)
{
    bool wasCurling = false;
    int stepCount = 0;

    while (!filterInitialized)
        vTaskDelay(pdMS_TO_TICKS(100));

    for (;;) {
        win_roll[win_idx]  = roll;
        win_pitch[win_idx] = pitch;
        win_idx++;
        if (win_idx >= MODEL_WINDOW_SIZE) {
            win_full = true;
            win_idx  = 0;
        }

        stepCount++;

        // only run inference every 5 samples — no need to classify every frame
        if (win_full && stepCount >= 5) {
            stepCount = 0;

            // read the circular buffer in order and normalize to match training
            float input[INPUT_SIZE];
            for (int i = 0; i < MODEL_WINDOW_SIZE; i++) {
                int ri = (win_idx + i) % MODEL_WINDOW_SIZE;
                input[i * 2 + 0] = (win_roll[ri]  - NORM_MIN[0]) / (NORM_MAX[0] - NORM_MIN[0] + 1e-7f);
                input[i * 2 + 1] = (win_pitch[ri] - NORM_MIN[1]) / (NORM_MAX[1] - NORM_MIN[1] + 1e-7f);
            }

            float prob = run_model(input);
            bool isCurling = prob > 0.5f;

            // count on rising edge only — one rep per curl motion
            if (isCurling && !wasCurling) {
                reps++;
                xTaskNotifyGive(motorTaskHandle);
            }
            wasCurling = isCurling;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void task_serial_control(void *pv)
{
    for (;;) {
        int c = getchar();
        if (c == 's') {
            collecting = !collecting;
            if (collecting) {
                printf("Collecting start\n");
            } else {
                printf("Collecting stop\n");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}


void app_main(void)
{
    i2c_bus_init();

    xTaskCreate(task_sample_imu,     "sampleIMU",    4096, NULL, 1, NULL);
    xTaskCreate(task_display_oled,   "displayOLED",  4096, NULL, 1, NULL);
    xTaskCreate(task_sample_button,  "sampleButton", 2048, NULL, 1, NULL);
    xTaskCreate(task_vibrate_motor,  "vibrateMotor", 2048, NULL, 1, &motorTaskHandle);
    xTaskCreate(task_serial_control, "serialCtrl",   2048, NULL, 1, NULL);
    xTaskCreate(task_inference,      "inference",    8192, NULL, 1, NULL);
}
