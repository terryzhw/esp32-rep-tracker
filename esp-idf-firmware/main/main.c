// Bicep curl counter. IMU on the forearm -> complementary filter -> tiny NN
// that says "curling / not curling" -> count the rising edges.
//
// Split into tasks mainly because the OLED redraw is slow and blocking on it
// would wreck the 100 Hz sample timing.

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

#define I2C_SDA_PIN         8
#define I2C_SCL_PIN         9
#define LED_PIN             10
#define BUTTON_PIN          4
#define MOTOR_PIN           3

#define I2C_PORT            I2C_NUM_0
#define I2C_FREQ_HZ         400000

// Button pulls to ground, so pressed reads low. Naming these stops me from
// getting the polarity backwards every time I touch this code.
#define BUTTON_RELEASED     1
#define BUTTON_PRESSED      0

// Gyro is smooth but drifts, accelerometer is drift-free but jumps around
// whenever the arm accelerates. 0.9 leans on the gyro for responsiveness and
// lets gravity slowly pull the estimate back straight. Worth re-tuning if the
// angle either wanders over a long set or looks too noisy to classify.
#define ALPHA               0.90f

// Sensor runs at 1 kHz internally and divides by (1 + SRD), so this is 100 Hz.
#define SAMPLE_RATE_DIV     9
#define DT                  ((1.0f + SAMPLE_RATE_DIV) / 1000.0f)

#define RAD_TO_DEG          (180.0f / (float)M_PI)

#define IMU_PERIOD_MS       10
#define DISPLAY_PERIOD_MS   50   // shares the bus with the IMU, so don't push it
#define BUTTON_PERIOD_MS    20
#define SERIAL_PERIOD_MS    50
#define INFERENCE_PERIOD_MS 10

#define MOTOR_BUZZ_MS       200  // short enough not to overlap the next rep

#define NUM_FEATURES        2
#define INPUT_SIZE          (MODEL_WINDOW_SIZE * NUM_FEATURES)
#define HIDDEN0_SIZE        16
#define HIDDEN1_SIZE        8

#define CURL_THRESHOLD      0.5f
#define INFERENCE_STRIDE    5

static mpu6500_t imu;
static u8g2_t u8g2;

// Shared between tasks. Everything here is word sized, so a torn read isn't
// possible and I skipped the mutex.
static float roll_deg  = 0.0f;
static float pitch_deg = 0.0f;
static volatile bool filter_ready = false;
static volatile bool collecting = false;
static volatile int button_level = BUTTON_RELEASED;
static volatile int rep_count = 0;
static volatile int set_count = 0;

static TaskHandle_t motor_task_handle = NULL;

static uint32_t millis(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

// u8g2 is platform agnostic, so it calls back into these two for delays and
// for anything that touches the bus.
static uint8_t u8x8_gpio_and_delay_esp32(u8x8_t *u8x8, uint8_t msg,
                                         uint8_t arg_int, void *arg_ptr)
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

// Collect the whole frame first and send it as one transaction. A separate
// I2C transfer per byte would spend most of the time on start/stop overhead.
static uint8_t u8x8_byte_esp32_i2c(u8x8_t *u8x8, uint8_t msg,
                                   uint8_t arg_int, void *arg_ptr)
{
    static uint8_t buf[128];
    static size_t buf_idx;
    uint8_t *data;

    switch (msg) {
        case U8X8_MSG_BYTE_START_TRANSFER:
            buf_idx = 0;
            break;

        case U8X8_MSG_BYTE_SEND:
            data = (uint8_t *)arg_ptr;
            while (arg_int > 0) {
                buf[buf_idx++] = *data++;
                arg_int--;
            }
            break;

        case U8X8_MSG_BYTE_END_TRANSFER: {
            // u8g2 keeps the address pre-shifted, ESP-IDF wants the raw 7 bits
            uint8_t addr = u8x8_GetI2CAddress(u8x8) >> 1;
            i2c_master_write_to_device(I2C_PORT, addr, buf, buf_idx,
                                       pdMS_TO_TICKS(100));
            break;
        }

        case U8X8_MSG_BYTE_INIT:
        case U8X8_MSG_BYTE_SET_DC:
            // bus is already up, and I2C has no DC line
            break;
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

static bool imu_init(void)
{
    if (!mpu6500_init(&imu, I2C_PORT)) {
        return false;
    }

    // 4 g leaves headroom over the 1 g resting reading for the fast part of
    // the curl. A curl is around 1 Hz, so the 20 Hz low pass drops tremor and
    // strap slap without touching the motion I care about.
    mpu6500_set_accel_range(&imu, MPU6500_ACCEL_RANGE_4G);
    mpu6500_set_gyro_range(&imu, MPU6500_GYRO_RANGE_500DPS);
    mpu6500_set_dlpf(&imu, MPU6500_DLPF_20HZ);
    mpu6500_set_srd(&imu, SAMPLE_RATE_DIV);
    return true;
}

static void task_sample_imu(void *pv)
{
    ESP_LOGI(TAG, "IMU task started");

    if (!imu_init()) {
        ESP_LOGE(TAG, "Error starting IMU");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "IMU initialized OK");

    // Header for the logging mode below, so the capture can be piped straight
    // into a csv and loaded by the training script.
    printf("timestamp_ms,roll,pitch,label\n");

    for (;;) {
        if (mpu6500_read(&imu)) {
            float ax = imu.accel[0];
            float ay = imu.accel[1];
            float az = imu.accel[2];
            float gx = imu.gyro[0];
            float gy = imu.gyro[1];

            // Gravity is the only absolute reference we have for tilt.
            float accel_roll  = atan2f(ay, -az) * RAD_TO_DEG;
            float accel_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;

            if (!filter_ready) {
                // Start from the accelerometer, otherwise the first couple of
                // seconds are spent crawling up from zero and the model sees
                // garbage.
                roll_deg  = accel_roll;
                pitch_deg = accel_pitch;
                filter_ready = true;
            } else {
                roll_deg  = ALPHA * (roll_deg  + gx * RAD_TO_DEG * DT)
                          + (1.0f - ALPHA) * accel_roll;
                pitch_deg = ALPHA * (pitch_deg + gy * RAD_TO_DEG * DT)
                          + (1.0f - ALPHA) * accel_pitch;
            }

            if (collecting) {
                // Holding the button is how I label a rep while recording.
                int label = (button_level == BUTTON_PRESSED) ? 1 : 0;
                printf("%lu,%.2f,%.2f,%d\n",
                       (unsigned long)millis(), roll_deg, pitch_deg, label);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(IMU_PERIOD_MS));
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

        // Filled box then color 0 text is how you get inverted video here,
        // there's no invert flag in u8g2.
        u8g2_SetDrawColor(&u8g2, 1);
        u8g2_DrawBox(&u8g2, 0, 0, 128, 12);
        u8g2_SetDrawColor(&u8g2, 0);
        u8g2_DrawStr(&u8g2, 20, 10, "CURL TRACKER");
        u8g2_SetDrawColor(&u8g2, 1);

        char line[32];
        snprintf(line, sizeof(line), "REPS: %d", rep_count);
        u8g2_DrawStr(&u8g2, 0, 28, line);

        snprintf(line, sizeof(line), "SETS: %d", set_count);
        u8g2_DrawStr(&u8g2, 0, 42, line);

        u8g2_DrawStr(&u8g2, 0, 56, "MQTT:OK");

        u8g2_SendBuffer(&u8g2);

        vTaskDelay(pdMS_TO_TICKS(DISPLAY_PERIOD_MS));
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

    // Both pins went through gpio_config as inputs, so put the LED back.
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    for (;;) {
        button_level = gpio_get_level(BUTTON_PIN);

        // LED is just so I can see the label state while recording data.
        gpio_set_level(LED_PIN, button_level == BUTTON_PRESSED ? 1 : 0);

        vTaskDelay(pdMS_TO_TICKS(BUTTON_PERIOD_MS));
    }
}

static void task_vibrate_motor(void *pv)
{
    gpio_config_t motor_conf = {
        .pin_bit_mask = (1ULL << MOTOR_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&motor_conf);

    for (;;) {
        // The buzz lives in its own task so the 200 ms delay doesn't stall
        // inference. Notifications are cheaper than a queue and there's
        // nothing to pass along anyway.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        gpio_set_level(MOTOR_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(MOTOR_BUZZ_MS));
        gpio_set_level(MOTOR_PIN, 0);
    }
}

// Circular buffer of the last MODEL_WINDOW_SIZE angle pairs.
static float window_roll[MODEL_WINDOW_SIZE];
static float window_pitch[MODEL_WINDOW_SIZE];
static int window_idx = 0;
static bool window_full = false;

static inline float relu(float x)
{
    return x > 0.0f ? x : 0.0f;
}

static inline float sigmoid(float x)
{
    return 1.0f / (1.0f + expf(-x));
}

// Weights are exported row major from the training script, so the index math
// below has to match that layout exactly.
static float run_model(const float *input)
{
    float hidden0[HIDDEN0_SIZE];
    for (int j = 0; j < HIDDEN0_SIZE; j++) {
        float sum = BIAS0[j];
        for (int i = 0; i < INPUT_SIZE; i++) {
            sum += input[i] * WEIGHT0[i * HIDDEN0_SIZE + j];
        }
        hidden0[j] = relu(sum);
    }

    float hidden1[HIDDEN1_SIZE];
    for (int j = 0; j < HIDDEN1_SIZE; j++) {
        float sum = BIAS1[j];
        for (int i = 0; i < HIDDEN0_SIZE; i++) {
            sum += hidden0[i] * WEIGHT1[i * HIDDEN1_SIZE + j];
        }
        hidden1[j] = relu(sum);
    }

    float sum = BIAS2[0];
    for (int i = 0; i < HIDDEN1_SIZE; i++) {
        sum += hidden1[i] * WEIGHT2[i];
    }
    return sigmoid(sum);
}

// Unwrap the circular buffer oldest first and apply the same min/max scaling
// the model was trained with. Feeding it raw degrees gives nonsense output.
static void build_model_input(float *input)
{
    for (int i = 0; i < MODEL_WINDOW_SIZE; i++) {
        int ri = (window_idx + i) % MODEL_WINDOW_SIZE;
        input[i * NUM_FEATURES + 0] =
            (window_roll[ri]  - NORM_MIN[0]) / (NORM_MAX[0] - NORM_MIN[0] + 1e-7f);
        input[i * NUM_FEATURES + 1] =
            (window_pitch[ri] - NORM_MIN[1]) / (NORM_MAX[1] - NORM_MIN[1] + 1e-7f);
    }
}

static void task_inference(void *pv)
{
    bool was_curling = false;
    int samples_since_inference = 0;

    // No point buffering the startup transient.
    while (!filter_ready) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    for (;;) {
        window_roll[window_idx]  = roll_deg;
        window_pitch[window_idx] = pitch_deg;
        window_idx++;
        if (window_idx >= MODEL_WINDOW_SIZE) {
            window_full = true;
            window_idx = 0;
        }

        samples_since_inference++;

        // Back to back windows share 49 of 50 samples, so running the model
        // every time would be almost pure wasted math.
        if (window_full && samples_since_inference >= INFERENCE_STRIDE) {
            samples_since_inference = 0;

            float input[INPUT_SIZE];
            build_model_input(input);

            bool is_curling = run_model(input) > CURL_THRESHOLD;

            // Rising edge only. The classifier stays high for the whole curl,
            // so counting on the level would add a rep every inference.
            if (is_curling && !was_curling) {
                rep_count++;
                xTaskNotifyGive(motor_task_handle);
            }
            was_curling = is_curling;
        }

        vTaskDelay(pdMS_TO_TICKS(INFERENCE_PERIOD_MS));
    }
}

// 's' over the serial monitor toggles data capture for training.
static void task_serial_control(void *pv)
{
    for (;;) {
        int c = getchar();
        if (c == 's') {
            collecting = !collecting;
            printf(collecting ? "Collecting start\n" : "Collecting stop\n");
        }
        vTaskDelay(pdMS_TO_TICKS(SERIAL_PERIOD_MS));
    }
}

void app_main(void)
{
    i2c_bus_init();

    // Equal priority everywhere, each task blocks on its own delay, so the
    // scheduler just round-robins. Inference gets the big stack because the
    // input vector and hidden layers are all on it.
    xTaskCreate(task_sample_imu,     "sampleIMU",    4096, NULL, 1, NULL);
    xTaskCreate(task_display_oled,   "displayOLED",  4096, NULL, 1, NULL);
    xTaskCreate(task_sample_button,  "sampleButton", 2048, NULL, 1, NULL);
    xTaskCreate(task_vibrate_motor,  "vibrateMotor", 2048, NULL, 1, &motor_task_handle);
    xTaskCreate(task_serial_control, "serialCtrl",   2048, NULL, 1, NULL);
    xTaskCreate(task_inference,      "inference",    8192, NULL, 1, NULL);
}
