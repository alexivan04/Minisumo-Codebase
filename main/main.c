#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"

#include "bdc_motor.h"
#include "main_config.h"
#include "robot_types.h"
#include "strategies.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"

#define WIFI_SSID      "MiniSumo_Telemetry"
#define WIFI_PASS      "robot123"
#define UDP_PORT       3333

// --- Global Variables ---
bdc_motor_handle_t motor1 = NULL, motor2 = NULL;
esp_timer_handle_t motorControlTimer = NULL;
static const char *TAG = "main";

int enemy_seen_counter = 0;
int64_t initial_move_start_time = 0;
int initial_move_duration_ms = 0;
bool is_in_timed_retreat = false;
int64_t retreat_start_time = 0;
int64_t retreat_ignore_time = 0;
int64_t last_enemy_seen_time = 0;
bool is_seen_right_line = false;
bool is_seen_left_line = false;

state_t state = PATROL;
int output = 0; 
state_t state_PID = PATROL;
int output_PID = 0;

uint8_t dist_center, dist_r90, dist_r, dist_l90, dist_l;
#ifdef ROBOT_VECHI
uint8_t dist_rf, dist_lf;
#endif
uint8_t line_right, line_left;

int mode = 0;
int wing_mode = 0;
bool is_running = false; 
bool initial_move_done = false; 

strategy_func_t current_strategy = NULL;

SemaphoreHandle_t sensorsMutex; 
SemaphoreHandle_t pidMutex;     

volatile imu_data_t g_imu_processed; 
QueueHandle_t imu_event_queue;

// --==================== TELEMETRY =====================--

static void wifi_init_softap(void) {
    ESP_LOGI(TAG, "Initializing WiFi SoftAP...");
    ESP_ERROR_CHECK(esp_netif_init());
    // Only create default loop if not already created
    esp_err_t err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = strlen(WIFI_SSID),
            .channel = 1,
            .password = WIFI_PASS,
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .required = false,
            },
        },
    };

    if (strlen(WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi AP Started. SSID:%s Password:%s", WIFI_SSID, WIFI_PASS);
}

void telemetry_task(void *pvParameters) {
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = inet_addr("192.168.4.255"); // Broadcast
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(UDP_PORT);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    int broadcast_en = 1;
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast_en, sizeof(broadcast_en));

    // Increase task priority so it doesn't get starved
    vTaskPrioritySet(NULL, 6); 

    char payload[256];
    uint32_t start_signal_ms = 0;
    bool timer_latched = false;

    while (1) {
        // Latch the time when we first receive the start signal
        if (is_running && !timer_latched) {
            start_signal_ms = esp_log_timestamp();
            timer_latched = true;
        } else if (!is_running) {
            timer_latched = false; // Reset if robot is stopped
        }

        uint32_t current_ms = esp_log_timestamp();
        uint32_t relative_ts = timer_latched ? (current_ms - start_signal_ms) : 0;

        // Human-friendly terminal-style formatting with timestamp relative to START SIGNAL
        // Digital Sensors: S:10101 (Center, R90, R, L90, L) - 1 if object SEEN
        // Line Sensors: L:RL (Right, Left) - 1 if whitespace SEEN
        snprintf(payload, sizeof(payload), 
            "[%7.2f s] [M:%d W:%d] [S:%d%d%d%d%d L:%d%d] [State:%d] Pwr:%4d | Yaw:%6.1f | Ax:%5.1fG | Impact:%s\n", 
            (float)relative_ts / 1000.0f,
            mode,
            wing_mode,
            dist_center, 
            dist_r90, 
            dist_r,
            dist_l90,
            dist_l,
            line_right, line_left,
            state_PID, 
            output_PID,
            g_imu_processed.yaw, 
            (float)(g_imu_processed.acc_x - g_imu_processed.acc_x_offset) / 2048.0f,
            g_imu_processed.impact_detected ? "!!HIT!!" : (g_imu_processed.impact_side_left ? "SIDE-L" : (g_imu_processed.impact_side_right ? "SIDE-R" : "SAFE")));
        
        int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
        }
        
        vTaskDelay(pdMS_TO_TICKS(20)); // ~25Hz
    }
}

// --==================== UTILITY FUNCTIONS =====================--

int percent_to_duty_cycle(int percent) {
    return (percent * BDC_MCPWM_DUTY_TICK_MAX) / 100;
}

static int clamp_percent_0_100(int p) {
    if (p < 0) return 0;
    if (p > 100) return 100;
    return p;
}

void blink_led(int cnt, int delay) {
    for (int i = 0; i < cnt; i++) {
        gpio_set_level(ONBOARD_LED, 1);
        vTaskDelay(delay / portTICK_PERIOD_MS);
        gpio_set_level(ONBOARD_LED, 0);
        vTaskDelay(delay / portTICK_PERIOD_MS);
    }
}

// --==================== HARDWARE SETUP =====================--

static esp_err_t mpu9250_register_write(uint8_t reg_addr, uint8_t data) {
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_write_to_device(I2C_MASTER_NUM, MPU9250_ADDR, write_buf, sizeof(write_buf), pdMS_TO_TICKS(100));
}

static esp_err_t mpu9250_register_read(uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU9250_ADDR, &reg_addr, 1, data, len, pdMS_TO_TICKS(100));
}

void mpu9250_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);

    mpu9250_register_write(MPU9250_PWR_MGMT_1, 0x01); 
    vTaskDelay(pdMS_TO_TICKS(100));
    mpu9250_register_write(0x1B, 0x18); 
    mpu9250_register_write(0x1C, 0x18);
    mpu9250_register_write(MPU9250_INT_ENABLE, 0x01);

    // Calibration logic
    ESP_LOGI(TAG, "Starting IMU Calibration (Keep robot still)...");
    gpio_set_level(ONBOARD_LED, 1);
    
    float gyro_sum = 0;
    float accel_x_sum = 0;
    float accel_y_sum = 0;
    float accel_z_sum = 0;
    int samples = 500;
    uint8_t data[14];
    
    for (int i = 0; i < samples; i++) {
        if (mpu9250_register_read(MPU9250_ACCEL_XOUT_H, data, 14) == ESP_OK) {
            int16_t ax = (int16_t)((data[0] << 8) | data[1]);
            int16_t ay = (int16_t)((data[2] << 8) | data[3]);
            int16_t az = (int16_t)((data[4] << 8) | data[5]);
            int16_t gz = (int16_t)((data[12] << 8) | data[13]);
            accel_x_sum += ax;
            accel_y_sum += ay;
            accel_z_sum += az;
            gyro_sum += gz;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    
    g_imu_processed.acc_x_offset = accel_x_sum / (float)samples;
    g_imu_processed.acc_y_offset = accel_y_sum / (float)samples;
    g_imu_processed.acc_z_nominal = accel_z_sum / (float)samples;
    g_imu_processed.gyro_z_offset = gyro_sum / (float)samples;
    g_imu_processed.yaw = 0.0f;
    
    ESP_LOGI(TAG, "Calibration Done! Gyro Offset: %.2f, AccZ Nominal: %.2f, AccXY Offsets: %.1f,%.1f", 
             g_imu_processed.gyro_z_offset, g_imu_processed.acc_z_nominal,
             g_imu_processed.acc_x_offset, g_imu_processed.acc_y_offset);
    
    gpio_set_level(ONBOARD_LED, 0);
}

static void IRAM_ATTR imu_gpio_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(imu_event_queue, &gpio_num, NULL);
}

void imu_interrupt_setup() {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,
        .pin_bit_mask = (1ULL << MPU9250_INT_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = 1,
    };
    gpio_config(&io_conf);
    
    imu_event_queue = xQueueCreate(10, sizeof(uint32_t));
    gpio_install_isr_service(0);
    gpio_isr_handler_add(MPU9250_INT_GPIO, imu_gpio_isr_handler, (void*) MPU9250_INT_GPIO);
}

void servo_init(void) {
    ledc_timer_config_t timer = {
        .speed_mode       = SERVO_MODE,
        .timer_num        = SERVO_TIMER,
        .duty_resolution  = SERVO_RESOLUTION,
        .freq_hz          = SERVO_FREQ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t channel = {
        .gpio_num       = SERVO_GPIO,
        .speed_mode     = SERVO_MODE,
        .channel        = SERVO_CHANNEL,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = SERVO_TIMER,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&channel);
}

void servo_set_angle(float angle) {
    if(angle < 0) angle = 0;
    if(angle > 180) angle = 180;
    uint32_t max_duty = (1 << SERVO_RESOLUTION) - 1;
    float period_ms = 20.0;     
    float min_pulse = 0.5;
    float max_pulse = 2.5;
    float pulse = min_pulse + (angle / 180.0) * (max_pulse - min_pulse);
    uint32_t duty = (pulse / period_ms) * max_duty;
    ledc_set_duty(SERVO_MODE, SERVO_CHANNEL, duty);
    ledc_update_duty(SERVO_MODE, SERVO_CHANNEL);
}

static int setup() {
    gpio_reset_pin(ONBOARD_LED);
    gpio_set_direction(ONBOARD_LED, GPIO_MODE_OUTPUT);
    gpio_reset_pin(MODE_BUTTON);
    gpio_set_direction(MODE_BUTTON, GPIO_MODE_INPUT);
    gpio_reset_pin(DIST_CENTER);
    gpio_set_direction(DIST_CENTER, GPIO_MODE_INPUT);
    gpio_reset_pin(DIST_R90);
    gpio_set_direction(DIST_R90, GPIO_MODE_INPUT);
    gpio_reset_pin(DIST_R);
    gpio_set_direction(DIST_R, GPIO_MODE_INPUT);
    gpio_reset_pin(DIST_L90);
    gpio_set_direction(DIST_L90, GPIO_MODE_INPUT);
    gpio_reset_pin(DIST_L);
    gpio_set_direction(DIST_L, GPIO_MODE_INPUT);
    gpio_reset_pin(LINE_RIGHT);
    gpio_set_direction(LINE_RIGHT, GPIO_MODE_INPUT);
    gpio_reset_pin(LINE_LEFT);
    gpio_set_direction(LINE_LEFT, GPIO_MODE_INPUT);

#ifdef ROBOT_VECHI
    gpio_reset_pin(DIST_RF);
    gpio_set_direction(DIST_RF, GPIO_MODE_INPUT);
    gpio_reset_pin(DIST_LF);
    gpio_set_direction(DIST_LF, GPIO_MODE_INPUT);
#endif

    bdc_motor_config_t motor_config1 = {
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
        .pwma_gpio_num = BDC_MCPWM_GPIO_1A,
        .pwmb_gpio_num = BDC_MCPWM_GPIO_1B,
    };
    bdc_motor_mcpwm_config_t mcpwm_config1 = {
        .group_id = 0,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config1, &mcpwm_config1, &motor1));
    ESP_ERROR_CHECK(bdc_motor_enable(motor1));
    ESP_ERROR_CHECK(bdc_motor_forward(motor1));

    bdc_motor_config_t motor_config2 = {
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
        .pwma_gpio_num = BDC_MCPWM_GPIO_2A,
        .pwmb_gpio_num = BDC_MCPWM_GPIO_2B,
    };
    bdc_motor_mcpwm_config_t mcpwm_config2 = {
        .group_id = 0,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config2, &mcpwm_config2, &motor2));
    ESP_ERROR_CHECK(bdc_motor_enable(motor2));
    ESP_ERROR_CHECK(bdc_motor_forward(motor2));
    
    return 0;
}

// --==================== SENSOR READING ======================--

void read_sensors() {
    uint8_t dist_center_local = gpio_get_level(DIST_CENTER);
    uint8_t dist_r90_local = gpio_get_level(DIST_R90);
    uint8_t dist_r_local = gpio_get_level(DIST_R);
    uint8_t dist_l90_local = gpio_get_level(DIST_L90);
    uint8_t dist_l_local = gpio_get_level(DIST_L);

#ifdef ROBOT_VECHI
    uint8_t dist_rf_local = gpio_get_level(DIST_RF);
    uint8_t dist_lf_local = gpio_get_level(DIST_LF);
#endif

#ifdef BLACK_FIELD
    uint8_t line_right_local = !gpio_get_level(LINE_RIGHT);
    uint8_t line_left_local = !gpio_get_level(LINE_LEFT);
#else
    uint8_t line_right_local = gpio_get_level(LINE_RIGHT);
    uint8_t line_left_local = gpio_get_level(LINE_LEFT);
#endif

    // RAW SENSOR WRITE: No buffering or confirmation logic needed per user request
    if (xSemaphoreTake(sensorsMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        dist_center = dist_center_local;
        dist_r90 = dist_r90_local;
        dist_r = dist_r_local;
        dist_l90 = dist_l90_local;
        dist_l = dist_l_local;
        line_right = line_right_local;
        line_left = line_left_local;
#ifdef ROBOT_VECHI
        dist_rf = dist_rf_local;
        dist_lf = dist_lf_local;
#endif
        xSemaphoreGive(sensorsMutex);
    }
}

// --==================== CORE TASKS =====================--

void motor_controller_callback(void *arg) {
    #ifdef USE_START_STOP_MODULE
    if (!gpio_get_level(START_STOP_MODULE)) {
        is_running = false;
    }
    #endif

    if (!is_running) {
        bdc_motor_brake(motor1);
        bdc_motor_brake(motor2);
        return;
    }
    
    state_t local_state;
    int local_output;
    if (xSemaphoreTake(pidMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        local_state = state;
        local_output = output;
        xSemaphoreGive(pidMutex);
    } else {
        bdc_motor_brake(motor1);
        bdc_motor_brake(motor2);
        return;
    }

    switch (local_state) {
        case PATROL:
        case ATTACK:
            bdc_motor_forward(motor1);
            bdc_motor_forward(motor2);
            {
                int left_percent = clamp_percent_0_100((int)((float)local_output * LEFT_MOTOR_MULTIPLIER));
                int right_percent = clamp_percent_0_100((int)((float)local_output * RIGHT_MOTOR_MULTIPLIER));
                bdc_motor_set_speed(motor1, percent_to_duty_cycle(left_percent));
                bdc_motor_set_speed(motor2, percent_to_duty_cycle(right_percent));
            }
            break;

        case FOUND:
            {
                int out_abs = abs(local_output);
                int left_percent = clamp_percent_0_100((int)((float)out_abs * LEFT_MOTOR_MULTIPLIER));
                int right_percent = clamp_percent_0_100((int)((float)out_abs * RIGHT_MOTOR_MULTIPLIER));
                if (output < 0) {
                    bdc_motor_reverse(motor1);
                    bdc_motor_forward(motor2);
                } else {
                    bdc_motor_forward(motor1);
                    bdc_motor_reverse(motor2);
                }
                bdc_motor_set_speed(motor1, percent_to_duty_cycle(left_percent));
                bdc_motor_set_speed(motor2, percent_to_duty_cycle(right_percent));
            }
            break;

        case ARCHING:
        case ARCHING_VIPER:
            {
                bdc_motor_forward(motor1);
                bdc_motor_forward(motor2);
                float ratio = (local_state == ARCHING) ? 0.68f : 0.55f;
                int base = abs(local_output);
                if (local_output > 0) { // Arc RIGHT
                    int left_percent = clamp_percent_0_100((int)((float)base * LEFT_MOTOR_MULTIPLIER));
                    int right_percent = clamp_percent_0_100((int)((float)base * ratio * RIGHT_MOTOR_MULTIPLIER));
                    bdc_motor_set_speed(motor1, percent_to_duty_cycle(left_percent));
                    bdc_motor_set_speed(motor2, percent_to_duty_cycle(right_percent));
                } else { // Arc LEFT
                    int left_percent = clamp_percent_0_100((int)((float)base * ratio * LEFT_MOTOR_MULTIPLIER));
                    int right_percent = clamp_percent_0_100((int)((float)base * RIGHT_MOTOR_MULTIPLIER));
                    bdc_motor_set_speed(motor1, percent_to_duty_cycle(left_percent));
                    bdc_motor_set_speed(motor2, percent_to_duty_cycle(right_percent));
                }
            }
            break;

        case RETREAT:
            bdc_motor_reverse(motor1);
            bdc_motor_reverse(motor2);
            if (is_seen_left_line) {
                bdc_motor_set_speed(motor1, percent_to_duty_cycle(100));
                bdc_motor_set_speed(motor2, percent_to_duty_cycle(65));
            } else {
                bdc_motor_set_speed(motor1, percent_to_duty_cycle(65));
                bdc_motor_set_speed(motor2, percent_to_duty_cycle(100));
            }
            vTaskDelay(pdMS_TO_TICKS(200));
            break;

        case IDLE:
        default:
            bdc_motor_brake(motor1);
            bdc_motor_brake(motor2);
            break;
    }
}

void run_strategy_task(void *arg) {
    bool initial_move_timer_started = false;
    while (1) {
        if (is_running && current_strategy != NULL) {
            if (is_in_timed_retreat) {
                int64_t elapsed_retreat_time = (esp_timer_get_time() - retreat_start_time) / 1000;
                if (elapsed_retreat_time < RETREAT_DURATION_MS) {
                    state_PID = RETREAT;
                } else {
                    is_in_timed_retreat = false;
                }
            }
            if (!is_in_timed_retreat) {
                read_sensors();
                int64_t elapsed_retreat_ignore_time = (esp_timer_get_time() - retreat_ignore_time) / 1000;
#ifdef USE_LINE_SENSORS
                if ((line_left || line_right) && (elapsed_retreat_ignore_time > RETREAT_IGNORE_MS)) {
                    if (line_left) { is_seen_left_line = true; is_seen_right_line = false; }
                    else { is_seen_right_line = true; is_seen_left_line = false; }
                    is_in_timed_retreat = true;
                    retreat_start_time = esp_timer_get_time();
                    retreat_ignore_time = esp_timer_get_time();
                    state_PID = RETREAT;
                } else {
                    current_strategy();
                }
#else
                current_strategy();
#endif
            }
            if (!initial_move_done && initial_move_duration_ms > 0) {
                if (!initial_move_timer_started) {
                    initial_move_start_time = esp_timer_get_time();
                    initial_move_timer_started = true;
                    // Reset yaw at the start of a timed/gyro move to have a fresh reference
                    g_imu_processed.yaw = 0.0f;
                }
                
                bool move_finished = false;
                
                // If it's a turn-based initial move, use Gyro
                if (current_strategy == &strategy_hunter) {
                    // Hunter usually does a scan turn. Let's make it scan 90 degrees.
                    float diff = fabs(g_imu_processed.yaw);
                    if (diff >= 90.0f) move_finished = true;
                } 
                else if (current_strategy == &strategy_matador_left || current_strategy == &strategy_matador_right) {
                    // Matador does an arc. Let's say 45 degrees of arcing.
                    float diff = fabs(g_imu_processed.yaw);
                    if (diff >= 45.0f) move_finished = true;
                }
                else if (current_strategy == &strategy_viper) {
                    // Viper initial move is a pre-turn. Let's say 30 degrees.
                    float diff = fabs(g_imu_processed.yaw);
                    if (diff >= 30.0f) move_finished = true;
                }
                else if (current_strategy == &strategy_dash_left || current_strategy == &strategy_dash_right) {
                    // For complex multi-stage moves, we check if the strategy itself 
                    // decided it's done (it internally calls hunter or sets initial_move_done)
                    // But we keep the time-based safety net.
                    int64_t elapsed_time_ms = (esp_timer_get_time() - initial_move_start_time) / 1000;
                    if (elapsed_time_ms >= initial_move_duration_ms) move_finished = true;
                }
                else {
                    // Default to time-based for straight moves like Bulldozer or DASH
                    int64_t elapsed_time_ms = (esp_timer_get_time() - initial_move_start_time) / 1000;
                    if (elapsed_time_ms >= initial_move_duration_ms) move_finished = true;
                }

                if (move_finished) {
                    initial_move_done = true;
                }
            }

            // --- Stall Supervisor (Flip detection disabled) ---
            if (is_running) {
                // Stall Detection: If at high power and not moving for too long
                // This now runs even if in timed_retreat or specialized modes to ensure robot never stays stuck
                static int64_t last_movement_time = 0;
                if (!g_imu_processed.robot_stalled) {
                    last_movement_time = esp_timer_get_time();
                } else if (abs(output_PID) >= 80) { // Only check stall if we are actually trying to move fast
                    int64_t stall_duration = (esp_timer_get_time() - last_movement_time) / 1000;
                    if (stall_duration >= STALL_POTENTIAL_MS) {
                        ESP_LOGW("Supervisor", "STALL DETECTED! Executing wiggle...");
                        // Trigger a small wiggle or retreat to unstuck
                        state_PID = FOUND;
                        output_PID = (output_PID > 0) ? -70 : 70; // Reverse the intended direction
                        // We reset the timer slightly so we don't spam wiggle
                        last_movement_time = esp_timer_get_time() - (500 * 1000); 
                    }
                }
            }
        } else {
            state_PID = IDLE;
            output_PID = 0;
        }
        if (xSemaphoreTake(pidMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            state = state_PID;
            output = output_PID;
            xSemaphoreGive(pidMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(SENSOR_READING_DELAY));
    }
}

void imu_task(void *arg) {
    uint32_t io_num;
    uint8_t data[14];
    int64_t last_time = esp_timer_get_time();
    int64_t last_impact_time = 0;
    const float gyro_scale = 16.4; 
    while (1) {
        if (xQueueReceive(imu_event_queue, &io_num, portMAX_DELAY)) {
            if (mpu9250_register_read(MPU9250_ACCEL_XOUT_H, data, 14) == ESP_OK) {
                int64_t now = esp_timer_get_time();
                float dt = (now - last_time) / 1000000.0f;
                last_time = now;
                
                int16_t ax = (int16_t)((data[0] << 8) | data[1]);
                int16_t ay = (int16_t)((data[2] << 8) | data[3]);
                int16_t az = (int16_t)((data[4] << 8) | data[5]);
                
                // Accelerometer raw to G (assuming +/- 16G range set during init)
                // 16G range = 2048 LSB/g
                float ax_g = (float)(ax - g_imu_processed.acc_x_offset) / 2048.0f;
                float ay_g = (float)(ay - g_imu_processed.acc_y_offset) / 2048.0f;
                float az_g = (float)az / 2048.0f;

                // Calculate impact magnitude (simple vector length on XY)
                float impact = sqrtf(ax_g*ax_g + ay_g*ay_g);

                if (xSemaphoreTake(sensorsMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                    g_imu_processed.acc_x = ax;
                    g_imu_processed.acc_y = ay;
                    g_imu_processed.acc_z = az;
                    
                    g_imu_processed.impact_magnitude = impact;
                    if (impact > IMPACT_THRESHOLD_G && (now - last_impact_time) > IMPACT_COOLDOWN_MS * 1000) {
                        g_imu_processed.impact_detected = true;
                        
                        // Determine side based on Acc Y (Lateral direction)
                        if (ay_g > 1.0f) { // Significant lateral G
                            g_imu_processed.impact_side_left = true;
                            g_imu_processed.impact_side_right = false;
                        } else if (ay_g < -1.0f) {
                            g_imu_processed.impact_side_left = false;
                            g_imu_processed.impact_side_right = true;
                        } else {
                            g_imu_processed.impact_side_left = false;
                            g_imu_processed.impact_side_right = false;
                        }

                        last_impact_time = now;
                    } else {
                        g_imu_processed.impact_detected = false;
                        g_imu_processed.impact_side_left = false;
                        g_imu_processed.impact_side_right = false;
                    }

                    // Flip Detection (Using Z acceleration)
                    // If Z-axis is reversed or significantly weak while robot is supposedly on ground
                    // Note: Z axis is UP, so nominal is ~ +1.0G (2048 LSB)
                    // We check if Z is less than -0.5G (strongly pointing down) to confirm flip
                    if (az_g < FLIP_THRESHOLD_Z) {
                        g_imu_processed.robot_flipped = true;
                    } else {
                        g_imu_processed.robot_flipped = false;
                    }

                    // Stall Detection
                    // If we are at high power (calculated in state machine) but X accel is near zero
                    // This is better checked in the strategy/supervisor level where 'output' is known.
                    // For now, we update the raw state.
                    if (fabsf(ax_g) < STALL_THRESHOLD_ACCEL && fabsf(ay_g) < STALL_THRESHOLD_ACCEL) {
                         // Potential stall - let strategy verify with motor power
                         g_imu_processed.robot_stalled = true; 
                    } else {
                         g_imu_processed.robot_stalled = false;
                    }

                    // TESTING: Log when stall or flip is detected (throttled)
                    static int log_div = 0;
                    if (++log_div >= 50) {
                        if (g_imu_processed.robot_stalled || g_imu_processed.robot_flipped) {
                            ESP_LOGI("IMU_TEST", "Status: %s %s | Z_G: %.2f | X_G: %.2f", 
                                g_imu_processed.robot_stalled ? "[STALL]" : "",
                                g_imu_processed.robot_flipped ? "[FLIP]" : "",
                                az_g, ax_g);
                        }
                        log_div = 0;
                    }

                    float gz_raw = (int16_t)((data[12] << 8) | data[13]) - g_imu_processed.gyro_z_offset;
                    if (fabsf(gz_raw) < 15.0f) gz_raw = 0;
                    g_imu_processed.yaw += (gz_raw / gyro_scale) * dt;
                    if (g_imu_processed.yaw >= 360.0f) g_imu_processed.yaw -= 360.0f;
                    if (g_imu_processed.yaw < 0.0f) g_imu_processed.yaw += 360.0f;
                    
                    // Tilt detection: If gravity on Z is weak, robot is tilted
                    if (az_g < 0.5f) {
                        g_imu_processed.is_tilted = true;
                    } else {
                        g_imu_processed.is_tilted = false;
                    }
                    xSemaphoreGive(sensorsMutex);
                }
                
                // // Print IMU data for testing
                // static int print_divider = 0;
                // if (++print_divider >= 10) { // Print every ~10 interrupts to avoid flooding
                //     printf("IMU: YAW:%.1f ACC:%d,%d,%d IMPACT:%.1f %s\n", 
                //         g_imu_processed.yaw, 
                //         g_imu_processed.acc_x, g_imu_processed.acc_y, g_imu_processed.acc_z,
                //         g_imu_processed.impact_magnitude,
                //         g_imu_processed.impact_detected ? "!!IMPACT!!" : "");
                //     print_divider = 0;
                // }
            }
        }
    }
}

// --========================= MODE SELECTION =========================--

static void mode_select_count() {
    const int COUNT_TIMEOUT_MS = 1500;
    int press_count = 0;
    int64_t last_press_time = 0;
    ESP_LOGI(TAG, "Entering count mode selection. Press button N times to select mode N.");
    gpio_set_level(ONBOARD_LED, 0);
    while (press_count == 0) {
        if (gpio_get_level(MODE_BUTTON) == 0) {
            vTaskDelay(50 / portTICK_PERIOD_MS);
            while (gpio_get_level(MODE_BUTTON) == 0) vTaskDelay(10 / portTICK_PERIOD_MS);
            press_count = 1;
            last_press_time = esp_timer_get_time();
            break;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    while (1) {
        if (gpio_get_level(MODE_BUTTON) == 0) {
            vTaskDelay(50 / portTICK_PERIOD_MS);
            while (gpio_get_level(MODE_BUTTON) == 0) vTaskDelay(10 / portTICK_PERIOD_MS);
            press_count++;
            last_press_time = esp_timer_get_time();
        } else {
            int64_t elapsed = (esp_timer_get_time() - last_press_time) / 1000;
            if (elapsed >= COUNT_TIMEOUT_MS) break;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    if (press_count < 1) press_count = 1;
    if (press_count > 10) press_count = 10;
    mode = press_count;
    blink_led(mode, 150);
    gpio_set_level(ONBOARD_LED, 1);
}

static void wing_select_count() {
    const int COUNT_TIMEOUT_MS = 1500;
    int press_count = 0;
    int64_t last_press_time = 0;
    ESP_LOGI(TAG, "Entering count wing selection.");
    while (press_count == 0) {
        if (gpio_get_level(MODE_BUTTON) == 0) {
            vTaskDelay(50 / portTICK_PERIOD_MS);
            while (gpio_get_level(MODE_BUTTON) == 0) vTaskDelay(10 / portTICK_PERIOD_MS);
            press_count = 1;
            last_press_time = esp_timer_get_time();
            break;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    while (1) {
        if (gpio_get_level(MODE_BUTTON) == 0) {
            vTaskDelay(50 / portTICK_PERIOD_MS);
            while (gpio_get_level(MODE_BUTTON) == 0) vTaskDelay(10 / portTICK_PERIOD_MS);
            press_count++;
            last_press_time = esp_timer_get_time();
        } else {
            int64_t elapsed = (esp_timer_get_time() - last_press_time) / 1000;
            if (elapsed >= COUNT_TIMEOUT_MS) break;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    if (press_count < 1) press_count = 1;
    if (press_count > 14) press_count = 14;
    wing_mode = press_count;
    blink_led(wing_mode, 150);
}

// --========================= MAIN APPLICATION =========================--

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_softap();
    xTaskCreate(telemetry_task, "telemetry_task", 4096, NULL, 5, NULL);

    setup();
    servo_init();
    servo_set_angle(90);
    srand((unsigned) (esp_timer_get_time() & 0xFFFFFFFF));
    
    // Turn off LED and wait for button press to start IMU init/calibration
    gpio_set_level(ONBOARD_LED, 0);
    ESP_LOGI(TAG, "Press MODE button once to start IMU calibration...");
    
    while (gpio_get_level(MODE_BUTTON) != 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    // Debounce
    vTaskDelay(pdMS_TO_TICKS(200));
    while (gpio_get_level(MODE_BUTTON) == 0) vTaskDelay(pdMS_TO_TICKS(10));
    
    vTaskDelay(pdMS_TO_TICKS(1000)); // 1 second before starting init
    mpu9250_init(); // This now handles LED and calibration details
    
    imu_interrupt_setup();
    gpio_set_level(ONBOARD_LED, 1);

    mode_select_count();
    wing_select_count();

    #ifdef USE_START_STOP_MODULE
    while (!gpio_get_level(START_STOP_MODULE)) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    is_running = true;
    #else
    is_running = true;
    #endif

    if(wing_mode == 1) servo_set_angle(1);
    else if (wing_mode == 2) servo_set_angle(170);
    else servo_set_angle(85);

    switch (mode) {
        case 1: current_strategy = &strategy_hunter; initial_move_duration_ms = HUNTER_INITIAL_MOVE_DURATION_MS; break;
        case 2: current_strategy = &strategy_bulldozer; initial_move_duration_ms = BULLDOZER_INITIAL_MOVE_DURATION_MS; break;
        case 3: current_strategy = &strategy_matador_right; initial_move_duration_ms = MATADOR_INITIAL_MOVE_DURATION_MS; break;
        case 4: current_strategy = &strategy_matador_left; initial_move_duration_ms = MATADOR_INITIAL_MOVE_DURATION_MS; break;
        case 5: current_strategy = &strategy_staggered_burst; initial_move_duration_ms = 0; break;
        case 6: current_strategy = &strategy_calibration_turn; initial_move_duration_ms = 0; break;
        case 7: current_strategy = &strategy_dash_left; initial_move_duration_ms = 0; break;
        case 8: current_strategy = &strategy_dash_right; initial_move_duration_ms = 0; break;
        case 9: current_strategy = &strategy_viper; initial_move_duration_ms = 0; break;
        case 10: current_strategy = &strategy_test_90deg_turns; initial_move_duration_ms = 0; break;
        default: current_strategy = &strategy_bulldozer; initial_move_duration_ms = BULLDOZER_INITIAL_MOVE_DURATION_MS; break;
    }

    sensorsMutex = xSemaphoreCreateMutex();
    pidMutex = xSemaphoreCreateMutex();

    xTaskCreate(run_strategy_task, "run_strategy_task", 4096, NULL, 5, NULL);
    xTaskCreate(imu_task, "imu_task", 4096, NULL, 10, NULL);

    esp_timer_create_args_t motor_timer_args = {
        .callback = &motor_controller_callback,
        .name = "motor_control_timer"
    };
    esp_timer_create(&motor_timer_args, &motorControlTimer);
    esp_timer_start_periodic(motorControlTimer, MOTOR_CONTROL_TIMER_PERIOD * 1000);
}
