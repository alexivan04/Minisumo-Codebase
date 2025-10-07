#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <stdint.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <driver/gpio.h>

#include "vl53l0x_esp32.h"
#include "mma845x.h"
#include "motor_control.h"
#include "bdc_motor.h"

#include <esp_wifi.h>
#include <nvs_flash.h>
#include <lwip/sockets.h>
#include <lwip/err.h>
#include <lwip/sys.h>
#include <esp_system.h>
#include <esp_event.h>
#include <esp_mac.h>
#include <esp_timer.h>

#include <math.h>
#include <freertos/queue.h>
#include <string.h>

// #include "soc/rtc_wdt.h"

#define ONBOARD_LED 2
#define DIST_SENSORS_CNT 7
#define USED_DIST_SENSORS_CNT 3

#define LINE_RIGHT 15 //RIGHT
#define LINE_LEFT 14 //LEFT 

#define MODE_BUTTON 35
#define START_STOP_MODULE 34
#define BLACK_FIELD 1

#define LONG_PRESS_DELAY 1000
#define DOUBLE_PRESS_DELAY 3000
// #define SINGLE_RANGING
#define CONTINUOUS_RANGING
#define MAX_RANGE 450
#define MIN_RANGE 10
#define CENTERED_TRESHOLD 20

#define MOTOR_CONTROL_TIMER_PERIOD 15
#define SENSOR_READING_DELAY 10
#define RETREAT_DELAY 100
#define WING_MOTOR_DELAY 25

#define BDC_MCPWM_TIMER_RESOLUTION_HZ 1000000 // 1MHz, 1 tick = 0.1us
#define BDC_MCPWM_FREQ_HZ             25000    // 25KHz PWM
#define BDC_MCPWM_DUTY_TICK_MAX       (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks
#define BDC_MCPWM_GPIO_1A              17
#define BDC_MCPWM_GPIO_1B              5
#define BDC_MCPWM_GPIO_2A              18
#define BDC_MCPWM_GPIO_2B              19
bdc_motor_handle_t motor1 = NULL, motor2 = NULL;

typedef enum
{
    FORWARD,
    BACKWARD
} direction_t;

typedef enum
{
    PATROL,
    ATTACK,
    FOUND,
    IDLE,
    RETREAT,
    SPIN
} state_t;

// #define ACTIVE_ACCEL
// #define ACTIVE_DEBUG
// #define SENSORS_DEBUG
// #define ACTIVE_DEBUG_DISTANCE_1
// #define ACTIVE_DEBUG_DISTANCE_2
// #define ACTIVE_DEBUG_DISTANCE_3
// #define ACTIVE_DEBUG_LINE
// #define SAFE_MODE
// #define USE_START_STOP_MODULE

esp_timer_handle_t motorControlTimer = NULL;
esp_timer_handle_t sensorReadingTimer = NULL;
esp_timer_handle_t retreatTimer = NULL;
esp_timer_handle_t wingMotorTimer = NULL;

static mma845x_sensor_t* accel;
int mode = 0;
int patrol_state = 0;
int digital_distace = 25;
TaskHandle_t sensor_task_handles[DIST_SENSORS_CNT];
static const char *MAIN_TAG = "main";

typedef struct 
{
    float move_forward_duty_cycle;
    float move_backward_duty_cycle;
    float turn_duty_cycle;
    int move_delay;
    int turn_delay;
} PatrolParams;

typedef struct
{
    int sensor_id;
} SensorTaskParams;

typedef struct
{
    double max_integral;

    double kp;
    double ki;
    double kd;
} SensorRange_PID;

enum PatrolSates
{
    PATROL1 = 1,
    PATROL2 = 2
};

enum AttackStates
{
    ATTACK1 = 1,
    ATTACK2 = 2
};

// SENSOR ORDER(ROBOT VIEW): FR F FL R90 R45 L45 L90
VL53L0X_Error status = VL53L0X_ERROR_NONE;
uint8_t dist_sensors_xshuts[DIST_SENSORS_CNT] = {13, 12, 27, 33, 32, 25, 26};
uint8_t dist_sensors_addrs[DIST_SENSORS_CNT] = {0x30, 0x32, 0x34, 0x36, 0x38, 0x40, 0x42};
uint8_t sensors_to_use[USED_DIST_SENSORS_CNT] = {1, 4, 5};
uint16_t dist_sensor_data[DIST_SENSORS_CNT];
VL53L0X_Dev_t dist_sensors[DIST_SENSORS_CNT];
SemaphoreHandle_t sensorsMutex;
SemaphoreHandle_t pidMutex;
SemaphoreHandle_t stateMutex;
int output;
int direction = 0;
state_t state = PATROL;

double max_integral = 1000000;
double kp = 0.2;
double ki = 0;
double kd = 0;
state_t state_PID = PATROL;

int output_PID = 0;
double integral_PID = 0;
double error_prev_PID = 0, error_PID;
double derivative_PID = 0;

VL53L0X_RangingMeasurementData_t measurement;
int line_right, line_left, line_back;

int output_MOTOR_CALLBACK = 0;
state_t state_MOTOR_CALLBACK = PATROL;

bool retreating = false;
bool is_running = false;


//TODO: set lower timing budget
static int setup()
{
    gpio_reset_pin(ONBOARD_LED);
    gpio_set_direction(ONBOARD_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(ONBOARD_LED, 1);
    gpio_set_level(ONBOARD_LED, 0);

    #ifdef ACTIVE_DEBUG
    ESP_LOGI(MAIN_TAG, "Create DC motors");
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
    #ifdef ACTIVE_DEBUG
    ESP_LOGI(MAIN_TAG, "Enable motor 1");
    #endif
    ESP_ERROR_CHECK(bdc_motor_enable(motor1));

    #ifdef ACTIVE_DEBUG
    ESP_LOGI(MAIN_TAG, "Forward motor 1");
    #endif
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
    #ifdef ACTIVE_DEBUG
    ESP_LOGI(MAIN_TAG, "Enable motor 2");
    #endif
    ESP_ERROR_CHECK(bdc_motor_enable(motor2));

    #ifdef ACTIVE_DEBUG
    ESP_LOGI(MAIN_TAG, "Forward motor 2");
    #endif
    ESP_ERROR_CHECK(bdc_motor_forward(motor2));

    //mode button
    gpio_reset_pin(MODE_BUTTON);
    gpio_set_direction(MODE_BUTTON, GPIO_MODE_INPUT);

    //start-stop module
    gpio_reset_pin(START_STOP_MODULE);
    gpio_set_direction(START_STOP_MODULE, GPIO_MODE_INPUT);
    gpio_set_pull_mode(START_STOP_MODULE, GPIO_PULLDOWN_ENABLE);

    gpio_reset_pin(4);
    gpio_set_direction(4, GPIO_MODE_INPUT);
    gpio_set_pull_mode(4, GPIO_PULLDOWN_ENABLE);

    //dist sensors
    #ifdef ACTIVE_DEBUG
    ESP_LOGI(MAIN_TAG, "XHSUT resetting");
    #endif

    init_xshuts(dist_sensors_xshuts, DIST_SENSORS_CNT);
    set_all_xshut_states(dist_sensors_xshuts, true, DIST_SENSORS_CNT);
    vTaskDelay(25 / portTICK_PERIOD_MS);
    set_all_xshut_states(dist_sensors_xshuts, false, DIST_SENSORS_CNT);
    vTaskDelay(25 / portTICK_PERIOD_MS);

    pidMutex = xSemaphoreCreateMutex();

    #ifdef ACTIVE_DEBUG
    ESP_LOGI(MAIN_TAG, "I2C initializing");
    #endif
    i2c_master_init();

    #ifdef ACTIVE_DEBUG
    ESP_LOGI(MAIN_TAG, "Dist sensor initializing");
    #endif
    int return_value = 1;
    for(int i = 0; i < USED_DIST_SENSORS_CNT; i++)
    {
        // Add the sensor device to the I2C bus
        status = init_dist_sensor(&dist_sensors[sensors_to_use[i]], dist_sensors_xshuts[sensors_to_use[i]], dist_sensors_addrs[sensors_to_use[i]], VL53L0X_HIGH_SPEED);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        if(status != VL53L0X_ERROR_NONE)
        {
            #ifdef ACTIVE_DEBUG
            ESP_LOGE(MAIN_TAG, "Dist sensor %d initialization failed", i);
            ESP_LOGE(MAIN_TAG, "Error: %d", status);
            #endif
            return_value = 0;
        }

        #ifdef CONTINUOUS_RANGING
        status = VL53L0X_SetDeviceMode(&dist_sensors[sensors_to_use[i]], VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
        status = VL53L0X_StartMeasurement(&dist_sensors[sensors_to_use[i]]);
        #endif

        #ifdef SINGLE_RANGING
        status = VL53L0X_SetDeviceMode(&dist_sensors[i], VL53L0X_DEVICEMODE_SINGLE_RANGING);
        #endif
    }

    //accelerometer
    #ifdef ACTIVE_ACCEL
    //ACCEL
    #ifdef ACTIVE_DEBUG
    ESP_LOGI(MAIN_TAG, "Accel initializing");
    #endif
    accel = mma845x_init_sensor(I2C_MASTER_NUM, MMA845X_I2C_ADDRESS_1);
    if (accel)
    {   
        // set polarity and type of INT signals if necessary
        mma845x_config_int_signals (accel, mma845x_high_active, mma845x_push_pull);

        // config HPF and enable it for accel output data
        mma845x_config_hpf (accel, 0, true);
        
        // set scale and mode
        mma845x_set_scale(accel, mma845x_scale_2_g);
        mma845x_set_mode (accel, mma845x_normal, mma845x_odr_50, true, false);
    }
    else
    {
        #ifdef ACTIVE_DEBUG
        ESP_LOGE(MAIN_TAG, "Could not initialize MMA845X accel. Error: %d", status);
        #endif
        return_value = 0;
    }
    #endif

    //line sensors
    #ifdef ACTIVE_DEBUG
    ESP_LOGI(MAIN_TAG, "Line sensors initializing");
    #endif
    gpio_reset_pin(LINE_RIGHT);
    gpio_set_direction(LINE_RIGHT, GPIO_MODE_INPUT);
    gpio_reset_pin(LINE_LEFT);
    gpio_set_direction(LINE_LEFT, GPIO_MODE_INPUT);
    return return_value;

}
  
static void mode_select()
{
    int press_time = 0;
    int double_press = 0;
    mode = 0;

    //detect long press
    while(!double_press)
    {
        if(gpio_get_level(MODE_BUTTON) == 0)
        {
            //detect long press
            while(gpio_get_level(MODE_BUTTON) == 0)
            {
                vTaskDelay(100 / portTICK_PERIOD_MS);
                press_time += 100;
            }

            if(press_time >= LONG_PRESS_DELAY)
            {
                mode++;
                press_time = 0;
                #ifdef ACTIVE_DEBUG
                ESP_LOGI(MAIN_TAG, "Mode %d selected", mode);
                #endif

                //blink LED for mode selection
                for(int i = 0; i < mode; i++)
                {
                    gpio_set_level(ONBOARD_LED, 1);
                    vTaskDelay(250 / portTICK_PERIOD_MS);
                    gpio_set_level(ONBOARD_LED, 0);
                    vTaskDelay(250 / portTICK_PERIOD_MS);
                }
            }

            else
            {
                //detect double press
                while(press_time < DOUBLE_PRESS_DELAY)
                {
                    if(gpio_get_level(MODE_BUTTON) == 0)
                    {
                        while(gpio_get_level(MODE_BUTTON) == 0)
                        {
                            vTaskDelay(100 / portTICK_PERIOD_MS);
                            press_time += 100;
                        }
                        double_press = 1;
                        break;
                    }
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                    press_time += 100;
                }
            }
        }
    }

    //TURN LED ON AFTER EXIT
    gpio_set_level(ONBOARD_LED, 1);

}

int percent_to_duty_cycle(int percent)
{
    return (percent * BDC_MCPWM_DUTY_TICK_MAX) / 100;
}

void read_sensors()
{
    for(int i = 0; i < USED_DIST_SENSORS_CNT; i++)
    {
        // status = VL53L0X_StartMeasurement(&dist_sensors[i]);
        status = VL53L0X_GetRangingMeasurementData(&dist_sensors[sensors_to_use[i]], &measurement);
        if(status != VL53L0X_ERROR_NONE)
        {
            dist_sensor_data[sensors_to_use[i]] = MAX_RANGE;
            continue;
        }
        // status = VL53L0X_StopMeasurement(&dist_sensors[i]);

        dist_sensor_data[i] = measurement.RangeMilliMeter;
        dist_sensor_data[i] = (dist_sensor_data[sensors_to_use[i]] > MAX_RANGE) ? MAX_RANGE : dist_sensor_data[i];
        ESP_LOGI("read sensors", "sensor %d: %d", sensors_to_use[i], dist_sensor_data[sensors_to_use[i]]);
    }

    //read line sensors
    // #ifdef BLACK_FIELD
    // line_right = !gpio_get_level(LINE_RIGHT);
    // line_left = !gpio_get_level(LINE_LEFT);
    // #endif

    #ifndef BLACK_FIELD
    local_line_right = gpio_get_level(LINE_RIGHT);
    local_line_left = gpio_get_level(LINE_LEFT);
    local_line_back = gpio_get_level(LINE_BACK);
    #endif
}

//check for flags
void mode_1_task(void *arg)
{
    while(1)
    {
        #ifdef USE_START_STOP_MODULE
        if(!is_running)
        {
            vTaskDelete(NULL);
        }
        #endif

        read_sensors();

        if(dist_sensor_data[1] < MAX_RANGE)
        {
            state_PID = ATTACK;
            output_PID = 65;
        }

        else if(dist_sensor_data[3] < MAX_RANGE)
        {
            state_PID = FOUND;
            output_PID = -65;
        }

        else if(dist_sensor_data[5] < MAX_RANGE)
        
        {
            state_PID = FOUND;
            output_PID = 40;
        }

        else
        {
            state_PID = PATROL;
            output_PID = 65;
        }

        if(xSemaphoreTake(pidMutex, pdTICKS_TO_MS(10)) == pdTRUE)
        {
            output = output_PID;
            state = state_PID;
            xSemaphoreGive(pidMutex);
        }
    }
}

void mode_2_task_woodpecker(void *arg)
{
    while(1)
    {
        #ifdef USE_START_STOP_MODULE
        if(!is_running)
        {
            vTaskDelete(NULL);
        }
        #endif

        read_sensors();

        if(line_right && line_left)
        {
            state_PID = IDLE;
            output_PID = 0;
        }

        else if(line_right || line_left)
        {
            state_PID = RETREAT;
            output_PID = 85;
        }

        else if(dist_sensor_data[1] < MAX_RANGE || dist_sensor_data[0] < MAX_RANGE || dist_sensor_data[2] < MAX_RANGE)
        {
            state_PID = ATTACK;
            if(dist_sensor_data[1] < 250)
                output_PID = 100;
            else output_PID = 75;

        }

        else if(dist_sensor_data[3] < MAX_RANGE || dist_sensor_data[4] < MAX_RANGE)
        {
            state_PID = FOUND;
            output_PID = -60;
        }

        else if(dist_sensor_data[5] < MAX_RANGE || dist_sensor_data[6] < MAX_RANGE)
        
        {
            state_PID = FOUND;
            output_PID = 60;
        }

        else
        {
            state_PID = PATROL;
            output_PID = 75;
            if(xSemaphoreTake(pidMutex, pdTICKS_TO_MS(10)) == pdTRUE)
            {
                output = output_PID;
                state = state_PID;
                xSemaphoreGive(pidMutex);
            }
            vTaskDelay(200 / portTICK_PERIOD_MS);
            state_PID = IDLE;
            if(xSemaphoreTake(pidMutex, pdTICKS_TO_MS(10)) == pdTRUE)
            {
                state = state_PID;
                xSemaphoreGive(pidMutex);
            }
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }

        if(xSemaphoreTake(pidMutex, pdTICKS_TO_MS(10)) == pdTRUE)
        {
            output = output_PID;
            state = state_PID;
            xSemaphoreGive(pidMutex);
        }
    }
}

void mode_3_task_wait_for_opponent()
{
    while(1)
    {
        #ifdef USE_START_STOP_MODULE
        if(!is_running)
        {
            vTaskDelete(NULL);
        }
        #endif

        read_sensors();

        if(line_right && line_left)
        {
            state_PID = IDLE;
            output_PID = 0;
        }

        else if(line_right || line_left)
        {
            state_PID = RETREAT;
            output_PID = 85;
        }

        if(dist_sensor_data[1] < MAX_RANGE || dist_sensor_data[0] < MAX_RANGE || dist_sensor_data[2] < MAX_RANGE)
        {
            state_PID = ATTACK;
            if(dist_sensor_data[1] < 300)
                output_PID = 100;
            else output_PID = 75;

        }

        else if(dist_sensor_data[3] < MAX_RANGE || dist_sensor_data[4] < MAX_RANGE)
        {
            state_PID = FOUND;
            output_PID = -60;
        }

        else if(dist_sensor_data[5] < MAX_RANGE || dist_sensor_data[6] < MAX_RANGE)
        
        {
            state_PID = FOUND;
            output_PID = 60;
        }

        else
        {
            state_PID = IDLE;
        }

        if(xSemaphoreTake(pidMutex, pdTICKS_TO_MS(10)) == pdTRUE)
        {
            output = output_PID;
            state = state_PID;
            xSemaphoreGive(pidMutex);
        }
    }
}

void mode_4_task_spin()
{
    while(1)
    {
        #ifdef USE_START_STOP_MODULE
        if(!is_running)
        {
            vTaskDelete(NULL);
        }
        #endif

        read_sensors();

        if(dist_sensor_data[1] < MAX_RANGE || dist_sensor_data[0] < MAX_RANGE || dist_sensor_data[2] < MAX_RANGE)
        {
            state_PID = ATTACK;
            if(dist_sensor_data[1] < 300)
                output_PID = 100;
            else output_PID = 75;

        }

        else if(dist_sensor_data[3] < MAX_RANGE || dist_sensor_data[4] < MAX_RANGE)
        {
            state_PID = FOUND;
            output_PID = -60;
        }

        else if(dist_sensor_data[5] < MAX_RANGE || dist_sensor_data[6] < MAX_RANGE)
        
        {
            state_PID = FOUND;
            output_PID = 60;
        }

        else
        {
            state_PID = SPIN;
            output_PID = 45;
        }

        if(xSemaphoreTake(pidMutex, pdTICKS_TO_MS(10)) == pdTRUE)
        {
            output = output_PID;
            state = state_PID;
            xSemaphoreGive(pidMutex);
        }
    }
}

void mode_5_task_fullforward(void *arg)
{
    state_PID = ATTACK;
    output_PID = 100;


    if(xSemaphoreTake(pidMutex, pdTICKS_TO_MS(10)) == pdTRUE)
    {
        output = output_PID;
        state = state_PID;
        xSemaphoreGive(pidMutex);
    }

}

void retreat_timer_callback(void *arg)
{
    state_MOTOR_CALLBACK = PATROL;
    retreating = false;

}

void motor_controller_callback(void *arg)
{
    #ifdef USE_START_STOP_MODULE
    if(!gpio_get_level(START_STOP_MODULE))
    {
        is_running = false;
        bdc_motor_brake(motor1);
        bdc_motor_brake(motor2);
        esp_timer_stop(motorControlTimer);
        return;
    }
    #endif

    // only run once at start of gam

    if(xSemaphoreTake(pidMutex, pdTICKS_TO_MS(10)) == pdTRUE)
    {
        output_MOTOR_CALLBACK = output;
        state_MOTOR_CALLBACK = state;
        xSemaphoreGive(pidMutex);
    }

    if(retreating)
    {
        ESP_LOGI("motor timer", "retreating");
        bdc_motor_reverse(motor1);
        bdc_motor_reverse(motor2);
        state_MOTOR_CALLBACK = RETREAT;
        output_MOTOR_CALLBACK = 100;
    }

    else
    {
        switch(state_MOTOR_CALLBACK)
        {
            case PATROL:
            case ATTACK:
                bdc_motor_forward(motor1);
                bdc_motor_forward(motor2);
                break;

            case FOUND:
                if(output_MOTOR_CALLBACK < 0)
                {
                    bdc_motor_forward(motor1);
                    bdc_motor_reverse(motor2);
                    output_MOTOR_CALLBACK = -output_MOTOR_CALLBACK;
                }

                else
                {
                    bdc_motor_reverse(motor1);
                    bdc_motor_forward(motor2);
                }
                break;

            case IDLE:
                bdc_motor_brake(motor1);
                bdc_motor_brake(motor2);
                break;

            case RETREAT:
                if (!retreating)
                {
                    retreating = true;
                    esp_timer_stop(retreatTimer);
                    esp_timer_start_once(retreatTimer, RETREAT_DELAY * 1000);
                }
                break;
            
            case SPIN:
                bdc_motor_reverse(motor1);
                bdc_motor_forward(motor2);
                break;
        }

    }
    
    bdc_motor_set_speed(motor1, percent_to_duty_cycle(output_MOTOR_CALLBACK));
    bdc_motor_set_speed(motor2, percent_to_duty_cycle(output_MOTOR_CALLBACK));

    // #ifdef ACTIVE_DEBUG
    ESP_LOGI("motor timer", "state: %d", state_MOTOR_CALLBACK);
    // #endif
}

void blink_led(int cnt, int delay)
{
    for(int i = 0; i < cnt; i++)
    {
        gpio_set_level(ONBOARD_LED, 1);
        vTaskDelay(delay / portTICK_PERIOD_MS);
        gpio_set_level(ONBOARD_LED, 0);
        vTaskDelay(delay / portTICK_PERIOD_MS);
    }
}

void app_main(void) 
{
    // int setup_status = setup();

    // //if setup ok, blink 3 times
    // if(setup_status == 1)
    //     blink_led(3, 150);
    // else
    //     gpio_set_level(ONBOARD_LED, 1);

    // mode = 1;

    // #ifdef ACTIVE_DEBUG
    // ESP_LOGI(MAIN_TAG, "Mode %d selected", mode);
    // #endif

    // #ifdef USE_START_STOP_MODULE
    // while(1)
    // {
    //     ESP_LOGI("start stop", "%d", gpio_get_level(START_STOP_MODULE));
    //     if(gpio_get_level(START_STOP_MODULE))
    //     {
    //         is_running = true;
    //         break;
    //     }
        
    // }
    // #endif

    // esp_timer_create_args_t motorControlTimerArgs = {
    //     .callback = &motor_controller_callback,
    //     .name = "motor_control_timer"
    // };
    // esp_timer_create(&motorControlTimerArgs, &motorControlTimer);
    // esp_timer_start_periodic(motorControlTimer, MOTOR_CONTROL_TIMER_PERIOD * 1000);

    // esp_timer_create_args_t retreatTimerArgs = {
    //     .callback = &retreat_timer_callback,
    //     .name = "retreat_timer"
    // };
    // esp_timer_create(&retreatTimerArgs, &retreatTimer);
    
    // //PATROL1 WITH PID
    // if(mode == 1)
    // {
    //     xTaskCreatePinnedToCore(mode_1_task, "Mode 1 Task", 8192, NULL, 6, NULL, 1);
    // }

    // //if mode not 1, reboot esp32
    // else if(mode != 1)
    // {
    //     esp_restart();
    // }
    gpio_reset_pin(ONBOARD_LED);
    gpio_set_direction(ONBOARD_LED, GPIO_MODE_OUTPUT);

    while(1)
    {
        gpio_set_level(ONBOARD_LED, 1);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        gpio_set_level(ONBOARD_LED, 0);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        ESP_LOGI("main", "loop");
    }
}