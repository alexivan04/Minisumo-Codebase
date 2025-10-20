/*******************************************************************************
 * @file    main.c
 * @author  Gemini AI (Refactored from user code)
 * @brief   Main application for an ESP32-S3 minisumo robot.
 *
 * @description
 * This firmware implements a flexible, multi-mode strategy framework for a
 * competitive minisumo robot. It uses a function pointer-based state machine
 * to allow for easy selection of different behaviors (e.g., aggressive,
 * defensive, calibration) before a match begins.
 *
 * Core Features:
 * - Multi-Mode Strategies via Function Pointers.
 * - Conditional, sensor-based retreat logic (no fixed timers).
 * - Proportional control for precise aiming in calibration mode.
 * - Clean separation of logic (strategy task) and execution (motor callback).
 *
 ******************************************************************************/

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "bdc_motor.h"

// --=========================== PIN DEFINITIONS ===========================--
#define ONBOARD_LED         38
#define MODE_BUTTON         2
#define START_STOP_MODULE   1

// Digital Distance Sensors
#define DIST_CENTER         16
#define DIST_R90            5
#define DIST_R              15
#define DIST_L90            6
#define DIST_L              4

// Line Sensors (Assuming white line on black surface)
#define LINE_RIGHT          35
#define LINE_LEFT           13

// --======================= CONFIGURATION & BEHAVIOR =======================--
#define USE_START_STOP_MODULE   // Comment out to disable the start/stop pin check
#define ACTIVE_DEBUG            // Enables verbose logging from key functions
#define BLACK_FIELD 1           // Set to 1 for black dohyo, 0 for white dohyo
#define DOUBLE_PRESS_TIMEOUT_MS 1000 // Time window for detecting double press

// --======================== TIMING & DELAY VALUES =========================--
#define MOTOR_CONTROL_TIMER_PERIOD  15      // ms, frequency of motor updates
#define SENSOR_READING_DELAY        10      // ms, delay in the main strategy loop
#define LONG_PRESS_DELAY            1000    // ms, for mode selection
#define DOUBLE_PRESS_DELAY          3000    // ms, for mode selection confirmation

// --==================== MOTOR & PWM CONFIGURATION =====================--
#define BDC_MCPWM_TIMER_RESOLUTION_HZ 1000000 // 1MHz, 1 tick = 1us
#define BDC_MCPWM_FREQ_HZ             25000   // 25KHz PWM
#define BDC_MCPWM_DUTY_TICK_MAX       (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ)
#define BDC_MCPWM_GPIO_1A             10      // Motor 1 (e.g., Left)
#define BDC_MCPWM_GPIO_1B             9
#define BDC_MCPWM_GPIO_2A             11      // Motor 2 (e.g., Right)
#define BDC_MCPWM_GPIO_2B             12

// --====================== STRATEGY TUNING CONSTANTS =======================--
// These are used for the Proportional Control in Calibration Mode (Mode 4)
const float Kp = 40.0;                  // Proportional Gain. Higher = faster/sharper turns.
const int MIN_TURN_SPEED = 35;          // Minimum power to overcome motor inertia.
const int MAX_TURN_SPEED = 80;          // Maximum power for controlled, non-overshooting turns.

// --========================== GLOBAL VARIABLES ============================--
// --- Hardware Handles ---
bdc_motor_handle_t motor1 = NULL, motor2 = NULL;
esp_timer_handle_t motorControlTimer = NULL;
static const char *MAIN_TAG = "main";

// --- Robot States ---
// This enum defines all possible actions the robot can take.
typedef enum {
    PATROL, // Searching for the enemy
    ATTACK, // Charging the enemy
    FOUND,  // Enemy is not centered, need to turn/aim
    IDLE,   // Stopped (either aimed, or match is over)
    RETREAT // Backing away from a line
} state_t;

// --- State Transfer Variables (Task -> Timer) ---
// These variables are updated by the strategy task and read by the motor timer.
// They are protected by the `pidMutex` for thread safety.
state_t state = PATROL;
int output = 0; // Represents motor speed (%) and turn direction (sign)
state_t state_PID = PATROL;
int output_PID = 0;

// --- Sensor Data Variables ---
// These hold the latest sensor readings and are protected by the `sensorsMutex`.
uint8_t dist_center, dist_r90, dist_r, dist_l90, dist_l;
uint8_t line_right, line_left;

// --- System & Mode Flags ---
int mode = 0;
bool is_running = false; // Controlled by the START_STOP_MODULE
bool initial_move_done = false; // Flag to ensure opening move runs only once

// --- Strategy Framework Variables ---
// A function pointer type that defines what a "strategy" function looks like.
typedef void (*strategy_func_t)(void);
// A global variable to hold the function pointer of the currently selected strategy.
strategy_func_t current_strategy = NULL;

// --- RTOS Synchronization ---
SemaphoreHandle_t sensorsMutex; // Protects access to global sensor data
SemaphoreHandle_t pidMutex;     // Protects access to `state` and `output` variables

// --===================== FORWARD DECLARATIONS =======================--
void strategy_bulldozer(void);
void strategy_hunter(void);
void strategy_matador(void);
void strategy_calibration_turn(void);
void run_strategy_task(void *arg);
static void mode_select();

// --==================== UTILITY & SETUP FUNCTIONS =====================--

/**
 * @brief Initializes all hardware peripherals (GPIO, Motors).
 */
static int setup() {
    // Onboard LED for status indication
    gpio_reset_pin(ONBOARD_LED);
    gpio_set_direction(ONBOARD_LED, GPIO_MODE_OUTPUT);

    // Input pins for sensors and buttons
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

    // --- Motor 1 (e.g., Left Motor) Initialization ---
    ESP_LOGI(MAIN_TAG, "Create DC motors");
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

    // --- Motor 2 (e.g., Right Motor) Initialization ---
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

/**
 * @brief Converts a percentage value (0-100) to a motor duty cycle value.
 */
int percent_to_duty_cycle(int percent) {
    return (percent * BDC_MCPWM_DUTY_TICK_MAX) / 100;
}

/**
 * @brief Blinks the onboard LED a specified number of times.
 */
void blink_led(int cnt, int delay) {
    for (int i = 0; i < cnt; i++) {
        gpio_set_level(ONBOARD_LED, 1);
        vTaskDelay(delay / portTICK_PERIOD_MS);
        gpio_set_level(ONBOARD_LED, 0);
        vTaskDelay(delay / portTICK_PERIOD_MS);
    }
}

// --==================== SENSOR & STATE LOGIC ======================--

/**
 * @brief Reads all digital sensors and updates global variables safely.
 */
void read_sensors() {
    // Read all sensor values into local variables first
    uint8_t dist_center_local = gpio_get_level(DIST_CENTER);
    uint8_t dist_r90_local = gpio_get_level(DIST_R90);
    uint8_t dist_r_local = gpio_get_level(DIST_R);
    uint8_t dist_l90_local = gpio_get_level(DIST_L90);
    uint8_t dist_l_local = gpio_get_level(DIST_L);

    // IMPORTANT: This logic assumes your line sensor outputs HIGH (1) on a white line.
    // If your sensor is inverted (outputs LOW on white), change this to !gpio_get_level().
#ifdef BLACK_FIELD
    uint8_t line_right_local = !gpio_get_level(LINE_RIGHT);
    uint8_t line_left_local = !gpio_get_level(LINE_LEFT);
#else
    uint8_t line_right_local = gpio_get_level(LINE_RIGHT);
    uint8_t line_left_local = gpio_get_level(LINE_LEFT);
#endif

    // Use a mutex to safely update the global variables
    if (xSemaphoreTake(sensorsMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        dist_center = dist_center_local;
        dist_r90 = dist_r90_local;
        dist_r = dist_r_local;
        dist_l90 = dist_l90_local;
        dist_l = dist_l_local;
        line_right = line_right_local;
        line_left = line_left_local;
        xSemaphoreGive(sensorsMutex);
    }
}

// --========================= STRATEGY MODES =========================--

/**
 * @brief STRATEGY 1: Pure aggression. Charges forward at the start and attacks
 *        any target it sees immediately with full power.
 */
void strategy_bulldozer(void) {
    // --- Initial Move: A timed, full-speed blitz forward ---
    if (!initial_move_done) {
        ESP_LOGI("Bulldozer", "Executing initial blitz!");
        state_PID = ATTACK;
        output_PID = 100;
        // This is a blocking delay, only acceptable for an initial move.
        vTaskDelay(pdMS_TO_TICKS(1500));
        initial_move_done = true;
        ESP_LOGI("Bulldozer", "Initial blitz complete.");
        return;
    }

    // --- Main Logic ---
    read_sensors();
    if (line_right || line_left) {
        state_PID = RETREAT;
    } else if (dist_center || dist_l || dist_r || dist_l90 || dist_r90) {
        state_PID = ATTACK;
        output_PID = 100; // Attack with full power
    } else {
        state_PID = PATROL;
        output_PID = 70; // Patrol aggressively
    }
}

/**
 * @brief STRATEGY 2: Search and Destroy. Actively seeks the opponent and
 *        attempts to center on it before launching a full-power attack.
 */
void strategy_hunter(void) {
    // --- Initial Move: A quick scan to find opponents not in the center ---
    if (!initial_move_done) {
        ESP_LOGI("Hunter", "Executing initial scan...");
        // Turn left briefly to scan
        state_PID = FOUND;
        output_PID = -70; // Turn left
        vTaskDelay(pdMS_TO_TICKS(300));
        initial_move_done = true;
        ESP_LOGI("Hunter", "Initial scan complete.");
        return;
    }
    
    // --- Main Logic ---
    read_sensors();
    if (line_right || line_left) {
        state_PID = RETREAT;
    } else if (dist_center) {
        state_PID = ATTACK; // Centered: attack!
        output_PID = 100;
    } else if (dist_l90 || dist_l) {
        state_PID = FOUND; // Target on left: turn left
        output_PID = -70;
    } else if (dist_r90 || dist_r) {
        state_PID = FOUND; // Target on right: turn right
        output_PID = 70;
    } else {
        state_PID = PATROL; // No target: search
        output_PID = 60;
    }
}

/**
 * @brief STRATEGY 3: Dodge and Counter. A sample strategy that tries to
 *        evade an initial rush and attack from the side.
 */
void strategy_matador(void) {
    if (!initial_move_done) {
        ESP_LOGI("Matador", "Executing side-step maneuver...");
        // Sharp turn to the side
        state_PID = FOUND;
        output_PID = 90; // Hard right turn
        vTaskDelay(pdMS_TO_TICKS(500));
        initial_move_done = true;
        ESP_LOGI("Matador", "Side-step complete.");
        return;
    }
    // Main logic would be similar to Hunter, but patrol in circles.
    strategy_hunter(); // For now, just defaults to Hunter logic after opening.
}


/**
 * @brief STRATEGY 4: Calibration Mode. Rotates in place to perfectly center
 *        on a target using proportional control, then stops.
 */
void strategy_calibration_turn(void) {
    read_sensors();

    if (line_right || line_left) {
        state_PID = RETREAT;
        return;
    }

    // Error: Negative = Left, Positive = Right, 0 = Centered or Not Seen
    float error = 0.0;

    // 1. Calculate the error based on which sensor is active
    if (dist_center)       { error = 0.0; } // Perfect!
    else if (dist_l90)     { error = -2.0;} // Far left
    else if (dist_l)       { error = -1.0;} // Near left
    else if (dist_r90)     { error = 2.0; } // Far right
    else if (dist_r)       { error = 1.0; } // Near right
    else { // No enemy detected
        state_PID = IDLE;
        output_PID = 0;
        return;
    }

    // 2. Decide the action based on the error
    if (error == 0.0) {
        state_PID = IDLE; // We are aimed, so stop.
        output_PID = 0;
    } else {
        state_PID = FOUND; // We need to turn.

        // 3. Calculate turn speed using Proportional Control
        int turn_speed = (int)(Kp * fabs(error));

        // 4. Clamp the speed to our defined min/max values
        if (turn_speed < MIN_TURN_SPEED) turn_speed = MIN_TURN_SPEED;
        if (turn_speed > MAX_TURN_SPEED) turn_speed = MAX_TURN_SPEED;

        // 5. Set final output, using the sign of the error for direction
        output_PID = (error < 0) ? -turn_speed : turn_speed;
    }
}

// --====================== CORE TASK & TIMER =======================--

/**
 * @brief The "Brain" of the robot. This task runs in a loop, executing the
 *        currently selected strategy function and safely updating the global
 *        state variables for the motor controller to use.
 */
void run_strategy_task(void *arg) {
    while (1) {
        if (is_running && current_strategy != NULL) {
            // Call the function pointer, executing the active strategy's logic
            current_strategy();
        } else {
            // If not running, ensure the state is idle.
            state_PID = IDLE;
            output_PID = 0;
        }

        // Safely transfer the calculated state/output to the global variables
        // that the motor controller will read.
        if (xSemaphoreTake(pidMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            state = state_PID;
            output = output_PID;
            xSemaphoreGive(pidMutex);
        }
        
        vTaskDelay(pdMS_TO_TICKS(SENSOR_READING_DELAY));
    }
}


/**
 * @brief The "Muscle" of the robot. This high-frequency timer callback reads
 *        the global state and output, and translates them into physical
 *        motor actions. It does not contain complex logic, it just executes.
 */
void motor_controller_callback(void *arg) {
    #ifdef USE_START_STOP_MODULE
    if (!gpio_get_level(START_STOP_MODULE)) {
        is_running = false; // The main task will see this and go idle
    }
    #endif

    // If the robot is not running, brake the motors and do nothing else.
    if (!is_running) {
        bdc_motor_brake(motor1);
        bdc_motor_brake(motor2);
        return;
    }
    
    // Safely copy the global state and output into local variables
    state_t local_state;
    int local_output;
    if (xSemaphoreTake(pidMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        local_state = state;
        local_output = output;
        xSemaphoreGive(pidMutex);
    } else {
        // If we fail to get the mutex, brake for safety and exit.
        bdc_motor_brake(motor1);
        bdc_motor_brake(motor2);
        return;
    }

    // --- Execute Actions Based on State ---
    switch (local_state) {
        case PATROL:
        case ATTACK:
            bdc_motor_forward(motor1);
            bdc_motor_forward(motor2);
            bdc_motor_set_speed(motor1, percent_to_duty_cycle(local_output));
            bdc_motor_set_speed(motor2, percent_to_duty_cycle(local_output));
            break;

        case FOUND:
            local_output = abs(local_output); // Use absolute value for speed
            if (output < 0) { // Negative output means turn LEFT
                bdc_motor_reverse(motor1);
                bdc_motor_forward(motor2);
            } else { // Positive output means turn RIGHT
                bdc_motor_forward(motor1);
                bdc_motor_reverse(motor2);
            }
            bdc_motor_set_speed(motor1, percent_to_duty_cycle(local_output));
            bdc_motor_set_speed(motor2, percent_to_duty_cycle(local_output));
            break;

        case RETREAT:
            // This is our "smart retreat". It reads sensor data directly
            // to execute the most efficient retreat maneuver.
            uint8_t local_line_left = 0, local_line_right = 0;
            if (xSemaphoreTake(sensorsMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                local_line_left = line_left;
                local_line_right = line_right;
                xSemaphoreGive(sensorsMutex);
            }
            
            bdc_motor_reverse(motor1);
            bdc_motor_reverse(motor2);

            if (local_line_left && !local_line_right) { // Left sensor only: back up and turn RIGHT
                bdc_motor_set_speed(motor1, percent_to_duty_cycle(30));
                bdc_motor_set_speed(motor2, percent_to_duty_cycle(100));
            } else if (local_line_right && !local_line_left) { // Right sensor only: back up and turn LEFT
                bdc_motor_set_speed(motor1, percent_to_duty_cycle(100));
                bdc_motor_set_speed(motor2, percent_to_duty_cycle(30));
            } else { // Both sensors or error: back up straight
                bdc_motor_set_speed(motor1, percent_to_duty_cycle(100));
                bdc_motor_set_speed(motor2, percent_to_duty_cycle(100));
            }
            break;

        case IDLE:
        default:
            bdc_motor_brake(motor1);
            bdc_motor_brake(motor2);
            break;
    }

    #ifdef ACTIVE_DEBUG
    // This log can be very spammy, use with caution.
    // ESP_LOGI("motor_timer", "State: %d, Output: %d", local_state, local_output);
    #endif
}


// --========================= MAIN APPLICATION =========================--

void app_main(void) {
    // 1. Initialize Hardware
    setup();
    gpio_set_level(ONBOARD_LED, 1); // Turn on LED to indicate setup is complete

    // 2. Select Mode
    // For testing, we can hardcode the mode. Uncomment mode_select() for button control.
    mode_select();
    // mode = 1; // <--- CHANGE THIS VALUE TO TEST DIFFERENT MODES (1-4)
    blink_led(mode, 200); // Blink to confirm which mode is selected

    // #ifdef ACTIVE_DEBUG
    // ESP_LOGI(MAIN_TAG, "Mode %d selected", mode);
    // #endif

    // 3. Wait for Start Signal (if enabled)
    #ifdef USE_START_STOP_MODULE
    ESP_LOGI(MAIN_TAG, "Waiting for start signal on GPIO %d...", START_STOP_MODULE);
    while (!gpio_get_level(START_STOP_MODULE)) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    is_running = true;
    ESP_LOGI(MAIN_TAG, "Start signal received! Go!");
    #else
    is_running = true; // If module is disabled, start immediately
    #endif

    // 4. Assign Strategy based on Mode
    switch (mode) {
        case 1:
            ESP_LOGI(MAIN_TAG, "Strategy selected: Bulldozer");
            current_strategy = &strategy_bulldozer;
            break;
        case 2:
            ESP_LOGI(MAIN_TAG, "Strategy selected: Hunter");
            current_strategy = &strategy_hunter;
            break;
        case 3:
            ESP_LOGI(MAIN_TAG, "Strategy selected: Matador");
            current_strategy = &strategy_matador;
            break;
        case 4:
            ESP_LOGI(MAIN_TAG, "Strategy selected: Calibration Aiming");
            current_strategy = &strategy_calibration_turn;
            break;
        default:
            ESP_LOGW(MAIN_TAG, "Invalid mode selected. Defaulting to Bulldozer.");
            current_strategy = &strategy_bulldozer;
            break;
    }

    // 5. Initialize RTOS Components
    sensorsMutex = xSemaphoreCreateMutex();
    pidMutex = xSemaphoreCreateMutex();

    // 6. Start the Brain and Muscle
    xTaskCreate(run_strategy_task, "run_strategy_task", 4096, NULL, 5, NULL);

    esp_timer_create_args_t motor_timer_args = {
        .callback = &motor_controller_callback,
        .name = "motor_control_timer"
    };
    esp_timer_create(&motor_timer_args, &motorControlTimer);
    esp_timer_start_periodic(motorControlTimer, MOTOR_CONTROL_TIMER_PERIOD * 1000);

    ESP_LOGI(MAIN_TAG, "Application startup complete.");
}


/**
 * @brief Handles mode selection via button presses.
 *
 * A short press increments the mode number. The function exits on a double press,
 * confirming the selection. The number of blinks indicates the current mode.
 */
static void mode_select() {
    int press_time = 0;
    int double_press = 0;
    mode = 0;

    ESP_LOGI(MAIN_TAG, "Entering mode selection. Short press to cycle, double press to confirm.");
    gpio_set_level(ONBOARD_LED, 0);

    while (!double_press) {
        if (gpio_get_level(MODE_BUTTON) == 0) {
            press_time = 0;
            // Wait for button release
            while (gpio_get_level(MODE_BUTTON) == 0) {
                vTaskDelay(100 / portTICK_PERIOD_MS);
                press_time += 100;
            }

            // A long press is ignored in this simplified version. A short press cycles modes.
            mode++;
            if (mode > 4) mode = 1; // Cycle through available modes (1-4)
            ESP_LOGI(MAIN_TAG, "Mode set to %d", mode);
            blink_led(mode, 150);

            // Check for a double press to exit
            int double_press_timeout = 0;
            while(double_press_timeout < DOUBLE_PRESS_TIMEOUT_MS) {
                if (gpio_get_level(MODE_BUTTON) == 0) {
                    double_press = 1;
                    break;
                }
                vTaskDelay(10 / portTICK_PERIOD_MS);
                double_press_timeout += 10;
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    
    ESP_LOGI(MAIN_TAG, "Final mode %d selected.", mode);
    gpio_set_level(ONBOARD_LED, 1); // Turn on LED to confirm exit
}