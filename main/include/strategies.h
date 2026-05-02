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

// --- Global Variables (declared in main.c, referenced here) ---
extern bdc_motor_handle_t motor1, motor2;
extern SemaphoreHandle_t sensorsMutex;
extern SemaphoreHandle_t pidMutex;

extern state_t state;
extern int output;
extern state_t state_PID;
extern int output_PID;

extern uint8_t dist_center, dist_r90, dist_r, dist_l90, dist_l;
#ifdef ROBOT_VECHI
extern uint8_t dist_rf, dist_lf;
#endif
extern uint8_t line_right, line_left;

extern int enemy_seen_counter;
extern bool initial_move_done;
extern int initial_move_duration_ms;
extern int64_t last_enemy_seen_time;
extern volatile imu_data_t g_imu_processed;

typedef void (*strategy_func_t)(void);
extern strategy_func_t current_strategy;

// --- Forward Declarations of Strategies ---
void strategy_bulldozer(void);
void strategy_hunter(void);
void strategy_matador_left(void);
void strategy_matador_right(void);
void strategy_calibration_turn(void);
void strategy_dash_left(void);
void strategy_dash_right(void);
void strategy_spiral_out(void);
void strategy_bounce_feint(void);
void strategy_random_micro(void);
void strategy_prescan_aim(void);
void strategy_staggered_burst(void);
void strategy_viper(void);

// Precision Movement
void turn_degrees(float target_relative_degrees, int speed);

// Testing Strategies
void strategy_test_90deg_turns(void);

// Utils (defined in main.c or here?)
void read_sensors(void);
