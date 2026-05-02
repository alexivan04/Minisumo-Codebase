#ifndef MAIN_CONFIG_H
#define MAIN_CONFIG_H

#include "driver/gpio.h"
#include "driver/ledc.h"

// #define ROBOT_VECHI
#define LONG_PRESS_THRESHOLD_MS 1000

#ifndef ROBOT_VECHI
// ROBOT NOU
// --=========================== PIN DEFINITIONS ===========================--
#define ONBOARD_LED         38
#define MODE_BUTTON         2
#define START_STOP_MODULE   1
#define SERVO_GPIO             21

// Digital Distance Sensors
#define DIST_CENTER         16
#define DIST_R90            5
#define DIST_R              15
#define DIST_L90            6
#define DIST_L              4

// Line Sensors (Assuming white line on black surface)
#define LINE_RIGHT          35
#define LINE_LEFT           13

#define BDC_MCPWM_GPIO_1A             9      // Motor 1 (e.g., Left)
#define BDC_MCPWM_GPIO_1B             10
#define BDC_MCPWM_GPIO_2A             11      // Motor 2 (e.g., Right)
#define BDC_MCPWM_GPIO_2B             12

#define BULLDOZER_INITIAL_MOVE_DURATION_MS      400
#define HUNTER_INITIAL_MOVE_DURATION_MS          25
#define MATADOR_INITIAL_MOVE_DURATION_MS         250
#define RETREAT_DURATION_MS                      25
#define EDGE_WALKER_INITIAL_TURN_DURATION_MS     2000 
#define VIPER_INITIAL_MOVE_DURATION_MS           300
#define ADAPTIVE_HUNTER_INITIAL_MOVE_DURATION_MS 25
#define ADAPTIVE_TIMEOUT_MS                      4000 
#define LEFT_MOTOR_MULTIPLIER 1
#define RIGHT_MOTOR_MULTIPLIER 1
#define VIPER_PRETURN_MS 750 
#define VIPER_RETURN_SPEED 100
#define VIPER_ARCH_MS 1000
#define VIPER_ARCING_SPEED 100

#define RETREAT_IGNORE_MS 125
#define PATROL_SPEED 55
#define ARCING_SPEED 100 

#define SERVO_FREQ 50
#define SERVO_TIMER LEDC_TIMER_0
#define SERVO_MODE LEDC_LOW_SPEED_MODE
#define SERVO_CHANNEL LEDC_CHANNEL_0
#define SERVO_RESOLUTION LEDC_TIMER_14_BIT

// I2C Definitions
#define I2C_MASTER_SCL_IO           37      
#define I2C_MASTER_SDA_IO           36      
#define I2C_MASTER_NUM              I2C_NUM_0 
#define I2C_MASTER_FREQ_HZ          400000  
#define MPU9250_ADDR                0x68    
#define MPU9250_INT_GPIO            7

// MPU9250 Registers
#define MPU9250_PWR_MGMT_1          0x6B
#define MPU9250_ACCEL_XOUT_H        0x3B
#define MPU9250_GYRO_XOUT_H         0x43
#define MPU9250_INT_ENABLE          0x38

#else

#define ONBOARD_LED         38
#define MODE_BUTTON         2
#define START_STOP_MODULE   1

// Robot vechi ===========================================================
#define DIST_CENTER         17
#define DIST_R90            16
#define DIST_RF             15
#define DIST_R              7
#define DIST_LF             5
#define DIST_L90            4
#define DIST_L              6

#define LINE_RIGHT          36
#define LINE_LEFT           35

#define BDC_MCPWM_GPIO_1A             10      
#define BDC_MCPWM_GPIO_1B             9
#define BDC_MCPWM_GPIO_2A             11      
#define BDC_MCPWM_GPIO_2B             12

#define BULLDOZER_INITIAL_MOVE_DURATION_MS       365
#define HUNTER_INITIAL_MOVE_DURATION_MS          25
#define MATADOR_INITIAL_MOVE_DURATION_MS         450
#define RETREAT_DURATION_MS                      175
#define EDGE_WALKER_INITIAL_TURN_DURATION_MS     2000 
#define VIPER_INITIAL_MOVE_DURATION_MS           500
#define ADAPTIVE_HUNTER_INITIAL_MOVE_DURATION_MS 25
#define ADAPTIVE_TIMEOUT_MS                      4000 
#define PATROL_SPEED 50
#define ARCING_SPEED 90
#define VIPER_PRETURN_MS 200 
#define VIPER_RETURN_SPEED 100
#define VIPER_ARCH_MS 250
#define VIPER_ARCING_SPEED 100
    
#define LEFT_MOTOR_MULTIPLIER 1.15
#define RIGHT_MOTOR_MULTIPLIER 1.0
#define RETREAT_IGNORE_MS 50                    
#endif

#define ACTIVE_DEBUG            
#define BLACK_FIELD 0           
#define DOUBLE_PRESS_TIMEOUT_MS 1000 
#define ENEMY_CONFIRMATION_THRESHOLD 7

#define MOTOR_CONTROL_TIMER_PERIOD  15      
#define SENSOR_READING_DELAY        10      
#define LONG_PRESS_DELAY            1000    
#define DOUBLE_PRESS_DELAY          3000    
#define CALIBRATION_AIM_MS         800

#define BDC_MCPWM_TIMER_RESOLUTION_HZ 1000000 
#define BDC_MCPWM_FREQ_HZ             25000   
#define BDC_MCPWM_DUTY_TICK_MAX       (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ)

static const float Kp = 40.0;                  
static const int MIN_TURN_SPEED = 35;          
static const int MAX_TURN_SPEED = 80;          

#endif // MAIN_CONFIG_H
