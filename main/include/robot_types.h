#ifndef ROBOT_TYPES_H
#define ROBOT_TYPES_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    PATROL, 
    ATTACK, 
    FOUND,  
    IDLE,   
    RETREAT, 
    ARCHING,  
    ARCHING_VIPER,
    FOLLOW_LINE,
    PUSH_CONFIRM,
    REPOSITION
} state_t;

typedef struct {
    int16_t acc_x, acc_y, acc_z;
    int16_t gyro_x, gyro_y, gyro_z;
    float yaw;          
    float pitch;        
    bool is_tilted;     
    float gyro_z_offset;
    float acc_x_offset;
    float acc_y_offset;
    float acc_z_nominal;
    float impact_magnitude;
    bool impact_detected;
    bool impact_side_left;
    bool impact_side_right;
    bool robot_flipped;
    bool robot_stalled;
} imu_data_t;

#endif // ROBOT_TYPES_H
