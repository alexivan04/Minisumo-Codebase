#include "strategies.h"

extern bool is_evading_stall;
extern int64_t evade_start_time;

void strategy_bulldozer(void) {
    static bool impact_confirmed = false;

    if (!initial_move_done) {
        ESP_LOGI("Bulldozer", "Executing initial blitz!");
        state_PID = ATTACK;
        output_PID = 100;

        // Use IMU to check if we hit something during the blitz
        if (g_imu_processed.impact_detected) {
            ESP_LOGI("Bulldozer", "Impact detected during initial move!");
            impact_confirmed = true;
        }

        if (impact_confirmed) {
            // If we hit something, check if we're still meeting resistance (pushing)
            // If acceleration is low but we are at full power, we are likely pushing or stalled
            if (abs(g_imu_processed.acc_x) < PUSHING_ACCEL_THRESHOLD) {
                ESP_LOGI("Bulldozer", "Pushing verified.");
                // Keep pushing - do nothing, let state_PID = ATTACK continue
            } else {
                ESP_LOGW("Bulldozer", "Impact but no resistance. Target might have slipped.");
                impact_confirmed = false;
                initial_move_done = true; // Switch to hunter to re-locate
            }
        }
    }
    else {
        strategy_hunter();
    }
}

void strategy_hunter(void) {
    // if (!initial_move_done) {
    //     state_PID = ATTACK;
    //     output_PID = PATROL_SPEED;
    //     return;
    // }
    // else {
    {
        // read_sensors();
        bool is_enemy_present = (dist_center || dist_l90 || dist_l || dist_r90 || dist_r
        #ifdef ROBOT_VECHI
        #ifdef USE_LATERAL_SENSORS
            || dist_rf || dist_lf
        #endif
        #endif
        );

        if (is_enemy_present) {
            if (enemy_seen_counter < ENEMY_CONFIRMATION_THRESHOLD)
                enemy_seen_counter++;
        } 
        static int missed_frames = 0;
        if (++missed_frames >= 3) {
            if (enemy_seen_counter > 0) enemy_seen_counter--;
            missed_frames = 0;
        }
        if (enemy_seen_counter >= ENEMY_CONFIRMATION_THRESHOLD) {
            if (g_imu_processed.robot_flipped) { state_PID = IDLE; return; }
            if (g_imu_processed.robot_stalled) {
                is_evading_stall = true;
                evade_start_time = esp_timer_get_time();
                state_PID = REPOSITION;
                return;
            }

            if (g_imu_processed.impact_detected) {
                if (g_imu_processed.impact_side_left) {
                    state_PID = FOUND; output_PID = -90; return;
                } else if (g_imu_processed.impact_side_right) {
                    state_PID = FOUND; output_PID = 90; return;
                }
            }

            // Priority 1: center + both sides = full commit, no turning
            if (dist_center) {
                state_PID = ATTACK;
                output_PID = 100;
                return;
            }

            // Priority 2: off-center but visible — snap turn then attack
            if (dist_l90) {
                // Far left — hard turn
                state_PID = FOUND;
                output_PID = -80;
            } else if (dist_l) {
                // Slight left — moderate turn, still moving
                state_PID = FOUND;
                output_PID = -60;
            } else if (dist_r90) {
                // Far right — hard turn
                state_PID = FOUND;
                output_PID = 80;
            } else if (dist_r) {
                // Slight right — moderate turn
                state_PID = FOUND;
                output_PID = 60;
            } else {
                // Had confirmation but lost sight — keep last direction briefly
                state_PID = ATTACK;
                output_PID = 100;
            }
        } else {
            state_PID = PATROL;
            output_PID = PATROL_SPEED;
        }
    }
}

void strategy_matador_left(void) {
    if (!initial_move_done) {
        ESP_LOGI("MatadorL", "Executing parabolic evade (LEFT)...");
        state_PID = ARCHING;
        output_PID = -ARCING_SPEED;
        return;
    }
    else strategy_hunter();
}

void strategy_matador_right(void) {
    if (!initial_move_done) {
        ESP_LOGI("MatadorR", "Executing parabolic evade (RIGHT)...");
        state_PID = ARCHING;
        output_PID = ARCING_SPEED;
        return;
    }
    else strategy_hunter();
}

void strategy_calibration_turn(void) {
    // read_sensors();
    bool is_enemy_present = (dist_center || dist_l90 || dist_l || dist_r90 || dist_r
    #ifdef ROBOT_VECHI
    #ifdef USE_LATERAL_SENSORS
        || dist_rf || dist_lf
    #endif
    #endif
    );

    if (is_enemy_present) {
        if (enemy_seen_counter < ENEMY_CONFIRMATION_THRESHOLD) {
            enemy_seen_counter++;
        }
    } else {
        enemy_seen_counter = 0;
    }

    static int calibrating = 0;
    static int64_t calibrate_start = 0;
    int64_t now = esp_timer_get_time();

    if (enemy_seen_counter >= ENEMY_CONFIRMATION_THRESHOLD || calibrating) {
        if (!calibrating) {
            calibrating = 1;
            calibrate_start = now;
            ESP_LOGI("Calibration", "Starting timed calibration aiming for %d ms", CALIBRATION_AIM_MS);
        }

        float error = 0.0;
        if (dist_center)       { error = 0.0; }
        else if (dist_l90)     { error = -2.0;}
        else if (dist_l)       { error = -1.0;}
        else if (dist_r90)     { error = 2.0; }
        else if (dist_r)       { error = 1.0; }
        #ifdef ROBOT_VECHI
        #ifdef USE_LATERAL_SENSORS
        else if (dist_lf)      { error = -0.5;}
        else if (dist_rf)      { error = 0.5; }
        #endif
        #endif

        if (error == 0.0) {
            state_PID = IDLE;
            output_PID = 0;
        } else {
            state_PID = FOUND;
            int turn_speed = (int)(Kp * fabs(error));
            if (turn_speed < MIN_TURN_SPEED) turn_speed = MIN_TURN_SPEED;
            if (turn_speed > MAX_TURN_SPEED) turn_speed = MAX_TURN_SPEED;
            output_PID = (error < 0) ? -turn_speed : turn_speed;
        }

        int64_t elapsed_ms = (now - calibrate_start) / 1000;
        if (elapsed_ms >= CALIBRATION_AIM_MS) {
            calibrating = 0;
            ESP_LOGI("Calibration", "Calibration complete - switching to Hunter (attack)");
            current_strategy = &strategy_hunter;
            initial_move_done = false;
            initial_move_duration_ms = HUNTER_INITIAL_MOVE_DURATION_MS;
            return;
        }
    } else {
        calibrating = 0;
        state_PID = IDLE;
        output_PID = 0;
    }
}

void strategy_viper(void) {
    // read_sensors();
    bool is_enemy_present = (dist_center || dist_l90 || dist_l || dist_r90 || dist_r
    #ifdef ROBOT_VECHI
    #ifdef USE_LATERAL_SENSORS
        || dist_rf || dist_lf
    #endif
    #endif
    );
    if (is_enemy_present) {
        if (enemy_seen_counter < ENEMY_CONFIRMATION_THRESHOLD) enemy_seen_counter++;
    } else {
        enemy_seen_counter = 0;
    }
    int64_t now = esp_timer_get_time();

    static int viper_phase = 0; 
    static int8_t viper_arch_dir = 0; 
    static int64_t viper_phase_start = 0;
    static int64_t viper_phase_arch_start = 0;
    static int64_t viper_hold_until = 0; 
    const int VIPER_HOLD_MS = 300; 

    bool confirmed = (enemy_seen_counter >= ENEMY_CONFIRMATION_THRESHOLD) || (now < viper_hold_until);

    if (confirmed) {
        if (dist_center && dist_l && dist_r) {
            viper_phase = 0;
            state_PID = ATTACK;
            output_PID = 100;
            viper_hold_until = now + (VIPER_HOLD_MS * 1000);
            return;
        }

        if (dist_l
        #ifdef ROBOT_VECHI
        #ifdef USE_LATERAL_SENSORS
            || dist_lf
        #endif
        #endif
        ) {
            if (viper_phase == 0) {
                viper_phase = 1;
                viper_arch_dir = 1; 
                viper_phase_start = now;
                state_PID = FOUND; output_PID = -VIPER_RETURN_SPEED; 
                return;
            }
            if (viper_phase == 1) {
                int64_t elapsed = (now - viper_phase_start) / 1000;
                if (elapsed < VIPER_PRETURN_MS) {
                    state_PID = FOUND; output_PID = VIPER_RETURN_SPEED; 
                    return;
                } else {
                    viper_phase = 2; 
                    viper_phase_arch_start = now;
                    state_PID = ARCHING_VIPER; output_PID = VIPER_ARCING_SPEED;
                    viper_hold_until = now + (VIPER_HOLD_MS * 1000);
                    return;
                }
            }
            if (viper_phase == 2) {
                int64_t elapsed = (now - viper_phase_arch_start) / 1000;
                if (elapsed < VIPER_ARCH_MS) {
                    state_PID = ARCHING_VIPER; output_PID = VIPER_ARCING_SPEED; return;
                } else {
                    viper_phase = 0; 
                }
            }
        }

        if (dist_r
        #ifdef ROBOT_VECHI
        #ifdef USE_LATERAL_SENSORS
            || dist_rf
        #endif
        #endif
        ) {
            if (viper_phase == 0) {
                viper_phase = 1;
                viper_arch_dir = -1; 
                viper_phase_start = now;
                state_PID = FOUND; output_PID = VIPER_RETURN_SPEED; 
                return;
            }
            if (viper_phase == 1) {
                int64_t elapsed = (now - viper_phase_start) / 1000;
                if (elapsed < VIPER_PRETURN_MS) {
                    state_PID = FOUND; output_PID = -VIPER_RETURN_SPEED; 
                    return;
                } else {
                    viper_phase = 2;
                    viper_phase_arch_start = now;
                    state_PID = ARCHING_VIPER; output_PID = -VIPER_ARCING_SPEED;
                    viper_hold_until = now + (VIPER_HOLD_MS * 1000);
                    return;
                }
            }
            if (viper_phase == 2) {
                int64_t elapsed = (now - viper_phase_arch_start) / 1000;
                if (elapsed < VIPER_ARCH_MS) {
                    state_PID = ARCHING_VIPER; output_PID = -VIPER_ARCING_SPEED; return;
                } else {
                    viper_phase = 0;
                }
            }
        }
        if (dist_l90 && !dist_center) { viper_phase = 0; state_PID = FOUND; output_PID = -70; return; }
        if (dist_r90 && !dist_center) { viper_phase = 0; state_PID = FOUND; output_PID = 70; return; }
        viper_phase = 0;
        state_PID = PATROL;
        output_PID = 50;
    } else {
        viper_phase = 0;
        state_PID = PATROL;
        output_PID = 50;
    }
}

void strategy_dash_left(void) {
    if (initial_move_done) {
        state_PID = PATROL;
        output_PID = PATROL_SPEED;
        return;
    }
    static int stage = 0; 
    static int64_t stage_start = 0;
    static float turn_start_angle_l = 0;
    const int pre_straight_ms = 150;
    const int post_straight_ms = 175;

    if (stage == 0) {
        stage = 1;
        stage_start = esp_timer_get_time();
        ESP_LOGI("DashL", "Starting dash-left sequence");
    }

    int64_t elapsed = (esp_timer_get_time() - stage_start) / 1000;

    if (stage == 1) {
        state_PID = ATTACK;
        output_PID = 100;
        turn_start_angle_l = g_imu_processed.yaw; // Keep updating until we jump to stage 2
        if (elapsed >= pre_straight_ms) {
            stage = 2;
            stage_start = esp_timer_get_time();
        }
    } else if (stage == 2) {
        float relative_yaw = g_imu_processed.yaw - turn_start_angle_l;
        float error = 90.0f - relative_yaw; // Target is +90.0 degrees
        
        static float integral = 0;
        static float prev_error = 0;
        integral += error;
        float derivative = error - prev_error;
        prev_error = error;
        
        float TURN_KP = 2.0f;
        float TURN_KI = 0.01f;
        float TURN_KD = 0.5f;
        
        float correction = (TURN_KP * error) + (TURN_KI * integral) + (TURN_KD * derivative);
        
        // Target is +90.0 degrees.
        if ((fabs(error) < 5.0f && fabs(derivative) < 1.0f) || elapsed >= 1500) { 
            ESP_LOGI("DashL", "Turn complete at %.2f deg (rel: %.2f)", g_imu_processed.yaw, relative_yaw);
            stage = 3;
            stage_start = esp_timer_get_time();
            integral = 0;
            prev_error = 0;
        } else {
            int pwr = -correction; 
            if (pwr < -100) pwr = -100;
            if (pwr > 100) pwr = 100;
            if (pwr < 0 && pwr > -35) pwr = -35; 
            if (pwr > 0 && pwr < 35) pwr = 35;
            
            state_PID = FOUND;
            output_PID = pwr; 
        }
    } else if (stage == 3) {
        state_PID = ATTACK;
        output_PID = 100;
        if (elapsed >= post_straight_ms) {
            stage = 0; // Reset for next time
            stage_start = 0;
            turn_start_angle_l = 0;
            ESP_LOGI("DashL", "Dash-left complete. Switching to Hunter.");
            current_strategy = &strategy_hunter;
            initial_move_done = true;
        }
    }
}

void strategy_dash_right(void) {
    if (initial_move_done) {
        state_PID = PATROL;
        output_PID = PATROL_SPEED;
        return;
    }
    static int stage = 0;
    static int64_t stage_start = 0;
    static float turn_start_angle_r = 0;
    const int pre_straight_ms = 150;
    const int post_straight_ms = 175;

    if (stage == 0) {
        stage = 1;
        stage_start = esp_timer_get_time();
        ESP_LOGI("DashR", "Starting dash-right sequence");
    }

    int64_t elapsed = (esp_timer_get_time() - stage_start) / 1000;

    if (stage == 1) {
        state_PID = ATTACK;
        output_PID = 100;
        turn_start_angle_r = g_imu_processed.yaw; // Latch until jump
        if (elapsed >= pre_straight_ms) {
            stage = 2;
            stage_start = esp_timer_get_time();
        }
    } else if (stage == 2) {
        float relative_yaw = g_imu_processed.yaw - turn_start_angle_r;
        float error = -90.0f - relative_yaw; // Target is -90.0 degrees
        
        static float integral = 0;
        static float prev_error = 0;
        integral += error;
        float derivative = error - prev_error;
        prev_error = error;
        
        float TURN_KP = 2.0f;
        float TURN_KI = 0.01f;
        float TURN_KD = 0.5f;
        
        float correction = (TURN_KP * error) + (TURN_KI * integral) + (TURN_KD * derivative);
        
        // Target is -90.0 degrees.
        if ((fabs(error) < 5.0f && fabs(derivative) < 1.0f) || elapsed >= 1500) { 
            ESP_LOGI("DashR", "Turn complete at %.2f deg (rel: %.2f)", g_imu_processed.yaw, relative_yaw);
            stage = 3;
            stage_start = esp_timer_get_time();
            integral = 0;
            prev_error = 0;
        } else {
            int pwr = -correction; 
            if (pwr < -100) pwr = -100;
            if (pwr > 100) pwr = 100;
            if (pwr < 0 && pwr > -35) pwr = -35; 
            if (pwr > 0 && pwr < 35) pwr = 35;
            
            state_PID = FOUND;
            output_PID = pwr; 
        }
    } else if (stage == 3) {
        state_PID = ATTACK;
        output_PID = 100;
        if (elapsed >= post_straight_ms) {
            stage = 0; // Reset
            stage_start = 0;
            turn_start_angle_r = 0;
            ESP_LOGI("DashR", "Dash-right complete. Switching to Hunter.");
            current_strategy = &strategy_hunter;
            initial_move_done = true;
        }
    }
}

void strategy_spiral_out(void) {
    if (initial_move_done) {
        state_PID = PATROL;
        output_PID = PATROL_SPEED;
        return;
    }
    static int stage = 0;
    static int64_t start = 0;
    const int duration_ms = 1200;
    if (stage == 0) {
        stage = 1;
        start = esp_timer_get_time();
        ESP_LOGI("Spiral", "Starting spiral-out scan");
    }
    int64_t elapsed = (esp_timer_get_time() - start) / 1000;
    if (elapsed < duration_ms) {
        int mag = 30 + (int)((float)elapsed * (70.0f / duration_ms));
        if (mag > 100) mag = 100;
        state_PID = ARCHING;
        if ((rand() & 1) == 0) output_PID = mag;
        else output_PID = -mag;
    } else {
        stage = 2;
        ESP_LOGI("Spiral", "Spiral complete. Switching to Hunter.");
        current_strategy = &strategy_hunter;
        initial_move_done = false;
        initial_move_duration_ms = HUNTER_INITIAL_MOVE_DURATION_MS;
    }
}

void strategy_bounce_feint(void) {
    if (initial_move_done) {
        state_PID = PATROL;
        output_PID = PATROL_SPEED;
        return;
    }
    static int stage = 0;
    static int64_t stage_start = 0;
    static float turn_start_angle = 0;
    const int forward_ms = 150;
    const int turn_ms = 300;
    const int forward2_ms = 200;
    if (stage == 0) { stage = 1; stage_start = esp_timer_get_time(); ESP_LOGI("Bounce", "Starting bounce-feint"); }
    int64_t elapsed = (esp_timer_get_time() - stage_start) / 1000;
    if (stage == 1) {
        state_PID = ATTACK; output_PID = 100; 
        turn_start_angle = g_imu_processed.yaw;
        if (elapsed >= forward_ms) { 
            stage = 2; 
            stage_start = esp_timer_get_time(); 
        }
    } else if (stage == 2) {
        state_PID = FOUND; output_PID = (rand() & 1) ? 100 : -100; 
        float angle = fabs(g_imu_processed.yaw - turn_start_angle);
        if ((angle >= 45.0f && elapsed > 50) || elapsed >= turn_ms) { // Quick 45 deg feint
            stage = 3; 
            stage_start = esp_timer_get_time(); 
        }
    } else if (stage == 3) {
        state_PID = ATTACK; output_PID = 100; 
        if (elapsed >= forward2_ms) {
            stage = 4;
            ESP_LOGI("Bounce", "Bounce complete. Switching to Hunter.");
            current_strategy = &strategy_hunter;
            initial_move_done = false;
            initial_move_duration_ms = HUNTER_INITIAL_MOVE_DURATION_MS;
        }
    }
}

void strategy_random_micro(void) {
    if (initial_move_done) {
        state_PID = PATROL; output_PID = PATROL_SPEED; return;
    }
    static int steps = 0;
    static int idx = 0;
    static int64_t start = 0;
    const int step_ms = 200;
    if (steps == 0) {
        steps = 3 + (rand() % 4); 
        idx = 0;
        start = esp_timer_get_time();
        ESP_LOGI("RandMicro", "Starting %d micro moves", steps);
    }
    int64_t elapsed = (esp_timer_get_time() - start) / 1000;
    if (idx < steps) {
        if (elapsed >= step_ms) {
            int r = rand();
            int choice = r % 3;
            if (choice == 0) { state_PID = FOUND; output_PID = -40; }
            else if (choice == 1) { state_PID = FOUND; output_PID = 40; }
            else { state_PID = ATTACK; output_PID = 30; }
            idx++;
            start = esp_timer_get_time();
        }
    } else {
        ESP_LOGI("RandMicro", "Micro moves complete. Switching to Hunter.");
        steps = 0;
        idx = 0;
        current_strategy = &strategy_hunter;
        initial_move_done = false;
        initial_move_duration_ms = HUNTER_INITIAL_MOVE_DURATION_MS;
    }
}

void strategy_prescan_aim(void) {
    static int stage = 0;
    static int64_t start = 0;
    const int scan_ms = 1200;
    if (initial_move_done) {
        state_PID = IDLE; output_PID = 0; return;
    }
    if (stage == 0) { stage = 1; start = esp_timer_get_time(); ESP_LOGI("PreScan", "Starting prescan aim"); }
    int64_t elapsed = (esp_timer_get_time() - start) / 1000;
    state_PID = FOUND; output_PID = -30;
    read_sensors();
    if (dist_center) {
        ESP_LOGI("PreScan", "Target seen during prescan - switching to Calibration");
        current_strategy = &strategy_calibration_turn;
        initial_move_done = false;
        return;
    }
    if (elapsed >= scan_ms) {
        ESP_LOGI("PreScan", "Prescan timeout - switching to Hunter");
        current_strategy = &strategy_hunter;
        initial_move_done = false;
        stage = 0;
    }
}

void strategy_staggered_burst(void) {
    if (initial_move_done) { state_PID = PATROL; output_PID = PATROL_SPEED; return; }
    static int pulse = 0;
    static int64_t start = 0;
    const int on_ms = 80;
    const int off_ms = 60;
    const int pulses_total = 3;
    if (pulse == 0) { pulse = 1; start = esp_timer_get_time(); ESP_LOGI("Stagger", "Starting staggered burst"); }
    int64_t elapsed = (esp_timer_get_time() - start) / 1000;
    int phase = (pulse % 2 == 1) ? 1 : 0; 
    if (phase == 1) {
        state_PID = ATTACK; output_PID = 100;
        if (elapsed >= on_ms) { start = esp_timer_get_time(); pulse++; }
    } else {
        state_PID = IDLE; output_PID = 0;
        if (elapsed >= off_ms) { start = esp_timer_get_time(); pulse++; }
    }
    if (pulse > pulses_total * 2) {
        ESP_LOGI("Stagger", "Staggered burst complete. Switching to Hunter.");
        pulse = 0;
        current_strategy = &strategy_hunter;
        initial_move_done = false;
        initial_move_duration_ms = HUNTER_INITIAL_MOVE_DURATION_MS;
    }
}

void turn_degrees(float target_relative_degrees, int max_speed) {
    static float start_yaw = -1;
    static float target_yaw = -1;
    static bool is_turning = false;

    if (!is_turning) {
        start_yaw = g_imu_processed.yaw;
        target_yaw = start_yaw + target_relative_degrees;
        is_turning = true;
    }

    float error = target_yaw - g_imu_processed.yaw; 

    // Smooth approach tolerance
    if (fabs(error) > 2.0f) { 
        state_PID = FOUND;
        
        // Softer positional gain to prevent aggressive stutter
        float p_gain = 1.0f; 
        float requested_speed = fabs(error) * p_gain;
        
        if (requested_speed > max_speed) requested_speed = max_speed;
        
        // Increased minimum speed to prevent stalling out at the very end of the turn
        if (requested_speed < 25) requested_speed = 25; 

        output_PID = (error > 0) ? -requested_speed : requested_speed;
    } else {
        state_PID = IDLE;
        output_PID = 0;
        is_turning = false;
    }
}

void strategy_test_forward_180(void) {
    if (initial_move_done) {
        state_PID = IDLE; output_PID = 0; return;
    }

    static int phase = 0;
    static int64_t phase_start = 0;
    static float ideal_target_yaw = 0.0f; // Tracks exact 0 or 180 mathematically
    
    int64_t now = esp_timer_get_time();

    if (phase == 0) {
        if (phase_start == 0) phase_start = now;
        state_PID = IDLE; output_PID = 0;
        
        if ((now - phase_start) / 1000 > 2000) {
            ideal_target_yaw = g_imu_processed.yaw; // Lock initial 0 degree baseline
            phase = 1;
            phase_start = now;
        }
    }
    else if (phase == 1) { // Drive Forward
        state_PID = ATTACK;
        output_PID = 60; 
        if ((now - phase_start) / 1000 > 1500) { 
            phase = 2;
            phase_start = now;
        }
    }
    else if (phase == 2) { // Pause
        state_PID = IDLE; output_PID = 0;
        if ((now - phase_start) / 1000 > 500) { 
            phase = 3;
            ideal_target_yaw += 180.0f; // Set mathematical 180 target
        }
    }
    else if (phase == 3) { // Turn +180 Absolute
        float error = ideal_target_yaw - g_imu_processed.yaw;
        if (fabs(error) > 2.0f) {
            state_PID = FOUND;
            float req_speed = fabs(error) * 1.0f;
            if (req_speed > 60) req_speed = 60;
            if (req_speed < 25) req_speed = 25;
            output_PID = (error > 0) ? -req_speed : req_speed;
        } else {
            state_PID = IDLE; output_PID = 0;
            phase = 4;
            phase_start = now;
        }
    }
    else if (phase == 4) { // Pause
        state_PID = IDLE; output_PID = 0;
        if ((now - phase_start) / 1000 > 500) { 
            phase = 5;
            phase_start = now;
        }
    }
    else if (phase == 5) { // Drive Forward Backwards
        state_PID = ATTACK;
        output_PID = 60; 
        if ((now - phase_start) / 1000 > 1500) { 
            phase = 6;
            phase_start = now;
        }
    }
    else if (phase == 6) { // Pause
        state_PID = IDLE; output_PID = 0;
        if ((now - phase_start) / 1000 > 500) { 
            phase = 7;
            ideal_target_yaw -= 180.0f; // Set mathematical 0 target
        }
    }
    else if (phase == 7) { // Turn -180 Absolute
        float error = ideal_target_yaw - g_imu_processed.yaw;
        if (fabs(error) > 2.0f) {
            state_PID = FOUND;
            float req_speed = fabs(error) * 1.0f;
            if (req_speed > 60) req_speed = 60;
            if (req_speed < 25) req_speed = 25;
            output_PID = (error > 0) ? -req_speed : req_speed;
        } else {
            state_PID = IDLE; output_PID = 0;
            phase = 1; // Loop back to start
            phase_start = now;
        }
    }
}

void strategy_test_straight(void) {
    if (initial_move_done) {
        state_PID = IDLE; output_PID = 0; return;
    }
    // Command continuous straight attack
    state_PID = ATTACK;
    output_PID = 60; // 60% power straight
}

void strategy_test_turn(void) {
    if (initial_move_done) {
        state_PID = IDLE; output_PID = 0; return;
    }
    
    static int phase = 0;
    static int64_t phase_start = 0;
    int64_t now = esp_timer_get_time();

    if (phase == 0) {
        // State 0: Paused at starting angle (0)
        if (phase_start == 0) phase_start = now;
        state_PID = IDLE; 
        output_PID = 0;
        
        if ((now - phase_start) / 1000 > 2000) { // 2 seconds pause
            phase = 1;
            phase_start = 0;
            ESP_LOGI("Test", "Phase 1: Turning +90 degrees");
        }
    } 
    else if (phase == 1) {
        // State 1: Execute +90 turn at 50% target turn rate
        // turn_degrees sets state to FOUND, passing through your Turn Rate PID
        turn_degrees(90.0f, 50); 
        
        if (state_PID == IDLE) { // turn_degrees sets state to IDLE when target is reached
            phase = 2;
            phase_start = now;
            ESP_LOGI("Test", "Phase 2: Reached +90. Pausing for 2s.");
        }
    } 
    else if (phase == 2) {
        // State 2: Paused at 90
        state_PID = IDLE; 
        output_PID = 0;
        
        if ((now - phase_start) / 1000 > 2000) { // 2 seconds pause
            phase = 3;
            phase_start = 0;
            ESP_LOGI("Test", "Phase 3: Turning back to 0 (-90 degrees)");
        }
    } 
    else if (phase == 3) {
        // State 3: Execute -90 turn back to original position
        turn_degrees(-90.0f, 50); 
        
        if (state_PID == IDLE) {
            phase = 0;
            phase_start = 0;
            ESP_LOGI("Test", "Phase 0: Reached 0. Looping.");
        }
    }
}


void strategy_test_90deg_turns(void) {
    static int phase = 0;
    static int64_t phase_start = 0;
    int64_t now = esp_timer_get_time();

    if (phase == 0) {
        // Wait 2 seconds before starting
        if (phase_start == 0) phase_start = now;
        if ((now - phase_start) / 1000 > 2000) {
            phase = 1;
            phase_start = 0;
            ESP_LOGI("Test", "Phase 1: Turning +90 degrees");
        }
    } else if (phase == 1) {
        // Turn +90
        turn_degrees(90, 40);
        if (state_PID == IDLE) { // turn_degrees sets IDLE when done
            phase = 2;
            phase_start = now;
            ESP_LOGI("Test", "Phase 1 complete. Pausing.");
        }
    } else if (phase == 2) {
        // Wait 2 seconds
        if ((now - phase_start) / 1000 > 2000) {
            phase = 3;
            phase_start = 0;
            ESP_LOGI("Test", "Phase 3: Turning -90 degrees");
        }
    } else if (phase == 3) {
        // Turn -90
        turn_degrees(-90, 40);
        if (state_PID == IDLE) {
            phase = 0; // Loop back
            phase_start = now;
            ESP_LOGI("Test", "Phase 3 complete. Looping back to Phase 0.");
        }
    }
}
