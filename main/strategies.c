#include "strategies.h"

void strategy_bulldozer(void) {
    if (!initial_move_done) {
        ESP_LOGI("Bulldozer", "Executing initial blitz!");
        state_PID = ATTACK;
        output_PID = 100;
    }
    else strategy_hunter();
}

void strategy_hunter(void) {
    if (!initial_move_done) {
        ESP_LOGI("Hunter", "Executing initial scan...");
        state_PID = FOUND;
        output_PID = -70; // Turn left
    }
    else {
        read_sensors();
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

        if (enemy_seen_counter >= ENEMY_CONFIRMATION_THRESHOLD) {
            if (dist_center && dist_l && dist_r) {
                state_PID = ATTACK;
                output_PID = 100;
            } else if (dist_l90 || dist_l
            #ifdef ROBOT_VECHI
            #ifdef USE_LATERAL_SENSORS
                || dist_lf
            #endif
            #endif
            ) {
                state_PID = FOUND;
                output_PID = -70;
            } else if (dist_r90 || dist_r
            #ifdef ROBOT_VECHI
            #ifdef USE_LATERAL_SENSORS
                || dist_rf
            #endif
            #endif
            ) {
                state_PID = FOUND;
                output_PID = 70;
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
    read_sensors();
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
    read_sensors();
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
    const int pre_straight_ms = 150;
    const int turn_ms = 300; 
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
        if (elapsed >= pre_straight_ms) {
            stage = 2;
            stage_start = esp_timer_get_time();
        }
    } else if (stage == 2) {
        state_PID = FOUND;
        output_PID = -100; 
        if (elapsed >= turn_ms) {
            stage = 3;
            stage_start = esp_timer_get_time();
        }
    } else if (stage == 3) {
        state_PID = ATTACK;
        output_PID = 100;
        if (elapsed >= post_straight_ms) {
            stage = 4;
            ESP_LOGI("DashL", "Dash-left complete. Switching to Hunter.");
            current_strategy = &strategy_hunter;
            initial_move_done = false;
            initial_move_duration_ms = HUNTER_INITIAL_MOVE_DURATION_MS;
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
    const int pre_straight_ms = 150;
    const int turn_ms = 320; 
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
        if (elapsed >= pre_straight_ms) {
            stage = 2;
            stage_start = esp_timer_get_time();
        }
    } else if (stage == 2) {
        state_PID = FOUND;
        output_PID = 100; 
        if (elapsed >= turn_ms) {
            stage = 3;
            stage_start = esp_timer_get_time();
        }
    } else if (stage == 3) {
        state_PID = ATTACK;
        output_PID = 100;
        if (elapsed >= post_straight_ms) {
            stage = 4;
            ESP_LOGI("DashR", "Dash-right complete. Switching to Hunter.");
            current_strategy = &strategy_hunter;
            initial_move_done = false;
            initial_move_duration_ms = HUNTER_INITIAL_MOVE_DURATION_MS;
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
    const int forward_ms = 150;
    const int turn_ms = 300;
    const int forward2_ms = 200;
    if (stage == 0) { stage = 1; stage_start = esp_timer_get_time(); ESP_LOGI("Bounce", "Starting bounce-feint"); }
    int64_t elapsed = (esp_timer_get_time() - stage_start) / 1000;
    if (stage == 1) {
        state_PID = ATTACK; output_PID = 100; 
        if (elapsed >= forward_ms) { stage = 2; stage_start = esp_timer_get_time(); }
    } else if (stage == 2) {
    state_PID = FOUND; output_PID = (rand() & 1) ? 100 : -100; 
        if (elapsed >= turn_ms) { stage = 3; stage_start = esp_timer_get_time(); }
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
