#pragma once

//#include <stdint.h>
#include <Arduino.h>

#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))

#define MOTOR_AC_EN_PIN 14
#define MOTOR_A_STEP_PIN 12
#define MOTOR_A_DIR_PIN 13
#define MOTOR_B_EN_PIN 15
#define MOTOR_B_STEP_PIN 2
#define MOTOR_B_DIR_PIN 0
#define MOTOR_C_STEP_PIN 4
#define MOTOR_C_DIR_PIN 25

#define v_max  100 // speed max, in cm/s
#define microsteps 4
#define steps 400
#define diameter 10 // cm
#define lever_ratio 0.09090909090909090909090909090909 //    12/132
#define lenght_btw_wheels 24 // cm

void engine_init();

void Motor_engine(void *);

void engine_update(unsigned long dt_loop);

void set_speed(float,float);

void set_angular_speed(float,float);

void set_acceleration(float,float);

void set_angular_acceleration(float,float);

float* get_speed();

float* get_dist();

void reset_dist();

void set_turn(float);

void emergency(bool);

bool stand(float,float);

void raise_dir(int);

bool is_ready();

