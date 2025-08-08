#include "motor_engine.h"
TaskHandle_t Motor_engine_task;

int long total_stepA = 0;
int long total_stepB = 0;
int long total_stepC = 0;

//motor dir
int dirA = 0;
int dirB = 0;
int dirC = 0;

//motor dt
int dt_MA = 0;
int dt_MB = 0;
int dt_MC = 0;

//motor avance speed, rot/s
float v_MA = 0;
float v_MB = 0;
float v_MC = 0;



// Total wheel speed
float v_tot_A = 0;
float v_tot_B = 0;
float v_tot_C = 0;

float v_diff =0.0;

//motor accelerations, only applied on motor avance speed, rot/s²
float a_MA = 0;
float a_MB = 0;
float a_MC = 0;

//motor dist
float total_dist_A = 0;
float total_dist_B = 0;
float total_dist_C = 0;

//motor freq
double f_MA = 0;
double f_MB = 0;
double f_MC = 0;

//Rise system parameters
int target_step = 0;
float Speed_stand =0;

float v_max_ang = v_max /(PI * diameter);

void engine_init() {

  pinMode(MOTOR_AC_EN_PIN, OUTPUT);
  pinMode(MOTOR_B_EN_PIN, OUTPUT);
  digitalWrite(MOTOR_AC_EN_PIN, LOW);
  digitalWrite(MOTOR_B_EN_PIN, LOW);

  pinMode(MOTOR_A_STEP_PIN, OUTPUT);
  pinMode(MOTOR_B_STEP_PIN, OUTPUT);
  pinMode(MOTOR_C_STEP_PIN, OUTPUT);
  digitalWrite(MOTOR_A_STEP_PIN, LOW);
  digitalWrite(MOTOR_B_STEP_PIN, LOW);
  digitalWrite(MOTOR_C_STEP_PIN, LOW);

  pinMode(MOTOR_A_DIR_PIN, OUTPUT);
  pinMode(MOTOR_B_DIR_PIN, OUTPUT);
  pinMode(MOTOR_C_DIR_PIN, OUTPUT);
  digitalWrite(MOTOR_A_DIR_PIN, LOW);
  digitalWrite(MOTOR_B_DIR_PIN, LOW);
  digitalWrite(MOTOR_C_DIR_PIN, HIGH);


  xTaskCreatePinnedToCore(
    Motor_engine,       /* Task function. */
    "Motor_engine",     /* name of task. */
    10000,              /* Stack size of task */
    (void*)NULL,        /* parameter of the task */
    1,                  /* priority of the task */
    &Motor_engine_task, /* Task handle to keep track of created task */
    1);                 /* pin task to core 1 */
}


void Motor_engine(void* Parameters) {
  //setup
  Serial.print("Motor Engine running on core ");
  Serial.println(xPortGetCoreID());


  unsigned long t_MA = 0;
  unsigned long t_MB = 0;
  unsigned long t_MC = 0;

  unsigned long t_motor = micros();
  unsigned long dt_motor;
  unsigned long new_t_motor;

  //loop
  while (true) {

    //time calculation
    new_t_motor = micros();
    dt_motor = new_t_motor - t_motor;
    t_motor = new_t_motor;
    t_MA += dt_motor;
    t_MB += dt_motor;
    t_MC += dt_motor;


    // incrementation of the stepper motors 
    if (dirA != 0 && t_MA > dt_MA) { 
      digitalWrite(MOTOR_A_STEP_PIN, HIGH);
      total_stepA += dirA;
      t_MA = 0;
    } else {
      digitalWrite(MOTOR_A_STEP_PIN, LOW);
    }

    if (dirB != 0 && t_MB > dt_MB) {
      digitalWrite(MOTOR_B_STEP_PIN, HIGH);
      total_stepB += dirB;
      t_MB = 0;
    } else {
      digitalWrite(MOTOR_B_STEP_PIN, LOW);
    }

    if (dirC != 0 && t_MC > dt_MC) {
      digitalWrite(MOTOR_C_STEP_PIN, HIGH);
      total_stepC += dirC;
      t_MC = 0;
    } else {
      digitalWrite(MOTOR_C_STEP_PIN, LOW);
    }

    engine_update(dt_motor); // uptate each motor step period
  }
}



void set_speed(float vA, float vC) {
  v_MA = vA/(PI * diameter);
  v_MC = vC/(PI * diameter);
}

void set_angular_speed(float vA, float vC) {
  v_MA = vA;
  v_MC = vC;
}

void set_acceleration(float aA, float aC) {
  a_MA = aA/(PI * diameter);
  a_MC = aC/(PI * diameter);
}

void set_angular_acceleration(float aA, float aC) {
  a_MA = aA;
  a_MC = aC;
}

float list_speed[3] = { 0.0, 0.0, 0.0 };
float* get_speed() {
  list_speed[0] = v_tot_A*(PI * diameter);
  list_speed[1] = v_tot_B * lever_ratio;
  list_speed[2] = v_tot_C*(PI * diameter);
  return list_speed;
}

float list_dist[3] = { 0.0, 0.0, 0.0 };
float* get_dist() {
  list_dist[0] = total_stepA * 1.0 / (steps * microsteps) * diameter * PI;
  list_dist[1] = total_stepB * 1.0 / (steps * microsteps) * lever_ratio;
  list_dist[2] = total_stepC * 1.0 / (steps * microsteps) * diameter * PI;
  return list_dist;
}

//low speed loop
void engine_update(unsigned long dt_loop) { 

  //acceleration calculation of motor A with limit to +/- vmax
  v_MA += a_MA * dt_loop * 1e-6 ;
  if (v_MA > v_max_ang) {
    v_MA = v_max_ang;
  } else if (v_MA < -v_max_ang) {
    v_MA = -v_max_ang;
  }

  //displacement calculation of motor B to set the extension/retraction speed 
  //and the wheel counter rotation 
  if (total_stepB > target_step){
    v_MB = Speed_stand / lever_ratio;
  
  } else if (total_stepB < target_step){
    v_MB = -Speed_stand / lever_ratio;
    
  } else {
    v_MB = 0;
  }
  
  //acceleration calculation of motor A with limit to +/- vmax
  v_MC += a_MC * dt_loop * 1e-6;
  if (v_MC > v_max_ang) {
    v_MC = v_max_ang;
  } else if (v_MC < -v_max_ang) {
    v_MC = -v_max_ang;
  }
  
  //total speeds 
  v_tot_A = v_MA + v_diff;
  v_tot_B = v_MB;
  v_tot_C = v_MC - v_diff;


  //compute freq from speed
  f_MA = v_tot_A * steps * microsteps;
  f_MB = v_tot_B * steps * microsteps;
  f_MC = v_tot_C * steps * microsteps;

  //compute the increment period from freq and set the direction
  if (f_MA < 0) {
    dt_MA = 1e6 / abs(f_MA);
    digitalWrite(MOTOR_A_DIR_PIN, HIGH);
    dirA = 1;
  } else if (f_MA > 0) {
    dt_MA = 1e6 / abs(f_MA);
    digitalWrite(MOTOR_A_DIR_PIN, LOW);
    dirA = -1;
  } else {
    dirA = 0;
  }


  if (f_MB > 0) {
    dt_MB = 1e6 / abs(f_MB);
    digitalWrite(MOTOR_B_DIR_PIN, LOW);
    dirB = -1;
  } else if (f_MB < 0) {
    dt_MB = 1e6 / abs(f_MB);
    digitalWrite(MOTOR_B_DIR_PIN, HIGH);
    dirB = 1;
  } else {
    dirB = 0;
  }

  if (f_MC > 0) {
    dt_MC = 1e6 / abs(f_MC);
    digitalWrite(MOTOR_C_DIR_PIN, HIGH);
    dirC = -1;
  } else if (f_MC < 0) {
    dt_MC = 1e6 / abs(f_MC);
    digitalWrite(MOTOR_C_DIR_PIN, LOW);
    dirC = 1;
  } else {
    dirC = 0;
  }
}

void reset_dist() { //reset function
  total_stepA = 0;
  total_stepB = 0;
  total_stepC = 0;
}

void set_turn(float angular_speed){ // apply a differential speed on the wheel to turn
  v_diff = (lenght_btw_wheels*PI*angular_speed / (180.0*2))/(PI * diameter) ;
}

void emergency(bool stop){ //shut off the power 
  if(stop){
    digitalWrite(MOTOR_AC_EN_PIN, HIGH);
    digitalWrite(MOTOR_B_EN_PIN, HIGH);
  }else{
    digitalWrite(MOTOR_AC_EN_PIN, LOW);
    digitalWrite(MOTOR_B_EN_PIN, LOW);
  }
}         

bool stand(float angle, float speed){
  target_step = angle/ (360 * lever_ratio) * steps * microsteps;
  Speed_stand = speed*1.0/360.0 ; // go from deg/s to rot/s
  return total_stepB == target_step;
}


bool is_ready(){
  return total_stepB == target_step;
}
