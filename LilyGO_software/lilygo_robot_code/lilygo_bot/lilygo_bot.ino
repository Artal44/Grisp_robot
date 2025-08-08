#include <SPI.h>
#include <LoRa.h>
#include <Arduino.h>
#include <Wire.h>

#include "motor_engine.h"

#define BAND 433E6
#define CONFIG_MOSI 27
#define CONFIG_MISO 19
#define CONFIG_CLK  5
#define CONFIG_NSS  18
#define CONFIG_RST  23
#define CONFIG_DIO0 26

#define SDCARD_MOSI 15
#define SDCARD_MISO 2
#define SDCARD_SCLK 14
#define SDCARD_CS   13

#define I2C_SLAVE_ADDR 0x40

// trapezoidal speed command parametter for turning, migration on GRiSP for this part
#define max_turn_speed 80
#define turn_acc 400 


float I2C_command[2] = {0.0, 0.0}; // value received from GRiSP : {wheels acceleration , turn speed}

float freq_lim [13] = {300,200,175,150,125,100,90,80,70,60,50,40,30};
int size_test_freq = sizeof(freq_lim)/sizeof(freq_lim[0]);
int index_lim = 0;

// time mesure variable
unsigned long t_GRiSP;
unsigned long t_LORA;
unsigned long t_test;
unsigned long t_ESP;

// freq and period variable
float dt_GRiSP = 10;
float freq_GRiSP = 200;
float dt_ESP = 0;

// control byte received
byte cmd = 0; // received from LoRa communication and transfered to GRiSP
byte GRiSP_flags = 0; // Received from GRiSP

//control flag
bool new_cmd =false;
bool test = false;
bool disturb = false;
bool ext_end = true;



void setup() {
  Serial.begin(115200);
  // SPI - LoRa init
  SPI.begin(CONFIG_CLK, CONFIG_MISO, CONFIG_MOSI, CONFIG_NSS);
  LoRa.setPins(CONFIG_NSS, CONFIG_RST, CONFIG_DIO0);
  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  // I2C Slave init, work with IRQ so no need to incorporate into the main loop
  Wire.begin(I2C_SLAVE_ADDR);
  Wire.onReceive(GRiSP_receiver);
  Wire.onRequest(GRiSP_sender);

  // motor init, works on core n°2
  engine_init();
  delay(1000);
  set_speed(0, 0);
  set_acceleration(0, 0);

  // time init
  t_GRiSP = millis();
  t_LORA = t_GRiSP;
  t_ESP = t_GRiSP;
}


void loop() {
  unsigned long new_t_ESP = millis();
  dt_ESP = (new_t_ESP - t_ESP) / 1000.0;
  t_ESP = new_t_ESP;
  LoRa_receiver();
  Event_handle();
  delay(1);
}



void GRiSP_receiver(int howMany) {
  if(howMany == 5){ // check if the packet match the expected lenght
    unsigned long new_t_GRiSP = millis();
    dt_GRiSP = (new_t_GRiSP - t_GRiSP) / 1000.0;
    freq_GRiSP = freq_GRiSP * 0.99 + 1.0 / dt_GRiSP * 0.01;
    t_GRiSP = new_t_GRiSP;

    byte A;
    byte B;
    if (Wire.available()) {

      // read and decode the wheel acceleration
      A = Wire.read();
      B = Wire.read();
      I2C_command[0] = decoder(A, B);

      // read and decode the differential turn speed
      A = Wire.read();
      B = Wire.read();
      I2C_command[1] = decoder(A, B);

      // read the flags
      GRiSP_flags = Wire.read();
    }

    //set acceleration
    if(!disturb && bitRead(GRiSP_flags, 4) && !bitRead(GRiSP_flags, 6)){
      set_acceleration(I2C_command[0], I2C_command[0]);
    }

    // Freeze
    if(bitRead(GRiSP_flags, 6)){ 
      set_acceleration(0, 0);
      set_speed(0, 0);
    }

    // extension/retraction of the rising system
    if(bitRead(GRiSP_flags, 5)){
      ext_end = stand(-30,30.0);
    } else {
      ext_end = stand(0,30.0);
    }
    
  }

  // Empty the stack
  while(Wire.available()){
    Wire.read();
  }
}

void GRiSP_sender() 
{ 
  byte v[5];
  float* speeds = get_speed();
  encoder(v, speeds[0]);
  encoder(v+2, speeds[2]);
  
  // control byte send to GRiSP witth : finsish extension/retraction flag and the command inputs
  v[4] = (cmd & 127) | (is_ready() * 128);
  
  //send 
  Wire.write((byte*) v, sizeof(v));
}

double decoder(byte X, byte Y) {
  // decode half float to double

  byte A = (X & 192);
  if ((A & 64) == 0) A = A | 63;  // fill the missing exponnent bytes with the right value
  byte B = ((X << 2) & 252) | ((Y >> 6) & 3);
  byte C = ((Y << 2) & 252);

  byte vals[] = { 0x00, 0x00, 0x00, 0x00, 0x00, C, B, A };
  double d = 0;
  memcpy(&d, vals, 8);

  return d;
}

void encoder(byte* res, double X){
  // encode double to half float 
  byte vals[8];
  memcpy(vals, &X,8);
  byte A = vals[7];
  byte B = vals[6];
  byte C = vals[5];
  
  res[0] = (A&192)|((B>>2)&63);
  res[1] = ((B<<6)&192)|((C>>2)&63);
  
  return ;
}

void LoRa_receiver(){
// receiption of LoRa packets
  if (LoRa.parsePacket()) {
    if(LoRa.available()>=2){
      byte cmd1 = LoRa.read();
      byte cmd2 = LoRa.read();
      if(cmd1 == cmd2){
        cmd = cmd1;
        new_cmd = true;
      }
    }
    while (LoRa.available()){
      LoRa.read();
    }
  }
}

void Event_handle(){

  //emergency stop 
  emergency(!bitRead(cmd, 7) || !bitRead(GRiSP_flags, 7));
  if (!bitRead(cmd, 7) || !bitRead(GRiSP_flags, 7)){
    set_acceleration(0, 0);
    set_speed(0, 0);
  } 


  // start test procedure
  if(bitRead(cmd, 5)){
    test = true;
    t_test = millis();
  }
  if(test){
    //start the disturbance 500ms to let the record start
    if(millis()> t_test + 500){ 
      //disturb = true;
      //set_acceleration(40, 40);
    }
    // the disturbance is only applied between t=500 and t=800 
    if(millis()> t_test + 800){ 
      disturb = false;
      test = false;
    }
  }

  set_turn(I2C_command[1]);

  new_cmd =false;
}