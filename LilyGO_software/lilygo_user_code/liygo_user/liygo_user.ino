#include <SPI.h>
#include <LoRa.h>
#include <Wire.h> 


#define Pin 15
#define Buzz 13

#define LORA_PERIOD 433
#define BAND 433E6


#define CONFIG_MOSI 27
#define CONFIG_MISO 19
#define CONFIG_CLK  5
#define CONFIG_NSS  18
#define CONFIG_RST  23
#define CONFIG_DIO0 26



unsigned long t ;
int state = 0;
int prevstate = 0;
byte cmd;

void setup()
{
  // init serial
  Serial.begin(115200);
  while (!Serial);


  // init LoRa
  SPI.begin(CONFIG_CLK, CONFIG_MISO, CONFIG_MOSI, CONFIG_NSS);
  LoRa.setPins(CONFIG_NSS, CONFIG_RST, CONFIG_DIO0);
  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  //init Pin
  pinMode(Pin, OUTPUT);
  pinMode(Pin, INPUT_PULLUP);
  pinMode(Buzz, OUTPUT);
  digitalWrite(Buzz, LOW);
  
  prevstate = esp_sleep_get_wakeup_cause()!=ESP_SLEEP_WAKEUP_TIMER && !digitalRead(Pin);
  t = millis();
}

int count = 0;

//main loop
void loop(){
  Keyboard_input();
  LoRA_sender();
}

void LoRA_sender(){ 

  state = digitalRead(Pin);

  if(state==HIGH){
    cmd = cmd & 127;
  }

  LoRa.beginPacket();
  // send two packet for redundancy
  LoRa.write(cmd);
  LoRa.write(cmd);
  LoRa.endPacket();
  Serial.println(cmd, BIN);


  //buzzer logic
  if(state == HIGH && prevstate == LOW){
    digitalWrite(Buzz, HIGH);
    delay(100);
    digitalWrite(Buzz, LOW);
  }


  if(state == LOW && prevstate == HIGH){
    digitalWrite(Buzz, HIGH);
    delay(100);
    digitalWrite(Buzz, LOW);
  }
  prevstate = state;
}


void Keyboard_input(){
  if (Serial.available() > 0) {
    cmd = Serial.read();
  }
  
  while(Serial.available()){
    Serial.read();
  }
}