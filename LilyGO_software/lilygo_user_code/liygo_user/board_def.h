#include <Arduino.h>

#define LORA_V1_0_OLED  0
#define LORA_V1_2_OLED  0
#define LORA_V1_6_OLED  1
#define LORA_V2_0_OLED  0

// Change here whether you define Sender or Receiver.
// The rest should be the same
//#define LORA_SENDER 0
#define LORA_SENDER 1

#define LORA_PERIOD 433
// #define LORA_PERIOD 915     
//#define LORA_PERIOD 433  

#if LORA_V1_0_OLED
#include <Wire.h> 
#include "SSD1306Wire.h" 
#define OLED_CLASS_OBJ  SSD1306Wire
#define OLED_ADDRESS    0x3C
#define OLED_SDA    4
#define OLED_SCL    15
#define OLED_RST    16
#define CONFIG_MOSI 27
#define CONFIG_MISO 19
#define CONFIG_CLK  5
#define CONFIG_NSS  18
#define CONFIG_RST  14
#define CONFIG_DIO0 26
// !    There are two versions of TTGO LoRa V1.0, 
// !    the 868 version uses the 3D WiFi antenna, and the 433 version uses the PCB antenna. 
// !    You need to change the frequency according to the board.

#define SDCARD_MOSI -1
#define SDCARD_MISO -1
#define SDCARD_SCLK -1
#define SDCARD_CS   -1

#elif LORA_V1_2_OLED
//Lora V1.2 ds3231
#include <Wire.h> 
#include "SSD1306Wire.h" 
#define OLED_CLASS_OBJ  SSD1306Wire
#define OLED_ADDRESS    0x3C
#define OLED_SDA    21
#define OLED_SCL    22
#define OLED_RST    -1
#define CONFIG_MOSI 27
#define CONFIG_MISO 19
#define CONFIG_CLK  5
#define CONFIG_NSS  18
#define CONFIG_RST  23
#define CONFIG_DIO0 26

#define SDCARD_MOSI -1
#define SDCARD_MISO -1
#define SDCARD_SCLK -1
#define SDCARD_CS   -1

#define ENABLE_DS3231

#elif LORA_V1_6_OLED
#include <Wire.h> 
#include "SSD1306Wire.h" 
#define OLED_CLASS_OBJ  SSD1306Wire
#define OLED_ADDRESS    0x3C
#define OLED_SDA    21
#define OLED_SCL    22
#define OLED_RST    -1

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

#elif LORA_V2_0_OLED
#include <Wire.h> 
#include "SSD1306Wire.h" 
#define OLED_CLASS_OBJ  SSD1306Wire
#define OLED_ADDRESS    0x3C
#define OLED_SDA    21
#define OLED_SCL    22
#define OLED_RST    -1

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

#else
#error "please select board"
#endif


#if LORA_PERIOD == 433
#define BAND 433E6
#elif LORA_PERIOD == 868
#define BAND 868E6
#elif LORA_PERIOD == 915
#define BAND 915E6
#else
#error "Please select the correct lora frequency"
#endif