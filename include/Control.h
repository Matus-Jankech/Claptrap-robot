/* Inludes */
#include <Arduino.h>

/* Definitions */
#define SCK_PIN 13
#define MISO_PIN 12
#define MOSI_PIN 11
#define CSN_PIN 10
#define CE_PIN 9
#define Leds_PIN 8
#define M1_front_PIN 7
#define M1_back_PIN 6
#define M2_front_PIN 5
#define M2_back_PIN 4
#define MPU_int_PIN 2
#define MP3_Rx_PIN 1 // yellow
#define MP3_Tx_PIN 0 // blue
#define Encoder_A2_PIN A0
#define Encoder_B2_PIN A1
#define Encoder_A1_PIN A2
#define Encoder_B1_PIN A3
#define MPU_SDA_PIN A4 // white 18
#define MPU_SCL_PIN A5 // blue 19

/* Function headers*/
void pins_init(void);
void radio_init(void);
void read_encoder_1(void);
void read_encoder_2(void);
void LEDs_init(void);
void MP3_init(void);
bool is_radio_connected(void);