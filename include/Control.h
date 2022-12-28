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

/* Structures */
typedef struct RADIO_DATA_STRUCT {
    byte j1PotX;
    byte j1PotY;
    byte j2PotX;
    byte j2PotY;
} radio_data_struct;

/* Function headers*/
void radio_init(void);
void LEDs_init(void);
void MP3_init(void);
void encoders_init(void);
void motors_init(void);
void read_encoder_1(void);
void read_encoder_2(void);
bool is_radio_connected(void);
void set_motor_pwm(int pwm_value, int pin_1, int pin_2);
void calculate_velocity_PID(double* ref_vel);
void get_velocity(double* velocity);
radio_data_struct read_radio(void);