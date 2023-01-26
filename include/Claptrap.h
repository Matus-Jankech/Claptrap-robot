/* Inludes */
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <DFPlayerMini_Fast.h>
#include <RF24.h>

/* ISR function headers */
void read_encoder_1();
void read_encoder_2(); 


//======================================
//              Defines
//======================================
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
#define pipe 0xE8E8F0F0E1LL


//======================================
//        Radio data structure
//======================================
typedef struct RADIO_DATA_STRUCT {
    byte j1PotX;
    byte j1PotY;
    byte j2PotX;
    byte j2PotY;
} radio_data_struct;


//======================================
//            Claptrap class
//======================================
class Claptrap {
    public:
        /* Constructor */
        Claptrap();

        /* Methods */
        void begin();
        void read_radio(void);
        bool is_radio_connected(void);
        void set_eye_color(uint8_t r, uint8_t g, uint8_t b);
        void MP3_play(uint8_t track_num);
        void MP3_set_volume(uint8_t volume);
        void read_encoder(uint8_t encoder_B_pin);
        void send_serial(char ident);
        void read_serial(void);
        void calculate_velocity_PID(double* ref_vel); // Later move to private methods  

        /* Variables */
        radio_data_struct radio_data;

    private:
        /* Methods */
        void radio_begin(void);
        void LEDs_begin(void);
        void MP3_begin(void);
        void encoders_begin(void);
        void motors_begin(void);     
        void set_motor_pwm(int pwm_value, int pin_1, int pin_2);
        void filter_velocity(double* velocity);
        void get_velocity(double* velocity);

        /* Radio variables */
        RF24 *radio;
        bool radio_status = false;
        unsigned long radio_last_receive_time;

        /* MP3 and LEDs variables */
        DFPlayerMini_Fast mp3;
        Adafruit_NeoPixel pixels;

        /* Encoders variables */
        volatile long current_encoder_pos[2];
        volatile long last_encoder_pos[2];
        unsigned long encoders_last_read_time;
        double vel_filtered_1[2];
        double vel_filtered_2[2];

        /* Motors variables */
        double Kp_vel = 0.7, Ki_vel = 4, Kd_vel = 0;
        double P_gain[2], I_gain[2];
        unsigned long PID_last_calc_time;
};