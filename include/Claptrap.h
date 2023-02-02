/* Inludes */
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <DFPlayerMini_Fast.h>
#include <RF24.h>
#include <Wire.h>

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
#define Encoder_A2_PIN A1
#define Encoder_B2_PIN A0
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
        void read_encoder(uint8_t encoder_B_pin, uint8_t index);
        void write_serial(char ident);
        void read_serial(void);
        void read_MPU(void);
        void set_ref_vel(double* vel);
        void set_ref_tilt(double tilt);
        void set_motor_stop_flag(bool state);
        void set_motor_pwm(int pwm_value, int pin_1, int pin_2);
        void inicialize_MPU_values(void);
        void inicialize_PID_values(void);
        void calibrate_gyro(void); 

        // Later move to private methods 
        void calculate_tilt_PID();
        void calculate_velocity_PID(); 

        /* Variables */
        radio_data_struct radio_data;
        int manual_pwm[2];

    private:
        /* Methods */
        void radio_begin(void);
        void LEDs_begin(void);
        void MP3_begin(void);
        void encoders_begin(void);
        void motors_begin(void); 
        void MPU_begin(void);    
        void filter_velocity(double* velocity);
        void get_velocity(double* velocity);
        void read_acc(void);
        void read_gyro(void); 
        void integrate_gyro(void);
        void kalman_filter(void);

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

        /* Motor (PID) variables */
        double Kp_vel = 1, Ki_vel = 1, Kd_vel = 0;
        double Kp_tilt = 3, Ki_tilt = 5, Kd_tilt = 0.5;
        double P_vel_gain[2], I_vel_gain[2];
        double P_tilt_gain, I_tilt_gain, D_tilt_gain, last_error;
        double ref_vel[2], ref_tilt;
        int16_t pwm[2];
        unsigned long PID_vel_last_calc_time;
        unsigned long PID_tilt_last_calc_time;
        bool motor_stop_flag;

        /* Acc + Gyro variables */
        double gyro_rates[3], gyro_angles[2], acc_angles[2]; 
        double kalman_angles[2], kalman_uncertainty_angles[2];
        double angles_filtered[2]; 
        unsigned long last_gyro_read_time = 0;
        unsigned long last_MPU_filter_time = 0;

};