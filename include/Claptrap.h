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
//        Radio data structures
//======================================
typedef struct RADIO_DATA_STRUCT {
    byte switch1;
    byte switch2;
    byte switch3;
    byte switch4;
    byte button1;
    byte button2;
    byte button3;
    byte button4; 
    byte j1PotX;
    byte j1PotY;
    byte j2PotX;
    byte j2PotY;
} radio_data_struct;

typedef struct LOCAL_RADIO_DATA_STRUCT{
    float switch1;
    float switch2;
    float switch3;
    float switch4;
    float button1;
    float button2;
    float button3;
    float button4; 
    float j1PotX;
    float j1PotY;
    float j2PotX;
    float j2PotY;
} local_radio_data_struct;


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
        void set_wheels_ref_vel(double* wheels_vel);
        void set_ref_pitch(double pitch);
        void set_angular_vel(double angular_vel);
        void set_motor_stop_flag(bool state);
        void set_motor_pwm(int pwm_value, int pin_1, int pin_2);
        void inicialize_MPU_values(void);
        void inicialize_PID_values(void);
        void calibrate_gyro(void); 
        bool is_standing(void);
        void print_radio(void);

        // Later move to private methods 
        void calculate_pitch_PID();
        void calculate_wheels_velocity_PID(); 
        void calculate_robot_velocity_PID(); 

        /* Variables */
        local_radio_data_struct radio_in;
        bool is_balancing = true;

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
        void input_filter(void);

        /* Radio variables */
        RF24* radio;
        radio_data_struct radio_data;
        byte* radio_members = &radio_data.switch1;
        float* local_radio_member = &radio_in.switch1;
        float radio_current_input_values[12];
        float radio_input_integrators[4];
        bool radio_status = false;
        unsigned long radio_last_receive_time;
        unsigned long radio_last_integration_time;

        /* MP3 and LEDs variables */
        DFPlayerMini_Fast mp3;
        Adafruit_NeoPixel pixels;
        unsigned long switch_last_read_time[4];
        int switch_last_state[4];

        /* Encoders variables */
        volatile long current_encoder_pos[2];
        volatile long last_encoder_pos[2];
        unsigned long encoders_last_read_time;
        double vel_filtered_1[2];
        double vel_filtered_2[2];

        /* Motor (PID) variables */
        double Kp_vel = 3, Ki_vel = 15, Kd_vel = 0;
        double Kp_pitch = 18, Ki_pitch = 222, Kd_pitch = 0;
        double Kp_robot_vel = 0, Ki_robot_vel = 0, Kd_robot_vel = 0;

        double P_wheels_vel_gain[2], I_wheels_vel_gain[2];
        double P_pitch_gain, I_pitch_gain, D_pitch_gain, pitch_last_error;
        double P_robot_vel_gain, I_robot_vel_gain, D_robot_vel_gain,robot_vel_last_error;
        double ref_wheels_vel[2], ref_pitch, ref_linear_vel, ref_angular_vel;

        unsigned long PID_wheels_vel_last_calc_time;
        unsigned long PID_pitch_last_calc_time;
        unsigned long PID_robot_vel_last_calc_time;
        bool motor_stop_flag;

        /* Acc + Gyro variables */
        double gyro_rates[3], gyro_angles[2], acc_angles[2]; 
        double kalman_angles[2], kalman_uncertainty_angles[2];
        double angles_filtered[2]; 
        unsigned long last_gyro_read_time = 0;
        unsigned long last_MPU_filter_time = 0;
};