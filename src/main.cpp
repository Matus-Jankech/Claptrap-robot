/* Includes */
#include <Claptrap.h>

/* Global variables */
Claptrap claptrap;
bool LED_switch = true;
double ref_vel[2];
unsigned long Last_serial_timer;

//======================================
//              SETUP
//======================================
void setup() {
    claptrap.begin();
    attachInterrupt(digitalPinToInterrupt(Encoder_A1_PIN), read_encoder_1, RISING);
    attachInterrupt(digitalPinToInterrupt(Encoder_A2_PIN), read_encoder_2, RISING);
    //claptrap.calibrate_gyro();
}

//======================================
//               LOOP
//======================================
void loop() {
    if(claptrap.is_radio_connected()){
        claptrap.read_radio();
        if(claptrap.radio_data.j1PotX == 0){
            if(LED_switch){
                claptrap.set_motor_stop_flag(false); 
                claptrap.set_eye_color(0,255,0);
                LED_switch = false;
            }
        }
        else{
            if(!LED_switch){
                claptrap.inicialize_PID_values();
                claptrap.set_motor_stop_flag(false); 
                claptrap.set_eye_color(255,0,0);
                LED_switch = true;
            }
        }
    }
    else{
        LED_switch = false;
        claptrap.set_motor_stop_flag(true); 
    }

    /* Dry friction test */
    claptrap.set_motor_pwm(claptrap.manual_pwm[0],M1_front_PIN,M1_back_PIN);
    claptrap.set_motor_pwm(claptrap.manual_pwm[1],M2_front_PIN,M2_back_PIN);

    claptrap.set_ref_tilt(80);
    claptrap.calculate_tilt_PID();

    /* Serial COM test*/
    claptrap.read_serial();
    if(millis() - Last_serial_timer > 200){
        Last_serial_timer = millis();
        claptrap.write_serial('S');
    }

    delay(10);
}

//======================================
//               ISR
//======================================
void read_encoder_1(){
    claptrap.read_encoder(Encoder_B1_PIN, 0);
}

void read_encoder_2(){
     claptrap.read_encoder(Encoder_B2_PIN, 1);
}