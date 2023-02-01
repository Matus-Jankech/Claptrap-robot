/* Includes */
#include <Claptrap.h>

/* Global variables */
Claptrap claptrap;
bool LED_switch = true;
double ref_tilt;
double ref_vel[2];

//======================================
//              SETUP
//======================================
void setup() {
    Serial.begin(115200);
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
            ref_vel[1] = 80;
            ref_vel[0] = 0;
            if(LED_switch){
                claptrap.set_eye_color(0,255,0);
                LED_switch = false;
            }
        }
        else{
            ref_vel[1] = -80;
            ref_vel[0] = 0; 
            if(!LED_switch){
                claptrap.set_eye_color(255,0,0);
                LED_switch = true;
            }
        }
    }
    else{
        ref_vel[0] = 0;
        ref_vel[1] = 0;      
    }

    claptrap.calculate_velocity_PID(ref_vel);
    /* Serial COM test*/
    //claptrap.read_serial();

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