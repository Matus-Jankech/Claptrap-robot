/* Includes */
#include <Claptrap.h>

/* Global variables */
Claptrap claptrap;
double ref_vel[2];
unsigned long Last_serial_timer, last_radio_reading;
unsigned long current_loop_time, last_loop_time;

//======================================
//              SETUP
//======================================
void setup() {
    claptrap.begin();
    attachInterrupt(digitalPinToInterrupt(Encoder_A1_PIN), read_encoder_1, RISING);
    attachInterrupt(digitalPinToInterrupt(Encoder_A2_PIN), read_encoder_2, RISING);
    claptrap.set_motor_stop_flag(true); 
    //claptrap.calibrate_gyro();
    delay(1000);
}

//======================================
//               LOOP
//======================================
void loop() {
    current_loop_time = micros();

    if(current_loop_time - last_loop_time > 2000){

        claptrap.calculate_tilt_PID();   

        if(claptrap.is_radio_connected()){
            if(millis() - last_radio_reading > 20){
                last_radio_reading = millis();
                claptrap.read_radio();
                //claptrap.print_radio();
            }
            if(claptrap.radio_in.switch4 == true){
                if(claptrap.is_balancing == false){
                    if(claptrap.is_standing()){
                        claptrap.inicialize_PID_values();
                        claptrap.set_motor_stop_flag(false); 
                        claptrap.set_eye_color(0,255,0);
                        claptrap.is_balancing = true;
                    }
                }
            }
            else if(claptrap.radio_in.switch4 == false){
                if(claptrap.is_balancing == true){
                        claptrap.set_motor_stop_flag(true); 
                        claptrap.set_eye_color(255,0,0);
                        claptrap.is_balancing = false; 
                }
            }
            claptrap.set_ref_tilt(claptrap.radio_in.j1PotY);
            claptrap.set_angular_vel(claptrap.radio_in.j2PotX);
        }
        else{
            claptrap.is_balancing = true;
            claptrap.set_motor_stop_flag(true); 
            claptrap.set_eye_color(0,0,255);
            claptrap.set_ref_tilt(0);
            claptrap.set_angular_vel(0);
        }

        /* Serial COM test*/
        /*claptrap.read_serial();
        if(millis() - Last_serial_timer > 200){
            Last_serial_timer = millis();
            claptrap.write_serial('D');
        }*/

        last_loop_time = current_loop_time;
    }
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