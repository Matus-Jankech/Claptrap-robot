#include <Control.h>

/* Global variables */
radio_data_struct radio_data;
Claptrap claptrap;
bool LED_switch = true;

void setup() {
    Serial.begin(115200);
    claptrap.begin();
}

void loop() {
    double ref_vel[2];

    if(claptrap.is_radio_connected()){
        radio_data = claptrap.read_radio();
        if(radio_data.j1PotX == 0){
            ref_vel[0] = 100;
            ref_vel[1] = 0;
            if(LED_switch){
                Serial.println("eye_green");
                claptrap.set_eye_color(0,255,0);
                LED_switch = false;
            }
        }
        else{
            ref_vel[0] = -100;
            ref_vel[1] = 0; 
            if(!LED_switch){
                Serial.println("eye_red");
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
    delay(10);
}