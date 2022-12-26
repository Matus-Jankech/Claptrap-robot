#include <Control.h>

radio_data_struct radio_data;

void setup() {
    Serial.begin(115200);
    motors_init();
    encoders_init();
    radio_init();
    LEDs_init();
    MP3_init();
}

void loop() {
    double ref_vel[2];

    if(is_radio_connected()){
        radio_data = read_radio();
        if(radio_data.j1PotX == 0){
            ref_vel[0] = 100;
            ref_vel[1] = 0;
        }
        else{
            ref_vel[0] = -100;
            ref_vel[1] = 0; 
        }
    }
    else{
        ref_vel[0] = 0;
        ref_vel[1] = 0;
    }

    calculate_velocity_PID(ref_vel);
    delay(10);
}