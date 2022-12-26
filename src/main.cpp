#include <Control.h>

double* vel;

void setup() {
    Serial.begin(115200);
    motors_init();
    encoders_init();
    radio_init();
    LEDs_init();
    MP3_init();
}

void loop() {
    if(is_radio_connected()){
        set_motor_velocity(true,100,M1_front_PIN,M1_back_PIN);
        set_motor_velocity(true,0,M2_front_PIN,M2_back_PIN);
    }
    else{
        set_motor_velocity(true,0,M1_front_PIN,M1_back_PIN);
        set_motor_velocity(true,0,M2_front_PIN,M2_back_PIN);
    }

    get_velocity(&vel);
    Serial.print(vel[0]);
    Serial.println(" ");
    delay(10);
}