/* Includes */
#include <Control.h>

void motors_init(){
    /* Set pinmodes */
    pinMode(M1_front_PIN,OUTPUT);
    pinMode(M1_back_PIN,OUTPUT); 
    pinMode(M2_front_PIN,OUTPUT); 
    pinMode(M2_back_PIN,OUTPUT); 

    /* Make sure motors are turned off at startup*/
    analogWrite(M1_front_PIN,0);
    analogWrite(M2_front_PIN,0);
    analogWrite(M1_back_PIN,0);
    analogWrite(M2_back_PIN,0); 
}

void set_motor_velocity(bool dir, int pwm_value, int pin_1, int pin_2){
    if(dir == true){
        analogWrite(pin_2,0);
        analogWrite(pin_1,pwm_value);
    }  
    else if(dir == false){
        analogWrite(pin_1,0);
        analogWrite(pin_2,pwm_value);
    }
}

