/* Includes */
#include <Control.h>

void set_motor_velocity(bool dir, int pwmVal, int pin1, int pin2){
    if(dir == true){
        analogWrite(pin2,0);
        analogWrite(pin1,pwmVal);
    }  
    if(dir == false){
        analogWrite(pin1,0);
        analogWrite(pin2,pwmVal);
    }
}

