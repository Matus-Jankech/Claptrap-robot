/* Includes */
#include <Claptrap.h>

/* Methods definition */
void Claptrap::motors_begin(){
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

void Claptrap::set_motor_pwm(int pwm_value, int pin_1, int pin_2){
    if(pwm_value > 35){
        analogWrite(pin_2,0);
        analogWrite(pin_1,pwm_value);
    }  
    else if(pwm_value < -35){
        analogWrite(pin_1,0);
        analogWrite(pin_2,-pwm_value);
    }
    else{
        analogWrite(pin_1,0);
        analogWrite(pin_2,0);
    }
}

void Claptrap::calculate_velocity_PID(double* ref_vel){
    const double Kp[2] = {0.7,0.7};
    const double Ki[2] = {4,4};
    const int NUM_OF_MOTORS = 2;
    const int DRY_FRICTION_CONST = 30;
    double delta_time;
    double current_vel[2];
    double error[2];
    int16_t pwm[2];
    
    /* Get current velocity on each motor */
    get_velocity(current_vel);

    for(int i = 0; i < NUM_OF_MOTORS; i++){
        error[i] = ref_vel[i] - current_vel[i]; 
        delta_time = (micros() - PID_last_calc_time)/1e6; 

        /* Proportional gain */
        P_gain[i] = Kp[i]*error[i];

        /* Integral gain */
        if((P_gain[i] + I_gain[i]) < 255 && (P_gain[i] + I_gain[i]) > -255){ // Anti-windup
            I_gain[i] = I_gain[i] + Ki[i]*error[i]*delta_time;
        }

        /* Input for motors */
        pwm[i] = P_gain[i] + I_gain[i];
        if(pwm[i] > 0){
            pwm[i] = pwm[i] + DRY_FRICTION_CONST;
        }
        else if(pwm[i] < 0){
            pwm[i] = pwm[i] - DRY_FRICTION_CONST;
        }

        /* Saturation of inputs for motors*/
        if(pwm[i] > 255){
            pwm[i] = 255;
        } 
        else if(pwm[i] < -255){
            pwm[i] = -255;   
        }
    }

    /* Write values into motors */
    set_motor_pwm(pwm[0],M1_front_PIN,M1_back_PIN);
    set_motor_pwm(pwm[1],M2_front_PIN,M2_back_PIN);
    PID_last_calc_time = micros();
    
    /* Serial output for debuging */
    Serial.print(ref_vel[0]);
    Serial.print(" , ");
    Serial.print(current_vel[0]);
    Serial.print(" , ");
    Serial.print(0);
    Serial.print(" , ");
    Serial.print(pwm[0]);
    Serial.println(" ");
}