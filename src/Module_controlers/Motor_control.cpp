/* Includes */
#include <Control.h>

/* Global variables */
double P_gain[2], I_gain[2];
unsigned long PID_last_calc_time;

/* Methods definition */
void Claptrap::motors_init(){
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
    double delta_time;
    double current_vel[2];
    double error[2];
    int16_t pwm[2];
    
    /* Get current velocity on each motor */
    get_velocity(current_vel);

    error[0] = ref_vel[0] - current_vel[0]; 
    error[1] = ref_vel[1] - current_vel[1];
    delta_time = (micros() - PID_last_calc_time)/1e6; 

    /* Proportional gain */
    P_gain[0] = Kp[0]*error[0];
    P_gain[1] = Kp[1]*error[1];

    /* Integral gain */
    if((P_gain[0] + I_gain[0]) < 255 && (P_gain[0] + I_gain[0]) > -255){ // Anti-windup
        I_gain[0] = I_gain[0] + Ki[0]*error[0]*delta_time;
    }
    if((P_gain[1] + I_gain[1]) < 255 && (P_gain[1] + I_gain[1]) > -255){ // Anti-windup
        I_gain[1] = I_gain[1] + Ki[1]*error[1]*delta_time;
    }

    /* Input for motors */
    pwm[0] = P_gain[0] + I_gain[0];
    pwm[1] = P_gain[1] + I_gain[1];

    /* Saturation of inputs for motors*/
    if(pwm[0] > 255){
        pwm[0] = 255;
    } 
    else if(pwm[0] < -255){
         pwm[0] = -255;   
    }

    if(pwm[1] > 255){
        pwm[1] = 255;
    } 
    else if(pwm[1] < -255){
         pwm[1] = -255;   
    }

    /* Write values into motors */
    set_motor_pwm(pwm[0],M1_front_PIN,M1_back_PIN);
    set_motor_pwm(pwm[1],M2_front_PIN,M2_back_PIN);

    PID_last_calc_time = micros();
    
    /*Serial.print(ref_vel[0]);
    Serial.print(" , ");
    Serial.print(current_vel[0]);
    Serial.print(" , ");
    Serial.print(0);
    Serial.print(" , ");
    Serial.print(pwm[0]);
    Serial.println(" ");*/
}