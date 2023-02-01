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
    const int DRY_FRICTION_CONST = 35;

    if(pwm_value > DRY_FRICTION_CONST){
        analogWrite(pin_2,0);
        analogWrite(pin_1,pwm_value);
    }  
    else if(pwm_value < -DRY_FRICTION_CONST){
        analogWrite(pin_1,0);
        analogWrite(pin_2,-pwm_value);
    }
    else{
        analogWrite(pin_1,0);
        analogWrite(pin_2,0);
    }
}

void Claptrap::calculate_velocity_PID(double* ref_vel){
    const int NUM_OF_MOTORS = 2;
    const int DRY_FRICTION_CONST = 30;
    const int MAX_OUTPUT_PWM = 255;
    unsigned long current_time = micros();
    double delta_time = (double)(current_time - PID_vel_last_calc_time)/1e6;
    double current_vel[2], error[2];
    int16_t pwm[2];
    
    /* Get current velocity on each motor */
    Claptrap::get_velocity(current_vel);

    for(int i = 0; i < NUM_OF_MOTORS; i++){
        error[i] = ref_vel[i] - current_vel[i]; 

        /* Proportional gain */
        P_vel_gain[i] = Kp_vel*error[i];

        /* Integral gain */
        if((P_vel_gain[i] + I_vel_gain[i]) < MAX_OUTPUT_PWM && (P_vel_gain[i] + I_vel_gain[i]) > -MAX_OUTPUT_PWM){ // Anti-windup
            I_vel_gain[i] = I_vel_gain[i] + Ki_vel*error[i]*delta_time;
        }

        /* Input for motors */
        pwm[i] = P_vel_gain[i] + I_vel_gain[i];
        if(pwm[i] > 0){
            pwm[i] = pwm[i] + DRY_FRICTION_CONST;
        }
        else if(pwm[i] < 0){
            pwm[i] = pwm[i] - DRY_FRICTION_CONST;
        }

        /* Saturation of inputs for motors*/
        if(pwm[i] > MAX_OUTPUT_PWM){
            pwm[i] = MAX_OUTPUT_PWM;
        } 
        else if(pwm[i] < -MAX_OUTPUT_PWM){
            pwm[i] = -MAX_OUTPUT_PWM;   
        }
    }

    /* Write values into motors */
    Claptrap::set_motor_pwm(pwm[0],M1_front_PIN,M1_back_PIN);
    Claptrap::set_motor_pwm(pwm[1],M2_front_PIN,M2_back_PIN);
    PID_vel_last_calc_time = current_time;


    Serial.print(ref_vel[0]);
    Serial.print(" , ");
    Serial.print(current_vel[0]);
    Serial.print(" , ");
    Serial.println(error[0]);
}

void Claptrap::calculate_tilt_PID(double ref_tilt){
    const int MAX_OUTPUT_VEL = 60;
    unsigned long current_time = micros();
    double delta_time = (double)(current_time - PID_tilt_last_calc_time)/1e6;
    double error, current_tilt[2], ref_vel[2];

    /* Read current tilt of robot */
    Claptrap::read_MPU(current_tilt);
    error = ref_tilt - current_tilt[1]; 

    /* Proportional gain */
    P_tilt_gain = Kp_tilt*error;

    /* Integral gain */
    if((P_tilt_gain + I_tilt_gain) < MAX_OUTPUT_VEL && (P_tilt_gain + I_tilt_gain) > -MAX_OUTPUT_VEL){ // Anti-windup
        I_tilt_gain = I_tilt_gain + Ki_tilt*error*delta_time;
    }

    /* Derivative gain */
    D_tilt_gain = Kd_tilt*(error - last_error)*delta_time;

    /* Input for velocity controller */
    ref_vel[0] = P_tilt_gain + I_tilt_gain + D_tilt_gain;

    /* Saturation of ref_vel*/
    if(ref_vel[0] > MAX_OUTPUT_VEL){
        ref_vel[0] = MAX_OUTPUT_VEL;
    } 
    else if(ref_vel[0] < -MAX_OUTPUT_VEL){
        ref_vel[0] = -MAX_OUTPUT_VEL;   
    }

    ref_vel[1] = ref_vel[0];
    
    /* Write values into motors */
    Claptrap::calculate_velocity_PID(ref_vel);
    last_error = error;
    PID_tilt_last_calc_time = current_time;
}