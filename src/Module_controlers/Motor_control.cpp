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

void Claptrap::inicialize_PID_values(){
    I_tilt_gain = 0;
    I_vel_gain[0] = 0;
    I_vel_gain[1] = 0;
    PID_vel_last_calc_time = micros();
    PID_tilt_last_calc_time = micros();
}

void Claptrap::set_motor_pwm(int pwm_value, int pin_1, int pin_2){
    const int DRY_FRICTION_CONST = 0;

    if(!motor_stop_flag){
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
    else{
        analogWrite(pin_1,0);
        analogWrite(pin_2,0);
    }
}

//======================================
//               Velocity PID
//======================================
void Claptrap::calculate_velocity_PID(){
    const float MOTOR_SCALE_FACTOR[2] = {1.2,1.0};
    const int NUM_OF_MOTORS = 2;
    const int MAX_OUTPUT_PWM = 255;
    unsigned long current_time = micros();
    double delta_time = (double)(current_time - PID_vel_last_calc_time)/1e6;
    double current_vel[2], angular_vel[2], error[2];
    
    Claptrap::get_velocity(current_vel);
    angular_vel[0] = ref_angular_velocity;
    angular_vel[1] = -ref_angular_velocity;

    Serial.print(ref_angular_velocity);
    Serial.print(",");
    Serial.print(angular_vel[0]);
    Serial.print(",");
    Serial.println(angular_vel[1]);

    for(int i = 0; i < NUM_OF_MOTORS; i++){
        error[i] = ref_vel[i] + angular_vel[i] - current_vel[i]; 

        /* PI gains */
        P_vel_gain[i] = Kp_vel*MOTOR_SCALE_FACTOR[i]*error[i];
        if((P_vel_gain[i] + I_vel_gain[i]) < MAX_OUTPUT_PWM && (P_vel_gain[i] + I_vel_gain[i]) > -MAX_OUTPUT_PWM) // Anti-windup
        { 
            I_vel_gain[i] = I_vel_gain[i] + Ki_vel*MOTOR_SCALE_FACTOR[i]*error[i]*delta_time;
            if(I_vel_gain[i] > MAX_OUTPUT_PWM){
                I_vel_gain[i] = MAX_OUTPUT_PWM;
            }
            else if(I_vel_gain[i] < -MAX_OUTPUT_PWM){
                I_vel_gain[i] = -MAX_OUTPUT_PWM;                   
            }
        }

        /* Input for motors */
        pwm[i] = P_vel_gain[i] + I_vel_gain[i];

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
}

//======================================
//              Tilt PID
//======================================
void Claptrap::calculate_tilt_PID(){
    const int MAX_OUTPUT_VEL = 200;
    unsigned long current_time = micros();
    double delta_time = (double)(current_time - PID_tilt_last_calc_time)/1e6;
    double error, ref_vel[2];

    /* Get error */
    Claptrap::read_MPU();
    error = -1*(ref_tilt - angles_filtered[1]); 

    /* PID gains */
    P_tilt_gain = Kp_tilt*error;
    if((P_tilt_gain + I_tilt_gain) < MAX_OUTPUT_VEL && (P_tilt_gain + I_tilt_gain) > -MAX_OUTPUT_VEL)  // Anti-windup
    {
        I_tilt_gain = I_tilt_gain + Ki_tilt*error*delta_time;
        if(I_tilt_gain > MAX_OUTPUT_VEL){
            I_tilt_gain = MAX_OUTPUT_VEL;
        }
        else if(I_tilt_gain < -MAX_OUTPUT_VEL){
            I_tilt_gain = -MAX_OUTPUT_VEL;                   
        }
    }
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
    
    /* Write values into velocity controller */
    Claptrap::set_ref_vel(ref_vel);
    Claptrap::calculate_velocity_PID();
    last_error = error;
    PID_tilt_last_calc_time = current_time;
}


//======================================
//               Setters
//======================================
void Claptrap::set_ref_vel(double* vel){
    ref_vel[0] = vel[0];
    ref_vel[1] = vel[1];
}

void Claptrap::set_ref_tilt(double tilt){
    ref_tilt = tilt;
}

void Claptrap::set_angular_vel(double angular_vel){
    ref_angular_velocity = angular_vel;
}

void Claptrap::set_motor_stop_flag(bool state){
    motor_stop_flag = state;
}
