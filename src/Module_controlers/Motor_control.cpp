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
    I_pitch_gain = 0;
    I_wheels_vel_gain[0] = 0;
    I_wheels_vel_gain[1] = 0;
    PID_wheels_vel_last_calc_time = micros();
    PID_pitch_last_calc_time = micros();
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
//           Wheels velocity PID
//======================================
void Claptrap::calculate_wheels_velocity_PID(){
    const float MOTOR_SCALE_FACTOR[2] = {1.25,1.0};
    const int NUM_OF_MOTORS = 2;
    const int MAX_OUTPUT_PWM = 255;
    unsigned long current_time = micros();
    double delta_time = (double)(current_time - PID_wheels_vel_last_calc_time)/1e6;
    double current_wheels_vel[2], angular_vel[2], error[2];
    int16_t output_pwm[2];
    
    Claptrap::get_velocity(current_wheels_vel);
    angular_vel[0] = ref_angular_vel;
    angular_vel[1] = -ref_angular_vel;

    for(int i = 0; i < NUM_OF_MOTORS; i++){
        error[i] = ref_wheels_vel[i] + angular_vel[i] - current_wheels_vel[i]; 

        /* PI gains */
        P_wheels_vel_gain[i] = Kp_vel*MOTOR_SCALE_FACTOR[i]*error[i];
        if((P_wheels_vel_gain[i] + I_wheels_vel_gain[i]) < MAX_OUTPUT_PWM && (P_wheels_vel_gain[i] + I_wheels_vel_gain[i]) > -MAX_OUTPUT_PWM) // Anti-windup
        { 
            I_wheels_vel_gain[i] = I_wheels_vel_gain[i] + Ki_vel*MOTOR_SCALE_FACTOR[i]*error[i]*delta_time;
            if(I_wheels_vel_gain[i] > MAX_OUTPUT_PWM){
                I_wheels_vel_gain[i] = MAX_OUTPUT_PWM;
            }
            else if(I_wheels_vel_gain[i] < -MAX_OUTPUT_PWM){
                I_wheels_vel_gain[i] = -MAX_OUTPUT_PWM;                   
            }
        }

        /* Input for motors */
        output_pwm[i] = P_wheels_vel_gain[i] + I_wheels_vel_gain[i];

        /* Saturation of inputs for motors*/
        if(output_pwm[i] > MAX_OUTPUT_PWM){
            output_pwm[i] = MAX_OUTPUT_PWM;
        } 
        else if(output_pwm[i] < -MAX_OUTPUT_PWM){
            output_pwm[i] = -MAX_OUTPUT_PWM;   
        }
    }

    /* Write values into motors */
    Claptrap::set_motor_pwm(output_pwm[0],M1_front_PIN,M1_back_PIN);
    Claptrap::set_motor_pwm(output_pwm[1],M2_front_PIN,M2_back_PIN);
    PID_wheels_vel_last_calc_time = current_time;
}

//======================================
//              Pitch PID
//======================================
void Claptrap::calculate_pitch_PID(){
    const int MAX_OUTPUT_WHEELS_VEL = 200;
    unsigned long current_time = micros();
    double delta_time = (double)(current_time - PID_pitch_last_calc_time)/1e6;
    double error, output_wheels_vel[2];

    /* Get error */
    Claptrap::read_MPU();
    error = -1*(ref_pitch - angles_filtered[1]); 

    /* PID gains */
    P_pitch_gain = Kp_pitch*error;
    if((P_pitch_gain + I_pitch_gain) < MAX_OUTPUT_WHEELS_VEL && (P_pitch_gain + I_pitch_gain) > -MAX_OUTPUT_WHEELS_VEL)  // Anti-windup
    {
        I_pitch_gain = I_pitch_gain + Ki_pitch*error*delta_time;
        if(I_pitch_gain > MAX_OUTPUT_WHEELS_VEL){
            I_pitch_gain = MAX_OUTPUT_WHEELS_VEL;
        }
        else if(I_pitch_gain < -MAX_OUTPUT_WHEELS_VEL){
            I_pitch_gain = -MAX_OUTPUT_WHEELS_VEL;                   
        }
    }
    D_pitch_gain = Kd_pitch*(error - pitch_last_error)*delta_time;

    /* Input for wheels velocity controller */
    output_wheels_vel[0] = P_pitch_gain + I_pitch_gain + D_pitch_gain;

    /* Saturation of output_wheels_vel*/
    if(output_wheels_vel[0] > MAX_OUTPUT_WHEELS_VEL){
        output_wheels_vel[0] = MAX_OUTPUT_WHEELS_VEL;
    } 
    else if(output_wheels_vel[0] < -MAX_OUTPUT_WHEELS_VEL){
        output_wheels_vel[0] = -MAX_OUTPUT_WHEELS_VEL;   
    }
    output_wheels_vel[1] = output_wheels_vel[0];
    
    /* Write values into wheels velocity controller */
    Claptrap::set_wheels_ref_vel(output_wheels_vel);
    Claptrap::calculate_wheels_velocity_PID();
    pitch_last_error = error;
    PID_pitch_last_calc_time = current_time;
}

//======================================
//           Robot velocity PID
//======================================
void Claptrap::calculate_robot_velocity_PID(){
    const float MAX_OUTPUT_PITCH = 3;
    const float ROT_LIN_RATIO = 0.5; // revision needed !!!
    unsigned long current_time = micros();
    double delta_time = (double)(current_time - PID_robot_vel_last_calc_time)/1e6;
    double error, output_pitch, current_lin_vel, current_wheels_vel[2];

    /* Get lin vel and error */
    get_velocity(current_wheels_vel);
    current_lin_vel = (current_wheels_vel[0] - ref_angular_vel)*ROT_LIN_RATIO; // revision needed !!!
    error = ref_linear_vel - current_lin_vel; 

    /* PID gains */
    P_robot_vel_gain = Kp_robot_vel*error;
    if((P_robot_vel_gain + I_robot_vel_gain) < MAX_OUTPUT_PITCH && (P_robot_vel_gain + I_robot_vel_gain) > -MAX_OUTPUT_PITCH)  // Anti-windup
    {
        I_robot_vel_gain = I_robot_vel_gain + Ki_robot_vel*error*delta_time;
        if(I_robot_vel_gain > MAX_OUTPUT_PITCH){
            I_robot_vel_gain = MAX_OUTPUT_PITCH;
        }
        else if(I_robot_vel_gain < -MAX_OUTPUT_PITCH){
            I_robot_vel_gain = -MAX_OUTPUT_PITCH;                   
        }
    }
    D_robot_vel_gain = Kd_robot_vel*(error - robot_vel_last_error)*delta_time;

    /* Input for pitch controller */
    output_pitch = P_robot_vel_gain + I_robot_vel_gain + D_robot_vel_gain;

    /* Saturation of output_pitch*/
    if(output_pitch > MAX_OUTPUT_PITCH){
        output_pitch = MAX_OUTPUT_PITCH;
    } 
    else if(output_pitch < -MAX_OUTPUT_PITCH){
        output_pitch = -MAX_OUTPUT_PITCH;   
    }

    /* Write values into pitch controller */
    Claptrap::set_ref_pitch(output_pitch);
    Claptrap::calculate_pitch_PID();
    robot_vel_last_error = error;
    PID_robot_vel_last_calc_time = current_time;
}

//======================================
//               Setters
//======================================
void Claptrap::set_wheels_ref_vel(double* wheels_vel){
    ref_wheels_vel[0] = wheels_vel[0];
    ref_wheels_vel[1] = wheels_vel[1];
}

void Claptrap::set_ref_pitch(double pitch){
    ref_pitch = pitch;
}

void Claptrap::set_angular_vel(double angular_vel){
    ref_angular_vel = angular_vel;
}

void Claptrap::set_motor_stop_flag(bool state){
    motor_stop_flag = state;
}
