/* Includes */
#include <Control.h>

/* Global variables */
uint64_t current_encoder_pos[2];
uint64_t last_encoder_pos[2];
unsigned long encoders_last_read_time;
double vel_filtered_1[2];
double vel_filtered_2[2];

/* Functions */
void read_encoder_1(){
    int b = digitalRead(Encoder_A1_PIN);

    if(b > 0){
        current_encoder_pos[0] = current_encoder_pos[0] + 1;
    }
    else{
        current_encoder_pos[0] = current_encoder_pos[0] - 1;
    }
}

void read_encoder_2(){
    int b = digitalRead(Encoder_A2_PIN);

    if(b > 0){
        current_encoder_pos[1] = current_encoder_pos[1] + 1;
    }
    else{
        current_encoder_pos[1] = current_encoder_pos[1] - 1;
    }
}

void filter_velocity(double* velocity_1,double* velocity_2){
    vel_filtered_1[0] = 0.854*vel_filtered_1[0] + 0.0728*(*velocity_1) + 0.0728*vel_filtered_1[1];
    vel_filtered_1[1] = *velocity_1;
    vel_filtered_2[0] = 0.854*vel_filtered_2[0] + 0.0728*(*velocity_2) + 0.0728*vel_filtered_2[1];
    vel_filtered_2[1] = *velocity_2;

    *velocity_1 = vel_filtered_1[0];
    *velocity_2 = vel_filtered_2[0];
}

void get_velocity(double* velocity_1, double* velocity_2){
    double delta_time = ((double)(micros() - encoders_last_read_time)/1.0e6);
    
    *velocity_1 = (current_encoder_pos[0] - last_encoder_pos[0])/delta_time;
    *velocity_1 = *velocity_1/1020*60; // RPM
    *velocity_2 = (current_encoder_pos[1] - last_encoder_pos[1])/delta_time;
    *velocity_2 = *velocity_2/1020*60; // RPM
    filter_velocity(velocity_1,velocity_2);

    encoders_last_read_time = micros();
    last_encoder_pos[0] = current_encoder_pos[0];
    last_encoder_pos[1] = current_encoder_pos[1];
}