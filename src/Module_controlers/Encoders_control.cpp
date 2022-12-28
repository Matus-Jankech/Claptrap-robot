/* Includes */
#include <Control.h>

/* Global variables */
volatile long current_encoder_pos[2];
volatile long last_encoder_pos[2];
unsigned long encoders_last_read_time;
double vel_filtered_1[2];
double vel_filtered_2[2];

/* Functions */
void encoders_init(){
    /* Set pinmodes */
    pinMode(Encoder_A1_PIN,INPUT); 
    pinMode(Encoder_B1_PIN,INPUT);
    pinMode(Encoder_A2_PIN,INPUT);
    pinMode(Encoder_B2_PIN,INPUT);

    /* Attach hardware interrupts */
    attachInterrupt(digitalPinToInterrupt(Encoder_A1_PIN), read_encoder_1, RISING);
    attachInterrupt(digitalPinToInterrupt(Encoder_A2_PIN), read_encoder_2, RISING);
}

void read_encoder_1(){
    int b = digitalRead(Encoder_B1_PIN);
    if(b > 0){
        current_encoder_pos[0] = current_encoder_pos[0] - 1;
    }
    else{
        current_encoder_pos[0] = current_encoder_pos[0] + 1;
    }
}

void read_encoder_2(){
    int b = digitalRead(Encoder_B2_PIN);
    if(b > 0){
        current_encoder_pos[1] = current_encoder_pos[1] - 1;
    }
    else{
        current_encoder_pos[1] = current_encoder_pos[1] + 1;
    }
}

void filter_velocity(double* velocity){
    vel_filtered_1[0] = 0.854*vel_filtered_1[0] + 0.0728*(velocity[0]) + 0.0728*vel_filtered_1[1];
    vel_filtered_1[1] = velocity[0];
    vel_filtered_2[0] = 0.854*vel_filtered_2[0] + 0.0728*(velocity[1]) + 0.0728*vel_filtered_2[1];
    vel_filtered_2[1] = velocity[1];

    velocity[0] = vel_filtered_1[0];
    velocity[1] = vel_filtered_2[0];
}

void get_velocity(double* velocity){
    double delta_time = ((double)(micros() - encoders_last_read_time)/1.0e6);

    velocity[0] = (current_encoder_pos[0] - last_encoder_pos[0])/delta_time;
    velocity[0] = velocity[0]/1020*60; // RPM
    velocity[1] = (current_encoder_pos[1] - last_encoder_pos[1])/delta_time;
    velocity[1] = velocity[1]/1020*60; // RPM
    filter_velocity(velocity);

    encoders_last_read_time = micros();
    last_encoder_pos[0] = current_encoder_pos[0];
    last_encoder_pos[1] = current_encoder_pos[1];
}