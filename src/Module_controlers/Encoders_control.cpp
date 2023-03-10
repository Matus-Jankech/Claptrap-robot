/* Includes */
#include <Claptrap.h>

/* Methods definition */
void Claptrap::encoders_begin(){
    pinMode(Encoder_A1_PIN,INPUT); 
    pinMode(Encoder_B1_PIN,INPUT);
    pinMode(Encoder_A2_PIN,INPUT);
    pinMode(Encoder_B2_PIN,INPUT);
}

void Claptrap::filter_velocity(double* velocity){
    vel_filtered_1[0] = 0.854*vel_filtered_1[0] + 0.0728*(velocity[0]) + 0.0728*vel_filtered_1[1];
    vel_filtered_1[1] = velocity[0];
    vel_filtered_2[0] = 0.854*vel_filtered_2[0] + 0.0728*(velocity[1]) + 0.0728*vel_filtered_2[1];
    vel_filtered_2[1] = velocity[1];
    
    velocity[0] = vel_filtered_1[0];
    velocity[1] = vel_filtered_2[0];
}

void Claptrap::get_velocity(double* velocity){
    unsigned long current_time = micros();
    double delta_time = ((double)(current_time - encoders_last_read_time)/1.0e6);

    velocity[0] = (current_encoder_pos[0] - last_encoder_pos[0])/delta_time;
    velocity[0] = velocity[0]/1020*60; // RPM
    velocity[1] = (current_encoder_pos[1] - last_encoder_pos[1])/delta_time;
    velocity[1] = velocity[1]/1020*60; // RPM
    Claptrap::filter_velocity(velocity);

    encoders_last_read_time = current_time;
    last_encoder_pos[0] = current_encoder_pos[0];
    last_encoder_pos[1] = current_encoder_pos[1];
}

void Claptrap::read_encoder(uint8_t encoder_B_pin, uint8_t index){
    int b = digitalRead(encoder_B_pin);
    if(b > 0){
        current_encoder_pos[index] = current_encoder_pos[index] - 1;
    }
    else{
        current_encoder_pos[index] = current_encoder_pos[index] + 1;
    }
}