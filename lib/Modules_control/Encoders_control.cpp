/* Includes */
#include <Control.h>

/* Global variables */
uint64_t current_encoder_pos[2];
uint64_t last_encoder_pos[2];

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