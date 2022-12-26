/* Inludes */
#include <Control.h>
#include <RF24.h>

/* Global variables */
uint64_t pipe = 0xE8E8F0F0E1LL;
radio_data_struct radio_control_data;
RF24 radio(CE_PIN,CSN_PIN);
bool radio_status = 0;
unsigned long radio_last_receive_time;

/* Functions */
void radio_init()
{
    radio.begin();
    radio.openReadingPipe(0, pipe);
    radio.setChannel(90);
    radio.setAutoAck(false);
    radio.setDataRate(RF24_250KBPS);
    radio.setPALevel(RF24_PA_LOW);
    radio.startListening();
}

bool is_radio_connected(){
    if (radio.available()){ 
        radio_last_receive_time = millis();
        radio_status = true;
    }
    else if (millis() - radio_last_receive_time > 100) { 
        radio_status = false; 
    } 
    return radio_status;
}

radio_data_struct read_radio(){
    radio.read(&radio_control_data, sizeof(radio_data_struct));
    return radio_control_data;
}