/* Inludes */
#include <Control.h>

/* Global variables */
uint64_t pipe = 0xE8E8F0F0E1LL;
RF24 radio(CE_PIN,CSN_PIN);
bool radio_status = 0;
unsigned long radio_last_receive_time;

/* Methods definition */
void Claptrap::radio_init(){
    radio.begin();
    radio.openReadingPipe(0, pipe);
    radio.setChannel(90);
    radio.setAutoAck(false);
    radio.setDataRate(RF24_250KBPS);
    radio.setPALevel(RF24_PA_LOW);
    radio.startListening();
}

bool Claptrap::is_radio_connected(){
    if (radio.available()){ 
        radio_last_receive_time = millis();
        radio_status = true;
    }
    else if (millis() - radio_last_receive_time > 100) { 
        radio_status = false; 
    } 
    return radio_status;
}

void Claptrap::read_radio(){
    radio.read(&radio_data, sizeof(radio_data_struct));
}