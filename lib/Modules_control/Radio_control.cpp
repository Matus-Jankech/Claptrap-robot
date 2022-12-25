/* Inludes */
#include <Control.h>
#include <RF24.h>

/* Structures */
typedef struct RADIO_DATA_STRUCT {
    byte j1PotX;
    byte j1PotY;
    byte j2PotX;
    byte j2PotY;
} radio_data_struct;

/* Global variables */
uint64_t pipe = 0xE8E8F0F0E1LL;
radio_data_struct radio_data;
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
        radio.read(&radio_data, sizeof(radio_data_struct)); 
        radio_last_receive_time = millis();
        radio_status = true;
    }
    else if (millis() - radio_last_receive_time > 100) { 
        radio_status = false; 
    } 

    return radio_status;
}