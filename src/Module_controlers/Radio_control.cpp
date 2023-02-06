/* Inludes */
#include <Claptrap.h>

/* Methods definition */
void Claptrap::radio_begin(){
    radio = new RF24(CE_PIN,CSN_PIN);
    radio->begin();
    radio->openReadingPipe(0, pipe);
    radio->setChannel(90);
    radio->setAutoAck(false);
    radio->setDataRate(RF24_250KBPS);
    radio->setPALevel(RF24_PA_LOW);
    radio->startListening();
}

bool Claptrap::is_radio_connected(){
    if (radio->available()){ 
        radio_last_receive_time = millis();
        radio_status = true;
    }
    else if (millis() - radio_last_receive_time > 100) { 
        radio_status = false; 
    } 
    return radio_status;
}

void Claptrap::read_radio(){
    radio->read(&radio_data, sizeof(radio_data_struct));
}

void Claptrap::print_radio(){
    for(int i = 0; i < 8; i++){
        Serial.print(radio_members[i]);
        Serial.print(",");
    }
    Serial.println("");
}