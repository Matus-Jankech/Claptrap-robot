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

void Claptrap::print_radio(){
    for(int i = 0; i < 12; i++){
        Serial.print(local_radio_member[i]);
        Serial.print(" , ");
    }
    Serial.println("");
}

void Claptrap::read_radio(){
    radio->read(&radio_data, sizeof(radio_data_struct));
    for(int i = 0; i < 12; i++){
        radio_current_input_values[i] = radio_members[i];
    }
    Claptrap::input_filter();
}

void Claptrap::input_filter(){
    unsigned long current_time = micros();
    double delta_time = (current_time - radio_last_integration_time)/1e6;
    const float K[2] = {5,10};

    for(int i = 0; i < 8; i++){
        local_radio_member[i] = radio_current_input_values[i];
    }
    // Code for joystick
    /*radio_input_integrators[0] = radio_input_integrators[0] + K*(radio_current_input_values[8] - radio_input_integrators[0])*delta_time;
    radio_input_integrators[1] = radio_input_integrators[1] + K*(radio_current_input_values[10] - radio_input_integrators[1])*delta_time;*/


    radio_input_integrators[0] = radio_input_integrators[0] - K[0]*(2*radio_current_input_values[4] + radio_input_integrators[0])*delta_time;
    radio_input_integrators[1] = radio_input_integrators[1] + K[0]*(2*radio_current_input_values[5] - radio_input_integrators[1])*delta_time;
    radio_input_integrators[2] = radio_input_integrators[2] - K[1]*(6*radio_current_input_values[6] + radio_input_integrators[2])*delta_time;
    radio_input_integrators[3] = radio_input_integrators[3] + K[1]*(6*radio_current_input_values[7] - radio_input_integrators[3])*delta_time;

    local_radio_member[8] = 0;
    local_radio_member[9] = radio_input_integrators[0] + radio_input_integrators[1];
    local_radio_member[10] = radio_input_integrators[2] + radio_input_integrators[3];
    local_radio_member[11] = 0;

    radio_last_integration_time = current_time;
}