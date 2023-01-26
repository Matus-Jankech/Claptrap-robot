/* Includes */
#include <Claptrap.h>

/* Constructor definition */
Claptrap::Claptrap(){}

/* Method definitions */
void Claptrap::begin(){
    motors_begin();
    MP3_begin();
    encoders_begin();
    radio_begin();
    LEDs_begin();
}

void Claptrap::send_serial(char ident){
    if(Serial.availableForWrite()){
        char buffer[50];

        switch(ident){
            case 'V':
                sprintf(buffer,"%c;%lf;%lf;%lf;",'v',Kp_vel,Ki_vel,Kd_vel);
                Serial.write(buffer);
            break;
        }
    }
}

void Claptrap::read_serial(){
    if(Serial.available()){
        char buffer[100];
        char identByte,inByte;
        int i = 0;

        inByte = Serial.read();
        while(inByte != '\n'){
            buffer[i] = inByte;
            inByte = Serial.read();  
            i++;            
        }
        
        identByte = buffer[0];
        switch(identByte){
            case 'V':
                Claptrap::send_serial('V');
            break;

            case 'v':
                sscanf(buffer,"%c;%lf;%lf;%lf;",&identByte,&Kp_vel,&Ki_vel,&Kd_vel);
            break;
        }       
    }
}