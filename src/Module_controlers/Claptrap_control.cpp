/* Includes */
#include <Claptrap.h>

/* Constructor definition */
Claptrap::Claptrap(){}

/* Method definitions */
void Claptrap::begin(){
    motors_begin();
    MPU_begin();
    MP3_begin();
    encoders_begin();
    radio_begin();
    LEDs_begin();
    Serial.begin(115200);
}

void Claptrap::write_serial(char identByte){
    if(Serial.availableForWrite() > 0){
        //char writeBuffer[50];

        switch(identByte){
            case 'V':
                /*sprintf(writeBuffer,"%c;%lf;%lf;%lf",'v',Kp_vel,Ki_vel,Kd_vel);
                Serial.println(writeBuffer);*/
                Serial.print("v;");
                Serial.print(Kp_vel,3);
                Serial.print(";");
                Serial.print(Ki_vel,3);
                Serial.print(";");
                Serial.println(Kd_vel);
            break;

            case 'T':
                /*sprintf(writeBuffer,"%c;%lf;%lf;%lf",'t',Kp_tilt,Ki_tilt,Kd_tilt);
                Serial.println(writeBuffer);*/
                Serial.print("t;");
                Serial.print(Kp_tilt,3);
                Serial.print(";");
                Serial.print(Ki_tilt,3);
                Serial.print(";");
                Serial.println(Kd_tilt,3);
            break;

            case 'S':
                /*sprintf(writeBuffer,"%c;%0.3lf;%0.3lf;%0.3f",'s',ref_vel[0],vel_filtered_1[0],(float)millis()/1000.0);
                Serial.println(writeBuffer);*/
                Serial.print("s;");
                Serial.print(ref_vel[0],3);
                Serial.print(";");
                Serial.print(vel_filtered_1[0],3);
                Serial.print(";");
                Serial.println(millis()/1000.0,3);
            break;
        }
    }
}

void Claptrap::read_serial(){
    if(Serial.available() > 0){
        char readBuffer[50];
        char identByte,inByte;
        int i = 0;

        inByte = Serial.read();
        while(inByte != '\n'){
            readBuffer[i] = inByte;
            inByte = Serial.read();  
            i++;            
        }
        
        identByte = readBuffer[0];
        switch(identByte){
            case 'V':
                write_serial('V');
            break;

            case 'v':
                sscanf(readBuffer,"%c;%lf;%lf;%lf;",&identByte,&Kp_vel,&Ki_vel,&Kd_vel);
            break;

            case 'T':
                write_serial('T');
            break;

            case 't':
                sscanf(readBuffer,"%c;%lf;%lf;%lf;",&identByte,&Kp_tilt,&Ki_tilt,&Kd_tilt);
            break;
        }       
    }
}