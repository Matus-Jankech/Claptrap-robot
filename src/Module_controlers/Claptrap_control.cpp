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

//======================================
//            Write to PC
//======================================
void Claptrap::write_serial(char identByte){
    if(Serial.availableForWrite() > 0){
        switch(identByte){
            case 'V':
                Serial.print("v;");
                Serial.print(Kp_vel,3);
                Serial.print(";");
                Serial.print(Ki_vel,3);
                Serial.print(";");
                Serial.println(Kd_vel);
            break;

            case 'T':
                Serial.print("t;");
                Serial.print(Kp_pitch,3);
                Serial.print(";");
                Serial.print(Ki_pitch,3);
                Serial.print(";");
                Serial.println(Kd_pitch,3);
            break;

            case 'D':
                Serial.print("d;");
                Serial.print(ref_wheels_vel[0],3);
                Serial.print(";");
                Serial.print(vel_filtered_1[0],3);
                Serial.print(";");
                Serial.print(vel_filtered_2[0],3);
                Serial.print(";");
                /*Serial.print(pwm[0]);
                Serial.print(";");
                Serial.print(pwm[1]);*/
                Serial.print(";");
                Serial.print(ref_pitch);
                Serial.print(";");
                Serial.println(angles_filtered[1]);
            break;
            
        }
    }
}

//======================================
//           Read from PC
//======================================
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
                sscanf(readBuffer,"%c;%lf;%lf;%lf;",&identByte,&Kp_pitch,&Ki_pitch,&Kd_pitch);
            break;
        }       
    }
}

bool Claptrap::is_standing(void){
    if(angles_filtered[1] < 0.75 && angles_filtered[1] > -0.75){
        return true;
    }
    else{
        return false;
    }
}