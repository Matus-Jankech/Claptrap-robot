/* Includes */
#include <Claptrap.h>

void Claptrap::MPU_begin(){
    /* I2C communication setup */
    Wire.setClock(400000);
    Wire.begin();
    delay(250);
    
    /* MPU setup */
    Wire.beginTransmission(0x68);
    Wire.write(0x6B); // Power register
    Wire.write(0x00); // Power on
     Wire.endTransmission();
 
    /* Acc and Gyro setup */
    Wire.beginTransmission(0x68); 
    Wire.write(0x1A); // Low pass filter register 
    Wire.write(0x5); // 5Hz
    Wire.endTransmission();

    Wire.beginTransmission(0x68);
    Wire.write(0x1C); // Acc scalling register
    Wire.write(0x10); // +-8g
    Wire.endTransmission();

    Wire.beginTransmission(0x68);
    Wire.write(0x1B); // Gyro scalling register
    Wire.write(0x8); // +-500 deg/s 
    Wire.endTransmission();

    Claptrap::inicialize_MPU_values();
}

void Claptrap::inicialize_MPU_values(){
    Claptrap::read_acc();
    gyro_angles[0] = acc_angles[0];
    gyro_angles[1] = acc_angles[1];
    kalman_angles[0] = acc_angles[0];
    kalman_angles[1] = acc_angles[1];
    kalman_uncertainty_angles[0] = 2*2;
    kalman_uncertainty_angles[1] = 2*2;
}

void Claptrap::read_MPU(){
    Claptrap::read_acc();
    Claptrap::read_gyro();
    Claptrap::kalman_filter();

    Serial.print(acc_angles[1],3);
    Serial.print(" , ");
    Serial.println(gyro_angles[1],3);
}

void Claptrap::read_acc(void){
    int16_t accXLSB, accYLSB, accZLSB;
    double accX, accY, accZ;

    Wire.beginTransmission(0x68);
    Wire.write(0x3B); // acc X,Y,Z register
    Wire.endTransmission();
    Wire.requestFrom(0x68,6);

    accXLSB = Wire.read()<<8 | Wire.read();
    accYLSB = Wire.read()<<8 | Wire.read();
    accZLSB = Wire.read()<<8 | Wire.read();

    accX = (double)accXLSB/4096 - 0.05;
    accY = (double)accYLSB/4096 + 0.0;
    accZ = (double)accZLSB/4096 + 0.03;

    acc_angles[0] = atan(accY/sqrt(pow(accX,2)+pow(accZ,2)))*180/PI;
    acc_angles[1] = atan(-accX/sqrt(pow(accY,2)+pow(accZ,2)))*180/PI;
}

void Claptrap::read_gyro(){
    int16_t gyroX,gyroY,gyroZ;
    
    Wire.beginTransmission(0x68);
    Wire.write(0x43); // gyro X,Y,Z register
    Wire.endTransmission();
    Wire.requestFrom(0x68,6);

    gyroX = Wire.read()<<8 | Wire.read();
    gyroY = Wire.read()<<8 | Wire.read();
    gyroZ = Wire.read()<<8 | Wire.read();

    gyro_rates[0] = (double)gyroX/65.5 + 1.91577;
    gyro_rates[1] = (double)gyroY/65.5 + 1.02266;
    gyro_rates[2] = (double)gyroZ/65.5 - 0.51854;
    Claptrap::integrate_gyro();
}

void Claptrap::integrate_gyro(void){
    unsigned long current_time = millis();
    float delta_time = (float)(current_time-last_gyro_read_time)/1000.0;

    gyro_angles[0] += gyro_rates[0]*delta_time;
    gyro_angles[1] += gyro_rates[1]*delta_time;

    last_gyro_read_time = current_time;
}

void Claptrap::kalman_filter(void){
    
}

void Claptrap::calibrate_gyro(){
    double calibRateRoll = 0, calibRatePitch = 0, calibRateYaw = 0;
    
    for(int i = 0; i < 2000; i++){
       Claptrap::read_gyro(); 
       calibRateRoll += gyro_rates[0];
       calibRatePitch += gyro_rates[1];
       calibRateYaw += gyro_rates[2];
       delay(1);
    }

    Serial.print("Roll rate calib: ");
    Serial.print(calibRateRoll/2000,5);
    Serial.print("  Pitch rate calib:");
    Serial.print(calibRatePitch/2000,5);
    Serial.print("  Yaw rate calib:");
    Serial.println(calibRateYaw/2000,5);
}