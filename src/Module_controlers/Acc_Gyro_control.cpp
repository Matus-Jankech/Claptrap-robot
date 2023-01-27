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

    rollAngle = atan(accY/sqrt(pow(accX,2)+pow(accZ,2)))*180/PI;
    pitchAngle = atan(-accX/sqrt(pow(accY,2)+pow(accZ,2)))*180/PI;

    /*Serial.print("AccX: ");
    Serial.print(accX,3);
    Serial.print("  AccY:");
    Serial.print(accY,3);
    Serial.print("  AccZ:");
    Serial.println(accZ,3);*/

    Serial.print("  roll [deg]:");
    Serial.print(rollAngle,3);
    Serial.print("  pitch [deg]:");
    Serial.println(pitchAngle,3);
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

    rollRate = (double)gyroX/65.5 + 1.91577;
    pitchRate = (double)gyroY/65.5 + 1.02266;
    yawRate = (double)gyroZ/65.5 - 0.51854;

    Serial.print("Roll rate: ");
    Serial.print(rollRate);
    Serial.print("  Pitch rate:");
    Serial.print(pitchRate);
    Serial.print("  Yaw rate:");
    Serial.println(yawRate);
}

void Claptrap::calibrate_gyro(){
    double calibRateRoll = 0, calibRatePitch = 0, calibRateYaw = 0;
    
    for(int i = 0; i < 2000; i++){
       read_gyro(); 
       calibRateRoll += rollRate;
       calibRatePitch += pitchRate;
       calibRateYaw += yawRate;
       delay(1);
    }

    Serial.print("Roll rate calib: ");
    Serial.print(calibRateRoll/2000,5);
    Serial.print("  Pitch rate calib:");
    Serial.print(calibRatePitch/2000,5);
    Serial.print("  Yaw rate calib:");
    Serial.println(calibRateYaw/2000,5);
}