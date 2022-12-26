#include "MPU9250.h"

MPU9250 mpu; // You can also use MPU9255 as is

float accX, accY, accZ;
float gyroX,gyroY,gyroZ;
float magX,magY,magZ;

float A_Pitch, A_Roll; 
float Pitch_old=0, Roll_old=0, Yaw_old=0; 
float Pitch_new, Roll_new, Yaw_new; 

float G_Pitch=0, G_Roll=0;
float Pitch, Roll, Yaw;
float dt;
unsigned long millisOld;

float Pitch_Rad,Roll_Rad;
float MagX, MagY, MagZ;
float Yaw_Comp; 

float first_coefficient = 0 ,scend_coefficient = 0 ,therd_coefficient = 0 ,Mxc = 0 ,Myc = 0 ,Mzc = 0 ;
float Mag_X, Mag_Y;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    //    delay(5000);
    mpu.setup(0x68);  
    mpu.calibrateAccelGyro();
    delay(2000);
    }


void loop() { 
 if (mpu.update()) {
         accX = mpu.getAccX();
         accY = mpu.getAccY();
         accZ = mpu.getAccZ();
         gyroX = mpu.getGyroX();
         gyroY = mpu.getGyroY();
         gyroZ = mpu.getGyroZ();
         magX = mpu.getMagX() ;
         magY = mpu.getMagY();
         magZ = mpu.getMagZ();
         
         /**
         //Mxc = 22(magX-295.870903) + 22(magY-597.917591) + 22(magZ-21.052387)
        
         magX = ( magX - 307.401914 ) ;
         magY = ( magY - 602.792045 ) ;
         magZ = ( magZ - 36.700158 ) ;
         /**/  

         /**/
         magX = 0.127677 * ( magX - 315.10636 ) ;
         magY = 0.119032 * ( magY - 589.102394 ) ;
         magZ = 0.1526 * ( magZ - 83.054573 ) ;
         
         //Mxc = 22(magX-295.870903) + 22(magY-597.917591) + 22(magZ-21.052387)
         
         first_coefficient = ( magX - -2.982943 );
         scend_coefficient = ( magY - -15.606394 );
         therd_coefficient = ( magZ - -0.026628 );
         
         Mxc = 0.923807 * first_coefficient + -0.002876 * scend_coefficient + -0.004330 * therd_coefficient ;
         Myc = -0.091560 * first_coefficient + 1.200741 * scend_coefficient + -0.043406 * therd_coefficient ;
         Mzc = -0.032603 * first_coefficient + -0.043406 * scend_coefficient + 0.939092 * therd_coefficient ;
        
         magX = Mxc ;
         magY = Myc ;
         magZ = Mzc ;
         /**/  

A_Roll = atan2(accY, sqrt( accX * accX + accZ * accZ) )*RAD_TO_DEG; //sqrt( accX * accX + accZ * accZ)
A_Pitch = atan2(-1*accX, sqrt( accY * accY + accZ * accZ) )*RAD_TO_DEG; //sqrt( accY * accY + accZ * accZ)
  
Pitch_new = 0.99 * Pitch_old + 0.01 * A_Pitch; 
Roll_new = 0.99 * Roll_old + 0.01 * A_Roll;
  
dt = (millis()-millisOld)/1000.;
millisOld = millis();

Pitch = (Pitch + gyroY * dt)*.9 + A_Pitch * .1;
Roll = (Roll + gyroX * dt)*.9 + A_Roll * .1;

//G_Pitch = G_Pitch + gyroY * dt;
//G_Roll = G_Roll - gyroX * dt;

Roll_Rad = Roll/RAD_TO_DEG;
Pitch_Rad = Pitch/RAD_TO_DEG;

Mag_X = magX * cos(Pitch_Rad) +  magZ * sin(Pitch_Rad);
Mag_Y = magX*sin(Roll_Rad)*sin(Pitch_Rad) - magY*cos(Roll_Rad) +  magZ*sin(Roll_Rad)*cos(Pitch_Rad) ;
Yaw = atan2(Mag_Y,abs(Mag_X))* RAD_TO_DEG;
          
//Mag_X = magX * cos(Pitch/RAD_TO_DEG) + magY * sin(Roll/RAD_TO_DEG) *sin(Pitch/RAD_TO_DEG) +  magZ * sin(Pitch/RAD_TO_DEG) * cos(Roll/RAD_TO_DEG);
//Mag_Y = magY * cos(Roll/RAD_TO_DEG) - magZ * sin(Roll/RAD_TO_DEG) + magX * sin(Roll/RAD_TO_DEG);
//Yaw = atan2( Mag_Y ,abs(Mag_X) )* RAD_TO_DEG;
//Yaw = atan2(magY,abs(magX) )* RAD_TO_DEG;

//Yaw = atan2( Mag_Y,abs(Mag_X) ) * RAD_TO_DEG;

//if (Yaw > 90)
//Yaw = 90 - (Yaw - 90);
//if (Yaw < -90)
//Yaw = -1*(Yaw + 90);

Yaw = 0.95 * Yaw_old + 0.05 * Yaw; 
Yaw_Comp = (Yaw - gyroZ * dt)*.9 + Yaw * .1;
   

Serial.print(Roll);
Serial.print(",");
Serial.print(Pitch);
Serial.print(",");
Serial.print(Yaw);
Serial.print(",");
Serial.println(Yaw_Comp);


 Pitch_old = Pitch_new; 
 Roll_old = Roll_new; 
 Yaw_old = Yaw; 
 delay(10);
  
 }
 }
