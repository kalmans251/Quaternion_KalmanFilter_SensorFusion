#include <Wire.h>

const int MPU_ADDR = 0x68; 
const int MAG_ADDR = 0x0C; 

unsigned int mag_xL, mag_xH, mag_yL, mag_yH, mag_zH;
unsigned int mag_zL;
int16_t mag_x, mag_y, mag_z;


int8_t Msens_x,Msens_y, Msens_z;

int8_t status_1;
float asax, asay, asaz;
float magSin,magCos;

void setup() {

  Serial.begin(28800);
  Wire.begin();

  // Acc & Gyro Registers********************************************
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6A);   // USER CONTROL  
  Wire.write(0x00);   // 0x00 is reset value
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);   
  Wire.write(0x37);   //  IMU INT PIN CONFIG    
  Wire.write(0x02);   //  0x02 activate bypass in order to communicate with magnetometer
  Wire.endTransmission(true);
  delay(200);

  // Magnetometer Registers*****************************************
  Wire.beginTransmission(MAG_ADDR); 
  Wire.write(0x0B);   //  CONTROL 2
  Wire.write(0b01);   //  0 NORMAL OR 1 RESET
  Wire.endTransmission(true);
  delay(200);

  Wire.beginTransmission(MAG_ADDR);   //SLEEP MODE
  Wire.write(0x0A);   //  CONTROL 1
  Wire.write(0b00010000);   // 1 for 16 bit or 0 for 14 bit output, 0000 SLEEP MODE
  Wire.endTransmission(true);
  delay(200);

  Wire.beginTransmission(MAG_ADDR);   //ROM WRITE MODE
  Wire.write(0x0A);   //  CONTROL 1
  Wire.write(0b00011111); // 1 for 16 bit or 0 for 14 bit output, 1111 FUSE ROM ACCESS MODE
  Wire.endTransmission(true);
  delay(200);

  Wire.beginTransmission(MAG_ADDR);   //GET MAGNETIC SENSITIVITY DATA FOR CONVERTING RAW DATA
  Wire.write(0x10);     //  ASAX  
  Wire.endTransmission(false);
  Wire.requestFrom(MAG_ADDR, 3 , true);  //GET SENSITIVITY ADJUSMENT VALUES STARTS AT ASAX
  Msens_x = Wire.read();    //GET X SENSITIVITY ADJUSMENT VALUE
  Msens_y = Wire.read();    //GET Y SENSITIVITY ADJUSMENT VALUE
  Msens_z = Wire.read();    //GET Z SENSITIVITY ADJUSMENT VALUE
  Wire.endTransmission(true);
  asax = (((Msens_x-128))/256.0f)+1.0f;
  asay = (((Msens_y-128))/256.0f)+1.0f;
  asaz = (((Msens_z-128))/256.0f)+1.0f;
  delay(200);
  
  Wire.beginTransmission(MAG_ADDR);   //SLEEP MODE
  Wire.write(0x0A);   //  CONTROL 1
  Wire.write(0b00010000);  // 1 for 16 bit or 0 for 14 bit output, 0000 SLEEP MODE
  Wire.endTransmission(true);
  delay(200);
    
  Wire.beginTransmission(MAG_ADDR);   //CONT MODE 2
  Wire.write(0x0A);
  Wire.write(0b00010110); // 1 for 16 bit or 0 for 14 bit output, 0110 FOR CONT MODE 2 (X Hz?) 
  Wire.endTransmission(true);
  delay(200);

}

void loop() {
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x02);
  Wire.endTransmission(false);
  Wire.requestFrom(MAG_ADDR, 1 , true);   
  status_1 = Wire.read();
  Wire.endTransmission(true);

  if(status_1 == 0b11) {
    Wire.beginTransmission(MAG_ADDR);
    Wire.write(0x03);
    Wire.endTransmission(false);
    Wire.requestFrom(MAG_ADDR, 7 , true);
   
    mag_yL = Wire.read();
    mag_yH = Wire.read();
    mag_y = (mag_yH << 8) | mag_yL ;   

    mag_xL = Wire.read();
    mag_xH = Wire.read();
    mag_x = (mag_xH << 8) | mag_xL ;   

   
    mag_zL = Wire.read();
    mag_zH = Wire.read();
    mag_z = -((mag_zH << 8) | mag_zL );   
  
    
    Wire.endTransmission(true);
    mag_x=100*(mag_x*asax*0.15-4.5);
    mag_y=100*(mag_y*asay*0.15-4.5);
    mag_z=100*(mag_z*asaz*0.15+5.5);
    // mag_x=(mag_x-4.5/(asax*0.15));
    // mag_y=(mag_y-4.5/(asay*0.15));
    // mag_z=(mag_z+5.5/(asaz*0.15));
    // Serial.print(mag_x);
    // Serial.print(" , ");
    // Serial.print(mag_y);
    // Serial.print(" , ");
    // Serial.println(mag_z);
  }

  if(mag_x<0 && mag_y<0){
    magSin=-(sqrt((sqrt(pow(mag_x,2)+pow(mag_y,2))+mag_y)/sqrt(pow(mag_x,2)+pow(mag_y,2)))+sqrt((sqrt(pow(mag_x,2)+pow(mag_y,2))-mag_y)/sqrt(pow(mag_x,2)+pow(mag_y,2))))/2.;
  }else if(mag_x<0){
    magSin=(sqrt((sqrt(pow(mag_x,2)+pow(mag_y,2))+mag_y)/sqrt(pow(mag_x,2)+pow(mag_y,2)))+sqrt((sqrt(pow(mag_x,2)+pow(mag_y,2))-mag_y)/sqrt(pow(mag_x,2)+pow(mag_y,2))))/2.;
  }else{
    magSin=(sqrt((sqrt(pow(mag_x,2)+pow(mag_y,2))+mag_y)/sqrt(pow(mag_x,2)+pow(mag_y,2)))-sqrt((sqrt(pow(mag_x,2)+pow(mag_y,2))-mag_y)/sqrt(pow(mag_x,2)+pow(mag_y,2))))/2.;
  }
  
  magCos=sqrt(1-pow(magSin,2));

}
