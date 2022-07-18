#include <Wire.h>

#define BAUD 115200
#define _addr 0x6A

/* Register definitions */
#define CTRL1_XL  0x10  // Accelerometer control register 1 (r/w)
#define CTRL2_G   0x11  // Gyroscope control register 2     (r/w)
#define WHO_AM_I  0x0f  //                                  (r)

#define OUT_TEMP_L 0x20 // Temperature data output register (r)

#define OUTX_L_G   0x22 // Angular rate sensor pitch axis (X) angular rate output register (r)
#define OUTY_L_G   0x24 // Angular rate sensor roll axis  (Y) angular rate output register (r)
#define OUTZ_L_G   0x26 // Angular rate sensor yaw axis   (Z) angular rate output register (r)

#define OUTX_L_A   0x28 // Linear acceleration sensor X-axis output register (r)
#define OUTY_L_A   0x2A // Linear acceleration sensor Y-axis output register (r)
#define OUTZ_L_A   0x2C // Linear acceleration sensor Z-axis output register (r)

void i2c_write(byte *buffer, uint8_t len){
  Wire.beginTransmission(_addr);
  Wire.write(buffer,len);
  Wire.endTransmission();
}

void i2c_write(byte b){
  Wire.beginTransmission(_addr);
  Wire.write(b);
  Wire.endTransmission();
}

bool i2c_read(byte* buffer, byte _register, uint8_t number_requested){
  Wire.beginTransmission(_addr);
  Wire.write(_register);
  Wire.endTransmission();
  
  bool a =  Wire.requestFrom(_addr, number_requested);
  for(int i=0; i<number_requested; i++){
    buffer[i] = Wire.read();
  }
  
  return a;
}

bool i2c_read(byte* buffer, byte _register){
  Wire.beginTransmission(_addr);
  Wire.write(_register);
  Wire.endTransmission();
  
  bool a = Wire.requestFrom(_addr, 1);
  *buffer = Wire.read();
  
  return a;
}

void setup() {
  Serial.begin(BAUD); 
  Wire  .begin();        // Start I2C library      

  _init();
}

void _init(){
  byte buffer[2];
  byte check_byte[1];

  // Enable Accelerometer with 12.5 Hz ODR
  buffer[0] = CTRL1_XL;
  buffer[1] = 0b10110000;
  i2c_write(buffer,2);
  
  // Enable GyroScope with 12.5 Hz ODR
  buffer[0] = CTRL2_G;
  buffer[1] = 0b00010010;
  i2c_write(buffer,2);
  
  i2c_read(check_byte,CTRL1_XL);
  Serial.print("CTRL1_XL: ");
  Serial.println(*check_byte);
  
  i2c_read(check_byte,CTRL2_G);
  Serial.print("CTRL2_G: ");
  Serial.println(*check_byte);
  
  delay(2000);
}


void loop() {
  byte a[1];
  i2c_read(a,OUTX_L_A);
  Serial.println(a[0]);
  
  
  delay(1000);
}
