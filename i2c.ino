void i2c_write(byte *buffer, uint8_t len){
  Wire.beginTransmission(ADDR);
  Wire.write(buffer,len);
  Wire.endTransmission();
}

void i2c_write(byte b){
  Wire.beginTransmission(ADDR);
  Wire.write(b);
  Wire.endTransmission();
}

bool i2c_read(byte* buffer, byte _register, uint8_t number_requested){
  Wire.beginTransmission(ADDR);
  Wire.write(_register);
  Wire.endTransmission();
  
  bool a =  Wire.requestFrom(ADDR, number_requested);
  for(int i=0; i<number_requested; i++){
    buffer[i] = Wire.read();
  }
  
  return a;
}

bool i2c_read(byte* buffer, byte _register){
  Wire.beginTransmission(ADDR);
  Wire.write(_register);
  Wire.endTransmission();
  
  bool a = Wire.requestFrom(ADDR, 1);
  *buffer = Wire.read();
  
  return a;
}
