void i2c_write(byte _addr, byte *buffer, uint8_t len){
  Wire.beginTransmission(_addr);
  Wire.write(buffer,len);
  Wire.endTransmission();
}

void i2c_write(byte _addr, byte b){
  Wire.beginTransmission(_addr);
  Wire.write(b);
  Wire.endTransmission();
}

bool i2c_read(byte _addr, byte* buffer, byte _register, uint8_t number_requested){
  Wire.beginTransmission(_addr);
  Wire.write(_register);
  Wire.endTransmission();
  
  bool a =  Wire.requestFrom(_addr, number_requested);
  for(int i=0; i<number_requested; i++){
    buffer[i] = Wire.read();
  }
  
  return a;
}

bool i2c_read(byte _addr, byte* buffer, byte _register){
  Wire.beginTransmission(_addr);
  Wire.write(_register);
  Wire.endTransmission();
  
  bool a = Wire.requestFrom(_addr, 1);
  *buffer = Wire.read();
  
  return a;
}
