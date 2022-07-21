#include <Wire.h>

#define BAUD 115200             // Serial COM baudrate

#define IMU_ADDR 0x6A           // IMU Chip I2C Address
#define MAG_ADDR 0x1C           // Magnetometer Chip I2C Address

#define EARTH_GRAVITY 9.80665F  // m/s^2 [standard value (defined)]

//** IMU Register definitions **//
#define CTRL1_XL  0x10  // Accelerometer control register (r/w)
#define CTRL2_G   0x11  // Gyroscope control register     (r/w)
#define CTRL3_C   0x12  // Control register 3 (r/w)
#define CTRL4_C   0x13  // Control register 4 (r/w)
#define CTRL5_C   0x14  // Control register 5 (r/w)
#define CTRL6_C   0x15  // Control register 6 (r/w)
#define CTRL7_G   0x16  // Control register 7 (r/w)
#define CTRL8_XL  0x17  // Control register 8 (r/w)

#define WHO_AM_I   0x0f  // Chip ID register  (r)
#define STATUS_REG 0x1E  //                   (r)

#define OUT_TEMP_L 0x20 // Temperature data output register (r)

#define OUTX_L_G   0x22 // Angular rate sensor pitch axis (X) angular rate output register (r)
#define OUTY_L_G   0x24 // Angular rate sensor roll axis  (Y) angular rate output register (r)
#define OUTZ_L_G   0x26 // Angular rate sensor yaw axis   (Z) angular rate output register (r)

#define OUTX_L_A   0x28 // Linear acceleration sensor X-axis output register (r)
#define OUTY_L_A   0x2A // Linear acceleration sensor Y-axis output register (r)
#define OUTZ_L_A   0x2C // Linear acceleration sensor Z-axis output register (r)

//** MAGNETOMETER Register definitions **//
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24

#define MAG_STATUS_REG 0x27


#define OUT_X_L   0x28
#define OUT_Y_L   0x2A
#define OUT_Z_L   0x2C

#define WHO_AM_I_MAG 0x0F

//** Motor control pins **//
#define M1FLT_PIN 2     // Motor 1 Fault 
#define M2FLT_PIN 3     // Motor 2 Fault 
#define M1PWM_PIN 4     // Motor 1 PWM 
#define M2PWM_PIN 5     // Motor 2 PWM 
#define M1EN_PIN  12    // Motor 1 Enable 
#define M2EN_PIN  13    // Motor 2 Enable 
#define M1DIR_PIN 8     // Motor 1 Direction 
#define M2DIR_PIN 9     // Motor 2 Direction 

/* Gyro data range */
typedef enum{
  GYRO_RANGE_125_DPS  = 0b0010,
  GYRO_RANGE_250_DPS  = 0b0000,
  GYRO_RANGE_500_DPS  = 0b0100,
  GYRO_RANGE_1000_DPS = 0b1000,
  GYRO_RANGE_2000_DPS = 0b1100
} gyro_range;

/* Accelerometer data range */
typedef enum{
  ACCEL_RANGE_2_G  = 0b0000,
  ACCEL_RANGE_16_G = 0b0100,
  ACCEL_RANGE_4_G  = 0b0100,
  ACCEL_RANGE_8_G  = 0b1100
} accel_range;

/* Motion sensor data rates */
typedef enum{
  RATE_OFF      = 0b00000000,
  RATE_12_5_HZ  = 0b00010000,
  RATE_26_HZ    = 0b00100000,
  RATE_52_HZ    = 0b00110000,
  RATE_104_HZ   = 0b01000000,
  RATE_208_HZ   = 0b01010000,
  RATE_416_HZ   = 0b01100000,
  RATE_833_HZ   = 0b01110000,
  RATE_1_66K_HZ = 0b10000000,
  RATE_3_33K_HZ = 0b10010000,
  RATE_6_66K_HZ = 0b10100000,
} data_rate;

/* Magnetometer data rates */
typedef enum{
  RATE_0_6_25_HZ  = 0b01100000,
  RATE_1_25_HZ    = 0b01100100,
  RATE_2_5_HZ     = 0b01101000,
  RATE_5_HZ       = 0b01101100,
  RATE_10_HZ      = 0b01110000,
  RATE_20_HZ      = 0b01110100,
  RATE_40_HZ      = 0b01111000,
  RATE_80_HZ      = 0b01111100,
  RATE_155_HZ     = 0b01100010,
  RATE_300_HZ     = 0b01100010,
  RATE_560_HZ     = 0b01100010,
  RATE_1K_HZ      = 0b10100010,
} mag_data_rate;

/* Magnetometer data range */
typedef enum{
  MAG_RANGE_4_GAUSS  = 0b00000000,
  MAG_RANGE_8_GAUSS  = 0b00100000,
  MAG_RANGE_12_GAUSS = 0b01000000,
  MAG_RANGE_16_GAUSS = 0b01100000
} mag_range;


float acceleration[3],rotation[3],mag[3],temperature[1];
float rotation_offset[3] = {0.,0.,0.};

int16_t combine_8_to_16(uint8_t *a){
  int16_t b = a[1] << 8 | a[0];
  return b;
}

bool get_mag(float *_mag){
  byte buffer[6];
  i2c_read(MAG_ADDR,buffer,OUT_X_L,6);

  int16_t rawMagX,rawMagY,rawMagZ;

  // Combine two 8-bit register's data into 16-bit value
  rawMagX = buffer[1] << 8 | buffer[0];
  rawMagY = buffer[3] << 8 | buffer[2];
  rawMagZ = buffer[5] << 8 | buffer[4];

  _mag[0] = rawMagX *6842/1000.;
  _mag[1] = rawMagY *6842/1000.;
  _mag[2] = rawMagZ *6842/1000.;
}

bool get_rotation(float *_rotation){
  byte buffer[6];
  i2c_read(IMU_ADDR,buffer,OUTX_L_G,6);

  int16_t rawGyroX,rawGyroY,rawGyroZ;

  // Combine two 8-bit register's data into 16-bit value
  rawGyroX = buffer[1] << 8 | buffer[0];
  rawGyroY = buffer[3] << 8 | buffer[2];
  rawGyroZ = buffer[5] << 8 | buffer[4];

  _rotation[0] = rawGyroX *4.375/1000.;
  _rotation[1] = rawGyroY *4.375/1000.;
  _rotation[2] = rawGyroZ *4.375/1000.;
}

bool get_acceleration(float *_acceleration){
  byte buffer[6];
  bool response = i2c_read(IMU_ADDR,buffer,OUTX_L_A,6);

  int16_t rawAccelX,rawAccelY,rawAccelZ;
  
  rawAccelX = buffer[1] << 8 | buffer[0];
  rawAccelY = buffer[3] << 8 | buffer[2];
  rawAccelZ = buffer[5] << 8 | buffer[4];

  _acceleration[0] = rawAccelX * .061 *EARTH_GRAVITY/1000.;
  _acceleration[1] = rawAccelY * .061 *EARTH_GRAVITY/1000.;
  _acceleration[2] = rawAccelZ * .061 *EARTH_GRAVITY/1000.;

  return response;
}

void get_temperature(float *_temperature){
  byte buffer[2];
  i2c_read(IMU_ADDR,buffer,OUT_TEMP_L,2);

  int16_t rawTemp;
  
  rawTemp = buffer[1] << 8 | buffer[0];

  temperature[0] = (rawTemp/256.)+25.;
}

void zero_gyro_bias(){
  float rx,ry,rz;
  rx = 0;
  ry = 0;
  rz = 0;

  for(uint16_t i = 0; i < 500; i++){
    get_rotation(rotation);
    rx += rotation[0];
    ry += rotation[1];
    rz += rotation[2];
    delay(15);
    
    Serial.print(rx);
    Serial.print(" ");
    Serial.print(ry);
    Serial.print(" ");
    Serial.println(rz);
  }

  delay(5000);
  rotation_offset[0] = rx/500.;
  rotation_offset[1] = ry/500.;
  rotation_offset[2] = rz/500.;

  Serial.print("Rotation offset: ");
  Serial.print(rotation_offset[0]);
  Serial.print(" ");
  Serial.print(rotation_offset[1]);
  Serial.print(" ");
  Serial.println(rotation_offset[2]);

  
}

void set_gyro_params(data_rate _rate, gyro_range _range){
  uint8_t buffer[2];
  
  buffer[0] = CTRL2_G;
  buffer[1] = _rate | _range;
  i2c_write(IMU_ADDR,buffer,2);
}

void set_accel_params(data_rate _rate, accel_range _range){
  uint8_t buffer[2];
  
  buffer[0] = CTRL1_XL;
  buffer[1] = _rate | _range;
  i2c_write(IMU_ADDR,buffer,2);
}

void set_mag_params(mag_data_rate _rate, mag_range _range){
  uint8_t buffer[2];

  buffer[0] = CTRL_REG1;
  buffer[1] = _rate;
  i2c_write(MAG_ADDR,buffer,2);

  buffer[0] = CTRL_REG2;
  buffer[1] = _range;
  i2c_write(MAG_ADDR,buffer,2);
}
void setup() {
  Serial.begin(BAUD); 
  Wire  .begin();        // Start I2C library      

  _init();
}

void _init(){
  byte buffer[2];
  byte check_byte[1];

  buffer[0] = CTRL_REG3;
  buffer[1] = 0b00000000;
  i2c_write(MAG_ADDR,buffer,2);

  buffer[0] = CTRL_REG4;
  buffer[1] = 0b00001100;
  i2c_write(MAG_ADDR,buffer,2);
  

  // Enable Accelerometer with 12.5 Hz ODR, +/- 2g sensitivity
  set_accel_params(RATE_12_5_HZ,ACCEL_RANGE_2_G);
  
  // Enable GyroScope with 104 Hz ODR, +/- 125 dps sensitivity
  set_gyro_params(RATE_104_HZ,GYRO_RANGE_125_DPS);

  // Enable Magnetometer with 80 Hz ODR, +/- 4 Gauss sensitivity
  set_mag_params(RATE_80_HZ,MAG_RANGE_4_GAUSS);

  zero_gyro_bias();
  
  i2c_read(IMU_ADDR,check_byte,CTRL1_XL);
  Serial.print("CTRL1_XL: ");
  Serial.println(*check_byte);
  
  i2c_read(IMU_ADDR,check_byte,CTRL2_G);
  Serial.print("CTRL2_G: ");
  Serial.println(*check_byte);

  i2c_read(MAG_ADDR,check_byte,WHO_AM_I_MAG);
  Serial.print("WHO AM I MAG: ");
  Serial.println(*check_byte);
  
  delay(2000);
}


void loop() {
  
  get_acceleration(acceleration);
  Serial.print("Accel: ");
  Serial.print(acceleration[0]);
  Serial.print(" ");
  Serial.print(acceleration[1]);
  Serial.print(" ");
  Serial.print(acceleration[2]);
  Serial.print(" ");
  Serial.println(sqrt(sq(acceleration[0])+sq(acceleration[1])+sq(acceleration[2])));

  get_rotation(rotation);
  Serial.print("rotation: ");
  Serial.print(rotation[0]);
  Serial.print(" ");
  Serial.print(rotation[1]);
  Serial.print(" ");
  Serial.print(rotation[2]);
  Serial.print(" ");
  Serial.println(sqrt(sq(rotation[0])+sq(rotation[1])+sq(rotation[2])));

  get_mag(mag);
  Serial.print("Magnetic Field: ");
  Serial.print(mag[0]);
  Serial.print(" ");
  Serial.print(mag[1]);
  Serial.print(" ");
  Serial.print(mag[2]);
  Serial.print(" ");
  Serial.println(sqrt(sq(mag[0])+sq(mag[1])+sq(mag[2])));

  get_temperature(temperature);
  Serial.print("Temp: ");
  Serial.println(temperature[0]);

  
  Serial.println();
  delay(3000);
}
