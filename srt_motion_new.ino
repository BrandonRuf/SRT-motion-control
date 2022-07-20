#include <Wire.h>

#define BAUD 115200             // Serial COM baudrate
#define IMU_ADDR 0x6A           // IMU Chip I2C Address
#define MAG_ADDR 0x1C            // Magnetometer Chip I2C Address

#define EARTH_GRAVITY 9.80665F  // m/s^2 [standard value (defined)]

//** IMU Register definitions **//
#define CTRL1_XL  0x10  // Accelerometer control register 1 (r/w)
#define CTRL2_G   0x11  // Gyroscope control register 2     (r/w)
#define CTRL3_C   0x12  // Control register 3 (r/w)
#define CTRL4_C   0x13  // Control register 4 (r/w)
#define CTRL5_C   0x14  // Control register 5 (r/w)
#define CTRL6_C   0x15  // Control register 6 (r/w)
#define CTRL7_G   0x16  // Control register 7 (r/w)
#define CTRL8_XL  0x17  // Control register 8 (r/w)
#define WHO_AM_I  0x0f  // Chip ID register   (r)

#define OUT_TEMP_L 0x20 // Temperature data output register (r)

#define OUTX_L_G   0x22 // Angular rate sensor pitch axis (X) angular rate output register (r)
#define OUTY_L_G   0x24 // Angular rate sensor roll axis  (Y) angular rate output register (r)
#define OUTZ_L_G   0x26 // Angular rate sensor yaw axis   (Z) angular rate output register (r)

#define OUTX_L_A   0x28 // Linear acceleration sensor X-axis output register (r)
#define OUTY_L_A   0x2A // Linear acceleration sensor Y-axis output register (r)
#define OUTZ_L_A   0x2C // Linear acceleration sensor Z-axis output register (r)

//** MAGNETOMETER Register definitions **//
#define OUT_X_L 0x28
#define OUT_Y_L 0x2A
#define OUT_Z_L 0x2C

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

/* Sensor data rates */
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

float acceleration[3],rotation[3],mag[3],temperature[1];
float rotation_offset[3] = {0.,0.,0.};

bool get_mag(float *_mag){
  byte buffer[6];
  i2c_read(MAG_ADDR,buffer,OUT_X_L,6);

  int16_t rawMagX,rawMagY,rawMagZ;
  
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
  
  rawGyroX = buffer[1] << 8 | buffer[0];
  rawGyroY = buffer[3] << 8 | buffer[2];
  rawGyroZ = buffer[5] << 8 | buffer[4];

  _rotation[0] = rawGyroX *4.375/1000. - rotation_offset[0];
  _rotation[1] = rawGyroY *4.375/1000. - rotation_offset[1];
  _rotation[2] = rawGyroZ *4.375/1000. - rotation_offset[2];
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
  double rx,ry,rz;
  for(uint16_t i = 0; i < 500; i++){
    get_rotation(rotation);
    rx += rotation[0];
    ry += rotation[1];
    rz += rotation[2];
    
    delay(15);
  }

  rotation_offset[0] = rx/500;
  rotation_offset[1] = ry/500;
  rotation_offset[2] = rz/500;
}

void set_gyro_params(data_rate _rate, gyro_range _range){
  byte buffer[2];
  
  buffer[0] = CTRL2_G;
  buffer[1] = _rate | _range;
  i2c_write(IMU_ADDR,buffer,2);
}

void set_accel_params(data_rate _rate, accel_range _range){
  byte buffer[2];
  
  buffer[0] = CTRL1_XL;
  buffer[1] = _rate | _range;
  i2c_write(IMU_ADDR,buffer,2);
}

void setup() {
  Serial.begin(BAUD); 
  Wire  .begin();        // Start I2C library      

  _init();
}

void _init(){
  byte buffer[2];
  byte check_byte[1];

  // Enable Accelerometer with 12.5 Hz ODR, +/- 2g sensitivity
  set_accel_params(RATE_12_5_HZ,ACCEL_RANGE_2_G);
  
  // Enable GyroScope with 104 Hz ODR, +/- 125 dps sensitivity
  set_gyro_params(RATE_104_HZ,GYRO_RANGE_125_DPS);

  //zero_gyro_bias();
  
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
  delay(1000);
}
