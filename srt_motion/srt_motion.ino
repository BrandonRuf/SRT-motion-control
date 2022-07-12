/*
 * For use in the McGill University physics labs.
 * Written by Brandon Ruffolo in 2021-22.
 * Email: brandon.ruffolo@mcgill.ca
*/

#include <Wire.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM6DSOX.h>

#define BAUD 115200

/** Motor control pins **/
#define M1FLT_PIN 2 
#define M2FLT_PIN 3
#define M1PWM_PIN 4
#define M2PWM_PIN 5
#define M1EN_PIN  12
#define M2EN_PIN  13
#define M1DIR_PIN 8
#define M2DIR_PIN 9

#define REVERSING_DELAY 0  // Motor direction reversing delay (in milliseconds) 

bool  motor_fault[2];
float acceleration[3];
float rotation[3];
float bfield[3];
float temperature;
float measure_time;

sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t temp;
sensors_event_t mag;

Adafruit_LSM6DSOX lsm6dsoxSensor;
Adafruit_LIS3MDL lis3mdl;

/** Serial data handling **/
const byte data_size = 64;        // Size of the data buffer receiving from the serial line 
char received_data[data_size];    // Array for storing received data
char temp_data    [data_size];    // Temporary array for use when parsing
char functionCall[20]  = {0};     //
boolean newData = false;          // Flag used to indicate if new data has been found on the serial line
char * strtok_index;              // Used by strtok() as an index

void initialize(){

  /* Directions */
  digitalWrite(M1DIR_PIN, LOW);
  digitalWrite(M2DIR_PIN, LOW);

  /* PWM pins held HIGH, can be PWM'd for speed control */
  digitalWrite(M1PWM_PIN, HIGH);
  digitalWrite(M2PWM_PIN, HIGH);   

  /* Enable pins, turn a given motor on when pull HIGH */
  digitalWrite(M1EN_PIN, LOW);
  digitalWrite(M2EN_PIN, LOW);
}

void setup() {
  Serial.begin(BAUD);               

  /* Fault pins must be pulled up to arduino's logic level */
  pinMode(M1FLT_PIN, INPUT_PULLUP); 
  pinMode(M2FLT_PIN, INPUT_PULLUP); 

  /* Each driver output is controlled by a
   * PWM, Enable, and Direction pin. */
  pinMode(M1PWM_PIN, OUTPUT); 
  pinMode(M2PWM_PIN, OUTPUT); 
  
  pinMode(M1EN_PIN, OUTPUT); 
  pinMode(M2EN_PIN, OUTPUT); 
  
  pinMode(M1DIR_PIN, OUTPUT); 
  pinMode(M2DIR_PIN, OUTPUT); 
  
  /* Enable MEMs sensors */
  lsm6dsoxSensor.begin_I2C();
  lis3mdl       .begin_I2C(0x1E);
  
  initialize();                       // Initialize relevant variables 

  lsm6dsoxSensor.setGyroRange(LSM6DS_GYRO_RANGE_125_DPS);
  /*Serial.print("Gyro range set to: ");
  switch (lsm6dsoxSensor.getGyroRange()) {
  case LSM6DS_GYRO_RANGE_125_DPS:
    Serial.println("125 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_250_DPS:
    Serial.println("250 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_500_DPS:
    Serial.println("500 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_1000_DPS:
    Serial.println("1000 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_2000_DPS:
    Serial.println("2000 degrees/s");
    break;
  case ISM330DHCX_GYRO_RANGE_4000_DPS:
    break; // unsupported range for the DSOX
  }*/
}

void loop() {
  receive_data();                       /* Look for and grab data on the serial line. */
                                        /* If new data is found, the newData flag will be set */ 
  if (newData == true) {
      strcpy(temp_data, received_data); /* this temporary copy is necessary to protect the original data    */
                                        /* because strtok() used in parseData() replaces the commas with \0 */
      parseData();                      // Parse the data for commands
      newData = false;                  // Reset newData flag
  }

  // Get MEMs sensor data
  lsm6dsoxSensor.getEvent(&accel, &gyro, &temp);
  lis3mdl.getEvent(&mag);

  // Transfer 
  acceleration[0] = accel.acceleration.x;
  acceleration[1] = accel.acceleration.y;
  acceleration[2] = accel.acceleration.z;

  rotation[0] = gyro.gyro.x;
  rotation[1] = gyro.gyro.y;
  rotation[2] = gyro.gyro.z;

  temperature = temp.temperature;

  bfield[0] = mag.magnetic.x;
  bfield[1] = mag.magnetic.y;
  bfield[2] = mag.magnetic.z;

  get_motor_fault();
}
