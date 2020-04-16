//libraries
#include <Wire.h>
#include "ESP8266WiFi.h" 
#include "FirebaseArduino.h" 

//define firebase and WiFi credentials
#define FIREBASE_HOST "getfit-70b66.firebaseio.com" //Without http:// or https:// schemes
#define FIREBASE_AUTH "Y07opUw8SqVJL0HzjrNLD7yLKnz84Y0SGFVv3rTR"
#define WIFI_SSID "Bankai"
#define WIFI_PASSWORD "nvvxiegj"


//defining variables
const float HI_THRESHOLD = 1.00; //used to count peaks
bool count_flag = false; // boolean for setting counter 
int n= 0; //will be used to store the count
int oldValue = n; //will be used to compare the count and send only 1 variable to the DB
int totalReps; // this value is recieved from the DB and is used to tell the system when to stop it is created in the webapp

const uint8_t MPU_addr = 0x68; // I2C address of the MPU-6050

//defining the sensor Raw values as can be found in the datasheet
const float MPU_ACCL_2_SCALE = 16384.0;
const float MPU_ACCL_4_SCALE = 8192.0;
const float MPU_ACCL_8_SCALE = 4096.0;
const float MPU_ACCL_16_SCALE = 2048.0;

//for filtering the 3 acceleratmeter demensions
float filteredAx = 0;
float filteredAy = 0;
float filteredAz = 0;


struct rawdata {
  int16_t AcX; //Acceleration
  int16_t AcY; //Acceleration
  int16_t AcZ; //Acceleration

};

// Using int variables to convert later on to degrees and decimals
struct scaleddata {
  float AcX; //Acceleration
  float AcY; //Acceleration
  float AcZ; //Acceleration

};

bool checkI2c(byte addr);
void mpu6050Begin(byte addr);
rawdata mpu6050Read(byte addr, bool Debug);
void setMPU6050scales(byte addr, uint8_t Accl);
void getMPU6050scales(byte addr, uint8_t &Accl);
scaleddata convertRawToScaled(byte addr, rawdata data_in, bool Debug);

//starting the sensor, connecting to teh wifi and firebase.
void setup() {
  Wire.begin();
  Serial.begin(115200);

  mpu6050Begin(MPU_addr);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(100);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();
    Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  totalReps = Firebase.getInt("Exercise/totalReps");
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  rawdata next_sample;
  setMPU6050scales(MPU_addr, 0b00010000);
  next_sample = mpu6050Read(MPU_addr, false);
  convertRawToScaled(MPU_addr, next_sample, false);
  
  //myString = String(n);
  if(oldValue != n){
     Firebase.setInt("Reps/Value", n);
     oldValue++;
  }

// lights are on untill the current amount of movements equals the pre-set goal(from the webapp)
  if(n == totalReps){

    digitalWrite(LED_BUILTIN, HIGH); 
    
    //delay (1000);
  }
  delay(100); // Wait 5 seconds and scan again


  
}


void mpu6050Begin(byte addr) {
  // This function initializes the MPU-6050 IMU Sensor
  // It verifys the address is correct and wakes up the
  // MPU.
  if (checkI2c(addr)) {
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B); // PWR_MGMT_1 register
    Wire.write(0); // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);

    delay(30); // Ensure gyro has enough time to power up
  }
}

bool checkI2c(byte addr) {
  // This function uses the return value of
  // the Write.endTransmisstion to see if
  // a device did acknowledge to the address.
  Serial.println(" ");
  Wire.beginTransmission(addr);

  if (Wire.endTransmission() == 0)
  {
    Serial.print(" Device Found at 0x");
    Serial.println(addr, HEX);
    return true;
  }
  else
  {
    Serial.print(" No Device Found at 0x");
    Serial.println(addr, HEX);
    return false;
  }
}



rawdata mpu6050Read(byte addr, bool Debug) {
  // This function reads the raw 16-bit data values from
  // the MPU-6050 the example in the datasheet are used
  rawdata values;

  Wire.beginTransmission(addr);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(addr, 14, true); // request a total of 14 registers
  values.AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  values.AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  values.AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)


  return values;
}
//here we change the raw value to readable scale value
void setMPU6050scales(byte addr, uint8_t Accl) {
  Wire.beginTransmission(addr);
  Wire.write(0x1B); // write to register starting at 0x1B
  Wire.write(Accl); // Self Tests Off and set Accl FS to 8g
  Wire.endTransmission(true);
}
//this method is used to retrieve teh values after creating them
void getMPU6050scales(byte addr, uint8_t &Accl) {
  Wire.beginTransmission(addr);
  Wire.write(0x1B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(addr, 2, true); // request a total of 14 registers
  Accl = (Wire.read() & (bit(3) | bit(4))) >> 3;
}



//now we calibrate the sensor
scaleddata convertRawToScaled(byte addr, rawdata data_in, bool Debug) {

  scaleddata values;
  float scale_value = 0.0;
  byte Accl;

  getMPU6050scales(MPU_addr, Accl);

  //defining teh speeds,values, 2, 4, 8, 16 can be found in the datasheet
  scale_value = 0.0;
  if (Debug) {
    Serial.print("Accl Full-Scale = ");
  }
  switch (Accl) {
    case 0:
      scale_value = MPU_ACCL_2_SCALE;
      if (Debug) {
        Serial.println("±2 g");
      }
      break;
    case 1:
      scale_value = MPU_ACCL_4_SCALE;
      if (Debug) {
        Serial.println("±4 g");
      }
      break;
    case 2:
      scale_value = MPU_ACCL_8_SCALE;
      if (Debug) {
        Serial.println("±8 g");
      }
      break;
    case 3:
      scale_value = MPU_ACCL_16_SCALE;
      if (Debug) {
        Serial.println("±16 g");
      }
      break;
    default:
      break;
  }

  //creating 1 single value out of 3 using pythagoras this gives us the ability to read and measure 3D space better 
  float filterConstant = 0.05;
  values.AcX = (float) data_in.AcX / scale_value;
  values.AcY = (float) data_in.AcY / scale_value;
  values.AcZ = (float) data_in.AcZ / scale_value;
  filteredAx = filteredAx * (1.0 - filterConstant) + values.AcX * filterConstant;
  filteredAy = filteredAy * (1.0 - filterConstant) + values.AcY * filterConstant;
  filteredAz = filteredAz * (1.0 - filterConstant) + values.AcZ * filterConstant;
  float result = sqrt((filteredAx * filteredAx) + (filteredAy * filteredAy ) + (filteredAz * filteredAx ));
  
  Serial.println(result);
//start the count of every movement by using a threshold
 if( result > HI_THRESHOLD && !count_flag) //if analog_in > Hi_THRESHOLD and count_flag == false
     {
           n++;
           count_flag = true;
     }
     
     if(result <= HI_THRESHOLD) //set count_flag to false only when below the threshold
           count_flag = false;
           
  Serial.println(n);



  return values;
}
