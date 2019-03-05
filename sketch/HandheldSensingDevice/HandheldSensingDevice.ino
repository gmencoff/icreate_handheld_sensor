/*------------------------------------------------------------------------------

This code publishes the handheld sensor information to the following ROS topic:

/Handheld_device_sensor

In order for this program to work, the user must have the arduino libraries downloaded and copied
in to the library folder.

Additionally, the sensors must be hooked up properly according to the wiring diagram


/*******************************************************************************/ 
  /*
  include libraries and initialize objects
  */
/******************************************************************************/

#include <Wire.h>
#include <LIDARLite.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

//Set the delay rate between fresh samples
#define BNO055_SAMPLERATE_DELAY_MS (100)

//Initialize the sensors
LIDARLite myLidarLite;
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// store Lidar Reading and lidar reading count
int LidarDistance;
int LidarReadingCount=0;

// store IMU quaternion and accel vector
imu::Quaternion imuQuat;
imu::Vector<3> imuAccel;

/**************************************************************************/
/*
    Display sensor calibration status
    */
/**************************************************************************/
void displayCalStatus(void)
{
    /* Get the four calibration values (0..3) */
    /* Any sensor data reporting 0 should be ignored, */
    /* 3 means 'fully calibrated" */
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);

    /* The data should be ignored until the system calibration is > 0 */
    Serial.print("\t");
    if (!system)
    {
        Serial.print("! ");
    }

    /* Display the individual values */
    Serial.print("Sys:");
    Serial.print(system, DEC);
    Serial.print(" G:");
    Serial.print(gyro, DEC);
    Serial.print(" A:");
    Serial.print(accel, DEC);
    Serial.print(" M:");
    Serial.print(mag, DEC);
}

/*********************************************************************
 * setup
 *********************************************************************/

void setup() {
  Serial.begin(115200);

  // initializes the lidar to default and the I2C to 100kHz
  myLidarLite.begin(0,false);

  // initialize the IMU
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  /*
   * 
   * The following setup looks for IMU calibration data stored in the arduino. If it exists, then
   * it is uploaded, otherwise the user is asked to calibrate the sensor at which point the data is
   * stored in the arduino
   * 
   * To recalibrate if calibration data currently exists, run the eeprom clear sketch in EEprom examples
   * and then reload this file
   */
  
  int eeAddress = 0;
  long bnoID;
  bool foundCalib = false;

  EEPROM.get(eeAddress, bnoID);

  adafruit_bno055_offsets_t calibrationData;
  sensor_t sensor;

  bno.getSensor(&sensor);
  if (bnoID != sensor.sensor_id)
  {
      Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
      delay(500);
  }
  else
  {
      Serial.println("\nFound Calibration for this sensor in EEPROM.");
      eeAddress += sizeof(long);
      EEPROM.get(eeAddress, calibrationData);

      Serial.println("\n\nRestoring Calibration data to the BNO055...");
      bno.setSensorOffsets(calibrationData);

      Serial.println("\n\nCalibration data loaded into BNO055");
      foundCalib = true;
  }

  //Crystal must be configured AFTER loading calibration data into BNO055.
  bno.setExtCrystalUse(true);

  sensors_event_t event;
  bno.getEvent(&event);
  if (foundCalib){
      Serial.println("Move sensor slightly to calibrate magnetometers");
      while (!bno.isFullyCalibrated())
      {
          bno.getEvent(&event);
          delay(BNO055_SAMPLERATE_DELAY_MS);
      }
  }
  else
  {
      Serial.println("Please Calibrate Sensor: ");
      while (!bno.isFullyCalibrated())
      {
          bno.getEvent(&event);

          Serial.print("X: ");
          Serial.print(event.orientation.x, 4);
          Serial.print("\tY: ");
          Serial.print(event.orientation.y, 4);
          Serial.print("\tZ: ");
          Serial.print(event.orientation.z, 4);

          /* Optional: Display calibration status */
          displayCalStatus();

          /* New line for the next sample */
          Serial.println("");

          /* Wait the specified delay before requesting new data */
          delay(BNO055_SAMPLERATE_DELAY_MS);
      }
  }

  Serial.println("\nFully calibrated!");
  Serial.println("--------------------------------");
  Serial.println("Calibration Results: ");
  adafruit_bno055_offsets_t newCalib;
  bno.getSensorOffsets(newCalib);

  Serial.println("\n\nStoring calibration data to EEPROM...");

  eeAddress = 0;
  bno.getSensor(&sensor);
  bnoID = sensor.sensor_id;

  EEPROM.put(eeAddress, bnoID);

  eeAddress += sizeof(long);
  EEPROM.put(eeAddress, newCalib);
  Serial.println("Data stored to EEPROM.");

  Serial.println("\n--------------------------------\n");
  delay(500);
}

/*********************************************************************
 * loop
 *********************************************************************/

void loop() {
 
  // reads the lidar distance with reciever bias correction every 100 measurements, according to 
  //operating manual
  
  if(LidarReadingCount%100==0){
    LidarDistance=myLidarLite.distance(); //read with reciever bias correction
    LidarReadingCount=1;
  }
  else{
    LidarDistance=myLidarLite.distance(false); //read without reciever bias correction
    LidarReadingCount=LidarReadingCount+1;
  }

  delay(BNO055_SAMPLERATE_DELAY_MS);
  
  // read the IMU quaternion and acceleration information, with gravity removed
  imuQuat=bno.getQuat();
  imuAccel=bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  
  Serial.println("");
  Serial.print("Distance: ");
  Serial.println(LidarDistance);
  Serial.print("xquat: ");Serial.print(imuQuat.x(),4);
  Serial.print(" yquat: ");Serial.print(imuQuat.y(),4);
  Serial.print(" zquat: ");Serial.print(imuQuat.z(),4);
  Serial.print(" wquat: ");Serial.println(imuQuat.w(),4);
  Serial.print(" xaccel: ");Serial.print(imuAccel.x(),4);
  Serial.print(" yaccel: ");Serial.print(imuAccel.y(),4);
  Serial.print(" zaccel: ");Serial.println(imuAccel.z(),4);

  delay(1000);
}
