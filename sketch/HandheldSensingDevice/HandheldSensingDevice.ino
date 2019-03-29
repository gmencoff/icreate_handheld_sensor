/*------------------------------------------------------------------------------

This code publishes the handheld sensor information to the following ROS topic:

/Handheld_device_sensor

In order for this program to work, the user must have the arduino libraries downloaded and copied
in to the library folder.

Additionally, the sensors must be hooked up properly according to the wiring diagram

May not to add calibration steps for the IMU, this is TBD

---------------------------------------------------------------*/


#include <ros.h>
#include <Wire.h>
#include <LIDARLite.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>
#include <handheld_device/handheld_device_data.h>
#include <std_msgs/String.h>

//Set the delay rate between fresh samples
#define BNO055_SAMPLERATE_DELAY_MS (10)

//Create a ros node handle and publish to the desired topic
ros::NodeHandle nh;
handheld_device::handheld_device_data handheld_device_data;
ros::Publisher Handheld_device("Handheld_device",&handheld_device_data);

//Initialize the sensors
LIDARLite myLidarLite;
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// store Lidar Reading and lidar reading count
int LidarDistance;
int LidarReadingCount=0;

//Store Switch Sensor
int laserSwitch=2;



/*******************************************************************************/ 
  /*
  Setup
  */
/******************************************************************************/



void setup() {
  
  // initialize ros node and advertise the handheld device topic
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(Handheld_device);
  
  //begin arduinio serial com
  Serial.begin(115200);

  // initializes the lidar to default and the I2C to 100kHz
  myLidarLite.begin(0,false);
  
  //set switch pin to input
  pinMode(laserSwitch, INPUT);

  // initialize the IMU
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
 
  delay(500);  
}

/*******************************************************************************/ 
  /*
  Setup
  */
/******************************************************************************/

void loop()
{
  
  
  //Get LIDAR data
  if(LidarReadingCount%100==0){
    handheld_device_data.device_distance=myLidarLite.distance(); //read with reciever bias correction
    LidarReadingCount=1;
  }
  else{
    handheld_device_data.device_distance=myLidarLite.distance(false); //read without reciever bias correction
    LidarReadingCount=LidarReadingCount+1;
  }
  
  
  imu::Quaternion quat = bno.getQuat();
  imu::Vector<3> linaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  handheld_device_data.quatx = quat.x();
  handheld_device_data.quaty = quat.y();
  handheld_device_data.quatz = quat.z();
  handheld_device_data.quatw = quat.w();
  handheld_device_data.accelx = linaccel(0);
  handheld_device_data.accely = linaccel(1);
  handheld_device_data.accelz = linaccel(2);
  
  //read switch state
  handheld_device_data.device_switch=digitalRead(laserSwitch);
  
  Handheld_device.publish(&handheld_device_data);
  nh.spinOnce();
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
