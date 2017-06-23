#include <PID_v1.h>
#include <Kalman.h>
#include "Wire.h"
#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>

#define   NUMLEDS   3
#define   LEDMODE_INITIALIZING    0
#define   INITIALIZING_BLINK_RATE   200
#define   LEDMODE_ERROR    1
#define   ERROR_BLINK_RATE   50
#define   LED13PIN    13

#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C
 
#define    GYRO_FULL_SCALE_250_DPS    0x00  
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18
 
#define    ACC_FULL_SCALE_2_G        0x00  
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18

//the variables for the IMU
struct ATTITUDE {
  double pitch;
  double roll;
  double yaw;
} attitude;

struct RAW_IMU_DATA {
  int16_t ax;
  int16_t ay;
  int16_t az;
  int16_t gx;
  int16_t gy;
  int16_t gz;
} rawIMUData;

uint32_t IMUTimer = 0;
// Create the Kalman instances
Kalman kalmanX; 
Kalman kalmanY;

//setting up the PID for the rate controller
//TODO
//I'm am not 100% sure where these calculations will be done.
//I think that any rate control will be done on the flight controller though

//Indicator LED variables
uint32_t LEDTimer[NUMLEDS] = {0, 0, 0};
int LED_13_State = LEDMODE_INITIALIZING;
int LED_A_State = LEDMODE_INITIALIZING;
int LED_B_State = LEDMODE_INITIALIZING;

//Setup ROS_SERIAL
ros::NodeHandle  nh;
geometry_msgs::Vector3Stamped attitude_msg;
ros::Publisher pub_attitude( "/m7/imu/euler", &attitude_msg);
sensor_msgs::Imu rawIMU_msg;
ros::Publisher pub_IMU( "/m7/imu/measurement", &rawIMU_msg);

void setup() {
  //set up the LEDS
  setupIndicatorLEDs();
  
  //Serial.begin(115200);
  Wire.begin();

  // Configure gyroscope range
  I2CwriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_2000_DPS);
  // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS,28,ACC_FULL_SCALE_4_G);
  // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);
 
  // Request first magnetometer single measurement
  I2CwriteByte(MAG_ADDRESS,0x0A,0x01);

  delayMicroseconds(500);
  //this will initialize the kalman filter with a starting angle
  //based off the accelerometer
  initializeKalmanFusion();

  //setup ROS Subs and Pubs
  nh.initNode();
  nh.advertise(pub_attitude);
  nh.advertise(pub_IMU);
}

void loop() {
  //update the LEDs for status
  updateIndicatorLEDs();
  
  //read the pitch and roll from IMU and set the new attitude est
  attitude = readAndFuseIMU(attitude, &rawIMUData); // read IMU a fuse it using a kalmna filter which requries previous state
  
  //contruct attitde message and publish it
  attitude_msg = contructAngleMessage(attitude);
  attitude_msg.header.stamp = nh.now();
  pub_attitude.publish(&attitude_msg);

  //conctruct the IMU msg and publish
  rawIMU_msg = constructIMUMessage(attitude, rawIMUData);
  rawIMU_msg.header.stamp = nh.now();
  pub_IMU.publish(&rawIMU_msg);
  
  nh.spinOnce();
}
