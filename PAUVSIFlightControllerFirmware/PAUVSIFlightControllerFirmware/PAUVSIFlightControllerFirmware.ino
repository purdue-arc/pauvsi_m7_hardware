#include <PID_v1.h>
#include <Kalman.h>
#include "Wire.h"

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
  double yaw_rate;
} attitude;

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

void setup() {
  //set up the LEDS
  setupIndicatorLEDs();
  
  Serial.begin(115200);
  Wire.begin();

  // Configure gyroscope range
  I2CwriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_2000_DPS);
  // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS,28,ACC_FULL_SCALE_16_G);
  // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);
 
  // Request first magnetometer single measurement
  I2CwriteByte(MAG_ADDRESS,0x0A,0x01);

  delayMicroseconds(500);
  //this will initialize the kalman filter with a starting angle
  //based off the accelerometer
  initializeKalmanFusion();
}

void loop() {
  updateIndicatorLEDs();
  attitude = readAndFuseIMU(attitude); // read IMU a fuse it using a kalmna filter which requries previous state
  Serial.printf("Pitch: %6.1f Roll: %6.1f Yaw_Rate: %6.1f\n", attitude.pitch, attitude.roll, attitude.yaw_rate);
}
