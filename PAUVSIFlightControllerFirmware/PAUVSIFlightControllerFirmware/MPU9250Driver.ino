// This function read Nbytes bytes from I2C device at address Address. 
// Put read bytes starting at register Register in the Data array. 
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
 
  // Read Nbytes
  Wire.requestFrom(Address, Nbytes); 
  uint8_t index=0;
  while (Wire.available())
    Data[index++]=Wire.read();
}
 
 
// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

ATTITUDE initializeKalmanFusion()
{
  IMUTimer = micros();
  // Read accelerometer and gyroscope
  uint8_t Buf[14];
  I2Cread(MPU9250_ADDRESS,0x3B,14,Buf);
  // Create 16 bits values from 8 bits data
  // Accelerometer
  int16_t ax=-(Buf[0]<<8 | Buf[1]);
  int16_t ay=-(Buf[2]<<8 | Buf[3]);
  int16_t az=Buf[4]<<8 | Buf[5];
 
  // Gyroscope
  int16_t gx=-(Buf[8]<<8 | Buf[9]);
  int16_t gy=-(Buf[10]<<8 | Buf[11]);
  int16_t gz=Buf[12]<<8 | Buf[13];

  //convert the acceleration vector to roll and pitch in degrees
  double accRoll  = atan2(ay, az) * RAD_TO_DEG;
  double accPitch = atan(-ax / sqrt(ay * ay + az * az)) * RAD_TO_DEG;

  // Set starting angle
  kalmanX.setAngle(accRoll); 
  kalmanY.setAngle(accPitch);

  ATTITUDE retAtt;
  retAtt.roll = accRoll;
  retAtt.pitch = accPitch;
  retAtt.yaw_rate = gz;

  return retAtt;
}

ATTITUDE kalmanFuseData(int ax, int ay, int az, int gx, int gy, int gz, double dt, ATTITUDE lastAtt)
{
  ATTITUDE retAtt; // the attitude to be returned
  
  //convert the acceleration vector to roll and pitch in degrees
  double accRoll  = atan2(ay, az) * RAD_TO_DEG;
  double accPitch = atan(-ax / sqrt(ay * ay + az * az)) * RAD_TO_DEG;

  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((accRoll < -90 && lastAtt.roll > 90) || (accRoll > 90 && lastAtt.roll < -90)) {
    kalmanX.setAngle(accRoll);
    retAtt.roll = accRoll;
  } else
    retAtt.roll = kalmanX.getAngle(accRoll, gx, dt); // Calculate the angle using a Kalman filter

  if (abs(retAtt.roll) > 90)
    gy = -gy; // Invert rate, so it fits the restriced accelerometer reading
  retAtt.pitch = kalmanY.getAngle(accPitch, gy, dt);
  
  retAtt.yaw_rate = gy;

  return retAtt;
}

ATTITUDE readAndFuseIMU(ATTITUDE lastAtt)
{
  // ____________________________________
  // :::  accelerometer and gyroscope ::: 
 
  // Read accelerometer and gyroscope
  uint8_t Buf[14];
  I2Cread(MPU9250_ADDRESS,0x3B,14,Buf);

  //calculate dt
  double dt = (double)(micros() - IMUTimer) / 1000000;
 
  // Create 16 bits values from 8 bits data
 
  // Accelerometer
  int16_t ax=-(Buf[0]<<8 | Buf[1]);
  int16_t ay=-(Buf[2]<<8 | Buf[3]);
  int16_t az=Buf[4]<<8 | Buf[5];
 
  // Gyroscope
  int16_t gx=-(Buf[8]<<8 | Buf[9]);
  int16_t gy=-(Buf[10]<<8 | Buf[11]);
  int16_t gz=Buf[12]<<8 | Buf[13];

  return kalmanFuseData(ax, ay, az, gx, gy, gz, dt, lastAtt);
}

