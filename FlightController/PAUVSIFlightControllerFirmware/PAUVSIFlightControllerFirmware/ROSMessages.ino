geometry_msgs::Vector3Stamped contructAngleMessage(ATTITUDE att)
{
  geometry_msgs::Vector3Stamped msg;
  msg.vector.x = float(att.roll);
  msg.vector.y = float(att.pitch);
  msg.vector.z = float(att.yaw);
  
  return msg;
}

sensor_msgs::Imu constructIMUMessage(ATTITUDE att, RAW_IMU_DATA rawData)
{
  sensor_msgs::Imu msg;

  //converts the euler angle to a quarternion
  double angle;

  angle = att.roll * 0.5;
  double sr = sin(angle);
  double cr = cos(angle);

  angle = att.pitch * 0.5;
  double sp = sin(angle);
  double cp = cos(angle);

  angle = att.yaw * 0.5;
  double sy = sin(angle);
  double cy = cos(angle);

  double cpcy = cp * cy;
  double spcy = sp * cy;
  double cpsy = cp * sy;
  double spsy = sp * sy;

  double x = sr * cpcy - cr * spsy;
  double y = cr * spcy + sr * cpsy;
  double z = cr * cpsy - sr * spcy;
  double w = cr * cpcy + sr * spsy;

  double n = x*x + y*y + z*z + w*w;
  //normalize quat
  if (n != 1){
    n = sqrt(n);
    x = x / n;
    y = y / n;
    z = z / n;
    w = w / n;
  }

  msg.orientation.x = x;
  msg.orientation.y = y;
  msg.orientation.z = z;
  msg.orientation.w = w;

  // convert to m/s/s (+/- 4) and degrees/s (+/- 2000)

  msg.angular_velocity.x = (double)rawData.gx / 16.3835;
  msg.angular_velocity.y = (double)rawData.gy / 16.3835;
  msg.angular_velocity.z = (double)rawData.gz / 16.3835;

  msg.linear_acceleration.x = ((double)rawData.ax / 8191.75) * 9.8;
  msg.linear_acceleration.y = ((double)rawData.ay / 8191.75) * 9.8;
  msg.linear_acceleration.z = ((double)rawData.az / 8191.75) * 9.8;

  //create the covariance for the sensors
  float covar[9] = {1.2184696791468346e-07, 0, 0, 0, 1.2184696791468346e-07, 0, 0, 0, 1.2184696791468346e-07};
  memcpy(msg.angular_velocity_covariance, covar, 9);
  msg.angular_velocity_covariance[0] = 1.2184696791468346e-07;
  msg.angular_velocity_covariance[4] = 1.2184696791468346e-07;
  msg.angular_velocity_covariance[8] = 1.2184696791468346e-07;
  //msg.angular_velocity_covariance = covar;
  
  float covar2[9] = {8.99999999e-08, 0, 0, 0, 8.99999999e-08, 0, 0, 0, 8.99999999e-08};
  memcpy(msg.linear_acceleration_covariance, covar2, 9);
  msg.linear_acceleration_covariance[0] = 8.99999999e-08;
  msg.linear_acceleration_covariance[4] = 8.99999999e-08;
  msg.linear_acceleration_covariance[8] = 8.99999999e-08;
  //msg.linear_acceleration_covariance = covar2;

  return msg;
}

