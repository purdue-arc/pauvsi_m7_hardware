#include <ros.h>
#include <ros/time.h>
#include <ax12.h>
#include <sensor_msgs/JointState.h>

#define NO_SERVO 6
#define setSpeed(id, pos) (ax12SetRegister2(id, AX_GOAL_SPEED_L, pos))
#define getSpeed(id)(ax12GetRegister(id, AX_PRESENT_SPEED_L, 2))

ros::NodeHandle nh;
sensor_msgs::JointState msg;
void jointStateCb(const sensor_msgs::JointState& inp)
{
    msg = inp;
}

ros::Subscriber<sensor_msgs::JointState> sub("arm_input", jointStateCb);


sensor_msgs::JointState opState;
ros::Publisher pub("arm_state", &opState);

unsigned int seq;

void setup()
{
  seq = 0;
  opState.velocity_length = NO_SERVO;
  opState.position_length = NO_SERVO;
  opState.effort_length = NO_SERVO;  
  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub);
}

void loop()
{
  
  int length = sizeof(msg.position)/sizeof(msg.position[0]);
  for(int i=0; i<length; ++i)
  {
    //9.55 rpm = 1 rad/s
    //0.111 rpm per unit
    setSpeed(i+1, int(9.55*msg.velocity[i] / 0.111));
    delay(10);
  }
  for(int i=0; i<length; ++i)
  {
     SetPosition(i+1, msg.position[i]);
     delay(10);      
  }
  
  opState.header.frame_id = msg.header.frame_id;
  opState.header.seq = seq++;
  opState.header.stamp = nh.now();
  for(int i=0; i<NO_SERVO;++i)
  {
    opState.position[i] = (GetPosition(i+1));
    opState.velocity[i] = (getSpeed(i+1)*0.111/9.55);
    opState.effort[i] = 0.0; // DO
  }
  
  pub.publish( &opState);
  nh.spinOnce();
  delay(500);
}
