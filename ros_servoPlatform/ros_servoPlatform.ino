#include <ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <Servo.h>

ros::NodeHandle nh;

Servo bottom;
Servo top;

void messageCb(const std_msgs::UInt16MultiArray& times)
{
  if(times.data[0] > 70 && times.data[0] < 120)
  {
    bottom.write(times.data[0]);
  }
  
  if(times.data[1] > 70 && times.data[1] < 120)
  {
    top.write(times.data[1]);
  }
}

ros::Subscriber<std_msgs::UInt16MultiArray> sub("servo_cmd", &messageCb);

void setup() {
  // put your setup code here, to run once:
  bottom.attach(9);
  top.attach(10);
  
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();
  delay(1);
}
