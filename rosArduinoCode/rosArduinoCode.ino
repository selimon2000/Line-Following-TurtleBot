#include <ros.h>
#include <std_msgs/Float32.h>

long duration;
int echoPin=9;
int trigPin=8;

ros::NodeHandle  nh;
std_msgs::Float32 distance;
ros::Publisher pub("UltrasonicDistance", &distance );

void setup()
{ 
  nh.initNode();
  nh.advertise(pub);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop()
{ 
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance.data = duration * 0.034 / 2;

  pub.publish(&distance);
  nh.spinOnce();
}
