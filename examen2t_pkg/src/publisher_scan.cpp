#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/LaserScan.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
sensor_msgs::LaserScan laser_msg;
std_msgs::Header header_msg;
int samples=720;
float angle_min=-1.5707999;
float angle_max=-1.5707999;
float angle_increment=0.0043694018;
float time_increment=0.0;
float scan_time=0.0;
float range_min=0.10000000149;
float range_max=10.0;
float ranges[720];

int main(int argc,char **argv)
{

  ros::init(argc,argv,"publisher_scan");//Node name--[unique]
  ros::NodeHandle node_obj;//create a handler
  FILE *iin;
  float dval;
  uint32_t sequence=0;
  char frame_id[]="hokuyo_link";
  iin=fopen("/home/josmar/imt_ws/src/examen2t_pkg/src/scan.txt","r");
  ros::Publisher number_publisher=node_obj.advertise<sensor_msgs::LaserScan>("/raw_scan",100);// topic "/numbers", data type : Int32, buffer:10 bytes
  ros::Rate loop_rate(20); // publish rate, frequency, 10 Hz, 10 msgs per second
  int pub_counter=0; // variable
  if(iin==NULL){ROS_INFO("File absent");}
else{ROS_INFO("OK");}
  laser_msg.angle_min=angle_min;
  laser_msg.angle_max=angle_max;
  laser_msg.angle_increment=angle_increment;
  laser_msg.time_increment=time_increment;
  laser_msg.scan_time=scan_time;
  laser_msg.range_min=range_min;
  laser_msg.range_max=range_max;
  ROS_INFO("stage 1 OK");
  while(ros::ok())
  {
    header_msg.seq=sequence;
    header_msg.stamp=ros::Time::now();
    header_msg.frame_id=frame_id;
    laser_msg.header=header_msg;
    ROS_INFO("stage 2 OK");
    std::vector<float> vector_ranges(0);
    for (int i = 0; i < samples; i++) {
      int a=fscanf(iin,"%f",&dval);
      //ROS_INFO("stage 3 OK %f",dval);
      ranges[i]=dval;
      vector_ranges.push_back(dval);
    }
    laser_msg.ranges=vector_ranges;
    ROS_INFO("stage 4 OK");
    std_msgs::Float64 msg;// Int32 variable called msg
    msg.data=dval; //assign pub_counter to msg data
    //msg.data=2.36;
    sequence++;
    ROS_INFO("Data: %f",msg.data); // print for debugging
    number_publisher.publish(laser_msg);// publish msg
    ros::spinOnce();// keep alive
    loop_rate.sleep();// delay 10Hz->0.1s
    pub_counter++;//increase counter
  }
  fclose(iin);
  return 0;
}
