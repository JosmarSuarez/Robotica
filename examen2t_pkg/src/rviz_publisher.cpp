#include <string>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
double  joint1_angle=0, joint2_angle=0, joint3_angle=0, joint4_dist=0;


void axes_callback(const geometry_msgs::Quaternion::ConstPtr& msg)
{
    //ROS_INFO("Data: %f", msg->data);
    joint1_angle= msg->x;
    joint2_angle= msg->y;
    joint3_angle= msg->z;
    joint4_dist= msg->w;
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "rviz_publisher"); // nombre del nodo
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Subscriber joint_subs = n.subscribe<geometry_msgs::Quaternion>("joint_values", 20,axes_callback);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(30);



    // robot state
    // el parcial
    //double tilt = 0, tinc = degree, joint1_angle=0, joint2_dist=0, joint3_dist=0, joint4_dist=0, hinc=0.005;

    // message declarations
    geometry_msgs::TransformStamped odom_trans;
    sensor_msgs::JointState joint_state; //importante
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "axis";

    while (ros::ok()) 
    {
        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(4); // 4 gDL
        joint_state.position.resize(4);
        joint_state.name[0] ="joint1";
        joint_state.position[0] = joint1_angle;
        joint_state.name[1] ="joint2";
        joint_state.position[1] = joint2_angle;
        joint_state.name[2] ="joint3";
        joint_state.position[2] = joint3_angle;
        joint_state.name[3] ="joint4";
        joint_state.position[3] = joint4_dist;
	    //joint_state.name[3] ="joint4"; // nombres deben coincidir con joint del rviz
        //joint_state.position[3] = joint4_angle;

        // update transform
        // (moving in a circle with radius=2)

        //send the joint state and transform
        joint_pub.publish(joint_state);

        // Create new robot state
       
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}

