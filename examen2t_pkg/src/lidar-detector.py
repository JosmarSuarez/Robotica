#! /usr/bin/env python
import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import LaserScan

pub_t = rospy.Publisher('sector', Int32, queue_size=10)
def callback_t(data):
    #rospy.loginfo( data.ranges)    
    minimo=min(data.ranges)
    try:
        value_index = data.ranges.index(minimo)
    except:
        value_index = -1
    
    cuadrante=Int32()
    if minimo<15:
        if value_index <180:
            cuadrante.data=1
        if value_index >= 180 and value_index <360:
            cuadrante.data=2
        if value_index >= 360 and value_index <540:
            cuadrante.data=3
        if value_index >= 540 and value_index <720:
            cuadrante.data=4
    else:
        cuadrante.data=5
    rospy.loginfo(cuadrante.data)
    pub_t.publish(cuadrante)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('lidar_detector', anonymous=True)
    #pub_t = rospy.Publisher('sector', Int32, queue_size=10)
    rospy.Subscriber("raw_scan", LaserScan, callback_t)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()