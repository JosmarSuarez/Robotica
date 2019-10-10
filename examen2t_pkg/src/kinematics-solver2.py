#! /usr/bin/env python
from sympy import *
import numpy as np
import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

t1=Symbol('t1')
t2=Symbol('t2')
t3=Symbol('t3')
d2=Symbol('d2')

msg2=Quaternion()
pub = rospy.Publisher('joint_values',Quaternion, queue_size=20)


def subs_dh(T,t1,t2,t3,d2,par):
	[m,n]=T.shape #tamano de filas y columnas
	T_subs=np.zeros(T.shape)
	for i in range(0,m):
		for j in range (0,n):
			T_subs[i,j]=T[i,j].subs([(t1,par[0,0]),(t2,par[0,1]),(t3,par[0,2]),(d2,par[0,3])])
	return T_subs

def DH_matrix(t,d,a,alph):
	T=np.array([[cos(t),-cos(alph)*sin(t),sin(alph)*sin(t),a*cos(t)],
		[sin(t),cos(alph)*cos(t),-sin(alph)*cos(t),a*sin(t)],
		[0,sin(alph),cos(alph),d],[0,0,0,1]])
	return T

def subscriber():

    rospy.init_node('kinematics_solver', anonymous=True)

    rospy.Subscriber("sector", Int32, callback)
	#pub.publish(1234)
    rospy.spin()

def publisher():

	#rospy.init_node('Kinematicainversa2', anonymous=True)
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		rospy.loginfo("EstamosEnviandoLosTitas")

		rate.sleep()

def callback(cuad):
    #rospy.loginfo(rospy.get_caller_id() + "Received Data: %f", Point.x)
	iterations =500
	if cuad.data==1:
		msg2.x =0.1326
		msg2.y =-0.8117
		msg2.z =0.8476
		msg2.w =0.1
	if cuad.data==2:
		msg2.x =1.2036
		msg2.y =45.0268
		msg2.z =-107.65
		msg2.w =0.1
	if cuad.data==3:
		msg2.x =1.7359
		msg2.y =13.372
		msg2.z =-32.28
		msg2.w =0.1
	if cuad.data==4:
		msg2.x =2.3561
		msg2.y =-2.1674+3.141592
		msg2.z =2.2667+3.141592/2
		msg2.w =0.1
	if cuad.data==5:
		msg2.x =-1.57
		msg2.y =-0.030
		msg2.z =1.7085
		msg2.w =0.1
	pub.publish(msg2)

if __name__ == '__main__':
    T01 = DH_matrix(t1,0.4,0.025,np.pi/2)
    T12 = DH_matrix(t2,0,0.455,0)
    T23 = DH_matrix(t3+np.pi/2,0,0.035,np.pi/2)
    T34 = DH_matrix(d2,0.420,0,0)
    T02 = np.dot(T01,T12)
    T03 = np.dot(T02,T23)
    T04 = np.dot(T03,T34)

    px= T04[0,3]
    py= T04[1,3]
    pz= T04[2,3]

    print "X\n" 
    print px
    print "\nY\n"
    print py
    print "\nZ\n"
    print pz

    try:
        subscriber()
        publisher()
    except rospy.ROSInterruptException:
        pass

#print "Y:" + TO4[1,3]
#print "Z:" + TO4[2,3]
#px=numeric_T[0,3]
#    	py=numeric_T[1,3]
#    	pz=numeric_T[2,3]
