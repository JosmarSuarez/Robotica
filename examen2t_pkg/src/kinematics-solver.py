from sympy import *
import numpy as np
import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

j1=Symbol('j1')
j2=Symbol('j2')
j3=Symbol('j3')
j4=Symbol('j4')

msg2=Quaternion()
pub = rospy.Publisher('joint_values',Quaternion, queue_size=20)


#Funcion que sustituye valores numericos en la matriz de transformacion
#subs_dh(Matriz de transformacion, j1, j2, j3, j4, parametros iniciales )
def subs_dh(T,j1,j2,j3,j4,par):
	[m,n]=T.shape #tamano de filas y columnas
	T_subs=np.zeros(T.shape)
	for i in range(0,m):
		for j in range (0,n):
			T_subs[i,j]=T[i,j].subs([(j1,par[0,0]),(j2,par[0,1]),(j3,par[0,2]),(j4,par[0,3])])
	return T_subs

#Creacion de la matriz DH
def DH_matrix(t,d,a,alph):
	T=np.array([[cos(t),-cos(alph)*sin(t),sin(alph)*sin(t),a*cos(t)],
		[sin(t),cos(alph)*cos(t),-sin(alph)*cos(t),a*sin(t)],
		[0,sin(alph),cos(alph),d],[0,0,0,1]])
	return T

#Inicializa el subscriptor
def subscriber():
	rospy.init_node('kinematics-solver', anonymous=True)
	rospy.Subscriber("sector", Int32, callback) #Callback realiza el calculo de los joints
	rospy.spin()


def callback(cuad):
    #En caso de recibir algun dato del 1 al 5 se le asigna un punto
	iterations =500
	if cuad.data==1:
		x=0.75
		y=0.1
		z=0.12
	if cuad.data==2:
		x=0.3
		y=0.78
		z=1.05
	if cuad.data==3:
		x=-0.15
		y=0.90
		z=0.8
	if cuad.data==4:
		x=-0.13
		y=0.13
		z=0.1
	if cuad.data==5:
		x=0
		y=-0.4
		z=0.8
	pxyz=np.array([[x,y,z]]) #target point
	alpha = 0.1 #learning rate
	qi = np.array([[0.5,0.5,0.5,0.5]])
	
    #jacobi matrix
	J=np.array([[diff(px,j1),diff(px,j2),diff(px,j3),diff(px,j4)],[diff(py,j1),diff(py,j2),diff(py,j3),diff(py,j4)],[diff(pz,j1),diff(pz,j2),diff(pz,j3),diff(pz,j4)]])
	print "Calculando ..."
    #print J.shape
	Ex = np.zeros(iterations)
	Ey = np.zeros(iterations)
    
	for i in range (0,iterations):
	    e = pxyz - np.array([[px.subs([(j1,qi[0,0]),(j2,qi[0,1]),(j3,qi[0,2]),(j4,qi[0,3])]),py.subs([(j1,qi[0,0]),(j2,qi[0,1]),(j3,qi[0,2]),(j4,qi[0,3])]),pz.subs([(j1,qi[0,0]),(j2,qi[0,1]),(j3,qi[0,2]),(j4,qi[0,3])])]])
	    par = np.array([[qi[0,0],qi[0,1],qi[0,2],qi[0,3]]])
	    J_subs=subs_dh(J,j1,j2,j3,j4,par)
	    dq = np.dot(np.linalg.pinv(J_subs),e.T)
	    qi = qi + alpha*dq.T
	print qi
	msg2.x =float(qi[0,0])
	msg2.y =float(qi[0,1])
	msg2.z =float(qi[0,2])
	msg2.w =float(qi[0,3])
	pub.publish(msg2)
#Main code
if __name__ == '__main__':
	#Los parametros DH del modelo se colocan aqui
	T01 = DH_matrix(j1,0.4,0.025,np.pi/2)
	T12 = DH_matrix(j2,0,0.455,0)
	T23 = DH_matrix(j3+np.pi/2,0,0.035,np.pi/2)
	T34 = DH_matrix(j4,0.420,0,0)
	#Multiplicamos las matrices previamente creadas
	T02 = np.dot(T01,T12)
	T03 = np.dot(T02,T23)
	T04 = np.dot(T03,T34)
	#Las formulas para hallar los ejes cartesianos x,y,z se guardan en px,py,pz obtenidos de la operacion anterior
	px= T04[0,3]
	py= T04[1,3]
	pz= T04[2,3]
	
	try:
		subscriber()
	except rospy.ROSInterruptException:
		pass

