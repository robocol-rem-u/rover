#!/usr/bin/env python
#Se importan las librerias necesarias junto con los mensajes a utilizar
import rospy, math, roslaunch, time
from master_msgs.msg import imu_Speed, imu_Magnetism, rpm, position
import numpy as np
from nav_msgs.msg import Odometry


odom = Odometry()

#El primero es la velocidad de la rueda izquierda y el segundo es la velocidad de la derecha.
vel = [0, 0, 0, 0, 0, 0]

#Son las constantes utilizadas para calcular la posicion y orientacion estimada del robot en cada momento.
dt = 0
dSl = 0
dSr = 0
dS = 0
dScos=0
dSsin=0
cose=0
seno=0

#Averiguar que es b. Por ahora en 1.
b = float(2*50)

#Constantes de la matriz de covarianza. Por ahora en 1.
kr = 0.1
kl = 0.1

#Theta. Es la orientacion del robot con respecto al eje z.
O = 0
dO = 0

#Es la tasa en Hz del nodo.
h = 10

#Covarianza
Covarianza=np.zeros((3,3))
CovarSrSl=np.zeros((2,2))

Fpt1=np.zeros((3,3))
Fpt1trans=np.transpose(Fpt1)
FdS=np.zeros((3,2))
FdStrans=np.transpose(Fpt1)

#  En esta variable se almacenan los ultimos 2 valores de tiempo.
t = [0, 0]

#  Indica cuando comenzar
empezar = False

def node_odometry():
    global pos
    # Se inicia el nodo de odometria
    rospy.init_node ('node_odometry', anonymous=True)
    # Se suscribe a al topico de la informacion de la velocidad segun IMU
    #rospy.Subscriber ('topic_imu_speed', imu_Speed, IMU_Speed_Callback)
    # Se suscribe a al topico de la informacion de RPM
    rospy.Subscriber ('topic_rpm', rpm, RPM_Callback)
    # Se suscribe a al topico de la informacion de 
    #rospy.Subscriber ('topic_imu_magnetism', imu_Magnetism, IMU_Magnetism_Callback)
    # Se crea referencia a topico para publicar posiciones
    pub_Position = rospy.Publisher ('odom', Odometry, queue_size=10)
    rate = rospy.Rate (10)
    t = [time.time(), time.time()]
    odom.pose.orientation.w=0
    odom.pose.position.x=0
    odom.pose.position.y=0

    while not rospy.is_shutdown ():
        rate.sleep ()

def RPM_Callback(msg):
    global pos, cov, vel, t, dt, dS, dSl, dSr, O, dO, Covarianza, pub_Position, pubCov, CovarSrSl, CovarSrSl, Fpt1, Fpt1trans, FdS, FdStrans, cose, seno, dScos, dSsin
    t.append(time.time())
    t = t[-2:]
    dt = (t[1]-t[0])
    dSl = (vel[0]+vel[1]+vel[2])*dt
    dSr = (vel[3]+vel[4]+vel[5])*dt
    dS = (dSl+dSr)/6
    dO = (dSr-dSl)/b
    O = odom.pose.orientation.w
    cose = math.cos(O+dO/2.0)
    seno = math.sin(O+dO/2.0)
    dScos = dS*cose
    dSsin = dS*seno
    odom.pose.position.x = odom.pose.position.x + dScos
    odom.pose.position.y = odom.pose.position.y + dSsin
    odom.pose.orientation.w = O + dO
    while odom.pose.orientation.w < -math.pi:
        odom.pose.orientation.w = odom.pose.orientation.w + 2*math.pi
    while odom.pose.orientation.w > math.pi:
        odom.pose.orientation.w = odom.pose.orientation.w - 2*math.pi
    CovarSrSl = np.array([[kr*np.absolute(dSr), 0], [0, kl*np.absolute(dSl)]])
    Fpt1 = np.array([[1.0, 0, -dSsin], [1.0, 0, dScos], [0, 0, 1.0]])
    Fpt1trans = Fpt1.transpose()
    FdS = np.array([[(1.0/2.0)*cose-(1.0/(2.0*b))*dSsin, (1.0/2.0)*cose+(1.0/(2.0*b))*dSsin], [(1.0/2.0)*seno+(1.0/(2.0*b))*dScos, (1.0/2.0)*seno-(1.0/(2.0*b))*dScos], [1.0/b, -(1.0/b)]])
    FdStrans = FdS.transpose()
    Covarianza = np.dot((np.dot(Fpt1, Covarianza)), Fpt1trans) + np.dot(np.dot(FdS, CovarSrSl), FdStrans)
    #cov.sigma11 = Covarianza[0, 0]
    #cov.sigma12 = Covarianza[0, 1]
    #cov.sigma13 = Covarianza[0, 2]
    #cov.sigma21 = Covarianza[1, 0]
    #cov.sigma22 = Covarianza[1, 1]
    #cov.sigma23 = Covarianza[1, 2]
    #cov.sigma31 = Covarianza[2, 0]
    #cov.sigma32 = Covarianza[2, 1]
    #cov.sigma33 = Covarianza[2, 2]
    pub_Position.publish(odom)
    vel = [msg.L0_V,msg.L1_V,msg.L2_V,msg.R0_V,msg.R1_V,msg.R2_V]

#def IMU_Speed_Callback(param):
 #   pass



#def IMU_Magnetism_Callback(param):
    #pass

# Metodo principal, crea el nodo de ROS, se suscribe a topico de informacion obstaculos e imprime su informacion
# mientras que el nodo se este ejecutando
if __name__ == '__main__':
    try:
    	node_odometry()
    except rospy.ROSInterruptException:
        pass
