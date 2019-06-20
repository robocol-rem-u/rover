#!/usr/bin/env python
#Se importan las librerias necesarias junto con los mensajes a utilizar
import rospy
from master_msgs.msg import IMU_Speed, imu_Magnetism, rpm, position



def IMU_Speed_Callback(param):
    pass

def RPM_Callback(param):
    pass


def IMU_Magnetism_Callback(param):
    pass


def node_Odometry():
    # Se inicia el nodo de odometria
    rospy.init_node ('node_Odometry', anonymous=True)
    # Se suscribe a al topico de la informacion de la velocidad segun IMU
    rospy.Subscriber ('topic_IMU_Speed', IMU_Speed, IMU_Speed_Callback)
    # Se suscribe a al topico de la informacion de RPM
    rospy.Subscriber ('topic_RPM', RPM, RPM_Callback)
    # Se suscribe a al topico de la informacion de 
    rospy.Subscriber ('topic_IMU_Magnetism', IMU_Magnetism, IMU_Magnetism_Callback)
    # Se crea referencia a topico para publicar posiciones
    pub_Position = rospy.Publisher ('topic_Position', position, queue_size=10)
    rate = rospy.Rate (10)
    while not rospy.is_shutdown ():
        rate.sleep ()

# Metodo principal, crea el nodo de ROS, se suscribe a topico de informacion obstaculos e imprime su informacion
# mientras que el nodo se este ejecutando
if __name__ == '__main__':
    try:
    	node_Odometry()
    except rospy.ROSInterruptException:
        pass
