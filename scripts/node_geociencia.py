#!/usr/bin/env python
import rospy
from master_msgs.msg import geociencia
from master_msgs.srv import service_enable
import serial
import threading
import time
import numpy as np


##msgs de diferentes tipos para los topicos y publicar###
geo = geociencia()





### NODO PRINCIPAL ###
def node_geociencia():
    global pub_geociencia
    #creacion del nodo
    rospy.init_node('node_geociencia',anonymous=True)
    # publica en RPM, Current y POTS
    pub_geociencia = rospy.Publisher('topic_geociencia', geociencia, queue_size=10)
    rate = rospy.Rate (10)
    while not rospy.is_shutdown ():
        rate.sleep ()




if __name__ == '__main__':
    try:
        node_geociencia()
    except rospy.ROSInterruptException:
        pass
