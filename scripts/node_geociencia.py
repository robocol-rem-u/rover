#!/usr/bin/env python
import rospy
from master_msgs.msg import geociencia
from master_msgs.srv import service_enable
import serial
import threading
import time
import numpy as np
import serial
import time

ser=serial.Serial(port='/dev/serial/by-id/usb-Arduino_Srl_Arduino_Uno_55632313838351C04022-if00',baudrate=9600)

##msgs de diferentes tipos para los topicos y publicar###
geo = geociencia()

msg = ''.encode()
x=''
Met=''
Hum=''
T=''
H2=''


### NODO PRINCIPAL ###
def node_geociencia():
    global pub_geociencia
    global x,Met,Hum,T,H2,geo, msg
    #creacion del nodo
    rospy.init_node('node_geociencia',anonymous=True)
    # publica en RPM, Current y POTS
    pub_geociencia = rospy.Publisher('topic_geociencia', geociencia, queue_size=10)
    rate = rospy.Rate (10)
    while not rospy.is_shutdown ():
        s=ser.read() #reading up to 100 bytes
        if s == '#'.encode():
            t = msg.decode('utf-8')
            #print(t)
            #print(msg.decode('utf-8'))
            msg = ''.encode()
            x= t.split()
            print(x)
            geo.tempratura=np.float32(x[0])
            geo.humedad=np.float32(x[1])
            geo.metano=np.float32(x[2])
            geo.hidrogeno=np.float32(x[3])
            pub_geociencia.publish(geo)
        else:
            msg+=s
        rate.sleep ()


if __name__ == '__main__':
    try:
        node_geociencia()
    except rospy.ROSInterruptException:
        ser.close()
        pass
