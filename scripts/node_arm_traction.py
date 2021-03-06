#!/usr/bin/env python
import rospy
from master_msgs.msg import pots
from master_msgs.msg import arm_Orders


def node_arm_traction():
    rospy.init_node('node_arm_traction',anonymous=True)
    # Se suscribe a al topico de la informacion de la conexion sobre quien manda informacion
    rospy.Subscriber ('topic_pots', pots, pots_Callback)
    #Se publica ordenes sobre las ordenes del brazo
    pub_Arm_Orders = rospy.Publisher ('topic_arm_orders', arm_Orders, queue_size=10)
    rate = rospy.Rate (10)
    while not rospy.is_shutdown ():
        rate.sleep ()


def pots_Callback(param):
	pass

if __name__ == '__main__':
    try:

        node_arm_traction()
        rate = rospy.Rate (10)
        while not rospy.is_shutdown ():
            rate.sleep ()
    except rospy.ROSInterruptException:
        pass
