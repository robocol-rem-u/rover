#!/usr/bin/env python
import rospy
from master_msgs.msg import pots
from master_msgs.msg import arm_Orders

def pots_Callback(param):
	pass




def node_Arm_Traction():
    
    rospy.init_node('node_Arm_Traction',anonymous=True)
    # Se suscribe a al topico de la informacion de la conexion sobre quien manda informacion
    rospy.Subscriber ('topic_Pots', pots, pots_Callback)
    #Se publica ordenes sobre las ordenes del brazo
    pub_Arm_Orders = rospy.Publisher ('topic_Arm_Orders', arm_Orders, queue_size=10)


    rate = rospy.Rate (10)
    while not rospy.is_shutdown ():
        rate.sleep ()




if __name__ == '__main__':
    try:

        node_Arm_Traction()
        rate = rospy.Rate (10)
        while not rospy.is_shutdown ():
            rate.sleep ()
    except rospy.ROSInterruptException:
        pass
