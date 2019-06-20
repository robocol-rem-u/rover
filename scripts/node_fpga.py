#!/usr/bin/env python
import rospy
from master_msgs.msg import traction_Orders, connection, arm_Orders, rpm, current,pots,sensibility


def node_fpga():
    #creacion del nodo
    rospy.init_node('node_fpga',anonymous=True)

    #se subscribe al topico traction orders
    rospy.Subscriber ('topic_Traction_Orders', traction_Orders, traction_Orders_Callback)
    #se subscribe al topico connection
    rospy.Subscriber ('topic_Connection', connection, connection_Callback)
    # se subscribe al topico Arm Orders
    rospy.Subscriber ('topic_Arm_Orders', arm_Orders, arm_Orders_Callback)
    #se subscribe a sensibility
    rospy.Subscriber ('topic_Sensibility', sensibility, sensibility_Callback)   
    # publica en RPM, Current y POTS
    pub_RPM = rospy.Publisher('topic_RPM', rpm, queue_size=10)
    pub_Current = rospy.Publisher('topic_Current', current, queue_size=10)
    pub_Pots= rospy.Publisher('topic_Pots',pots,queue_size=10)
    rate = rospy.Rate (10)
    while not rospy.is_shutdown ():
        rate.sleep ()

def traction_Orders_Callback(param):
    pass

def connection_Callback(param):
    pass

def arm_Orders_Callback(param):
    pass

def sensibility_Callback(param):
    pass



if __name__ == '__main__':
    try:

        node_fpga()

    except rospy.ROSInterruptException:
        pass
