#!/usr/bin/env python
import rospy
from master_msgs.msg import goal, position, traction_Orders


def node_autonomous_traction():
    # Creacion del nodo
    rospy.init_node('node_autonomous_traction',anonymous=True)
    #Se subscribe al topico del Goal y al de la posicion
    rospy.Subscriber ('topic_Goal', goal, goal_Callback)
    rospy.Subscriber ('topic_Position', position, position_Callback)
    # publica en el topico traction orders
    pub_Traction_Orders = rospy.Publisher('topic_Traction_Orders', traction_Orders, queue_size=10)
    rate = rospy.Rate (10)
    while not rospy.is_shutdown ():
        rate.sleep ()


def goal_Callback(param):
    pass

def position_Callback(param):
    pass


if __name__ == '__main__':
    try:

        node_autonomous_traction()
    except rospy.ROSInterruptException:
        pass
