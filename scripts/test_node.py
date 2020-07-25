#!/usr/bin/env python3
import rospy
from master_msgs.msg import pots

def callback(param):
	print('Si')

def node_ros():
	# Se inicia el nodo de odometria
    rospy.init_node ('node_imu', anonymous=True)
    rospy.Subscriber('topic_pots',pots,callback)
    rate = rospy.Rate (10)
    while not rospy.is_shutdown ():
        rate.sleep ()

if __name__ == '__main__':
    try:
    	node_ros()
    except rospy.ROSInterruptException:
        pass
