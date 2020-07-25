#!/usr/bin/env python3 
import numpy as np
from matplotlib import pyplot as plt
import rospy
from master_msgs.msg import traction_Orders,rpm
import time

#counter = 0
rpm_present = rpm()
traction_present = traction_Orders()

time_rpm = []
rpm_plot = []

time_order = []
order_plot = []
start = 0.0
#state = 0

def callback_rpm(param):
	global time_rpm,rpm_plot,start
	time_rpm.append(round(time.time()-start,4))
	rpm_plot.append(param.R0_V)
	#print(param)
	
def callback_traction(param):
    global time_order,order_plot
    time_order.append(round(time.time()-start,4))
    order_plot.append(param.rpm_l)    

def guardar():
    global time_rpm,rpm_plot
    global time_order,order_plot
    #array = np.array([time,rpm_plot])
    print(time_rpm)
    print(rpm_plot)
    print(' ')
    print(time_order)
    print(order_plot)

def node_rpm_plot():
    global rpm_present,traction_present
    global time_rpm,rpm_plot
    global start
    rospy.init_node("plotter_rpm")
    rospy.Subscriber('topic_traction_orders',traction_Orders,callback_traction)
    rospy.Subscriber('topic_rpm',rpm,callback_rpm)
    pub_traction = rospy.Publisher('topic_traction_orders',traction_Orders,queue_size=10)
    rate = rospy.Rate (10)
    start = time.time()
    a = 1
    state = 0
    rpm_act = 0
    traction_present.rpm_r = rpm_act
    traction_present.rpm_l = rpm_act
    pub_traction.publish(traction_present)
    while not rospy.is_shutdown():
        te = round(time.time()-start,4)
        if a==1:
            if state==0:
                rpm_act = 0
                if te > 5:
                    print(te)
                    state = 1
            if state==1:
                rpm_act = 50
                if te > 10:
                    print(te)
                    state = 2
            elif state==2:
                rpm_act = 100
                if te > 15:
                    print(te)
                    state = 3
            elif state==3:
                rpm_act = 150
                if te > 20:
                    print(te)
                    state = 4
            elif state==4:
                rpm_act = 200
                if te > 25:
                    print(te)
                    state = 5
            elif state==5:
                rpm_act = 255
                if te > 30:
                    print(te)
                    state = 6
            elif state==6:
                rpm_act = 200
                if te > 35:
                    print(te)
                    state = 7
            elif state==7:
                rpm_act = 150
                if te > 40:
                    print(te)
                    state = 8
            elif state==8:
                rpm_act = 100
                if te > 45:
                    print(te)
                    state = 9
            elif state==9:
                rpm_act = 50
                if te > 50:
                    print(te)
                    state = 10
            elif state==10:
                rpm_act = 0
                if te > 55:
                    print(te)
                    state = 11
            elif state==11:
                if te>60:
                    a = 0
        elif a==0:
            guardar()
            break
        traction_present.rpm_r = rpm_act
        traction_present.rpm_l = rpm_act
        pub_traction.publish(traction_present)
        rate.sleep ()
            # a = input('stop = ')
            # if int(a)==1:
            #     traction_present.rpm_l = 0
            #     traction_present.rpm_r = 0
            #     pub_traction.publish(traction_present)
            #     guardar()
        # else:
        #     traction_present.rpm_l = 0
        #     traction_present.rpm_r = 0
        #     pub_traction.publish(traction_present)

if __name__ == '__main__':
    try:
        node_rpm_plot()
    except rospy.ROSInterruptException:
        pass
