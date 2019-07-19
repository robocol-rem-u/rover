#!/usr/bin/env python
import rospy
from master_msgs.msg import traction_Orders, connection, arm_Orders, rpm, current,pots,sensibility
from master_msgs.srv import service_enable
import serial
import threading
import time
import numpy as np


#Variables globales
global latitude, longitude, azimuth, lineal_speed, steering_speed
global L0_speed, L1_speed, L2_speed, R0_speed, R1_speed, R2_speed
global L0_current, L1_current, L2_current, R0_current, R1_current, R2_current
global rover_temp, bat0, bat1, bat2, bat3
global joint0, joint1, joint2, joint3, joint4, joint5, joint6
ultimo_izquierdo=999
ultimo_derecho=999
EnviarMensaje=True #varibale de incilizacion
almacenar=True
almacenar2=True
start_motor=0

SEPARADOR_POSITIVO = "#"
SEPARADOR_NEGATIVO = "!"
inicio_rec = False #Booleano para iniciar thread de fpga recibir valores
##msgs de diferentes tipos para los topicos y publicar###
rpm_present = rpm()
current_present = current()
pots_present = pots()
traction_present=traction_Orders()
arm_present=arm_Orders()

# Conexion serial a la FPGA
ser = serial.Serial(port='/dev/ttyTHS2', baudrate = 115200)
# Pin de MUX en la FPGA



### NODO PRINCIPAL ###
def node_fpga():
    global pub_RPM,pub_Current,pub_Pots, inicio_rec,start_motor
    #creacion del nodo
    rospy.init_node('node_fpga',anonymous=True)
    #se subscribe al topico traction orders
    rospy.Subscriber ('topic_traction_orders', traction_Orders, traction_Orders_Callback)
    #se subscribe al topico connection
   # rospy.Subscriber ('topic_connection', connection, connection_Callback)
    # se subscribe al topico Arm Orders
    rospy.Subscriber ('topic_arm_orders', arm_Orders, arm_Orders_Callback)
    # publica en RPM, Current y POTS
    pub_RPM = rospy.Publisher('topic_rpm', rpm, queue_size=10)
    pub_Current = rospy.Publisher('topic_current', current, queue_size=10)
    pub_Pots= rospy.Publisher('topic_pots',pots,queue_size=10)
    threading.Thread(target=enviarMensajeInicializacion).start()
    enable = rospy.Service('service_enable', service_enable, handle_enable)
    threading.Thread(target=StartServerFPGA).start()

    rate = rospy.Rate (10)
    while not rospy.is_shutdown ():
        rate.sleep ()

def enviarMensajeInicializacion():
    global EnviarMensaje
    while EnviarMensaje:
        nMsg = 1  # 3
        print("Enviando Inicializacion")
        WriteFPGA("A" + str(nMsg) + "#I0#I1#I2#I3#I4#I5#")
        time.sleep(1)




###NODOS PARA TOPICOS ###
def traction_Orders_Callback(param):
    global traction_present
    traction_present=param
    procesarJoystick(traction_present.rpm_l, traction_present.rpm_r)
    pass

#def connection_Callback(param):
  #  pass

def arm_Orders_Callback(param):
    global arm_present
    arm_present=param.message
    almacenar=WriteFPGA(arm_present)
    pass


def handle_enable(param):
    mensage=param.message
    almacenar2=WriteFPGA(mensage)
    return []

###METODOS EXTERNOS A ROS####



def procesarJoystick(RPM_I, RPM_D):
    global EnviarMensaje, ultimo_izquierdo, ultimo_derecho


    (calc_RPM_izq, calc_RPM_der) = (int(RPM_I), int(RPM_D))

    StringIzquierda = ("L" + str(calc_RPM_izq) + SEPARADOR_NEGATIVO) if (calc_RPM_izq >= 0) else (
    "L" + str(-calc_RPM_izq) + SEPARADOR_POSITIVO)
    StringDerecha = ("R" + str(calc_RPM_der) + SEPARADOR_NEGATIVO) if (calc_RPM_der >= 0) else (
    "R" + str(-calc_RPM_der) + SEPARADOR_POSITIVO)

    MensajeSeguridadMotores = ""

    if np.sign(ultimo_izquierdo) != np.sign(calc_RPM_izq) and calc_RPM_izq != 0 and np.sign(ultimo_izquierdo) != 0:
        MensajeSeguridadMotores += "L0#"

    if np.sign(ultimo_derecho) != np.sign(calc_RPM_der) and calc_RPM_der != 0 and np.sign(ultimo_derecho) != 0:
        MensajeSeguridadMotores += "R0#"

    if np.abs(ultimo_izquierdo - calc_RPM_izq) > 0.1 or np.abs(ultimo_derecho - calc_RPM_der) > 0.1 or (
            calc_RPM_der == 0 and ultimo_derecho != 0) or (calc_RPM_izq == 0 and ultimo_izquierdo != 0):
        EnviarMensaje=not WriteFPGA(MensajeSeguridadMotores + StringIzquierda + StringDerecha)

        ultimo_izquierdo = calc_RPM_izq
        ultimo_derecho = calc_RPM_der



# Recepcion de datos de la base a la Rpi y reenvio a la FPGA, manejo del MUX
def WriteFPGA(rcv):
    ser.write(rcv.encode())
    return True


##METODOS PARA RECIBIR INFORMACION POR SERIAL ###
#Recibir datos de la FPGA por serial
def StartServerFPGA():

    global L0_speed, L1_speed, L2_speed, R0_speed, R1_speed, R2_speed
    global L0_current, L1_current, L2_current, R0_current, R1_current, R2_current
    global joint0, joint1, joint2, joint3, joint4, joint5, joint6
    global pub_RPM, pub_Current, pub_Pots,rpm_present,current_present,pots_present

    line = ""

    while(True):
        try:
            received = ser.read().decode()
            line += received

            if received == "!" or received == "#":

                signo = 1 if received=="!" else -1
                numero = signo * int(line[1:5])
                codigo = line[0]
                ## Se define que tipo de mensaje esta llegando y a lo que corresponde
                if codigo == 'A':
                    #L0_current = numero
                    current_present.L0_C=numero
                elif codigo == 'B':
                    #L1_current = numero
                    current_present.L1_C = numero
                elif codigo == 'C':
                    #L2_current = numero
                    current_present.L2_C = numero
                elif codigo == 'D':
                    #R0_current = numero
                    current_present.R0_C = numero
                elif codigo == 'E':
                    #R1_current = numero
                    current_present.R1_C = numero
                elif codigo == 'F':
                    #R2_current = numero
                    current_present.R2_C = numero
                elif codigo == 'G':
                    #joint0 = numero
                    pots_present.J0 = numero
                elif codigo == 'H':
                    #joint1 = numero
                    pots_present.J1 = numero
                elif codigo == 'I':
                    #joint2 = numero
                    pots_present.J2 = numero
                elif codigo == 'J':
                    #joint3 = numero
                    pots_present.J3 = numero
                elif codigo == 'K':
                    #joint4 = numero
                    pots_present.J4 = numero
                elif codigo == 'L':
                    #joint5 = numero
                    pots_present.J5 = numero
                elif codigo == 'M':
                    #joint6 = numero
                    pots_present.J6 = numero
                elif codigo == 'N':
                    #L0_speed = numero
                    rpm_present.L0_V = numero
                elif codigo == 'O':
                    #L1_speed = numero
                    rpm_present.L1_V = numero
                elif codigo == 'P':
                    #L2_speed = numero
                    rpm_present.L2_V = numero
                elif codigo == 'Q':
                    #R0_speed = numero
                    rpm_present.R0_V = numero
                elif codigo == 'R':
                    #R1_speed = numero
                    rpm_present.R1_V = numero
                elif codigo == 'S':
                    #R2_speed = numero
                    rpm_present.R2_V = numero
                line = ""
            pub_Current.publish(current_present)
            pub_Pots.publish(pots_present)
            pub_RPM.publish(rpm_present)


        except:
            line=""





if __name__ == '__main__':
    try:
        node_fpga()
    except rospy.ROSInterruptException:
        pass
