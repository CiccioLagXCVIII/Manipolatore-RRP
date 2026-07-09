#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import JointState
import kinematicsUtils
from directKinematics import computeDirectKinematics
# import inverseKinematics


# AA Impostazioni Stampa Di NumPy
# BB Precisione, Notazione Scientifica, Larghezza Massima Di Stampa
np.set_printoptions(precision=4, suppress=True, linewidth=120)

# AA Inizializzazione Nodo
rospy.init_node("jointController")

# AA Configurazione Del Publisher (Canale Di Comunicazione) Per Lo Stato Dei Giunti
publisher = rospy.Publisher("/joint_states", JointState, queue_size=10)
# BB Frequenza Di Aggiornamento
# CC L'Argomento Indica Il Numero Di Aggiornamenti Da Eseguire In Un Secondo
rate = rospy.Rate(50) # 50 Hz


# AA Ciclo Principale Di Esecuzione Fino Alla Terminazione Da Terminale
while not rospy.is_shutdown():
    # # AA =================== INSERIMENTO VALORI GIUNTI ===================
    # BB Definizione Posizioni Dei Giunti Nel Messaggio
    q1 = 3.14           # Valore In Radianti Per Il Giunto 1
    q2 = -0.428         # Valore In Radianti Per Il Giunto 2
    q3 = 0.15           # Valore In Metri Per Il Giunto 3

    # BB Controllo Dei Limiti Dei Giunti
    q1, q2, q3 = kinematicsUtils.checkJointLimits(q1, q2, q3)
    
    # BB Creazione Del Messaggio ROS
    msg = kinematicsUtils.createJointStateMsg(q1, q2, q3)
    
    # BB Pubblicazione Messaggio Sul Topic
    publisher.publish(msg)

    # AA =================== INIZIO CINEMATICA DIRETTA ===================
    TWorldEE, positionWorld = computeDirectKinematics(q1, q2, q3)
    print("Matrice Trasformazione Omogenea world - endEffector:")
    print(np.round(TWorldEE, 3))
    print(f"\nPosizione End Effector In World (X, Y, Z): {positionWorld}\n")

    # AA ==================== FINE CINEMATICA DIRETTA ====================

    # BB Sospensione Per Rispettare La Frequenza Impostata
    rate.sleep()