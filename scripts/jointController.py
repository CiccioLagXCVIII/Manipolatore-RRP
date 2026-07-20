#!/usr/bin/env python3

import os
import rospy
import numpy as np
from sensor_msgs.msg import JointState
import kinematicsUtils
from directKinematics import computeDirectKinematics
from inverseKinematics import computeInverseKinematics


# AA Impostazioni Stampa Di NumPy
# BB Precisione, Notazione Scientifica, Larghezza Massima Di Stampa
np.set_printoptions(precision=4, suppress=True, linewidth=120)

# AA Inizializzazione Del Nodo
rospy.init_node("jointController")

# AA Configurazione Del Publisher Per Lo Stato Dei Giunti
publisher = rospy.Publisher("/joint_states", JointState, queue_size=10)

# AA Definizione Delle Variabili Dei Giunti Iniziali
q1 = 0.0
q2 = 0.0
q3 = 0.0

# AA Posizione EE Default
# x = 2.55
# y = 0.0
# z = 0.36

# AA Pubblicazione Iniziale Per Configurare RViz Al Momento Dell Avvio
# BB Pubblica Per Un Breve Periodo Per Allineare I Frame In RViz (Non Basta Un Solo Messaggio)
for i in range(10):
    msg = kinematicsUtils.createJointStateMsg(q1, q2, q3)
    publisher.publish(msg)
    rospy.sleep(0.1)

# AA Ciclo Principale Di Input Utente Da Terminale
while not rospy.is_shutdown():
    kinematicType = input("Inserisci Il Tipo Di Cinematica (Diretta/Inversa/Esci): ").strip().lower()
        
    if kinematicType == "diretta":
        print("\nCinematica Diretta")
        try:
            # BB Acquisizione Dei Valori Inseriti Da Tastiera Per Ogni Giunto
            inputQ1 = float(input("Inserisci Il Valore Per Il Giunto 1 (q1 In Radianti): "))
            inputQ2 = float(input("Inserisci Il Valore Per Il Giunto 2 (q2 In Radianti): "))
            inputQ3 = float(input("Inserisci Il Valore Per Il Giunto 3 (q3 In Metri):    "))
        except ValueError:
            print("[ERRORE] Inserimento Non Valido. Digitare Esclusivamente Numeri.")
            continue
            
        # BB Controllo Dei Limiti Fisici E Saturazione
        q1, q2, q3 = kinematicsUtils.checkJointLimits(inputQ1, inputQ2, inputQ3)
        
        # BB Calcolo E Stampa Della Posa Dell End Effector Una Sola Volta
        tWorldEE, positionWorld = computeDirectKinematics(q1, q2, q3)
        print("\nMatrice Trasformazione Omogenea world - endEffector:")
        print(np.round(tWorldEE, 3))
        print(f"\nPosizione End Effector In World (X, Y, Z): {positionWorld}\n")
        
        # BB Pubblicazione Del Nuovo Stato Per Aggiornare RViz
        # CC (Si Potrebbe Pubblicare Più Volte Come Prima Per Sicurezza, Ma Anche Con Una Funziona)
        msg = kinematicsUtils.createJointStateMsg(q1, q2, q3)
        publisher.publish(msg)
        rospy.sleep(0.02)
        
    elif kinematicType == "inversa":
        print("\nCinematica Inversa")
        try:
            # BB Acquisizione Delle Coordinate Target Per L End Effector
            targetX = float(input("Inserisci La Coordinata Target X (In Metri): "))
            targetY = float(input("Inserisci La Coordinata Target Y (In Metri): "))
            targetZ = float(input("Inserisci La Coordinata Target Z (In Metri): "))
        except ValueError:
            print("[ERRORE] Inserimento Non Valido. Digitare Esclusivamente Numeri.")
            continue

        # BB Controllo Della Raggiungibilita Dello Spazio Di Lavoro Prima Di Calcolare I Giunti
        if not kinematicsUtils.checkWorkspace(targetX, targetY, targetZ):
            print("[ATTENZIONE] Il Target Inserito Si Trova Fuori Dallo Spazio Operativo Consentito.")
            continue

        # BB Esecuzione Dei Calcoli Per Risolvere La Cinematica Inversa
        solQ1, solQ2, solQ3 = computeInverseKinematics(targetX, targetY, targetZ)

        if solQ1 is not None:
            # CC Aggiornamento Delle Variabili Locali Dei Giunti Con I Nuovi Valori
            q1, q2, q3 = solQ1, solQ2, solQ3
            print(f"\nSoluzione Trovata:")
            print(f"\t Giunto 1 (q1): {q1:.4f} Rad")
            print(f"\t Giunto 2 (q2): {q2:.4f} Rad")
            print(f"\t Giunto 3 (q3): {q3:.4f} Metri")

            # CC Valutazione Dell Indice Di Singolarita Sulla Configurazione Raggiunta
            detJ, statusSing = kinematicsUtils.checkSingularity(q1, q2, q3)
            print(f"\t Determinante Jacobiano: {detJ:.6f} ({statusSing})")

            # CC Pubblicazione Del Nuovo Stato Dei Giunti Su ROS Per Aggiornare RViz
            msg = kinematicsUtils.createJointStateMsg(q1, q2, q3)
            publisher.publish(msg)
            rospy.sleep(0.02)
        else:
            print("[ERRORE] Impossibile Trovare Una Soluzione Valida Per I Giunti.")
    
    elif kinematicType == "esci":
        # BB Spegnimento Del Nodo ROS E Chiusura Forzata Di Tutti I Thread
        rospy.signal_shutdown("Richiesta Chiusura Utente")
        os._exit(0)

    else:
        # CC Messaggio Di Avviso Per Scelte Non Valide
        print("[ATTENZIONE] Opzione Non Valida. Riprovare.")