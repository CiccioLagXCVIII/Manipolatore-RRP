#!/usr/bin/env python3

import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
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
# q1 = 0.0
# q2 = 0.0
# q3 = 0.0

# AA Posizione EE Default
# x = 2.55
# y = 0.0
# z = 0.36

# AA Definizione Delle Variabili Iniziali Dei Giunti
currentQ1 = 0.0
currentQ2 = 0.0
currentQ3 = 0.0

# AA Definizione Delle Variabili Target Dei Giunti
targetQ1 = 0.0
targetQ2 = 0.0
targetQ3 = 0.0

# AA Pubblicazione Iniziale Per Configurare RViz Al Momento Dell Avvio
# BB Pubblica A 20Hz Per Allineare I Frame In RViz (Non Basta Un Solo Messaggio)
def publishJointStatesCallback(event):
    global currentQ1, currentQ2, currentQ3

    # CC Interpolazione Lineare Dei Giunti Per Evitare Salti Bruschi In RViz
    alpha = 0.15
    currentQ1 += alpha * (targetQ1 - currentQ1)
    currentQ2 += alpha * (targetQ2 - currentQ2)
    currentQ3 += alpha * (targetQ3 - currentQ3)

    msg = kinematicsUtils.createJointStateMsg(currentQ1, currentQ2, currentQ3)
    publisher.publish(msg)

rospy.Timer(rospy.Duration(0.05), publishJointStatesCallback)

# AA Ciclo Principale Di Input Utente Da Terminale
while not rospy.is_shutdown():
    try:
        kinematicType = input("Inserisci Il Tipo Di Cinematica (Diretta/Inversa/Esci): ").strip().lower()
    except (KeyboardInterrupt, EOFError):
        print("\n[INFO] Interruzione Da Tastiera. Chiusura...")
        rospy.signal_shutdown("Chiusura Da Terminale")
        break

    if not kinematicType:
        print("[ATTENZIONE] Nessun Input Rilevato. Riprovare.")
        continue
    elif kinematicType == "diretta":
        print("\n[MODALITA] Cinematica Diretta")
        try:
            # BB Acquisizione Dei Valori Inseriti Da Tastiera Per Ogni Giunto
            inputQ1 = float(input("Inserisci Il Valore Per Il Giunto 1 (q1 In Radianti): "))
            inputQ2 = float(input("Inserisci Il Valore Per Il Giunto 2 (q2 In Radianti): "))
            inputQ3 = float(input("Inserisci Il Valore Per Il Giunto 3 (q3 In Metri):    "))
        except ValueError:
            print("[ERRORE] Inserimento Non Valido. Digitare Esclusivamente Numeri.")
            continue
        except (KeyboardInterrupt, EOFError):
            print("\n[INFO] Interruzione Da Tastiera. Chiusura...")
            rospy.signal_shutdown("Chiusura Da Terminale")
            break

        # BB Controllo Dei Limiti Fisici E Saturazione
        q1, q2, q3 = kinematicsUtils.checkJointLimits(inputQ1, inputQ2, inputQ3)

        # BB Calcolo E Stampa Della Posa Dell End Effector Una Sola Volta
        tWorldEE, positionWorld = computeDirectKinematics(q1, q2, q3)
        print("\nMatrice Trasformazione Omogenea world - endEffector:")
        print(np.round(tWorldEE, 3))
        print(f"\nPosizione End Effector In World (X, Y, Z): {positionWorld}\n")

        # BB Pubblicazione Del Nuovo Stato Per Aggiornare RViz
        # CC Aggiornamento Dello Stato Globale Tramite La Funzione Che Pubblica A 20Hz
        targetQ1, targetQ2, targetQ3 = q1, q2, q3

    elif kinematicType == "inversa":
        print("\n[MODALITA] Cinematica Inversa")
        try:
            # BB Acquisizione Delle Coordinate Target Per L'End Effector
            targetX = float(input("Inserisci La Coordinata Target X (In Metri): "))
            targetY = float(input("Inserisci La Coordinata Target Y (In Metri): "))
            targetZ = float(input("Inserisci La Coordinata Target Z (In Metri): "))
        except ValueError:
            print("[ERRORE] Inserimento Non Valido. Digitare Esclusivamente Numeri.")
            continue
        except (KeyboardInterrupt, EOFError):
            print("\n[INFO] Interruzione Da Tastiera. Chiusura...")
            rospy.signal_shutdown("Chiusura Da Terminale")
            break

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

            # BB Pubblicazione Del Nuovo Stato Per Aggiornare RViz
            # CC Aggiornamento Dello Stato Globale Tramite La Funzione Che Pubblica A 20Hz
            targetQ1, targetQ2, targetQ3 = q1, q2, q3
        else:
            print("[ERRORE] Impossibile Trovare Una Soluzione Valida Per I Giunti.")

    elif kinematicType == "esci":
        # BB Spegnimento Del Nodo ROS E Chiusura Forzata Di Tutti I Thread
        rospy.signal_shutdown("Richiesta Chiusura Utente")
        break

    else:
        # CC Messaggio Di Avviso Per Scelte Non Valide
        print("[ATTENZIONE] Opzione Non Valida. Riprovare.")
