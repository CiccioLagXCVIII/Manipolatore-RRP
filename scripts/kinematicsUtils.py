# AA ========================================== Funzioni Di Utilita Per I Giunti E I Messaggi ==========================================

import rospy
import numpy as np
import tf.transformations as tft
from sensor_msgs.msg import JointState

# AA Definizione Dei Parametri Fisici Del Robot Come Variabili Globali 
# BB Siccome Poi Chiamo La Funzione loadRobotParameters() Per Caricare I Parametri Dal Server ROS
# BB Verranno Sovrascritti, Ma Comunque È Necessario Dichiararli Come Variabili Globali Per Evitare Errori
# CC Questi Valori Per I Parametri Fisici Del Robot (Che Poi Sono Quelli Veri) Sono Definiti Solo Per Sicurezza
# CC Se Si Modifica Il File Di Configurazione YAML Non Sarà Necessario Modificarle Manualmente
# BB Base
baseWidth = 0.3             # Larghezza Base
baseLength = 0.3            # Lunghezza Base
baseHeight = 0.06           # Altezza Base
worldBase = 2.0             # Distanza World Base

# BB Link
l1 = 0.55                   # Lunghezza Link 1
l2 = 0.45                   # Lunghezza Link 2
l3 = 0.35                   # Lunghezza Link 3
linkRadius = 0.035          # Raggio Link (Diametro 7cm)

# BB Giunti
jointRadius = 0.05          # Raggio Sfere Giunti Revolute (Diametro 10cm)
boxSize = 0.10              # Dimensione Box Giunto Prismatico (Lato 11cm)

# BB End Effector
eeBaseWidth = 0.05          # Larghezza Base Pinza 5 cm
eeBaseLength = 0.10         # Lunghezza Totale Base Pinza 10 cm
eeBaseHeight = 0.015        # Spessore Piastra Base Pinza 1.5cm
eeFingerLength = 0.05       # Lunghezza Dita Pinza 5cm
eeFingerThickness = 0.01    # Spessore Singolo Dito Pinza 1cm


# AA Funzione Che Carica I Parametri Fisici Del Robot Dal Server ROS E Li Rende Disponibili Come Variabili Globali
def loadRobotParameters():
    global baseWidth, baseLength, baseHeight, worldBase
    global l1, l2, l3, linkRadius
    global jointRadius, boxSize
    global eeBaseWidth, eeBaseLength, eeBaseHeight, eeFingerLength, eeFingerThickness

    # BB Importazione Dei Parametri Fisici Del Robot Dal Server ROS
    try:
        # CC Lettura Dal Server Con Valore Di Default Di Sicurezza Se La Chiave Non Esiste
        baseWidth = rospy.get_param("/baseWidth", 0.3)      # Larghezza Base
        baseLength = rospy.get_param("/baseLength", 0.3)    # Lunghezza Base
        baseHeight = rospy.get_param("/baseHeight", 0.06)   # Altezza Base
        worldBase = rospy.get_param("/worldBase", 2.0)      # Distanza World Base

        l1 = rospy.get_param("/l1", 0.55)
        l2 = rospy.get_param("/l2", 0.45)
        l3 = rospy.get_param("/l3", 0.35)
        linkRadius = rospy.get_param("/linkRadius", 0.035)

        jointRadius = rospy.get_param("/jointRadius", 0.05)
        boxSize = rospy.get_param("/boxSize", 0.10)

        eeBaseWidth = rospy.get_param("/eeBaseWidth", 0.05)
        eeBaseLength = rospy.get_param("/eeBaseLength", 0.10)
        eeBaseHeight = rospy.get_param("/eeBaseHeight", 0.015)
        eeFingerLength = rospy.get_param("/eeFingerLength", 0.05)
        eeFingerThickness = rospy.get_param("/eeFingerThickness", 0.01)

    except Exception as e:
        rospy.logerr(f"Errore Durante Il Caricamento Dei Parametri Del Robot: {e}")
        rospy.logwarn("Impostati I Valori Di Default Per I Parametri Del Robot")

# BB Limiti
# Giunto 1: Da -3.142 a 3.142 (360 gradi)
# Giunto 2: Da -1.571 a 0.785 (Non Serve Andare Oltre La Verticale Per Evitare Ridondanza)
# Giunto 3: Da 0 a L3 - Lunghezza Dita Pinza (Per Evitare Che La Pinza Entri Nel Box)

# AA Funzione Che Verifica Se I Valori Dei Giunti Superano I Limiti E Li Modifica Se Necessario
def checkJointLimits(q1, q2, q3):
    # BB # BB Importazione Dei Parametri Fisici Del Robot Dal Server ROS
    loadRobotParameters()

    # BB Controllo Del Giunto Uno
    # CC Verifica E Limita Il Primo Giunto Rotatorio Tra Meno Pi Greco E Piu Pi Greco
    if q1 < -np.pi:
        rospy.logwarn("Valore Giunto 1 Inferiore Al Limite Minimo [-3.142]")
        q1 = -np.pi
    elif q1 > np.pi:
        rospy.logwarn("Valore Giunto 1 Superiore Al Limite Massimo [3.142]")
        q1 = np.pi

    # BB Controllo Del Giunto Due
    # CC Verifica E Limita Il Secondo Giunto Rotatorio Tra Meno Pi Greco Mezzi E Pi Greco Quarti
    if q2 < -(np.pi/2):
        rospy.logwarn("Valore Giunto 2 Inferiore Al Limite Minimo [-1.571]")
        q2 = -(np.pi/2)
    elif q2 > (np.pi/4):
        rospy.logwarn("Valore Giunto 2 Superiore Al Limite Massimo [0.785]")
        q2 = (np.pi/4)

    # BB Controllo Del Giunto Tre
    # CC Verifica E Limita Il Giunto Prismatico Tra Zero E La Lunghezza Libera Dell'Asta
    if q3 < 0:
        rospy.logwarn("Valore Giunto 3 Inferiore Al Limite Minimo [0]")
        q3 = 0.0
    elif q3 > (l3 - eeFingerLength):
        rospy.logwarn(f"Valore Giunto 3 Superiore Al Limite Massimo [ {l3 - eeFingerLength} ]")
        q3 = l3 - eeFingerLength

    q1, q2, q3 = float(q1), float(q2), float(q3)

    return q1, q2, q3

# AA Funzione Che Genera Il Messaggio ROS Con I Nomi E Le Posizioni Validati In Formato JointState
def createJointStateMsg(q1, q2, q3):
    # AA Definzione Del Messaggio Per JointState
    msg = JointState()
    # BB Messaggio Di Joint State
    # std_msgs/Header header
    #   uint32 seq
    #   time stamp
    #   string frame_id
    # string[]  name
    # float64[] position
    # float64[] velocity
    # float64[] effort
    # BB Definizione Timestamp Header Del Messaggio
    # CC seq E frame_id Non Sono Necessari In Questo Caso Perchè
    msg.header.stamp = rospy.Time.now()
    
    # BB Definizione Nomi Dei Giunti Coerenti Con Il File URDF
    # CC giuntoWorld e giuntoEE Sono Fissi E Non Ricevono Valori Quindi Non È Necessario Includerli
    msg.name = ["giunto1", "giunto2", "giunto3"]

    # BB Definizione Posizioni Dei Giunti Nel Messaggio
    # CC A Questo Punto È Già Stato Verificato Che I Valori Dei Giunti Siano All'Interno Dei Limiti
    msg.position = [q1, q2, q3]

    # BB Definizione Velocità Dei Giunti Nel Messaggio
    # CC Opzionale: Se Si Vuole Pubblicare È Necessario Che Abbiano La Stessa Lunghezza Di Position (Un Valore Per Ogni Giunto)
    msg.velocity = []

    # BB Definizione Sforzi Dei Giunti Nel Messaggio
    # CC Opzionale: Se Si Vuole Pubblicare È Necessario Che Abbiano La Stessa Lunghezza Di Position (Un Valore Per Ogni Giunto)
    msg.effort = []

    return msg

# AA Funzione Che Calcola La Matrice Di Trasformazione Omogenea 4x4 A Partire Dal Vettore Di Traslazione E Dal Quaternione Di Rotazione
def getTransformationMatrix(translation, rotation):
    
    # BB Conversione Vettore Di Traslazione E Quaternione In Array NumPy
    translationVector = np.array([translation.x, translation.y, translation.z])
    quaternionVector = np.array([rotation.x, rotation.y, rotation.z, rotation.w])

    # BB Calcolo Matrice Di Rotazione 3x3 A Partire Dal Quaternione
    quaternionMatrix = tft.quaternion_matrix(quaternionVector)
    rotationMatrix   = quaternionMatrix[:3, :3]

    # print("\nMatrice Di Rotazione 3x3:\n")
    # print(np.round(rotationMatrix, 4)) 

    # print("\nVettore Di Traslazione:\n")
    # print(np.round(translationVector, 4))

    # BB Calcolo Matrice Di Trasformazione Omogenea 4x4
    transformationMatrix = np.identity(4)
    # CC Matrice Di Rotazione Nelle Prime Tre Righe E Tre Colonne Colonne
    transformationMatrix[:3, :3] = rotationMatrix
    # CC Vettore Di Traslazione Nelle Prime Tre Righe Dell'Ultima Colonna
    transformationMatrix[:3, 3] = translationVector
    # CC L'Ultima Riga Resta [0, 0, 0, 1] Perché È Una Matrice Omogenea

    return transformationMatrix