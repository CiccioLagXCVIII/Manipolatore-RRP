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
    # CC Verifica E Limita Il Giunto Prismatico Tra La Metà Della Dimensione Del Box E La Lunghezza Libera Dell'Asta
    if q3 < 0:
        rospy.logwarn(f"Valore Giunto 3 Inferiore Al Limite Minimo [ {boxSize/2} ]")
        q3 = boxSize/2
    elif q3 > (l3):
        rospy.logwarn(f"Valore Giunto 3 Superiore Al Limite Massimo [ {l3} ]")
        q3 = l3

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

# AA Funzione Che Controlla Se Il Target È Raggiungibile All'Interno Del Workspace Del Robot
# AA Funzione Che Controlla Se Il Target È Raggiungibile All'Interno Del Workspace Del Robot
def checkWorkspace(xTarget, yTarget, zTarget):
    # BB Caricamento Dei Parametri Geometrici Del Robot Dal Parameter Server
    loadRobotParameters()

    # CC Definizioni Valori Della Tabella Di Denavit Hartenberg
    d1 = 3 * jointRadius + l1
    a2 = jointRadius + l2 + (boxSize / 2.0)

    # BB Calcolo Dei Raggi Della Circonferenza
    rDMax = np.sqrt(a2**2 + (l3 + (boxSize / 2.0))**2)
    rDMin = np.sqrt(a2**2 + (boxSize / 2.0)**2)

    # BB Calcolo Coordinate Del Centro Del Secondo Giunto Nel Frame Globale World
    xShoulder = worldBase
    yShoulder = 0.0
    zShoulder = baseHeight + d1

    # CC Calcolo Distanza Euclidea Tra Il Giunto 2 E End Effector
    rD = np.sqrt((xTarget - xShoulder)**2 + (yTarget - yShoulder)**2 + (zTarget - zShoulder)**2)

    # BB Verifica Se La Distanza Calcolata Rientra Nei Limiti Del Workspace Del Robot
    # CC Il Valore Di Ritorno Sara True Se Il Target E Raggiungibile Altrimenti False
    if rD < rDMin:
        rospy.logwarn(f"Target Troppo Vicino Al Giunto 2 Del Robot: Distanza Calcolata {rD:.4f} Inferiore A {rDMin:.4f}")
        return False
    elif rD > rDMax:
        rospy.logwarn(f"Target Fuori Dal Raggio Di Estensione Massimo: Distanza Calcolata {rD:.4f} Superiore A {rDMax:.4f}")
        return False

    # CC Rimossa La Verifica Sulla Distanza Orizzontale Minore Di a2 Perche Il Giunto Di Spalla
    # DD Permette Di Ruotare Di Beccheggio Riducendo La Proiezione Orizzontale Fino A Valori Molto Piccoli

    return True


def checkSingularity(q1, q2, q3):
    # BB Caricamento Dei Parametri Geometrici Del Robot Dal Parameter Server
    loadRobotParameters()

    # CC Definizione Dei Parametri Della Tabella Di Denavit Hartenberg
    a2 = jointRadius + l2 + (boxSize / 2.0)
    d3 = q3 - (l3 + (boxSize / 2.0))

    # BB Calcolo Determinante Della Matrice Jacobiana
    # CC Equazione Determinante Calcolata Nel File "5) cinematicaInversaRRP.pdf"
    detJ = -d3 * (d3 * np.sin(q2) + a2 * np.cos(q2))

    # CC Analisi Delle Singolarita Geometriche
    # DD Condizione 1: d_3 = 0
    firstCond = np.abs(d3)
    # DD Condizione 2: d_3 \sin(q_2) + a_2 \cos(q_2) = 0
    secondCond = np.abs(d3 * np.sin(q2) + a2 * np.cos(q2))

    # BB Logic Per Identificare Vicinanza A Configurazioni Singolari
    # CC Soglia Di Tolleranza Definizione Di Condizione Singolare
    tolerance = 1e-3
    singularityStatus = "Sicuro"

    if firstCond < tolerance:
        singularityStatus = "Singolarita Giunto Prismatico d3 Vicino A Zero"
        rospy.logwarn(f"Attenzione: Robot Vicino A Singolarita Prismatico Con d3 = {d3:.4f}")
    elif secondCond < tolerance:
        singularityStatus = "Singolarita Giunto 2 Con Raggio Vicino A Zero"
        rospy.logwarn(f"Attenzione: Robot Vicino A Singolarita Di Allineamento Con Raggio = {secondCond:.4f}")

    return detJ, singularityStatus