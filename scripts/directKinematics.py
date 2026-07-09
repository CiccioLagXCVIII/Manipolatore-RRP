import numpy as np
import kinematicsUtils

# AA Funzione Che Calcola Le Matrici Di Trasformazione Necessarie Per Calcolare La Matrice Di Trasformazione 
def calculateTransformationMatrix(theta, d, a, alpha):
    # BB $R_z(\theta_i)$: Rotazione Attorno A $Z_{i-1}$ Di Angolo $\theta_i$ (*Variabile Di Giunto*)
    cosTheta = np.cos(theta)
    sinTheta = np.sin(theta)
    Rz_theta = np.array([
        [cosTheta, -sinTheta, 0.0, 0.0],
        [sinTheta,  cosTheta, 0.0, 0.0],
        [0.0,          0.0,         1.0, 0.0],
        [0.0,          0.0,         0.0, 1.0]
    ])
    # BB $T_z(d_i)$: Traslazione Lungo $Z_{i-1}$ Di Distanza $d_i$ (*Variabile Di Giunto Se Prismatico, Costante Se Revolute*), ovvero finché $Z_{i-1}$ e $Z_i$ sono complanari.
    Tz_d = np.array([
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, d],
        [0.0, 0.0, 0.0, 1.0]
    ])
    # BB $T_x(a_i)$: Traslazione Lungo $X_i$ Di Distanza $a_i$ (*Costante*), ovvero finché le origini dei due frame coincidono.
    Tx_a = np.array([
        [1.0, 0.0, 0.0, a],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ])
    # BB $R_x(\alpha_i)$: Rotazione Attorno A $X_i$ Di Angolo $\alpha_i$ finche $Z_{i-1}$ si allinea a $Z_i$.
    cosAlpha = np.cos(alpha)
    sinAlpha = np.sin(alpha)
    Rx_alpha = np.array([
        [1.0, 0.0,          0.0,           0.0],
        [0.0, cosAlpha, -sinAlpha, 0.0],
        [0.0, sinAlpha,  cosAlpha, 0.0],
        [0.0, 0.0,          0.0,           1.0]
    ])

    # DD Ovviamente Si Può Combinare Tutte Le Matrici In Un'unica Matrice Di Trasformazione Omogenea,
    # DD Ma Per Chiarezza Le Mantengo Singole
    # T = np.array([
    #     [cosTheta, -sinTheta * cosAlpha,  sinTheta * sinAlpha, a * cosTheta],
    #     [sinTheta,  cosTheta * cosAlpha, -cosTheta * sinAlpha, a * sinTheta],
    #     [0.0,       sinAlpha,             cosAlpha,            d],
    #     [0.0,       0.0,                  0.0,                 1.0]
    # ])
    T = Rz_theta @ Tz_d @ Tx_a @ Rx_alpha
    
    return T

# AA Funzione Che Restituisce La Matrice Di Trasformazione Omogenea
def computeTransformMatrix(q1, q2, q3):
    # BB Importazione Dei Parametri Fisici Del Robot Dal Server ROS
    kinematicsUtils.loadRobotParameters()
    l1 = kinematicsUtils.l1
    l2 = kinematicsUtils.l2
    l3 = kinematicsUtils.l3
    jointRadius = kinematicsUtils.jointRadius

    # BB Definizione Tabella DH Per Il Robot RRP
    # CC Dizionario Chiave-Valore Dove La Chiave Rappresenta Il Numero Del Link E Il Valore È La Lista Dei Parametri DH [Theta, D, A, Alpha]
    tableDH = {
        1: [q1, l1 + 2 * jointRadius, 0.0, -np.pi / 2.0],   # Da RF0 A RF1 
        2: [q2, 0.0, l2 + 2 * jointRadius, np.pi / 2.0],    # Da RF1 A RF2 
        3: [0.0, q3 - l3, 0.0, -np.pi]                      # Da RF2 A End Effector
    } 
    # L'EE Si Trova Alla Fine Del Link 3 (l3) Ma La Distanza Effettiva Dell'End Effector Da RF2 È q3 - l3 (A Riposo Vale -l3 Infatti L'EE È Verso Il Basso).
    # E Il Suo RF È Ruotato Di 90 Gradi Attorno A X Rispetto A RF2
    
    # BB Calcolo Singole Matrici Di Trasformazione
    # CC L'Operatore * Permette Di Passare Gli Elementi Della Lista Come Argomenti Separati Alla Funzione
    # DD Facendo Una Lista Di Liste E Iterando Su Di Essa, Si Può Calcolare La Matrice Di Trasformazione 
    # DD Di Ogni Lista Sempre Con * E Non È Necessario Definire Le Varibili Intermedie Ma Tanto Per Soli
    # DD Tre Link Non Cambia Niente In Termini Di Costo Computazionale
    T01 = calculateTransformationMatrix(*tableDH[1])
    T12 = calculateTransformationMatrix(*tableDH[2])
    T23 = calculateTransformationMatrix(*tableDH[3])

    # BB Calcolo Della Matrice Di Trasformazione Complessiva Dal Frame 0 Al Frame 3 (End Effector)
    T03 = T01 @ T12 @ T23

    return T03

def computeDirectKinematics(q1, q2, q3):
    
    # BB Calcolo Della Cinematica Diretta Dal Frame Zero Al Frame Tre
    # CC La Funzione All'Interno Del Modulo Restituisce La Matrice Quattro Per Quattro T03
    T03 = computeTransformMatrix(q1, q2, q3)
    
    # BB # BB Importazione Dei Parametri Fisici Del Robot Dal Server ROS
    kinematicsUtils.loadRobotParameters()
    baseHeight = kinematicsUtils.baseHeight
    worldBase = kinematicsUtils.worldBase
    
    # BB Creazione Della Matrice Di Trasformazione Omogenea Da RF0 A RF Base
    TBase0 = np.array([
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, baseHeight],
        [0.0, 0.0, 0.0, 1.0]
    ])

    # BB Creazione Della Matrice Di Trasformazione Omogenea Da RF Base A RF World
    TWorldBase = np.array([
        [1.0, 0.0, 0.0, worldBase],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ])

    # BB Calcolo Della Matrice Da World A End Effector
    TWorldEE = TWorldBase @ TBase0 @ T03

    # BB Estrazione Posizione X, Y, Z Dalla Matrice Di Trasformazione
    # CC (Primi Tre Elementi Della Quarta Colonna)
    positionWorld = TWorldEE[:3, 3]

    return TWorldEE, positionWorld