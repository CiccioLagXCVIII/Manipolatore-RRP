#!/usr/bin/env python3

# AA Spiegazione Approfondita Nel File "5) Cinematica Inversa RRP"

import rospy
import numpy as np
import kinematicsUtils

def computeInverseKinematics(xTarget, yTarget, zTarget):
    # BB Caricamento Dei Parametri Geometrici Del Robot Dal Parameter Server
    kinematicsUtils.loadRobotParameters()
    
    # CC Assegnazione Delle Variabili
    worldBase = kinematicsUtils.worldBase
    baseHeight = kinematicsUtils.baseHeight
    l1 = kinematicsUtils.l1
    l2 = kinematicsUtils.l2
    l3 = kinematicsUtils.l3
    jointRadius = kinematicsUtils.jointRadius
    boxSize = kinematicsUtils.boxSize

    # CC Calcolo Valori Della Tabella Di Denavit Hartenberg
    d1 = 3 * jointRadius + l1
    a2 = jointRadius + l2 + (boxSize / 2.0)

    # DD Traslazione Delle Coordinate Target Dal Frame World Al Frame Zero Della Base Rotante
    x = xTarget - worldBase
    y = yTarget
    z = zTarget - baseHeight

    # AA Calcolo Variabile Di Giunto Prismatico q3
    squareRootArg = x**2 + y**2 + (z - d1)**2 - a2**2

    # BB Verifica Della Condizione Di Esistenza Per Evitare Radici Di Numeri Negativi
    if squareRootArg < 0.0:
        rospy.logwarn("Target Non Raggiungibile. Si Trova Fuori Dal Workspace Del Robot.")
        return None, None, None

    # CC Calcolo Delle Due Possibili Soluzioni Per Il Giunto Prismatico q3
    # DD Soluzione Con Radice Positiva
    q3P = (l3 + boxSize / 2.0) + np.sqrt(squareRootArg)
    # DD Soluzione Con Radice Negativa
    q3M = (l3 + boxSize / 2.0) - np.sqrt(squareRootArg)

    # DD Scelta Della Soluzione Che Rientra Nei Limiti Del Giunto Prismatico
    q3 = None
    if 0.0 <= q3M <= l3:
        q3 = q3M
    elif 0.0 <= q3P <= l3:
        q3 = q3P
    else:
        rospy.logwarn("Nessuna Delle Due Soluzioni Di q3 Rientra Nei Limiti Fisici Del Giunto 3")
        return None, None, None

    # BB Calcolo Distanza d3 Corrispondente Alla Soluzione Selezionata
    d3 = q3 - (l3 + boxSize / 2.0)

    # AA Calcolo Della Variabile Di Giunto Rotatorio q2
    # BB Calcolo Raggio Workspace (Ovviamente Si Deve Considerare Il Valore Posivo E Il Valore Negativo)
    r = np.sqrt(x**2 + y**2)

    # BB Definizione Configurazioni Di Gomito Alto E Gomito Basso
    # CC Calcolo Valore q2 Gomito Alto Sfruttando Il Raggio Positivo
    q2Up = np.arctan2(d3 * r - a2 * (z - d1), a2 * r + d3 * (z - d1))
    # CC Calcolo Valore q2 Gomito Basso Sfruttando Il Raggio Negativo
    q2Down = np.arctan2(d3 * (-r) - a2 * (z - d1), a2 * (-r) + d3 * (z - d1))

    # DD Selezione Della Configurazione Di Gomito Che Rispetta I Limiti Del Secondo Giunto
    q2 = None
    limitMinQ2 = -np.pi / 2.0
    limitMaxQ2 = np.pi / 4.0

    if limitMinQ2 <= q2Up <= limitMaxQ2:
        q2 = q2Up
    elif limitMinQ2 <= q2Down <= limitMaxQ2:
        q2 = q2Down
    else:
        rospy.logwarn("Nessuna Configurazione Di Gomito Per q2 Rientra Nei Limiti Del Giunto")
        return None, None, None

    # AA Calcolo Della Variabile Di Giunto Rotatorio q1
    # CC Si Calocola Tramite Arcotangente A Quattro Quadranti
    denom = d3 * np.sin(q2) + a2 * np.cos(q2)

    if np.abs(denom) < 1e-6:
        # DD Gestione Del Caso Limite Con Denominatore Nullo Per Evitare Divisioni Per Zero
        q1 = 0.0
    else:
        # DD Calcolo Della Rotazione Della Base Tenendo Conto Del Segno Del denome
        q1 = np.arctan2(y / denom, x / denom)

    # CC Controllo Dei Valori Dei Giunti Rispetto Ai Limiti Fisici Del Robot
    q1, q2, q3 = kinematicsUtils.checkJointLimits(q1, q2, q3)

    return q1, q2, q3