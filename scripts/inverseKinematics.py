import numpy as np

# BB Calcolo Dei Valori Dei Giunti
# CC Riceve La Posizione Target E Calcola I Tre Valori Dei Giunti
def computeInverseKinematics(xTarget, yTarget, zTarget):
    # BB Le Coordinate Target Ovviamente Sono Espresse Rispetto A RF World Quindi Si Devono Trasformare Nel Sistema Di Riferimento Del Primo Giunto
    # CC Rispetto A RF World, Il Primo Giunto RF0, Si Trova A 2m E Sullla Base Che È Alta 6cm 
    # CC Quindi Un Qualsiasi Punto Target Espresso Rispetto A RF World, Poichè RF0 È Spostato In Avanti Di 2m E In Alto Di 6cm, Per Essere Espresso Rispetto A RF0, Si Deve Traslare Di -2m Sull'Asse X E Di -6cm Sull'Asse Z (Per Sovrapporre RF0 A RF World)
    x = xTarget - 2.0
    y = yTarget
    z = zTarget - 0.06

    # BB Dall' Equazione ${}^{0}T_{EE} = {}^{0}T_{1} \cdot {}^{1}T_{2} \cdot {}^{2}T_{3} = {}^{0}T_{2} \cdot {}^{2}T_{3}$ Svolgendo Il Prodotto Matriciale
    # BB A Blocchi Si Ottiene Che ${}^{0}d_{EE}$ (La Posizione Dell' End Effector Rispetto A RF0, Che È L'Input Traslato) È Data Da ${}^{0}R_{2} \cdot {}^{2}d_{EE} + {}^{0}d_{2}$



    return q1, q2, q3
    