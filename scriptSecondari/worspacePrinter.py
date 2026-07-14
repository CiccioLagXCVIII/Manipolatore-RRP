#!/usr/bin/env python3

# AA ======================= Importazione Delle Librerie Necessarie =======================

# BB Importazione Dei Moduli Standard Per Il Calcolo E La Grafica
import numpy as np
import matplotlib.pyplot as plt

# AA ======================= Definizione Dei Parametri Geometrici =======================

# BB Parametri Fisici Del Robot Definiti Come Costanti Locali
baseHeight = 0.06           # Spessore Della Piastra Di Base
l1 = 0.55                   # Lunghezza Della Colonna Verticale Del Primo Link
l2 = 0.45                   # Lunghezza Del Braccio Orizzontale Del Secondo Link
l3 = 0.35                   # Lunghezza Dell Asta Prismatica Del Terzo Link
jointRadius = 0.05          # Raggio Delle Sfere Di Giunzione Estetiche
boxSize = 0.10              # Dimensione Del Box Di Guida Del Giunto Prismatico

# AA ======================= Calcolo Dei Parametri Derivati E Dei Raggi =======================

# BB Calcolo Delle Distanze Geometriche DH E Dell Altezza Della Spalla
d1 = 2 * jointRadius + l1 + jointRadius  
a2 = jointRadius + l2 + (boxSize / 2)    

# BB Altezza Complessiva Del Centro Della Spalla Rispetto Al Terreno
shoulderHeight = d1 + baseHeight  

# BB Calcolo Dei Raggi Dei Due Cerchi Concentrici
# CC Valore Massimo
# Raggio Massimo Si Ottiene Con q_{3} = 0
maxRadius = np.sqrt(a2**2 + (l3 + (boxSize / 2.0))**2)  
# CC Valore Minimo
# Raggio Minimo Si Ottiene Con q_{3} = l3
minRadius = np.sqrt(a2**2 + (boxSize / 2.0)**2)

# CC Stampa Dei Raggi Calcolati
print("Raggio Massimo: ", maxRadius)
print("Raggio Minimo: ", minRadius)

# AA ======================= Generazione Dei Due Cerchi Interi =======================

# BB Creazione Del Vettore Angolare Per Disegnare I Cerchi Concentrici Interi
theta = np.linspace(0, 2 * np.pi, 500)

# BB Calcolo Dei Punti Del Cerchio Esterno Centrato Sulla Spalla
outerCircleX = maxRadius * np.cos(theta)
outerCircleY = shoulderHeight + maxRadius * np.sin(theta)

# BB Calcolo Dei Punti Del Cerchio Interno Centrato Sulla Spalla
innerCircleX = minRadius * np.cos(theta)
innerCircleY = shoulderHeight + minRadius * np.sin(theta)

# AA ======================= Generazione Della Figura Grafica =======================

# BB Inizializzazione Del Grafico Con Dimensioni Quadrate Per Evitare Distorsioni
plt.figure(figsize=(9, 9))
ax = plt.gca()

# BB Disegno Cerchi Concentrici
# CC Circonferenza Esterna (Rossa)
plt.plot(outerCircleX, outerCircleY, label=f"Limite Massimo (R_max = {maxRadius:.3f} m)", color="red", linestyle="-", linewidth=1.5)
# CC Freccia Circonferenza Esterna
# DD Angolo Per La Freccia
angleMax = np.pi / 4.0
arrowMaxX = maxRadius * np.cos(angleMax)
arrowMaxY = shoulderHeight + maxRadius * np.sin(angleMax)
# DD Freccia Dal Centro Della Spalla Al Bordo Esterno
plt.annotate("", xy=(arrowMaxX, arrowMaxY), xytext=(0, shoulderHeight), arrowprops=dict(arrowstyle="->", color="red", lw=2.0))
# DD Etichetta Freccia
maxRadiusLabelX = arrowMaxX / 2.0 + 0.03
maxRadiusLabelY = shoulderHeight + (arrowMaxY - shoulderHeight) / 2.0 + 0.03
plt.text(maxRadiusLabelX, maxRadiusLabelY, f"R_max = {maxRadius:.3f} m", color="red", fontsize=10, fontweight="bold")

# CC Circonferenza Interna (Blu)
plt.plot(innerCircleX, innerCircleY, label=f"Limite Minimo (R_min = {minRadius:.3f} m)", color="blue", linestyle="-", linewidth=1.5)
# CC Freccia Circonferenza  Interna
# DD Angolo
angleMin = 2.0 * np.pi / 4.0
arrowMinX = minRadius * np.cos(angleMin)
arrowMinY = shoulderHeight + minRadius * np.sin(angleMin)
# DD Freccia Dal Centro Della Spalla Al Bordo Interno
plt.annotate("", xy=(arrowMinX, arrowMinY), xytext=(0, shoulderHeight), arrowprops=dict(arrowstyle="->", color="blue", lw=2.0))
# DD Etichetta Freccia
minRadiusLabelX = arrowMinX / 2.0 - 0.15
minRadiusLabelY = shoulderHeight + (arrowMinY - shoulderHeight) / 2.0 + 0.03
plt.text(minRadiusLabelX, minRadiusLabelY, f"R_min = {minRadius:.3f} m", color="blue", fontsize=10, fontweight="bold")

# BB Disegno Link 1 (Verticale)
# CC Link 1
plt.plot([0, 0], [0, shoulderHeight], color="gray", linewidth=4.0, label="Link 1 (Braccio Verticale)")
# CC Centro Giunto 2
plt.plot(0, shoulderHeight, "o", color="orange", markersize=12, label="Centro Giunto 2")

# BB Disegno Della Linea Orizzontale Del Terreno
plt.axhline(0, color="black", linewidth=1.5, label="Livello World")

# BB Definizione Dei Titoli E Delle Etichette Degli Assi
plt.title("Workspace Manipolatore RRP (Vista Laterale X-Z)", fontsize=14, fontweight="bold", pad=15)
plt.xlabel("Distanza Orizzontale X (m)", fontsize=11)
plt.ylabel("Altezza Verticale Z (m)", fontsize=11)

# BB Forzatura Delle Proporzioni Uguali Per Mantenere I Cerchi Rotondi
ax.set_aspect("equal", adjustable="box")

# BB Abilitazione Della Griglia Di Riferimento E Della Legenda
plt.grid(True, linestyle=":", alpha=0.6)
plt.legend(loc="upper right", frameon=True, shadow=True)

# BB Impostazione Dei Limiti Di Visualizzazione Degli Assi Per Centrare Il Robot
plt.xlim(-maxRadius - 0.2, maxRadius + 0.2)
plt.ylim(-0.1, shoulderHeight + maxRadius + 0.2)

# BB Visualizzazione Del Grafico
plt.show()