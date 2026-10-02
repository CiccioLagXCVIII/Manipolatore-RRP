#!/usr/bin/env python3

# AA ======================= Importazione Delle Librerie Necessarie =======================

# BB Importazione Dei Moduli Per Il Calcolo Numerico E La Rappresentazione Grafica
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patheffects as pe

# AA ======================= Definizione Della Tavolozza Dei Colori =======================

# BB Codici Colori Esadecimali
workspaceColor                  = "#00bcd4"
specularWorkspaceColor          = "#607d8b"
specularWorkspaceBorderColor    = "#37474f"

baseColor        = "#383D42"
linkColor        = "#A6A6A6"
jointColor       = "#F2B814"
endEffectorColor = "#CC1F1F"

outerCircleColor = "#FF0000"
innerCircleColor = "#0000FF"
maxLimitColor    = "#FF8C00"
minLimitColor    = "#800080"
worldColor       = "#000000"
axesColor        = "#808080"

# AA ======================= Definizione Dei Parametri Geometrici =======================

# BB Parametri Dimensionali Del Manipolatore
baseWidth = 0.30            # Larghezza Della Base Quadrata (m)
baseLength = 0.30           # Lunghezza Della Base Quadrata (m)
baseHeight = 0.05           # Spessore Della Piastra Di Base (m)
l1 = 0.55                   # Lunghezza Della Colonna Verticale Del Primo Link (m)
l2 = 0.45                   # Lunghezza Del Braccio Orizzontale Del Secondo Link (m)
l3 = 0.35                   # Corsa Massima Dell Asta Prismatica Del Terzo Link (m)
jointRadius = 0.05          # Raggio Delle Sfere Di Giunzione (m)
boxSize = 0.10              # Dimensione Del Box Di Guida Del Giunto Prismatico (m)

# BB Dimensioni End-Effector (Pinza)
eeBaseWidth = 0.05          # Larghezza Della Piastra Di Base Pinza (m)
eeBaseLength = 0.10         # Lunghezza Della Piastra Di Base Pinza (m)
eeBaseHeight = 0.015        # Spessore Della Piastra Di Base Pinza (m)
eeFingerLength = 0.05       # Lunghezza Delle Dita Della Pinza (m)
eeFingerThickness = 0.01    # Spessore Delle Dita Della Pinza (m)

# BB Limiti Di Corsa Dei Giunti RRP
limitMinQ2 = -np.pi / 2.0   # Limite Inferiore Del Giunto 2 (-90 Gradi)
limitMaxQ2 = np.pi / 4.0    # Limite Superiore Del Giunto 2 (+45 Gradi)
limitMinQ3 = 0.0            # Estensione Massima Del Prismatico Verso Il Basso (m)
limitMaxQ3 = l3             # Corsa Massima Del Prismatico Nel Box Verso L Alto (m)

# AA ======================= Calcolo Dei Parametri Geometrici E Dei Raggi =======================

# BB Parametri Della Tabella Denavit-Hartenberg (DH)
d1 = 3 * jointRadius + l1
a2 = jointRadius + l2 + (boxSize / 2.0)

# BB Quota Verticale Del Centro Di Rotazione Della Spalla (Giunto 2)
shoulderHeight = d1 + baseHeight

# BB Valori Limite Della Distanza d3 Rispetto All Origine Del Giunto 2
d3FullyExtended = limitMinQ3 - (l3 + boxSize / 2.0)   # -0.40 m
d3FullyRetracted = limitMaxQ3 - (l3 + boxSize / 2.0)  # -0.05 m

# BB Raggi Sferici Estremo Ed Interno Centrati Sulla Spalla
maxRadius = np.sqrt(a2**2 + d3FullyExtended**2)
minRadius = np.sqrt(a2**2 + d3FullyRetracted**2)

# BB Raggi Limite Proiettati Sul Piano Orizzontale (Vista X-Y)
minRadiusXY = np.abs(d3FullyRetracted)
maxRadiusXY = maxRadius

# CC Stampa Dei Valori Calcolati Nel Terminale
print(f"Raggio Massimo Dalla Spalla: {maxRadius:.4f} m")
print(f"Raggio Minimo Dalla Spalla:  {minRadius:.4f} m")
print(f"Raggio Minimo Sul Piano X-Y: {minRadiusXY:.4f} m")
print(f"Raggio Massimo Sul Piano X-Y: {maxRadiusXY:.4f} m")

# AA ======================= VISTA LATERALE (PIANO X-Z) =======================

numSamples = 200
q2Samples = np.linspace(limitMinQ2, limitMaxQ2, numSamples)
d3Samples = np.linspace(d3FullyExtended, d3FullyRetracted, numSamples)

# BB Arco Esterno
extArcX = d3FullyExtended * np.sin(q2Samples) + a2 * np.cos(q2Samples)
extArcZ = shoulderHeight + d3FullyExtended * np.cos(q2Samples) - a2 * np.sin(q2Samples)

# BB Segmento Al Limite Superiore (q2 = +45°)
lineMaxQ2X = d3Samples * np.sin(limitMaxQ2) + a2 * np.cos(limitMaxQ2)
lineMaxQ2Z = shoulderHeight + d3Samples * np.cos(limitMaxQ2) - a2 * np.sin(limitMaxQ2)

# BB Arco Interno
innerArcX = d3FullyRetracted * np.sin(q2Samples[::-1]) + a2 * np.cos(q2Samples[::-1])
innerArcZ = shoulderHeight + d3FullyRetracted * np.cos(q2Samples[::-1]) - a2 * np.sin(q2Samples[::-1])

# BB Segmento Al Limite Inferiore (q2 = -90°)
lineMinQ2X = d3Samples[::-1] * np.sin(limitMinQ2) + a2 * np.cos(limitMinQ2)
lineMinQ2Z = shoulderHeight + d3Samples[::-1] * np.cos(limitMinQ2) - a2 * np.sin(limitMinQ2)

# BB Impostazione Spazio Di Lavoro
workspaceX = np.concatenate([extArcX, lineMaxQ2X, innerArcX, lineMinQ2X])
workspaceZ = np.concatenate([extArcZ, lineMaxQ2Z, innerArcZ, lineMinQ2Z])

# BB Figura Vista Laterale
fig1, ax1 = plt.subplots(figsize=(11, 6))

# BB Workspace E Bordi (con q1, q2, q3 e costanti a pedice in formato LaTeX)
ax1.fill(workspaceX, workspaceZ, color=workspaceColor, alpha=0.18, zorder=1, label="Area Raggiungibile ($q_1 = 0$ rad)")
ax1.plot(extArcX, extArcZ, color=outerCircleColor, linewidth=2.0, zorder=6, label=f"Raggio Esterno ($q_3 = 0$, $R_{{max}} = {maxRadius:.3f}$ m)")
ax1.plot(innerArcX, innerArcZ, color=innerCircleColor, linewidth=2.0, zorder=3, label=f"Raggio Interno ($q_3 = l_3$, $R_{{min}} = {minRadius:.3f}$ m)")
ax1.plot(lineMaxQ2X, lineMaxQ2Z, color=maxLimitColor, linestyle="--", linewidth=1.5, zorder=2, label=r"Limite Giunto 2 ($q_2 = +45^\circ$)")
ax1.plot(lineMinQ2X, lineMinQ2Z, color=minLimitColor, linestyle="--", linewidth=1.5, zorder=2, label=r"Limite Giunto 2 ($q_2 = -90^\circ$)")

# BB Workspace Speculare
ax1.fill(-workspaceX, workspaceZ, color=specularWorkspaceColor, alpha=0.25, edgecolor=specularWorkspaceBorderColor, linestyle="--", linewidth=1.2, zorder=1, label=r"Area Raggiungibile Speculare ($q_1 = \pm\pi$ rad)")

# CC Livello World
ax1.axhline(0, color=worldColor, linewidth=2.0, zorder=3)

# BB Struttura Manipolatore
# CC Base Manipolatore
baseBox = plt.Rectangle((-baseWidth / 2.0, 0), baseWidth, baseHeight, facecolor=baseColor, edgecolor=worldColor, zorder=4)
ax1.add_patch(baseBox)

# CC Colonna Verticale (Link 1 con bordo e taglio netto a filo della base)
ax1.plot(
    [0, 0], [baseHeight, shoulderHeight],
    color=linkColor, linewidth=6.0, zorder=4,
    solid_capstyle="butt",
    path_effects=[pe.Stroke(linewidth=8.0, foreground=worldColor), pe.Normal()]
)

# CC Centro Del Giunto Di Spalla (Giunto 2)
ax1.plot(0, shoulderHeight, "o", color=jointColor, markersize=10, markeredgecolor=worldColor, zorder=6)

# CC Braccio Orizzontale (Link 2 con bordo)
ax1.plot(
    [0, a2], [shoulderHeight, shoulderHeight],
    color=linkColor, linewidth=5.0, zorder=4,
    solid_capstyle="butt",
    path_effects=[pe.Stroke(linewidth=7.0, foreground=worldColor), pe.Normal()]
)

# CC Box Di Guida Del Giunto Prismatico
boxPrismatic = plt.Rectangle((a2 - boxSize / 2.0, shoulderHeight - boxSize / 2.0), boxSize, boxSize, facecolor=jointColor, edgecolor=worldColor, linewidth=1.5, zorder=5)
ax1.add_patch(boxPrismatic)

# CC Asta Prismatica (Link 3 con bordo)
prismaticRodZBottom = shoulderHeight + d3FullyExtended
ax1.plot(
    [a2, a2], [shoulderHeight - boxSize / 2.0, prismaticRodZBottom],
    color=linkColor, linewidth=4.0, zorder=4,
    solid_capstyle="butt",
    path_effects=[pe.Stroke(linewidth=6.0, foreground=worldColor), pe.Normal()]
)

# CC Piastra Di Base Della Pinza (End-Effector)
eeBasePlate = plt.Rectangle((a2 - eeBaseWidth / 2.0, prismaticRodZBottom), eeBaseWidth, eeBaseHeight, facecolor=endEffectorColor, edgecolor=worldColor, linewidth=0.75, zorder=8)
ax1.add_patch(eeBasePlate)

# CC Dita Della Pinza
fingerLeft = plt.Rectangle((a2 - eeBaseWidth / 2.0, prismaticRodZBottom - eeFingerLength), eeFingerThickness, eeFingerLength, facecolor=endEffectorColor, edgecolor=worldColor, linewidth=0.75, zorder=8)
fingerRight = plt.Rectangle((a2 + eeBaseWidth / 2.0 - eeFingerThickness, prismaticRodZBottom - eeFingerLength), eeFingerThickness, eeFingerLength, facecolor=endEffectorColor, edgecolor=worldColor, linewidth=0.75, zorder=8)
ax1.add_patch(fingerLeft)
ax1.add_patch(fingerRight)

# BB Quote E Frecce Dei Raggi Sferici (Con Sfondo Protettivo e zorder=7)
# DD Quota Del Raggio Massimo
angleMaxArrow = np.deg2rad(15.0)
arrowMaxX = d3FullyExtended * np.sin(angleMaxArrow) + a2 * np.cos(angleMaxArrow)
arrowMaxZ = shoulderHeight + d3FullyExtended * np.cos(angleMaxArrow) - a2 * np.sin(angleMaxArrow)
ax1.annotate("", xy=(arrowMaxX, arrowMaxZ), xytext=(0, shoulderHeight), arrowprops=dict(arrowstyle="->", color=outerCircleColor, lw=2.0), zorder=7)
ax1.text(
    arrowMaxX / 2.0 + 0.03,
    shoulderHeight + (arrowMaxZ - shoulderHeight) / 2.0,
    f"$R_{{max}} = {maxRadius:.3f}$ m",
    color=outerCircleColor,
    fontsize=10,
    bbox=dict(boxstyle="round,pad=0.25", facecolor="#FFFFFF", edgecolor="none", alpha=0.85),
    zorder=7
)

# DD Quota Del Raggio Minimo
angleMinArrow = np.deg2rad(-60.0)
arrowMinX = d3FullyRetracted * np.sin(angleMinArrow) + a2 * np.cos(angleMinArrow)
arrowMinZ = shoulderHeight + d3FullyRetracted * np.cos(angleMinArrow) - a2 * np.sin(angleMinArrow)
ax1.annotate("", xy=(arrowMinX, arrowMinZ), xytext=(0, shoulderHeight), arrowprops=dict(arrowstyle="->", color=innerCircleColor, lw=2.0), zorder=7)
ax1.text(
    arrowMinX / 2.0 - 0.22,
    shoulderHeight + (arrowMinZ - shoulderHeight) / 2.0 - 0.04,
    f"$R_{{min}} = {minRadius:.3f}$ m",
    color=innerCircleColor,
    fontsize=10,
    bbox=dict(boxstyle="round,pad=0.25", facecolor="#FFFFFF", edgecolor="none", alpha=0.85),
    zorder=7
)

# BB Impostazioni Vista Laterale
ax1.set_title("Workspace Manipolatore RRP (Vista Laterale X-Z)", fontsize=14,  pad=15)
ax1.set_xlabel("Distanza Orizzontale X Rispetto Alla Base (m)", fontsize=11)
ax1.set_ylabel("Distanza Verticale Z Rispetto Al Terreno (m)", fontsize=11)
ax1.set_aspect("equal", adjustable="box")
ax1.grid(True, linestyle=":", alpha=0.6, zorder=0)
ax1.set_xlim(-maxRadius - 0.20, maxRadius + 0.20)
ax1.set_ylim(-0.05, shoulderHeight + a2 + 0.1)
ax1.legend(loc="upper left", bbox_to_anchor=(1.05, 1.0), fontsize=9, frameon=True, shadow=True)
fig1.subplots_adjust(right=0.6)

# AA ======================= VISTA DALL ALTO (PIANO X-Y) =======================

thetaCamp = np.linspace(0, 2 * np.pi, 400)
outerCircleX = maxRadiusXY * np.cos(thetaCamp)
outerCircleY = maxRadiusXY * np.sin(thetaCamp)
innerCircleX = minRadiusXY * np.cos(thetaCamp)
innerCircleY = minRadiusXY * np.sin(thetaCamp)

fig2, ax2 = plt.subplots(figsize=(11, 6))

# BB Corona Circolare
corCircX = np.concatenate([outerCircleX, innerCircleX[::-1]])
corCircY = np.concatenate([outerCircleY, innerCircleY[::-1]])
ax2.fill(corCircX, corCircY, color=workspaceColor, alpha=0.20, zorder=1, label=f"Area Raggiungibile ({minRadiusXY:.2f} m ≤ R ≤ {maxRadiusXY:.2f} m)")
ax2.plot(outerCircleX, outerCircleY, color=outerCircleColor, linewidth=2.0, linestyle="-", zorder=2, label=f"Raggio Massimo XY ($R_{{max}} = {maxRadiusXY:.3f}$ m)")
ax2.plot(innerCircleX, innerCircleY, color=innerCircleColor, linewidth=2.0, linestyle="-", zorder=8, label=f"Raggio Minimo XY ($R_{{min}} = {minRadiusXY:.3f}$ m)")

# CC Assi Cartesiani Di Riferimento
ax2.axhline(0, color=axesColor, linestyle="--", linewidth=0.8, zorder=2)
ax2.axvline(0, color=axesColor, linestyle="--", linewidth=0.8, zorder=2)

# BB Struttura Manipolatore
# CC Base Fissa Quadrata
baseTopView = plt.Rectangle((-baseWidth / 2.0, -baseLength / 2.0), baseWidth, baseLength, facecolor=baseColor, edgecolor=worldColor, linewidth=1.5, alpha=0.45, zorder=4)
ax2.add_patch(baseTopView)

# CC Braccio Orizzontale (Link 2 con bordo)
ax2.plot(
    [0, a2], [0, 0],
    color=linkColor, linewidth=6.0, zorder=5,
    solid_capstyle="butt",
    path_effects=[pe.Stroke(linewidth=8.0, foreground=worldColor), pe.Normal()]
)

# CC Box Del Giunto Prismatico
boxTopView = plt.Rectangle((a2 - boxSize / 2.0, -boxSize / 2.0), boxSize, boxSize, facecolor=jointColor, edgecolor=worldColor, linewidth=1.2, zorder=6)
ax2.add_patch(boxTopView)

# CC Centro Di Rotazione Giunto 1
ax2.plot(0, 0, "o", color=jointColor, markersize=10, markeredgecolor=worldColor, zorder=7)

# BB Quote E Frecce
arrowAngleMaxXY = np.pi / 4.0
arrowMaxX_XY = maxRadiusXY * np.cos(arrowAngleMaxXY)
arrowMaxY_XY = maxRadiusXY * np.sin(arrowAngleMaxXY)
textMinX = 0.0
textMinY = -0.25
ax2.annotate("", xy=(arrowMaxX_XY, arrowMaxY_XY), xytext=(0, 0), arrowprops=dict(arrowstyle="->", color=outerCircleColor, lw=2.0), zorder=8)
ax2.annotate("", xy=(0, -minRadiusXY), xytext=(textMinX, textMinY + 0.04), arrowprops=dict(arrowstyle="->", color=innerCircleColor, lw=1.5), zorder=8)
ax2.text(arrowMaxX_XY * 0.65, arrowMaxY_XY * 0.65 - 0.05, f"$R_{{max}} = {maxRadiusXY:.3f}$ m",
         color=outerCircleColor, fontsize=10,
         bbox=dict(boxstyle="round,pad=0.25", facecolor="#FFFFFF", edgecolor="none", alpha=0.85),
         zorder=8
        )
ax2.text(textMinX, textMinY, f"$R_{{min}} = {minRadiusXY:.3f}$ m", color=innerCircleColor, fontsize=10,
          ha="center", va="center",
         bbox=dict(boxstyle="round,pad=0.25", facecolor="#FFFFFF", edgecolor="none", alpha=0.85),
         zorder=8)

# BB Impostazioni Vista Dall'Alto
ax2.set_title("Workspace Manipolatore RRP (Vista Dall'Alto X-Y)", fontsize=14,  pad=15)
ax2.set_xlabel("Distanza Orizzontale X Rispetto Alla Base (m)", fontsize=11)
ax2.set_ylabel("Distanza Orizzontale Y Rispetto Alla Base (m)", fontsize=11)
ax2.set_aspect("equal", adjustable="box")
ax2.grid(True, linestyle=":", alpha=0.6, zorder=0)
ax2.set_xlim(-maxRadiusXY - 0.20, maxRadiusXY + 0.20)
ax2.set_ylim(-maxRadiusXY - 0.20, maxRadiusXY + 0.20)
ax2.legend(loc="upper left", bbox_to_anchor=(1.05, 1.0), fontsize=9, frameon=True, shadow=True)
fig2.subplots_adjust(right=0.7)

# AA ======================= Visualizzazione Conclusiva =======================
plt.show()
