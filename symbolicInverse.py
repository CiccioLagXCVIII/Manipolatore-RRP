import sympy as sp

# AA ======================= Definizione Variabili Simboliche ======================= #

# BB Definizione Dei Parametri Geometrici E Delle Variabili Di Giunto
# CC Configurazione Delle Variabili Come Reali E Positive
# DD Inizializzazione Dei Simboli Per NumPy E SymPy
q1, q2, q3 = sp.symbols('q1 q2 q3', real=True)
l1, l2, l3 = sp.symbols('l1 l2 l3', positive=True)
jointRadius, boxSize = sp.symbols('jointRadius boxSize', positive=True)

# CC Definizione Delle Abbreviazioni Trigonometriche
c1 = sp.cos(q1)
s1 = sp.sin(q1)
c2 = sp.cos(q2)
s2 = sp.sin(q2)

# CC Definizione Dei Parametri DH Ricavati Dalla Geometria
d1 = 2 * jointRadius + l1 + jointRadius
a2 = jointRadius + l2 + sp.Rational(1, 2) * boxSize
d3 = q3 - (l3 + sp.Rational(1, 2) * boxSize)

# AA ======================= Definizione Matrici Denavit Hartenberg ======================= #

# BB Creazione Delle Matrici Di Trasformazione Per Ogni Giunto
# CC Matrice Di Trasformazione Dal Frame Zero Al Frame Uno
A_01 = sp.Matrix([
    [c1, 0, -s1, 0],
    [s1, 0,  c1, 0],
    [0,  -1, 0, d1],
    [0,  0,  0, 1]
])

# CC Matrice Di Trasformazione Dal Frame Uno Al Frame Due
A_12 = sp.Matrix([
    [c2, 0, s2, a2 * c2],
    [s2, 0, -c2, a2 * s2],
    [0,  1, 0, 0],
    [0,  0, 0, 1]
])

# CC Matrice Di Trasformazione Dal Frame Due All'End Effector
A_2EE = sp.Matrix([
    [1, 0, 0, 0],
    [0, -1, 0, 0],
    [0, 0, -1, d3],
    [0, 0, 0, 1]
])

# AA ======================= Calcolo Trasformazioni Composite ======================= #

# BB Calcolo Della Matrice Omogenea Da Frame Zero A Frame Due
A_02 = sp.simplify(A_01 * A_12)

# BB Estrazione Delle Matrici Di Rotazione E Dei Vettori Di Traslazione
# CC Estrazione Della Rotazione Zero Uno
R01_DH = A_01[:3, :3]
# CC Estrazione Della Rotazione Uno Due
R12_DH = A_12[:3, :3]
# CC Estrazione Della Traslazione Zero Due
d_02 = A_02[:3, 3]
# CC Estrazione Della Traslazione Due End Effector
d_2EE = A_2EE[:3, 3]

# BB Composizione Della Posizione Totale Dell'End Effector
d_0EE = sp.simplify(R01_DH * R12_DH * d_2EE + d_02)

# AA ======================= Verifica Equazioni Cinematica Inversa ======================= #

# AA ======================= Sezione Uno: Calcolo Di q3 ======================= #

# BB Definizione Delle Singole Coordinate Spaziali X Y Z
x = d_0EE[0]
y = d_0EE[1]
z = d_0EE[2]

# BB Calcolo Della Norma Quadra Spaziale Con Offset Verticale Isolato
# CC Questa Quantita Corrisponde Al Termine d3 Al Quadrato Piu a2 Al Quadrato
norm_squared = sp.simplify(x**2 + y**2 + (z - d1)**2)

# AA ======================= Sezione Due: Calcolo Di q2 ======================= #

# BB Definizione Delle Equazioni Del Sistema Lineare In Forma Matriciale
# CC Il Simbolo w Rappresenta La Radice Quadrata Di X Al Quadrato Piu Y Al Quadrato
w = sp.symbols('w', real=True)

# CC Definizione Della Matrice Dei Coefficienti E Del Vettore Dei Termini Noti
M = sp.Matrix([
    [d3, a2],
    [-a2, d3]
])
b = sp.Matrix([
    [w],
    [z - d1]
])

# CC Verifica Del Determinante Della Matrice Dei Coefficienti
det_M = sp.simplify(M.det())

# CC Calcolo Della Soluzione Tramite Inversione Della Matrice
M_inv = sp.simplify(M.inv())
sol_q2 = sp.simplify(M_inv * b)
sin_q2_expr = sol_q2[0]
cos_q2_expr = sol_q2[1]

# CC Sostituzione Dei Termini Spaziali Per Verificare L'Identita Di q2
# DD Sostituendo Il Valore Di w Con La Sua Espressione Analitica Verifichiamo Il Sistema
sin_q2_verified = sp.simplify(sin_q2_expr.subs(w, d3 * s2 + a2 * c2))
cos_q2_verified = sp.simplify(cos_q2_expr.subs(w, d3 * s2 + a2 * c2))

# AA ======================= Sezione Tre: Calcolo Di q1 ======================= #

# BB Verifica Delle Espressioni Di Seno E Coseno Di q1 Tramite Divisione
# CC Calcolo Del Denominatore Comune Presente Nelle Equazioni Di Spostamento
denom = d3 * s2 + a2 * c2
# CC La Semplificazione Di SymPy Deve Restituire Esattamente cos(q1) E sin(q1)
cos_q1_check = sp.simplify(x / denom)
sin_q1_check = sp.simplify(y / denom)

# AA ======================= Stampa Dei Risultati Simbolici ======================= #

# BB Funzione Di Supporto Per Visualizzare Le Equazioni In Modo Leggibile
print("=" * 80)
print("1. VERIFICA DI q3")
print("=" * 80)
print("Posizione End Effector d_0EE (x, y, z):")
sp.pprint(d_0EE, use_unicode=True)
print("\nNorma quadra con offset isolato: x^2 + y^2 + (z - d1)^2 =")
sp.pprint(norm_squared, use_unicode=True)

print("\n" + "=" * 80)
print("2. VERIFICA DI q2")
print("=" * 80)
print("Determinante della matrice M (deve valere d3^2 + a2^2):")
sp.pprint(det_M, use_unicode=True)
print("\nEspressione per sin(q2):")
sp.pprint(sin_q2_expr, use_unicode=True)
print("\nEspressione per cos(q2):")
sp.pprint(cos_q2_expr, use_unicode=True)
print("\nVerifica identità sin(q2) (deve restituire sin(q2)):")
sp.pprint(sin_q2_verified, use_unicode=True)
print("\nVerifica identità cos(q2) (deve restituire cos(q2)):")
sp.pprint(cos_q2_verified, use_unicode=True)

print("\n" + "=" * 80)
print("3. VERIFICA DI q1")
print("=" * 80)
print("Verifica espressione cos(q1) = x / (d3*sin(q2) + a2*cos(q2)) (deve restituire cos(q1)):")
sp.pprint(cos_q1_check, use_unicode=True)
print("\nVerifica espressione sin(q1) = y / (d3*sin(q2) + a2*cos(q2)) (deve restituire sin(q1)):")
sp.pprint(sin_q1_check, use_unicode=True)
print("=" * 80)