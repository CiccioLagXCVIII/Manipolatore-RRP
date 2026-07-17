#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# AA ======================= Importazione Delle Librerie E Impostazioni Iniziali =======================

# BB Importazione Di SymPy Per Il Calcolo Simbolico
import sympy as sp
from sympy import Matrix, cos, sin, sqrt, atan2, pi, simplify, factor, expand, trigsimp, Eq, pprint, N

# BB Abilitazione Della Stampa Con Caratteri Unicode Di SymPy
sp.init_printing(use_unicode=True)


# AA ======================= Dichiarazione Dei Simboli E Dei Parametri Fisici =======================

# BB Definizione Delle Variabili Di Giunto Del Manipolatore
q1, q2, q3 = sp.symbols('q1 q2 q3', real=True)

# BB Definizione Delle Coordinate Target Rispetto Al Frame Globale Di World
xTarget, yTarget, zTarget = sp.symbols('xTarget yTarget zTarget', real=True)

# BB Definizione Delle Variabili Cartesiane Generiche Per Lo End-Effector
x, y, z = sp.symbols('x y z', real=True)

# BB Definizione Dei Parametri Geometrici Costanti Del Robot Come Valori Positivi Reali
jointRadius, l1, l2, l3, boxSize, eeFingerLength, baseHeight = sp.symbols(
    'jointRadius l1 l2 l3 boxSize eeFingerLength baseHeight', positive=True, real=True
)

# BB Definizione Dei Parametri Compatti Per Semplificare Le Equazioni Cinematiche
d1, a2, d3 = sp.symbols('d1 a2 d3', real=True)

# BB Definizione Del Raggio Radiale Nel Piano Di Sezione Come Valore Non Negativo Reale
r = sp.symbols('r', nonnegative=True, real=True)


# AA ======================= Funzione Per La Generazione Della Matrice Di Denavit-Hartenberg =======================

# BB Calcolo Della Matrice Di Trasformazione Omogenea Secondo La Convenzione Standard
# DD Questa Macro Restituisce La Matrice Quattro Per Quattro Legando I Sistemi Di Riferimento Successivi
def computeDhMatrix(thetaI, dI, aI, alphaI):
    return Matrix([
        [sp.cos(thetaI), -sp.sin(thetaI)*sp.cos(alphaI),  sp.sin(thetaI)*sp.sin(alphaI), aI*sp.cos(thetaI)],
        [sp.sin(thetaI),  sp.cos(thetaI)*sp.cos(alphaI), -sp.cos(thetaI)*sp.sin(alphaI), aI*sp.sin(thetaI)],
        [0,              sp.sin(alphaI),               sp.cos(alphaI),                dI],
        [0,              0,                           0,                            1]
    ])


# AA ======================= Sezione Uno: Trasformazione Del Target E Cinematica Diretta =======================

# BB Trasformazione Delle Coordinate Target Dal Frame World Al Frame Zero Della Base
# DD Sottrazione Dello Offset Di Due Metri Lungo X E Della Altezza Della Base Lungo Z
print("\n" + "="*100)
print("1) TRASFORMAZIONE DEL TARGET DA RFworld A RF0")
print("="*100)

xTransformed = xTarget - sp.Float(2.0)
yTransformed = yTarget
zTransformed = zTarget - sp.Float(0.06)

print("\nxTransformed =")
pprint(xTransformed, use_unicode=True)
print("\nyTransformed =")
pprint(yTransformed, use_unicode=True)
print("\nzTransformed =")
pprint(zTransformed, use_unicode=True)

# BB Definizione Del Vettore Posizione Dello End-Effector Rispetto Al Frame Due
# DD La Asta Prismatica Si Estende Verso Il Basso Lungo Z Con Un Offset Dovuto Alla Lunghezza L3 E Al Box
print("\n" + "-"*100)
print("Definizione del vettore posizione dell’end-effector rispetto a RF2")
print("-"*100)

positionVectorEe2 = Matrix([0, 0, q3 - (l3 + boxSize/sp.Integer(2))])
print("\npositionVectorEe2 =")
pprint(positionVectorEe2, use_unicode=True)

# BB Generazione Della Matrice Di Trasformazione Omogenea Generica
print("\n" + "-"*100)
print("Matrice DH generica i-1 A_i(q_i)")
print("-"*100)

thetaI, dI, aI, alphaI = sp.symbols('thetaI dI aI alphaI', real=True)
genericTransformationMatrix = computeDhMatrix(thetaI, dI, aI, alphaI)
print("\ngenericTransformationMatrix =")
pprint(genericTransformationMatrix, use_unicode=True)

# BB Calcolo Delle Tre Matrici Di Trasformazione DH Per Ciascun Link Del Manipolatore RRP
# DD Configurazione Geometrica Dei Tre Giunti Compresi Gli Offset Fisici E Le Rotazioni Angolari
print("\n" + "-"*100)
print("Calcolo delle tre matrici DH del manipolatore")
print("-"*100)

transformationMatrix01 = computeDhMatrix(q1, 3*jointRadius + l1, 0, -pi/sp.Integer(2))
transformationMatrix12 = computeDhMatrix(q2, 0, jointRadius + l2 + boxSize/sp.Integer(2), pi/sp.Integer(2))
transformationMatrix2Ee = computeDhMatrix(0, q3 - (l3 + boxSize/sp.Integer(2)), 0, -pi)

print("\ntransformationMatrix01 =")
pprint(transformationMatrix01, use_unicode=True)
print("\ntransformationMatrix12 =")
pprint(transformationMatrix12, use_unicode=True)
print("\ntransformationMatrix2Ee =")
pprint(transformationMatrix2Ee, use_unicode=True)

# BB Calcolo Della Matrice Di Composizione Parziale Tra Frame Zero E Frame Due
print("\n" + "-"*100)
print("Prodotto 0T2 = 0A1 * 1A2")
print("-"*100)

transformationMatrix02 = trigsimp(transformationMatrix01 * transformationMatrix12)
print("\ntransformationMatrix02 =")
pprint(transformationMatrix02, use_unicode=True)

# BB Estrazione Della Rotazione E Della Posizione Dalle Matrici Parziali
print("\n" + "-"*100)
print("Estrazione dei vettori di posizione e delle matrici di rotazione")
print("-"*100)

positionVector02 = transformationMatrix02[:3, 3]
rotationMatrix01 = transformationMatrix01[:3, :3]
rotationMatrix12 = transformationMatrix12[:3, :3]

print("\npositionVector02 =")
pprint(positionVector02, use_unicode=True)
print("\nrotationMatrix01 =")
pprint(rotationMatrix01, use_unicode=True)
print("\nrotationMatrix12 =")
pprint(rotationMatrix12, use_unicode=True)

# BB Composizione Della Posizione Dello End-Effector Rispetto Al Frame Della Base Zero
# DD Somma Del Contributo Rototraslato E Della Origine Del Frame Precedente
print("\n" + "-"*100)
print("Composizione 0d_EE = 0R1 * 1R2 * 2d_EE + 0d2")
print("-"*100)

positionVectorEe0Full = trigsimp(rotationMatrix01 * rotationMatrix12 * positionVectorEe2 + positionVector02)
print("\npositionVectorEe0Full =")
pprint(positionVectorEe0Full, use_unicode=True)

# BB Definizione Delle Sostituzioni Parametriche Compatte Per La Cinematica Diretta
print("\n" + "-"*100)
print("Definizione dei parametri compatti")
print("-"*100)

parameterSubstitutionMap = {
    d1: 3*jointRadius + l1,
    a2: jointRadius + l2 + boxSize/sp.Integer(2),
    d3: q3 - (l3 + boxSize/sp.Integer(2)),
}
for keySubstitute, valueSubstitute in parameterSubstitutionMap.items():
    print(f"\n{keySubstitute} =")
    pprint(valueSubstitute, use_unicode=True)

positionVectorEe0 = trigsimp(positionVectorEe0Full.subs(parameterSubstitutionMap))
print("\npositionVectorEe0 =")
pprint(positionVectorEe0, use_unicode=True)

# BB Estrazione Delle Tre Equazioni Scalari Corrispondenti Agli Assi Cartesiani X, Y, Z
print("\n" + "-"*100)
print("Equazioni scalari della cinematica diretta")
print("-"*100)

xExpression = trigsimp(positionVectorEe0[0])
yExpression = trigsimp(positionVectorEe0[1])
zExpression = trigsimp(positionVectorEe0[2])

print("\nxExpression =")
pprint(xExpression, use_unicode=True)
print("\nyExpression =")
pprint(yExpression, use_unicode=True)
print("\nzExpression =")
pprint(zExpression, use_unicode=True)


# AA ======================= Sezione Due: Risoluzione Analitica Del Giunto Prismatico q3 =======================

# BB Sviluppo Algebrico E Quadratura Dei Singoli Assi Per Il Calcolo Della Norma
# DD Isolamento Delle Coordinate Per Sfruttare La Identita Trigonometrica Fondamentale
print("\n" + "="*100)
print("2) CALCOLO DI q3 PASSO PER PASSO")
print("="*100)

print("\n" + "-"*100)
print("Quadrato della prima componente")
print("-"*100)
xSquaredStep = expand(xExpression**2)
print("\nxSquaredStep =")
pprint(xSquaredStep, use_unicode=True)

print("\n" + "-"*100)
print("Quadrato della seconda componente")
print("-"*100)
ySquaredStep = expand(yExpression**2)
print("\nySquaredStep =")
pprint(ySquaredStep, use_unicode=True)

print("\n" + "-"*100)
print("Somma x^2 + y^2")
print("-"*100)
xySum = trigsimp(expand(xSquaredStep + ySquaredStep))
print("\nxySum =")
pprint(xySum, use_unicode=True)

print("\n" + "-"*100)
print("Quadrato della terza componente traslata: (z - d1)^2")
print("-"*100)
zShiftSquared = expand((zExpression - d1)**2)
print("\nzShiftSquared =")
pprint(zShiftSquared, use_unicode=True)

# BB Calcolo Della Somma Dei Quadrati Delle Tre Componenti Traslate
# DD La Norma Risultante Corrisponde Alla Distanza Geometrica Al Quadrato Tra Il Giunto Due E Lo End-Effector
print("\n" + "-"*100)
print("Somma totale x^2 + y^2 + (z - d1)^2")
print("-"*100)
normSquared = trigsimp(expand(xySum + zShiftSquared))
print("\nnormSquared =")
pprint(normSquared, use_unicode=True)

# BB Isolamento Del Parametro Variabile d3 Dalla Relazione Di Distanza Totale
print("\n" + "-"*100)
print("Isolamento di d3^2")
print("-"*100)
d3Squared = sp.solve(Eq(sp.Symbol('rho2'), d3**2 + a2**2), d3**2)[0]
print("\nd3Squared =")
pprint(d3Squared, use_unicode=True)

print("\n" + "-"*100)
print("Sostituzione di rho2 = x^2 + y^2 + (z - d1)^2")
print("-"*100)
d3SquaredTarget = x**2 + y**2 + (z - d1)**2 - a2**2
print("\nd3SquaredTarget =")
pprint(d3SquaredTarget, use_unicode=True)

# BB Formulazione Della Equazione Quadratica In Funzione Del Giunto q3
print("\n" + "-"*100)
print("Sostituzione d3 = q3 - (l3 + boxSize/2)")
print("-"*100)
q3EquationSquared = Eq((q3 - (l3 + boxSize/sp.Integer(2)))**2, d3SquaredTarget)
print("\nq3EquationSquared =")
pprint(q3EquationSquared, use_unicode=True)

# BB Risoluzione Della Equazione Rispetto A q3 Ottenendo Le Due Soluzioni Simmetriche
# DD Le Due Soluzioni Rappresentano I Due Rami Matematici Del Giunto Lineare
print("\n" + "-"*100)
print("Radice per ottenere q3")
print("-"*100)
q3SolutionPlus = simplify((l3 + boxSize/sp.Integer(2)) + sqrt(d3SquaredTarget))
q3SolutionMinus = simplify((l3 + boxSize/sp.Integer(2)) - sqrt(d3SquaredTarget))

print("\nq3SolutionPlus =")
pprint(q3SolutionPlus, use_unicode=True)
print("\nq3SolutionMinus =")
pprint(q3SolutionMinus, use_unicode=True)
print('\nCondizione di esistenza reale: x^2 + y^2 + (z - d1)^2 - a2^2 >= 0')


# AA ======================= Sezione Tre: Risoluzione Analitica Del Giunto Di Spalla q2 =======================

# BB Impostazione Del Sistema Di Equazioni Lineari Rispetto A Seno E Coseno Di q2
# DD Risoluzione Mediante La Inversione Della Matrice Dei Coefficienti Del Sistema Lineare
print("\n" + "="*100)
print("3) CALCOLO DI q2 PASSO PER PASSO")
print("="*100)

print("\n" + "-"*100)
print("Partenza da sqrt(x^2+y^2) = d3 sin(q2) + a2 cos(q2) e z-d1 = d3 cos(q2) - a2 sin(q2)")
print("-"*100)
radialDistance = sp.symbols('radialDistance', real=True)
radialDistanceDefinition = sqrt(x**2 + y**2)
print("\nradialDistanceDefinition =")
pprint(radialDistanceDefinition, use_unicode=True)

coefficientMatrix = Matrix([[d3, a2], [-a2, d3]])
unknownVector = Matrix([sp.Symbol('sinq2'), sp.Symbol('cosq2')])
knownVectorPlus = Matrix([radialDistance, z - d1])
knownVectorMinus = Matrix([-radialDistance, z - d1])

print("\ncoefficientMatrix =")
pprint(coefficientMatrix, use_unicode=True)
print("\nknownVectorPlus =")
pprint(knownVectorPlus, use_unicode=True)
print("\nknownVectorMinus =")
pprint(knownVectorMinus, use_unicode=True)

# BB Calcolo Del Determinante Della Matrice Dei Coefficienti Del Giunto Di Spalla
print("\n" + "-"*100)
print("Determinante della matrice dei coefficienti")
print("-"*100)
determinantCoefficientMatrix = simplify(coefficientMatrix.det())
print("\ndeterminantCoefficientMatrix =")
pprint(determinantCoefficientMatrix, use_unicode=True)

# BB Calcolo Della Matrice Inversa Per Isolare Le Funzioni Trigonometriche
print("\n" + "-"*100)
print("Inversa della matrice dei coefficienti")
print("-"*100)
inverseCoefficientMatrix = simplify(coefficientMatrix.inv())
print("\ninverseCoefficientMatrix =")
pprint(inverseCoefficientMatrix, use_unicode=True)

# BB Calcolo Dei Vettori Soluzione Contenenti Seno E Coseno Per Entrambi I Rami Riferiti Alla Distanza
print("\n" + "-"*100)
print("Soluzione per sin(q2), cos(q2) con il ramo positivo")
print("-"*100)
solutionVectorPlus = simplify(inverseCoefficientMatrix * knownVectorPlus)
print("\nsolutionVectorPlus =")
pprint(solutionVectorPlus, use_unicode=True)

print("\n" + "-"*100)
print("Soluzione per sin(q2), cos(q2) con il ramo negativo")
print("-"*100)
solutionVectorMinus = simplify(inverseCoefficientMatrix * knownVectorMinus)
print("\nsolutionVectorMinus =")
pprint(solutionVectorMinus, use_unicode=True)

# BB Calcolo Dello Angolo q2 Mediante La Arcotangente A Quattro Quadranti
# DD Estrazione Delle Due Espressioni In Forma Compatta Semplificata Come Riportate Nel PDF
print("\n" + "-"*100)
print("Formula finale di q2 tramite atan2")
print("-"*100)
q2SolutionPlus = simplify(atan2(solutionVectorPlus[0], solutionVectorPlus[1]))
q2SolutionMinus = simplify(atan2(solutionVectorMinus[0], solutionVectorMinus[1]))

print("\nq2SolutionPlus =")
pprint(q2SolutionPlus, use_unicode=True)
print("\nq2SolutionMinus =")
pprint(q2SolutionMinus, use_unicode=True)

q2SolutionPlusPdf = atan2(d3*sqrt(x**2+y**2) - a2*(z-d1), a2*sqrt(x**2+y**2) + d3*(z-d1))
q2SolutionMinusPdf = atan2(-d3*sqrt(x**2+y**2) - a2*(z-d1), -a2*sqrt(x**2+y**2) + d3*(z-d1))

print("\nq2SolutionPlusPdf =")
pprint(q2SolutionPlusPdf, use_unicode=True)
print("\nq2SolutionMinusPdf =")
pprint(q2SolutionMinusPdf, use_unicode=True)


# AA ======================= Sezione Quattro: Risoluzione Analitica Del Giunto Di Base q1 =======================

# BB Fattorizzazione Delle Espressioni Scalari Cartesiane Per Semplificare Lo Angolo Di Rotazione Primario
# DD Isolo Le Funzioni Di Seno E Coseno Rispetto Al Fattore Di Scala Comune A
print("\n" + "="*100)
print("4) CALCOLO DI q1 PASSO PER PASSO")
print("="*100)

print("\n" + "-"*100)
print("Fattorizzazione delle prime due equazioni")
print("-"*100)
scalarCoefficientA = simplify(d3*sin(q2) + a2*cos(q2))
print("\nscalarCoefficientA =")
pprint(scalarCoefficientA, use_unicode=True)
print("\nx_factored =")
pprint(trigsimp(scalarCoefficientA*cos(q1)), use_unicode=True)
print("\ny_factored =")
pprint(trigsimp(scalarCoefficientA*sin(q1)), use_unicode=True)

# BB Calcolo Dei Rapporti Per Lo Isolamento Di Seno E Coseno
print("\n" + "-"*100)
print("Isolamento di cos(q1) e sin(q1)")
print("-"*100)
cosQ1 = simplify(x / scalarCoefficientA)
sinQ1 = simplify(y / scalarCoefficientA)
print("\ncosQ1 =")
pprint(cosQ1, use_unicode=True)
print("\nsinQ1 =")
pprint(sinQ1, use_unicode=True)

# BB Estrazione Della Formula Finale Di q1 Tramite Arcotangente A Quattro Quadranti
print("\n" + "-"*100)
print("Formula finale di q1 tramite atan2")
print("-"*100)
q1Final = atan2(y/scalarCoefficientA, x/scalarCoefficientA)
print("\nq1Final =")
pprint(q1Final, use_unicode=True)


# AA ======================= Sezione Cinque: Analisi Dello Spazio Di Lavoro E Dei Limiti Fisici =======================

# BB Calcolo Della Distanza Radiale Rd Dal Giunto Due Allo End-Effector Nel Piano Verticale
# DD Determinazione Dei Limiti Estremi Sostituendo Le Configurazioni Limite Del Prismatico q3
print("\n" + "="*100)
print("5) SPAZIO DI LAVORO E LIMITI")
print("="*100)

print("\n" + "-"*100)
print("Distanza dal giunto 2 all’end-effector")
print("-"*100)
radialDistanceD = sqrt(a2**2 + d3**2)
print("\nradialDistanceD =")
pprint(radialDistanceD, use_unicode=True)

print("\n" + "-"*100)
print("Raggi minimo e massimo della corona nel piano di sezione")
print("-"*100)
radialDistanceDMax = simplify(radialDistanceD.subs(q3, 0).subs(d3, q3 - (l3 + boxSize/sp.Integer(2))))
radialDistanceDMax = simplify(radialDistanceDMax)
radialDistanceDMin = simplify(radialDistanceD.subs(q3, l3).subs(d3, q3 - (l3 + boxSize/sp.Integer(2))))
radialDistanceDMin = simplify(radialDistanceDMin)

print("\nradialDistanceDMax =")
pprint(radialDistanceDMax, use_unicode=True)
print("\nradialDistanceDMin =")
pprint(radialDistanceDMin, use_unicode=True)

# BB Definizione Degli Intervalli Fisici E Dei Limiti Di Escursione Per Ciascuna Variabile Di Giunto
print("\n" + "-"*100)
print("Limiti di q3 e di d3")
print("-"*100)
q3Interval = sp.Interval(0, l3)
print("\nq3Interval =")
pprint(q3Interval, use_unicode=True)

d3MinExpression = simplify((q3 - (l3 + boxSize/sp.Integer(2))).subs(q3, 0))
d3MaxExpression = simplify((q3 - (l3 + boxSize/sp.Integer(2))).subs(q3, l3))
print("\nd3MinExpression =")
pprint(d3MinExpression, use_unicode=True)
print("\nd3MaxExpression =")
pprint(d3MaxExpression, use_unicode=True)

print("\n" + "-"*100)
print("Limiti di q2 e q1")
print("-"*100)
print("\nq2Interval =")
pprint(sp.Interval(-pi/sp.Integer(2), pi/sp.Integer(4)), use_unicode=True)
print("\nq1Interval =")
pprint(sp.Interval(-pi, pi), use_unicode=True)

# BB Calcolo Numerico Degli Intervalli Rispetto Ai Parametri Geometrici Reali Del PDF
print("\n" + "-"*100)
print("Caso numerico del pdf: l3 = 0.35, boxSize = 0.10")
print("-"*100)
numericalParametersBasic = {l3: sp.Rational(35, 100), boxSize: sp.Rational(10, 100)}
print("\nd3MinNumerical =")
pprint(N(d3MinExpression.subs(numericalParametersBasic), 6), use_unicode=True)
print("\nd3MaxNumerical =")
pprint(N(d3MaxExpression.subs(numericalParametersBasic), 6), use_unicode=True)
print("\nq3NumericalInterval =")
pprint(sp.Interval(0, sp.Rational(35, 100)), use_unicode=True)
print("\nq2NumericalInterval =")
pprint(sp.Interval(N(-pi/sp.Integer(2), 6), N(pi/sp.Integer(4), 6)), use_unicode=True)
print("\nq1NumericalInterval =")
pprint(sp.Interval(N(-pi, 6), N(pi, 6)), use_unicode=True)

print("\n" + "-"*100)
print("Caso numerico del pdf per a2: a2 = 0.55")
print("-"*100)
numericalParameterA2 = {a2: sp.Rational(55, 100)}
print("\na2Numerical =")
pprint(N(a2.subs(numericalParameterA2), 6), use_unicode=True)


# AA ======================= Sezione Sei: Derivazione Della Matrice Jacobiana Posizionale E Sviluppo Del Determinante =======================

# BB Calcolo Delle Derivate Parziali Esplicite Per Ciascun Asse Cartesiano Rispetto Alle Variabili Di Giunto
# DD Le Derivate Calcolate Rispetto A q3 Tengono Conto Della Equazione d3 In Funzione Di q3
print("\n" + "="*100)
print("6) JACOBIANA POSIZIONALE E SINGOLARITÀ")
print("="*100)

print("\n" + "-"*100)
print("Derivate parziali esplicite")
print("-"*100)
dxDq1 = simplify(sp.diff(xExpression, q1))
dxDq2 = simplify(sp.diff(xExpression, q2))
dxDq3 = simplify(sp.diff(xExpression.subs(d3, q3 - (l3 + boxSize/sp.Integer(2))), q3))

dyDq1 = simplify(sp.diff(yExpression, q1))
dyDq2 = simplify(sp.diff(yExpression, q2))
dyDq3 = simplify(sp.diff(yExpression.subs(d3, q3 - (l3 + boxSize/sp.Integer(2))), q3))

dzDq1 = simplify(sp.diff(zExpression, q1))
dzDq2 = simplify(sp.diff(zExpression, q2))
dzDq3 = simplify(sp.diff(zExpression.subs(d3, q3 - (l3 + boxSize/sp.Integer(2))), q3))

print("\ndxDq1 =")
pprint(dxDq1, use_unicode=True)
print("\ndxDq2 =")
pprint(dxDq2, use_unicode=True)
print("\ndxDq3 =")
pprint(dxDq3, use_unicode=True)
print("\ndyDq1 =")
pprint(dyDq1, use_unicode=True)
print("\ndyDq2 =")
pprint(dyDq2, use_unicode=True)
print("\ndyDq3 =")
pprint(dyDq3, use_unicode=True)
print("\ndzDq1 =")
pprint(dzDq1, use_unicode=True)
print("\ndzDq2 =")
pprint(dzDq2, use_unicode=True)
print("\ndzDq3 =")
pprint(dzDq3, use_unicode=True)

# BB Definizione Dei Coefficienti Compatti A E B Utilizzati Nella Documentazione Analitica
print("\n" + "-"*100)
print("Definizione compatta A e B")
print("-"*100)
scalarCoefficientB = simplify(d3*cos(q2) - a2*sin(q2))
print("\nscalarCoefficientA =")
pprint(scalarCoefficientA, use_unicode=True)
print("\nscalarCoefficientB =")
pprint(scalarCoefficientB, use_unicode=True)

# BB Costruzione Della Matrice Jacobiana Posizionale Completa E Di Quella Compatta Semplificata
# DD Verifica Della Coerenza Delle Due Formulazioni Tramite Sottrazione Diretta
jacobianMatrix = Matrix([
    [dxDq1, dxDq2, dxDq3],
    [dyDq1, dyDq2, dyDq3],
    [dzDq1, dzDq2, dzDq3],
])
print("\njacobianMatrix =")
pprint(jacobianMatrix, use_unicode=True)

compactJacobianMatrix = Matrix([
    [-scalarCoefficientA*sin(q1), scalarCoefficientB*cos(q1), sin(q2)*cos(q1)],
    [ scalarCoefficientA*cos(q1), scalarCoefficientB*sin(q1), sin(q2)*sin(q1)],
    [          0,       -scalarCoefficientA,          cos(q2)]
])
print("\ncompactJacobianMatrix =")
pprint(compactJacobianMatrix, use_unicode=True)

print("\n" + "-"*100)
print("Verifica che le due forme coincidano")
print("-"*100)
jacobianDifference = simplify(jacobianMatrix - compactJacobianMatrix)
print("\njacobianDifference =")
pprint(jacobianDifference, use_unicode=True)

# BB Calcolo Analitico Del Determinante Della Matrice Jacobiana
# DD Sviluppo Ed Espansione Dei Termini Rispetto Ai Coefficienti Compatti Sostituiti
print("\n" + "-"*100)
print("Determinante della Jacobiana")
print("-"*100)
determinantJacobianMatrix = factor(trigsimp(simplify(jacobianMatrix.det())))
print("\ndeterminantJacobianMatrix =")
pprint(determinantJacobianMatrix, use_unicode=True)

print("\n" + "-"*100)
print("Sviluppo del determinante come nel pdf")
print("-"*100)
determinantJacobianMatrixSubstituted = factor(trigsimp(determinantJacobianMatrix.subs({
    scalarCoefficientA: d3*sin(q2)+a2*cos(q2),
    scalarCoefficientB: d3*cos(q2)-a2*sin(q2)
})))
print("\ndeterminantJacobianMatrixSubstituted =")
pprint(determinantJacobianMatrixSubstituted, use_unicode=True)
print("\ndeterminantJacobianMatrixExpanded =")
pprint(trigsimp(expand(determinantJacobianMatrixSubstituted)), use_unicode=True)

print("\n" + "-"*100)
print("Condizioni singolari")
print("-"*100)
print("\nsingularityCondition1 =")
pprint(Eq(d3, 0), use_unicode=True)
print("\nsingularityCondition2 =")
pprint(Eq(d3*sin(q2) + a2*cos(q2), 0), use_unicode=True)


# AA ======================= Sezione Sette: Analisi Numerica E Geometrica Delle Singolarità Meccaniche Non Raggiungibili =======================

# BB Analisi Della Prima Condizione Di Singolarità Riferita Al Giunto Prismatico
# DD Dimostrazione Che La Variabile d3 Rimane Sempre Negativa Entro I Suoi Limiti Di Corsa Fisici
print("\n" + "="*100)
print("7) ANALISI DELLE SINGOLARITÀ NON RAGGIUNGIBILI")
print("="*100)

print("\n" + "-"*100)
print("Singolarità del giunto prismatico: d3 = 0")
print("-"*100)
print('Dal pdf: d3 = q3 - (l3 + boxSize/2)')
print('Con q3 in [0, l3], l3 = 0.35 m e boxSize = 0.10 m:')
print('d3 appartiene a [-0.40, -0.05], quindi d3 non può annullarsi.')
print("\nd3NumericalInterval =")
pprint(sp.Interval(sp.Rational(-40, 100), sp.Rational(-5, 100)), use_unicode=True)

# BB Analisi Della Seconda Condizione Di Singolarità Riferita Dello Allineamento Con Lo Asse Verticale Di Spalla
# DD Dimostrazione Che Lo Angolo Richiesto Per La Singolarita Supera Ampiamente I Limiti Di Rotazione Di q2
print("\n" + "-"*100)
print("Singolarità di spalla/allineamento: d3 sin(q2) + a2 cos(q2) = 0")
print("-"*100)
print("\nsingularityEquation =")
pprint(Eq(d3*sin(q2) + a2*cos(q2), 0), use_unicode=True)
print("\nsingularityStep1 =")
pprint(Eq(d3*sin(q2), -a2*cos(q2)), use_unicode=True)
print("\nsingularityStep2 =")
pprint(Eq(sp.tan(q2), -a2/d3), use_unicode=True)

print('\nPoiché a2 > 0 e d3 < 0 nell’intervallo fisico, il rapporto -a2/d3 è sempre positivo, quindi q2_sing > 0.')

print("\n" + "-"*100)
print("Estremi numerici del pdf: a2 = 0.55, d3 in [-0.40, -0.05]")
print("-"*100)
q2SingularMin = sp.atan(sp.Rational(55, 100) / sp.Rational(40, 100))
q2SingularMax = sp.atan(sp.Rational(55, 100) / sp.Rational(5, 100))

print("\nq2SingularMinRad =")
pprint(N(q2SingularMin, 8), use_unicode=True)
print("\nq2SingularMinDeg =")
pprint(N(sp.deg(q2SingularMin), 8), use_unicode=True)
print("\nq2SingularMaxRad =")
pprint(N(q2SingularMax, 8), use_unicode=True)
print("\nq2SingularMaxDeg =")
pprint(N(sp.deg(q2SingularMax), 8), use_unicode=True)

print("\n" + "-"*100)
print("Confronto con il limite fisico di q2")
print("-"*100)
print("\nq2PhysicalIntervalDeg =")
pprint(sp.Interval(-90, 45), use_unicode=True)
print("\nq2SingularIntervalDeg =")
pprint(sp.Interval(N(sp.deg(q2SingularMin), 6), N(sp.deg(q2SingularMax), 6)), use_unicode=True)
print('\nPoiché il minimo angolo singolare è circa 53.97° e il massimo fisicamente ammesso è 45°, anche questa singolarità è non raggiungibile.')


# AA ======================= Sezione Otto: Formule Analitiche Finali Della Cinematica Inversa =======================

# BB Riassunto Delle Soluzioni Ottenute Passo Per Passo Per La Implementazione Dello Algoritmo
# DD Raccolta Dei Risultati Per Ciascun Giunto q1, q2, q3 Dai Rispettivi Sviluppi Matematici
print("\n" + "="*100)
print("8) FORMULE FINALI RIASSUNTIVE COME NEL PDF")
print("="*100)

print("\nq3SolutionPlusFinal =")
pprint(q3SolutionPlus, use_unicode=True)
print("\nq3SolutionMinusFinal =")
pprint(q3SolutionMinus, use_unicode=True)
print("\nq2SolutionPlusPdfFinal =")
pprint(q2SolutionPlusPdf, use_unicode=True)
print("\nq2SolutionMinusPdfFinal =")
pprint(q2SolutionMinusPdf, use_unicode=True)
print("\nq1FinalFormula =")
pprint(q1Final, use_unicode=True)

print('\nNOTA: q2 ha due rami principali legati al segno di sqrt(x^2+y^2), corrispondenti alle configurazioni tipo gomito alto / gomito basso;')
print('la soluzione fisicamente ammessa va poi selezionata imponendo i limiti dei giunti.')