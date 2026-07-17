import sympy as sp

from sympy import symbols, sin, cos, Matrix, latex

# Variabili simboliche
q1, q2, d3, a2 = symbols('q1 q2 d3 a2')
A, B = symbols('A B')

# Matrice con A e B come simboli
J = Matrix([
    [-A*sin(q1), B*cos(q1), sin(q2)*cos(q1)],
    [ A*cos(q1), B*sin(q1), sin(q2)*sin(q1)],
    [        0,       -A,       cos(q2)]
])

sp.pprint(J)

print("Determinante Matrice J:")
det_J = J.det()
det_J = sp.simplify(det_J)
sp.pprint(det_J)

print("Determinante Matrice J (Codice Latex):")
print(latex(det_J))
