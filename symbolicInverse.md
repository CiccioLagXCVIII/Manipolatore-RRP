# Cinematica Inversa Simbolica

## Prima Parte
Le coordinate target ovviamente sono espresse rispetto a RF World, quindi si devono trasformare nel sistema di riferimento del primo giunto.
Rispetto a RF World, il primo giunto RF0 si trova a 2m e sulla base che è alta 6cm, e quindi avremo una traslazione di 2m lungo X e -baseHeight lungo Z. Quindi:

```
x = xTarget - 2.0
y = yTarget
z = zTarget - 0.06
```

Dall'equazione ${}^{0}T_{EE} = {}^{0}T_{1} \cdot {}^{1}T_{2} \cdot {}^{2}T_{3} = {}^{0}T_{2} \cdot {}^{2}T_{3}$, svolgendo il prodotto matriciale a blocchi si ottiene che ${}^{0}d_{EE}$ (la posizione dell'end effector rispetto a RF0, che è l'input traslato) è data da ${}^{0}R_{2} \cdot {}^{2}d_{EE} + {}^{0}d_{2}$.

Siccome ${}^{0}R_{2}(q_1, q_2) = {}^{0}R_{1}(q_1){}^{1}R_{2}(q_2)$, sostituendo si ottiene ${}^{0}R_{1}(q_1){}^{1}R_{2}(q_2) \cdot {}^{2}d_{EE} + {}^{0}d_{2}(q_1, q_2)$.

Sapendo che ${}^{2}d_{EE}(q_3)$ è la posizione dell'EE vista dal frame 2 (posizione, non posa — per posa si intende posizione e rotazione), e ${}^{1}R_{2}(q_2)$ è la matrice di rotazione che trasforma le coordinate da RF2 a RF1, siccome le matrici di rotazione permettono di "allineare" i sistemi di riferimento, possiamo dire che ${}^{1}R_{2}(q_2) \cdot {}^{2}d_{EE}(q_3)$ è il vettore posizione dell'end effector rispetto a RF2 trasformato in RF1 dalla matrice di rotazione.

Nel nostro caso la posizione dell'EE rispetto a RF2 corrisponde a una traslazione di $q_3 - \left(l_3 + \frac{\text{boxSize}}{2}\right)$ lungo $Z_2$ (asse Z di RF2), e quindi:

$${}^{2}d_{EE}(q_3) = \begin{bmatrix} 0 \\ 0 \\ q_3 - \left(l_3 + \frac{\text{boxSize}}{2}\right) \end{bmatrix}$$

Infatti con $q_2 = 0$, ovvero prima che venga applicata la rotazione di $q_2$ attorno all'asse $Y_1$ (asse Y di RF1) che è l'asse di rotazione del giunto 2, l'EE si trova a una distanza pari a $\left(l_3 + \frac{\text{boxSize}}{2}\right)$ lungo $Z_2$ da RF2 (lunghezza del link 3 + metà della lunghezza del lato del box lungo la direzione negativa di $Z_2$).

Andando a sostituire:

$${}^{0}d_{EE} = {}^{0}R_{1}(q_1) \; {}^{1}R_{2}(q_2) \begin{bmatrix} 0 \\ 0 \\ q_3 - \left(l_3 + \frac{\text{boxSize}}{2}\right) \end{bmatrix} + {}^{0}d_{2}$$

Ricordando che la formulazione standard della matrice di trasformazione DH $A_i$ (ovvero ${}^{i-1}T_i$) per un giunto $i$ generico è:

$${}^{i-1}\mathbf{A}_i(q_i) = \begin{bmatrix} \cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\ \sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\ 0 & \sin\alpha_i & \cos\alpha_i & d_i \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

Si possono calcolare:

$${}^{0}\mathbf{A}_1(q_1) = \begin{bmatrix} \cos q_1 & -\sin q_1 \cos\!\left(-\tfrac{\pi}{2}\right) & \sin q_1 \sin\!\left(-\tfrac{\pi}{2}\right) & 0 \\ \sin q_1 & \cos q_1 \cos\!\left(-\tfrac{\pi}{2}\right) & -\cos q_1 \sin\!\left(-\tfrac{\pi}{2}\right) & 0 \\ 0 & \sin\!\left(-\tfrac{\pi}{2}\right) & \cos\!\left(-\tfrac{\pi}{2}\right) & 2\,\text{jointRadius} + l_1 + \text{jointRadius} \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

$${}^{1}\mathbf{A}_2(q_2) = \begin{bmatrix} \cos q_2 & -\sin q_2 \cos\!\left(\tfrac{\pi}{2}\right) & \sin q_2 \sin\!\left(\tfrac{\pi}{2}\right) & \left(\text{jointRadius} + l_2 + \tfrac{\text{boxSize}}{2}\right)\cos q_2 \\ \sin q_2 & \cos q_2 \cos\!\left(\tfrac{\pi}{2}\right) & -\cos q_2 \sin\!\left(\tfrac{\pi}{2}\right) & \left(\text{jointRadius} + l_2 + \tfrac{\text{boxSize}}{2}\right)\sin q_2 \\ 0 & \sin\!\left(\tfrac{\pi}{2}\right) & \cos\!\left(\tfrac{\pi}{2}\right) & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

$${}^{2}\mathbf{A}_\text{EE}(q_3) = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & -1 & 0 & 0 \\ 0 & 0 & -1 & q_3 - \left(l_3 + \tfrac{\text{boxSize}}{2}\right) \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

Ovvero, sostituendo i valori di $\alpha_i$:

$${}^{0}\mathbf{A}_1(q_1) = \begin{bmatrix} \cos q_1 & 0 & -\sin q_1 & 0 \\ \sin q_1 & 0 & \cos q_1 & 0 \\ 0 & -1 & 0 & 3\,\text{jointRadius} + l_1 \\ 0 & 0 & 0 & 1 \end{bmatrix} \qquad {}^{1}\mathbf{A}_2(q_2) = \begin{bmatrix} \cos q_2 & 0 & \sin q_2 & \left(\text{jointRadius} + l_2 + \tfrac{\text{boxSize}}{2}\right)\cos q_2 \\ \sin q_2 & 0 & -\cos q_2 & \left(\text{jointRadius} + l_2 + \tfrac{\text{boxSize}}{2}\right)\sin q_2 \\ 0 & 1 & 0 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

$${}^{2}\mathbf{A}_\text{EE}(q_3) = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & -1 & 0 & 0 \\ 0 & 0 & -1 & q_3 - \left(l_3 + \tfrac{\text{boxSize}}{2}\right) \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

Da cui si può calcolare ${}^{0}T_2(q_1, q_2) = {}^{0}A_1(q_1) \cdot {}^{1}A_2(q_2)$:

$${}^{0}\mathbf{A}_2(q_1, q_2) = \begin{bmatrix} \cos q_1 \cos q_2 & -\sin q_1 & \sin q_2 \cos q_1 & \left(\tfrac{\text{boxSize}}{2} + \text{jointRadius} + l_2\right)\cos q_1 \cos q_2 \\ \sin q_1 \cos q_2 & \cos q_1 & \sin q_1 \sin q_2 & \left(\tfrac{\text{boxSize}}{2} + \text{jointRadius} + l_2\right)\sin q_1 \cos q_2 \\ -\sin q_2 & 0 & \cos q_2 & 3\,\text{jointRadius} + l_1 - \left(\tfrac{\text{boxSize}}{2} + \text{jointRadius} + l_2\right)\sin q_2 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

Estraendo le prime 3 righe della quarta colonna di ${}^{0}A_2(q_1, q_2)$ si ottiene il vettore posizione di RF2 rispetto a RF0, ${}^{0}d_2(q_1, q_2)$; estraendo le prime 3 righe della quarta colonna di ${}^{2}A_\text{EE}(q_3)$ si ottiene il vettore posizione dell'EE rispetto al frame 2, ${}^{2}d_\text{EE}(q_3)$:

$${}^{0}\mathbf{d}_2(q_1, q_2) = \begin{bmatrix} \left(\tfrac{\text{boxSize}}{2} + \text{jointRadius} + l_2\right)\cos q_1 \cos q_2 \\ \left(\tfrac{\text{boxSize}}{2} + \text{jointRadius} + l_2\right)\sin q_1 \cos q_2 \\ 3\,\text{jointRadius} + l_1 - \left(\tfrac{\text{boxSize}}{2} + \text{jointRadius} + l_2\right)\sin q_2 \end{bmatrix} \qquad {}^{2}\mathbf{d}_\text{EE}(q_3) = \begin{bmatrix} 0 \\ 0 \\ q_3 - \left(l_3 + \tfrac{\text{boxSize}}{2}\right) \end{bmatrix}$$

Estraendo le prime 3 righe e le prime 3 colonne di $A_{01}$ e $A_{12}$ si ottengono rispettivamente ${}^{0}R_1(q_1)$ e ${}^{1}R_2(q_2)$, che devono essere estratte direttamente da $A_{01}$ e $A_{12}$ per includere le rotazioni:

$${}^{0}\mathbf{R}_1(q_1) = \begin{bmatrix} \cos q_1 & 0 & -\sin q_1 \\ \sin q_1 & 0 & \cos q_1 \\ 0 & -1 & 0 \end{bmatrix} \qquad {}^{1}\mathbf{R}_2(q_2) = \begin{bmatrix} \cos q_2 & 0 & \sin q_2 \\ \sin q_2 & 0 & -\cos q_2 \\ 0 & 1 & 0 \end{bmatrix}$$

Mettendo tutto insieme si ottiene:

$${}^{0}d_{EE} = \underbrace{\begin{bmatrix} \cos q_1 & 0 & -\sin q_1 \\ \sin q_1 & 0 & \cos q_1 \\ 0 & -1 & 0 \end{bmatrix}}_{{}^{0}\mathbf{R}_1(q_1)} \underbrace{\begin{bmatrix} \cos q_2 & 0 & \sin q_2 \\ \sin q_2 & 0 & -\cos q_2 \\ 0 & 1 & 0 \end{bmatrix}}_{{}^{1}\mathbf{R}_2(q_2)} \underbrace{\begin{bmatrix} 0 \\ 0 \\ q_3 - \left(l_3 + \tfrac{\text{boxSize}}{2}\right) \end{bmatrix}}_{{}^{2}\mathbf{d}_\text{EE}(q_3)} + \underbrace{\begin{bmatrix} \left(\tfrac{\text{boxSize}}{2} + \text{jointRadius} + l_2\right)\cos q_1 \cos q_2 \\ \left(\tfrac{\text{boxSize}}{2} + \text{jointRadius} + l_2\right)\sin q_1 \cos q_2 \\ 3\,\text{jointRadius} + l_1 - \left(\tfrac{\text{boxSize}}{2} + \text{jointRadius} + l_2\right)\sin q_2 \end{bmatrix}}_{{}^{0}\mathbf{d}_2(q_1,q_2)}$$

Quindi ponendo:

$$d_1 = 2\,\text{jointRadius} + l_1 + \text{jointRadius} = 3\,\text{jointRadius} + l_1$$
$$a_2 = \text{jointRadius} + l_2 + \frac{\text{boxSize}}{2}$$
$$d_3 = q_3 - \left(l_3 + \frac{\text{boxSize}}{2}\right)$$

la precedente equazione diventa:

$${}^{0}d_{EE} = \begin{bmatrix} \cos q_1 & 0 & -\sin q_1 \\ \sin q_1 & 0 & \cos q_1 \\ 0 & -1 & 0 \end{bmatrix} \begin{bmatrix} \cos q_2 & 0 & \sin q_2 \\ \sin q_2 & 0 & -\cos q_2 \\ 0 & 1 & 0 \end{bmatrix} \begin{bmatrix} 0 \\ 0 \\ d_3 \end{bmatrix} + \begin{bmatrix} a_2 \cos q_1 \cos q_2 \\ a_2 \sin q_1 \cos q_2 \\ d_1 - a_2 \sin q_2 \end{bmatrix}$$

Svolgendo i calcoli si ottiene un vettore in cui ogni elemento è una funzione di $q_1$, $q_2$ e $q_3$, e quindi si può risolvere il sistema di equazioni per trovare i valori dei giunti dati i target di posizione dell'end effector:

$${}^{0}d_{EE} = \begin{bmatrix} \cos q_1 \cos q_2 & -\sin q_1 & \sin q_2 \cos q_1 \\ \sin q_1 \cos q_2 & \cos q_1 & \sin q_1 \sin q_2 \\ -\sin q_2 & 0 & \cos q_2 \end{bmatrix} \begin{bmatrix} 0 \\ 0 \\ d_3 \end{bmatrix} + \begin{bmatrix} a_2 \cos q_1 \cos q_2 \\ a_2 \sin q_1 \cos q_2 \\ d_1 - a_2 \sin q_2 \end{bmatrix} = \begin{bmatrix} d_3 \sin q_2 \cos q_1 + a_2 \cos q_1 \cos q_2 \\ d_3 \sin q_1 \sin q_2 + a_2 \sin q_1 \cos q_2 \\ d_3 \cos q_2 + d_1 - a_2 \sin q_2 \end{bmatrix}$$

Il calcolo matriciale è stato fatto con Python quindi è corretto (a meno di errori nella definizione delle matrici simboliche).

## Seconda Parte

