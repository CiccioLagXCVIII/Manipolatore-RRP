## Struttura Robot Dal File XACRO

Catena Cinematica Del Robot RRP:

```text
world  -->  [Fixed: 0 0 0]                                              -->  base
base   -->  [giunto1: Rotazione Attorno Z, origin 0 0 baseHeight]       -->  RF0
RF0    -->  [giunto2: Rotazione Attorno Y, origin 0 0 l1+2*jointRadius] -->  RF1  
RF1    -->  [giunto3: prismatic Z, origin l2+2*jointRadius 0 0]         -->  RF2
RF2    -->  [Fixed: 0 0 -l3, rpy π 0 0]                                 -->  endEffector
```

---

## Assegnazione Frame DH

**Convenzione DH:** Asse $Z_i$ Lungo L'Asse Di Giunto $i+1$, Asse $X_i$ Normale Comune Tra $Z_{i-1}$ e $Z_i$

* **RF Base:**
  * $Z_{\text{Base}}$ Deve Essere Lungo L'Asse Del Giunto0 $\rightarrow$ **$Z_{\text{Base}} = Z_0$**
* **RF0:**
  * $Z_0$ Deve Essere Lungo L'Asse Del Giunto1 $\rightarrow$ **$Z_0 = Z_1$** (*giunto1 Ruota Attorno A Z*)
* **RF1:**
  * $Z_1$ Deve Essere Lungo L'Asse Del Giunto2 $\rightarrow$ **$Z_1 = Y_0$** (*giunto2 Ruota Attorno A Y*)
  * $X_1$ = normale comune tra $Z_0$ e $Z_1$
* **RF2:**
  * $Z_2$ Deve Essere Lungo L'Asse Del Giunto3 $\rightarrow$ **$Z_2 = Z_1$** (*giunto3 Trasla Lungo Z1, Ovvero Lungo L'Asse Z Di RF1*)
  * Giunto3 Ha `origin xyz="l2+2*jointRadius 0 0"` Quindi Si Ha Un Offset Lungo $X_1$

---

## Parametri DH

Ricordiamo la convenzione: 
$$^{i-1}T_i = R_z(\theta_i) \cdot T_z(d_i) \cdot T_x(a_i) \cdot R_x(\alpha_i)$$

| $i$ | $\theta_i$ | $d_i$ | $a_i$ | $\alpha_i$ | Note |
| :---: | :---: | :---: | :---: | :---: | :--- |
| **1** | $q_1$ | $l_1 + 2 \cdot \text{jointRadius}$ | $0$ | $-\pi/2$ | Giunto Revolute; $Z_0 \rightarrow Z_1$ Rotazione Di $-90^\circ$ Attorno A $X_1$ Perché $Z_1 = Y_0$ (*Giunto2 Ruota Attorno A $Y_0$ Se Ci Pensi*) |
| **2** | $q_2$ | $0$ | $l_2 + 2 \cdot \text{jointRadius}$ | $+\pi/2$ | Giunto Revolute Y; Offset $l_2+2 \cdot \text{jointRadius}$ Lungo $X_2$; $Z_2$ È Parallelo A $Z_0$ |
| **3** | $0$ | $q_3$ | $0$ | $0$ | Giunto Prismatico Lungo $Z_2$; Non Ruota Quindi Theta Non Cambia, Cambia d |

### Analisi Geometrica dei Singoli Link

#### Link 1 ($\theta_1 = q_1$, $d_1 = l_1 + 2 \cdot \text{jointRadius}$, $a_1 = 0$, $\alpha_1 = -\pi/2$)
* **Asse del Giunto ($Z_0$):** Diretto lungo la verticale.
* **Asse del Giunto Successivo ($Z_1$):** Poiché il giunto 2 ruota attorno a $Y_0$ (asse di beccheggio), l'asse $Z_1$ deve essere orientato lungo l'asse di rotazione di questo secondo giunto.
* **Angolo $\alpha_1 = -\pi/2$:** Per allineare il vettore $Z_0$ (verticale) con $Z_1$ (orizzontale lungo $Y_0$), è necessario eseguire una rotazione di $-90^\circ$ attorno all'asse normale comune $X_1$. Utilizzando la regola della mano destra, una rotazione negativa attorno a $X_1$ orienta correttamente $Z_0$ sull'asse orizzontale.
* **Spostamento $d_1$:** Rappresenta la distanza lungo $Z_0$ dall'origine di frame 0 fino all'asse $Z_1$. Se l'origine di frame 0 viene posta sulla sommità del blocco di base (all'altezza del giunto 1), il valore $l_1 + 2 \cdot \text{jointRadius}$ è corretto.

#### Link 2 ($\theta_2 = q_2$, $d_2 = 0$, $a_2 = l_2 + 2 \cdot \text{jointRadius}$, $\alpha_2 = +\pi/2$)
* **Asse del Giunto ($Z_1$):** Orizzontale, parallelo a $Y_0$.
* **Asse del Giunto Successivo ($Z_2$):** Coincide con la direzione di scorrimento del giunto prismatico, ovvero l'asse locale $Z$ del link $RF1$.
* **Angolo $\alpha_2 = +\pi/2$:** Per riportare l'asse $Z_1$ (orizzontale) sulla direzione verticale locale $Z_2$, occorre una rotazione di $+90^\circ$ attorno alla normale comune $X_2$.
* **Distanza $a_2 = l_2 + 2 \cdot \text{jointRadius}$:** Questo valore rappresenta lo sbraccio orizzontale del braccio ed è misurato lungo la normale comune $X_2$ (parallela alla direzione di estensione del braccio).

#### Link 3 ($\theta_3 = 0$, $d_3 = q_3$, $a_3 = 0$, $\alpha_3 = 0$)
* **Giunto Prismatico:** Poiché il giunto compie unicamente una traslazione lineare senza ruotare, l'angolo di giunto $\theta_3$ rimane nullo e costante, mentre la variabile di giunto corrisponde allo spostamento lineare $d_3 = q_3$ lungo l'asse $Z_2$.

---

### Gestione degli Offset Generali (`baseHeight` e `world`)

Nel definire la cinematica globale rispetto a `world`, occorre prestare attenzione a dove si sceglie di posizionare l'origine del frame di riferimento iniziale (Frame 0). 

In questo caso siccome stiamo considerando **RF0 Posizionato Sul Giunto 1**, per mappare le coordinate finali nel frame globale `world`, si deve moltiplicare la matrice complessiva della catena DH per una matrice di trasformazione omogenea fissa iniziale:
$$^{world}T_0 = T_x(2.0) \cdot T_z(\text{baseHeight})$$

Questo metodo è pulito e mantiene la tabella DH focalizzata esclusivamente sulle lunghezze dei link mobili.

Si può anche includere l'altezza della base direttamente all'interno della catena DH, spostando l'origine del Frame 0 sul terreno. Il parametro $d_1$ diventerà:
$$baseHeight + l_1 + 2 \cdot \text{jointRadius}$$
e la trasformazione fissa iniziale si ridurrà alla sola traslazione lungo l'asse $X$.

---

### Calcolo Della Matrice Di Trasformazione Complessiva

Ogni matrice $^{i-1}T_i$ descrive **come passare dal frame $i-1$ al frame $i$**. Il problema è che due frame consecutivi possono differire in traslazione, rotazione o entrambe, e anche su assi diversi.

Con la convenzione di DH è che **qualunque** configurazione relativa tra due frame (rispettando i vincoli di assegnazione) si può sempre decomporre in **esattamente 4 trasformazioni elementari in sequenza fissa**:

$$^{i-1}T_i = \underbrace{R_z(\theta_i)}_{\text{1}} \cdot \underbrace{T_z(d_i)}_{\text{2}} \cdot \underbrace{T_x(a_i)}_{\text{3}} \cdot \underbrace{R_x(\alpha_i)}_{\text{4}}$$

Dove:
* $R_z(\theta_i)$: Rotazione Attorno A $Z_{i-1}$ Di Angolo $\theta_i$ (*Variabile Di Giunto*)
* $T_z(d_i)$: Traslazione Lungo $Z_{i-1}$ Di Distanza $d_i$ (*Variabile Di Giunto Se Prismatico, Costante Se Revolute*), ovvero finché $Z_{i-1}$ e $Z_i$ sono complanari.
* $T_x(a_i)$: Traslazione Lungo $X_i$ Di Distanza $a_i$ (*Costante*), ovvero finché le origini dei due frame coincidono.
* $R_x(\alpha_i)$: Rotazione Attorno A $X_i$ Di Angolo $\alpha_i$ finche $Z_{i-1}$ si allinea a $Z_i$.

Queste quattro trasformazioni elementari sono rappresentate da matrici $4 \times 4$:

$$R_z(\theta_i) = \begin{pmatrix} c\theta_i & -s\theta_i & 0 & 0 \\ s\theta_i & c\theta_i & 0 & 0 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{pmatrix} \quad T_z(d_i) = \begin{pmatrix} 1 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 \\ 0 & 0 & 1 & d_i \\ 0 & 0 & 0 & 1 \end{pmatrix}$$

$$T_x(a_i) = \begin{pmatrix} 1 & 0 & 0 & a_i \\ 0 & 1 & 0 & 0 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{pmatrix} \quad R_x(\alpha_i) = \begin{pmatrix} 1 & 0 & 0 & 0 \\ 0 & c\alpha_i & -s\alpha_i & 0 \\ 0 & s\alpha_i & c\alpha_i & 0 \\ 0 & 0 & 0 & 1 \end{pmatrix}$$

Moltiplicandole in sequenza si ottiene la **forma chiusa generale**:

$$^{i-1}T_i = \begin{pmatrix} c\theta_i & -s\theta_i c\alpha_i & s\theta_i s\alpha_i & a_i c\theta_i \\ s\theta_i & c\theta_i c\alpha_i & -c\theta_i s\alpha_i & a_i s\theta_i \\ 0 & s\alpha_i & c\alpha_i & d_i \\ 0 & 0 & 0 & 1 \end{pmatrix}$$

## Comandi Per Avviare Il Nodo ROS
rm -f /root/.bash_history  && history -c  && history -w && clear
roslaunch manipolatoreRRP manipolatoreRRP.launch
roslaunch manipolatoreRRP manipolatoreRRP.launch gui:=false