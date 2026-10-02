# Contesto di Progetto: Manipolatore Antropomorfo RRP (Industrial Robotics)

Questo documento fornisce la descrizione concettuale, la struttura software e le linee guida di sviluppo per il progetto di un manipolatore antropomorfo a tre gradi di libertà (RRP - Rotatorio, Rotatorio, Prismatico), sviluppato in ambiente **ROS 1 Noetic** su container Docker (Ubuntu 20.04).

---




## 1. Visione Geometrica e Filosofia del Progetto

Il progetto si basa su un'analogia spaziale e meccanica coerente, mirata a unire realismo costruttivo e pulizia estetica:

*   **Riferimento Spaziale:** Il piano cartesiano globale $X$-$Y$ ($Z=0$) rappresenta il piano del tavolo di lavoro. L'asse $Z$ è perpendicolare a esso e punta verso l'alto (verso il soffitto).
*   **Posizionamento di Home:** Il manipolatore è ancorato al suolo con un offset di **2 metri** lungo l'asse $X$ rispetto all'origine globale di RViz (`world`).
*   **Realismo Meccanico e Strutturale:**
    *   Nel definire il giunto prismatico, si evita l'effetto irrealistico in cui l'intero alloggiamento della guida si muove con l'asta. La scatola di guida (il box del prismatico) è parte integrante e fissa del link padre precedente (`RF1`). Solo lo stelo interno (`RF2`) scorre fisicamente lungo l'asse lineare, simulando un vero cilindro telescopico.
    *   Per eliminare i vuoti d'aria antiestetici o le compenetrazioni visive tipiche dei modelli CAD approssimativi, ogni giunto rotante è coperto da sfere estetiche modellate nel file XACRO. Le coordinate di posizionamento dei link tengono conto del raggio di queste sfere per garantire incastri geometrici fluidi e continui.

---

## 2. Descrizione Fisica del Robot (RRP)

Il robot è composto da una catena cinematica seriale a tre gradi di libertà principali più un End Effector fisso:

1.  **Base del Manipolatore:** Un blocco rettangolare solido (`base`) fissato a $X=2$ metri sul frame `world` e poggiato direttamente sul terreno.
2.  **Giunto 1 (Rotatorio Z):** Ruota intorno all'asse verticale $Z$. È posizionato sulla faccia superiore della base.
3.  **Link 1 (Colonna Verticale - `RF0`):** Una colonna cilindrica che si sviluppa verticalmente lungo $Z$. All'estremità inferiore presenta una sfera di giunzione estetica.
4.  **Giunto 2 (Rotatorio Y):** Posizionato in cima alla colonna verticale, ruota attorno all'asse trasversale $Y$ (movimento di beccheggio/spalla). Ha limiti di movimento progettati per evitare auto-collisioni con la colonna.
5.  **Link 2 (Braccio Orizzontale - `RF1`):** Un braccio cilindrico che si estende in direzione radiale. All'estremità distale di questo link è montata la scatola di guida cubica del giunto prismatico.
6.  **Giunto 3 (Prismatico Z):** Giunto lineare che scorre lungo l'asse verticale locale del braccio.
7.  **Link 3 (Asta Prismatica - `RF2`):** Un'asta cilindrica interna che scorre verticalmente dentro la scatola di guida.
8.  **Giunto EE (Fisso):** Un giunto rigido posto all'estremità inferiore dell'asta prismatica, ruotato in modo da orientare la pinza verso il basso.
9.  **End Effector (`endEffector`):** Una pinza fissa composta da una piastra di base e due dita parallele simmetriche destinate a compiere operazioni sul piano di lavoro.

---

## 3. Struttura del Workspace ROS

Il pacchetto ROS è denominato `manipolatoreRRP` (rispetta lo stile *lowerCamelCase*) ed è strutturato come segue:

*   `urdf/manipolatoreRRP.urdf.xacro`: Modello geometrico parametrizzato del robot.
*   `config/robotParameters.yaml`: File di configurazione in formato YAML contenente i parametri dimensionali e i limiti fisici del manipolatore.
*   `launch/manipolatoreRRP.launch`: File di avvio principale che gestisce il caricamento del robot, i nodi di stato, la GUI per il controllo manuale dei giunti e l'apertura di RViz.
*   `scripts/tfPrinter.py`: Listener TF che intercetta la trasformazione tra l'origine del mondo (`world`) e l'End Effector (`endEffector`), stampando a terminale la matrice di trasformazione omogenea $4 \times 4$ in tempo reale.
*   `scripts/directKinematics.py`: Modulo Python dedicato al calcolo della cinematica diretta mediante la convenzione e i parametri di Denavit-Hartenberg.
*   `scripts/inverseKinematics.py`: File segnaposto predisposto per ospitare i futuri sviluppi legati alla cinematica inversa.
*   `scripts/jointController.py`: Script di controllo principale che pubblica periodicamente lo stato dei giunti, ne gestisce i limiti fisici e calcola la posa dell'End Effector tramite cinematica diretta.
*   `scripts/kinematicsUtils.py`: Libreria di utilità contenente le funzioni per il caricamento dei parametri dal server ROS, la validazione a soglia dei giunti, la formattazione dei messaggi ROS e le conversioni geometriche.

---

## 4. Logica dei File Implementati

### 4.1 Modello URDF/XACRO (`manipolatoreRRP.urdf.xacro`)
Il robot è interamente parametrizzato tramite macro e proprietà XACRO (es. lunghezze dei link, spessori, raggi dei giunti e dimensioni della pinza). Questa struttura consente di modificare le proporzioni del manipolatore aggiornando solo i valori delle variabili all'inizio del file. I colori dei link sono gestiti tramite una macro di configurazione cromatica dedicata per differenziare visivamente le parti strutturali (base, giunti, link, pinza).

### 4.2 File Launch (`manipolatoreRRP.launch`)
Il file coordina l'intero ecosistema di nodi:
*   Converte lo XACRO in URDF standard e lo carica nel parametro `robot_description`.
*   Avvia `joint_state_publisher_gui` (se l'interfaccia grafica è abilitata) per controllare i giunti tramite slider, oppure `joint_state_publisher` standard.
*   Esegue `robot_state_publisher` per calcolare e pubblicare i frame di trasformazione (TF) dinamici.
*   Avvia RViz pre-caricando un file di configurazione specifico e silenzia i messaggi di avviso legati al ciclo di vita di ROS 1.
*   Esegue in background lo script `tfPrinter.py`.

### 4.3 Script Listener TF (`tfPrinter.py`)
Lo script Python sfrutta la libreria `tf2_ros` per interrogare il buffer dei trasformati. Converte il quaternione della posa dell'End Effector in una matrice di rotazione $3 \times 3$ usando le funzioni di `tf.transformations` e ricostruisce la matrice di trasformazione omogenea $4 \times 4$. I risultati sono formattati e stampati a terminale a cadenza regolare (1 Hz).

### 4.4 File di Configurazione (`robotParameters.yaml`)
Questo file centralizza i parametri geometrici e operativi del robot. I valori definiti al suo interno includono le dimensioni della base, le lunghezze dei link (`l1`, `l2`, `l3`), le dimensioni dei giunti e i dettagli costruttivi dell'End Effector. Il file viene caricato sul Parameter Server di ROS all'avvio del sistema dal file `.launch`, in modo da essere condiviso dinamicamente tra la generazione del modello in XACRO e l'elaborazione degli script di calcolo in Python.

### 4.5 Script di Cinematica Diretta (`directKinematics.py`)
Il modulo implementa le relazioni matematiche necessarie per la risoluzione della cinematica diretta del manipolatore:
*   **Matrice di Trasformazione Omogenea:** La funzione `calculateTransformationMatrix` calcola la matrice relativa al singolo giunto applicando la convenzione di Denavit-Hartenberg come prodotto ordinato delle matrici elementari $R_z(\theta) \cdot T_z(d) \cdot T_x(a) \cdot R_x(\alpha)$.
*   **Tabella DH:** Definisce la rappresentazione tabellare dei parametri geometrici dei tre bracci per collegare i sistemi di riferimento locali (`RF0`, `RF1`, `RF2` ed `endEffector`).
*   **Posa Globale:** Estende la catena cinematica calcolando la trasformazione complessiva dall'origine globale (`world`) fino alla flangia dell'End Effector, estraendone la posizione cartesiana tridimensionale.

### 4.6 Script di Controllo Giunti (`jointController.py`)
Il nodo simula l'invio dei comandi di posizionamento ai giunti del manipolatore con una frequenza di aggiornamento di 50 Hz:
*   Definisce le variabili di giunto di prova e le sottopone alle funzioni di verifica geometrica.
*   Pubblica i messaggi formattati di tipo `sensor_msgs/JointState` sul topic `/joint_states`.
*   Esegue a ogni ciclo il calcolo in tempo reale della cinematica diretta per stampare a terminale la matrice omogenea globale e la posizione cartesiana risultante dell'End Effector.

### 4.7 Script Utility Cinematiche (`kinematicsUtils.py`)
Fornisce il supporto infrastrutturale per la gestione delle operazioni ricorrenti degli script principali:
*   **`loadRobotParameters()`**: Interroga il Parameter Server di ROS per aggiornare le variabili globali del modulo, integrando valori di ripiego di sicurezza (default) in caso di assenza delle chiavi.
*   **`checkJointLimits()`**: Applica funzioni di saturazione (*clamping*) ai valori angolari e lineari dei giunti per prevenire il superamento dei limiti strutturali e l'insorgenza di auto-collisioni.
*   **`createJointStateMsg()`**: Popola un'istanza del messaggio `JointState` associandovi i timestamp correnti e la mappatura dei nomi dei giunti coerente con il file URDF.
*   **`getTransformationMatrix()`**: Ricostruisce una matrice omogenea $4 \times 4$ integrando un vettore tridimensionale di traslazione e un quaternione di rotazione.

### 4.8 Script Cinematica Inversa (`inverseKinematics.py`)
Il file è predisposto come modulo vuoto per futuri ampliamenti metodologici. È destinato ad accogliere gli algoritmi matematici per il calcolo delle coordinate di giunto del robot ($q_1, q_2, q_3$) partendo da una posa cartesiana obiettivo assegnata all'End Effector.

---

## 5. Stile di Programmazione e Linee Guida di Scrittura

Per garantire la coerenza con il codice esistente in qualsiasi futuro sviluppo, devono essere rispettate le seguenti e rigorose regole di stile, nomenclatura e commento:

### 5.1 Nomenclatura e Codice
*   **Proprietà e Variabili:** Si utilizza lo stile **lowerCamelCase** (es. `baseWidth`, `linkRadius`, `translationVector`).
*   **Nodi e File:** Si utilizza lo stile **lowerCamelCase** per i nomi dei file e dei pacchetti (es. `manipolatoreRRP`, `tfPrinter.py`, `moveScript.py`).
*   **Librerie:** Per i calcoli matriciali in Python si utilizza esclusivamente `numpy` abbinato alle funzioni native di trasformazione di ROS.

### 5.2 Struttura e Stile dei Commenti (Fondamentale)
Tutti i commenti all'interno dei file (sia XML/XACRO che Python) devono aderire a uno standard molto specifico:
1.  **Prefisso Gerarchico:** Ogni blocco di commenti deve iniziare con un prefisso che identifica il livello di importanza:
    *   `AA` per le sezioni principali o definizioni globali.
    *   `BB` per le sottosezioni o i componenti logici intermedi.
    *   `CC` per i dettagli implementativi o spiegazioni specifiche.
    *   `DD` per note geometriche, descrizioni di traslazioni/rotazioni e allineamenti fisici.
2.  **Stile Grafico dei Separatori:** Le sezioni principali (`AA`) devono essere delimitate visivamente da una linea di uguali (es. `<!-- AA ======================= ... ======================= -->`).
3.  **Iniziale Maiuscola su Ogni Singola Parola (Capitalizzazione Forzata):** All'interno dei commenti, **ogni singola parola deve iniziare con la lettera maiuscola**, incluse le preposizioni, gli articoli, le congiunzioni e i pronomi (es. `# AA Impostazioni Stampa Di NumPy`, `<!-- DD Sfera Centrata Perfettamente Sull'Origine Del Secondo Giunto Rotante -->`, `# CC Matrice Di Rotazione Nelle Prime Tre Righe E Tre Colonne`). Non sono ammesse parole interamente in minuscolo nei commenti.


