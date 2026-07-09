#!/usr/bin/env python3

import rospy
import tf2_ros
import numpy as np
import kinematicsUtils

# AA Impostazioni Stampa Di NumPy
# BB Precisione, Notazione Scientifica, Larghezza Massima Di Stampa
np.set_printoptions(precision=4, suppress=True, linewidth=120)

# AA Inizializzazione
rospy.init_node('tfPrinter')

# AA Definizione Frame Di Riferimento
TARGET_FRAME = 'world'
SOURCE_FRAME = 'endEffector'

# AA Creazione Buffer E Listener TF
tf_buffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tf_buffer)

# AA Frequenza Di Aggiornamento
# BB L'Argomento Indica Il Numero Di Aggiornamenti Da Eseguire In Un Secondo
rate = rospy.Rate(1) # 1 Hz

# AA Ciclo Principale Del Nodo ROS
while not rospy.is_shutdown():
      try:
            # BB Intercettazione Messaggio Di Trasformazione Tra Frame Source E Target
            # CC rospy.Time(0) Permette Di Ottenere L'Ultima Trasformazione Disponibile
            dataTF = tf_buffer.lookup_transform(TARGET_FRAME, SOURCE_FRAME, rospy.Time(0))

            # BB Estrazione Della Traslazione E Della Rotazione Dal Messaggio Di Trasformazione
            translation = dataTF.transform.translation
            rotation = dataTF.transform.rotation 

            # BB Stampa Dati Grezzi
            print("-----------------------------------------------------")

            print(f"Trasformazione Da '{SOURCE_FRAME}' a '{TARGET_FRAME}'")

            print(f"\nTimestamp: {dataTF.header.stamp.to_sec()}")

            print("\nTraslazione (x, y, z):")
            print(f"\t x: {translation.x:.4f}")
            print(f"\t y: {translation.y:.4f}")
            print(f"\t z: {translation.z:.4f}")

            print("\nQuaternione (Rotazione x, y, z, w) :")
            print(f"\t x: {rotation.x:.4f}")
            print(f"\t y: {rotation.y:.4f}")
            print(f"\t z: {rotation.z:.4f}")
            print(f"\t w: {rotation.w:.4f}")

            # BB Calcolo Della Matrice Di Trasformazione Omogenea
            transformationMatrix = kinematicsUtils.getTransformationMatrix(translation, rotation)

            # BB Stampa Della Matrice Di Trasformazione Omogenea
            print(f"\nMatrice Trasformazione Omogenea {TARGET_FRAME} - {SOURCE_FRAME}:\n")
            print(np.round(transformationMatrix, 3))

      except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(5.0, "In attesa Che La Trasformazione Sia Disponibile...")

      rate.sleep()