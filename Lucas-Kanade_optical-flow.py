import numpy as np
import cv2
import time

## Paramètres de l'algorithme de suivi optique Lucas-Kanade

lk_params = dict(winSize  = (15, 15),  # Taille des fenêtres de recherche pour les algorithmes de suivi optique (Lucas-Kanade) 
                maxLevel = 2, # Nombre de niveaux de pyramide pour les algorithmes de suivi optique (Lucas-Kanade), pyramide pour diminuer la qualité de l'image
                criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03)) # Critères d'arrêt soit qualitatif soit quantitatif

# 10: Ce paramètre spécifie le nombre maximal d'itérations que l'algorithme doit effectuer. Dans ce cas, l'algorithme s'arrêtera après 10 itérations si le critère de précision n'est pas atteint avant.
# 0.03: Ce paramètre spécifie la précision souhaitée. L'algorithme s'arrêtera si les changements entre les itérations successives sont inférieurs à cette valeur.

feature_params = dict(maxCorners = 20, # Nombre maximal de coins à détecter
                    qualityLevel = 0.3, # Niveau de qualité minimal pour les coins à détecter
                    minDistance = 10, # Distance minimale entre les coins détectés
                    blockSize = 7 ) # Taille du bloc pour le calcul de la dérivée de l'image


trajectory_len = 40 # Longueur de la trajectoire
detect_interval = 5 # Intervalle de détection des nouvelles caractéristiques
trajectories = [] # Liste des trajectoires
frame_idx = 0 # Initialisation de l'index de la frame

## Capture de la vidéo avec une boucle infinie

cap = cv2.VideoCapture(0) # Webcam capture

while True: 

    # start time to calculate FPS
    start = time.time() # start chrono

    suc, frame = cap.read() # Capture frame-by-frame avec suc comme flag de succès
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # Conversion de l'image en niveaux de gris
    img = frame.copy() # Copie de l'image pour dessiner les trajectoires

    # Calculer le flux optique pour les trajectoires existantes et les nouvelles caractéristiques détectées   
    if len(trajectories) > 0:
        img0, img1 = prev_gray, frame_gray # Image précédente et image actuelle
        p0 = np.float32([trajectory[-1] for trajectory in trajectories]).reshape(-1, 1, 2) # Dernier point de la trajectoire reshape en 1x2 
        p1, _st, _err = cv2.calcOpticalFlowPyrLK(img0, img1, p0, None, **lk_params) # argument : image précédente, image actuelle, points à suivre, None, paramètres Lucas-Kanade
        p0r, _st, _err = cv2.calcOpticalFlowPyrLK(img1, img0, p1, None, **lk_params) # argument : image actuelle, image précédente, points suivis, None, paramètres Lucas-Kanade
        d = abs(p0-p0r).reshape(-1, 2).max(-1) # Calcul de la distance entre les points suivis et les points suivis en arrière
        good = d < 1 # Si la distance est inférieure à 1, alors c'est un bon point

        new_trajectories = [] 

        # Get all the trajectories
        for trajectory, (x, y), good_flag in zip(trajectories, p1.reshape(-1, 2), good): # zip : regroupe les éléments de chaque liste argument : trajectoire, points suivis, bon point
            if not good_flag:
                continue
            trajectory.append((x, y))
            if len(trajectory) > trajectory_len: # Si la longueur de la trajectoire est supérieure à la longueur max de la trajectoire
                del trajectory[0] # Supprimer le premier point de la trajectoire
            new_trajectories.append(trajectory)
            # Newest detected point
            cv2.circle(img, (int(x), int(y)), 2, (0, 0, 255), -1) # Dessiner un cercle sur le point détecté argument : image, centre, rayon, couleur, épaisseur

        trajectories = new_trajectories

        # Draw all the trajectories
        cv2.polylines(img, [np.int32(trajectory) for trajectory in trajectories], False, (0, 255, 0)) # arguments : image, points, isClosed, color, isClosed : True pour fermer la forme, False pour une ligne ouverte
        cv2.putText(img, 'track count: %d' % len(trajectories), (20, 50), cv2.FONT_HERSHEY_PLAIN, 1, (0,255,0), 2) # arguments : image, texte, position, font, fontScale, color, thickness, fontScale : taille de la police


    # Update interval - When to update and detect new features
    if frame_idx % detect_interval == 0: # Si l'index de la frame est un multiple de l'intervalle de détection
        mask = np.zeros_like(frame_gray) # Création d'un masque de la taille de l'image
        mask[:] = 255 # Remplir le masque avec des pixels blancs

        # Lastest point in latest trajectory
        for x, y in [np.int32(trajectory[-1]) for trajectory in trajectories]: # Prendre le dernier point de chaque trajectoire
            cv2.circle(mask, (x, y), 5, 0, -1) # argument : image, centre, rayon, couleur, épaisseur

        # Detect the good features to track
        p = cv2.goodFeaturesToTrack(frame_gray, mask = mask, **feature_params) # arguments : image, mask, maxCorners, qualityLevel, minDistance, blockSize
        if p is not None: # Si des bonnes caractéristiques peuvent être suivies - les ajouter aux trajectoires
            # If good features can be tracked - add that to the trajectories
            for x, y in np.float32(p).reshape(-1, 2): # Convertir les points en float32 et les redimensionner en 1x2
                trajectories.append([(x, y)]) 


    frame_idx += 1 # Incrémenter l'index de la frame
    prev_gray = frame_gray # Mettre à jour l'image précédente

    # End time
    end = time.time() # end chrono
    # calculate the FPS for current frame detection
    fps = 1 / (end-start) 
    
    # Show Results
    cv2.putText(img, f"{fps:.2f} FPS", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2) # arguments : image, texte, position, font, fontScale, color, thickness
    cv2.imshow('Optical Flow', img) # Afficher l'image avec les trajectoires
    cv2.imshow('Mask', mask) # Afficher le masque

    if cv2.waitKey(10) & 0xFF == ord('q'):
        break


cap.release()
cv2.destroyAllWindows()

# Pour détecter des caractéristiques sur la vidéo d'une webcam avec une personne, les caractéristiques typiques à détecter incluent :

# Coins et points d'intérêt :

# Les coins des yeux, de la bouche, du nez, et des oreilles.
# Les coins des objets dans l'arrière-plan.
# Contours et bords :

# Les contours du visage, des yeux, de la bouche, et des autres parties du corps.
# Les bords des objets dans l'arrière-plan.
# Textons et textures :

# Les motifs de texture sur les vêtements.
# Les textures de la peau.
# Dans le code fourni, les caractéristiques sont détectées en utilisant la fonction cv2.goodFeaturesToTrack, qui détecte les coins dans l'image. Les paramètres utilisés pour cette détection sont définis dans feature_params
# qui est dans la parmtie où on défini les paramètres de l'algorithme de suivi optique Lucas-Kanade.