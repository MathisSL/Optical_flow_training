import numpy as np
import cv2

class KalmanFilter:
    def __init__(self, dt, point):
        self.dt = dt

        # Vecteur d'état initial [x, y, vx, vy]
        self.E = np.matrix([[point[0]], [point[1]], [0], [0]])

        # Matrice de transition (modélisation des déplacements)
        self.A = np.matrix([[1, 0, self.dt, 0],
                            [0, 1, 0, self.dt],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])

        # Matrice d'observation
        self.H = np.matrix([[1, 0, 0, 0],
                            [0, 1, 0, 0]])

        # Bruit de processus et d'observation
        self.Q = np.eye(4) * 0.03
        self.R = np.eye(2) * 10

        # Matrice de covariance de l'erreur
        self.P = np.eye(4)

    def predict(self):
        self.E = np.dot(self.A, self.E)
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q
        return self.E[:2]

    def update(self, z):
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.E = self.E + np.dot(K, (z - np.dot(self.H, self.E)))
        I = np.eye(self.A.shape[0])
        self.P = np.dot((I - np.dot(K, self.H)), self.P)
        return self.E[:2]

# Chargement du modèle de détection des visages
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

if face_cascade.empty():
    print("Erreur : Impossible de charger le fichier cascade pour la détection des visages.")
    exit(1)

def detect_visage(image):
    points = []
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.2, minNeighbors=3)
    for x, y, w, h in faces:
        points.append(np.array([int(x + w / 2), int(y + h / 2)]))  # Centre du rectangle
        cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 2)
    return points

# Initialisation
webcam = cv2.VideoCapture(0)
if not webcam.isOpened():
    print("Erreur : Impossible d'accéder à la webcam.")
    exit(1)

kalman_filters = []
tracked_points = []

# Boucle principale
while True:
    ret, frame = webcam.read()
    if not ret:
        print("Erreur : Impossible de lire l'image depuis la webcam.")
        break

    # Détection des visages
    detected_faces = detect_visage(frame)

    # Mise à jour ou création de filtres de Kalman
    new_filters = []
    for face in detected_faces:
        matched = False
        for kf, tp in zip(kalman_filters, tracked_points):
            dist = np.linalg.norm(face - tp)
            if dist < 50:  # Distance seuil pour associer une détection à un filtre existant
                tp[:] = kf.update(np.expand_dims(face, axis=1)).A1  # Mise à jour avec observation
                matched = True
                break
        if not matched:
            # Créer un nouveau filtre de Kalman pour les nouveaux visages détectés
            kf = KalmanFilter(dt=1, point=face)
            new_filters.append(kf)
            tracked_points.append(kf.E[:2].A1)

    kalman_filters.extend(new_filters)

    # Prédiction des positions pour tous les filtres existants
    for kf, tp in zip(kalman_filters, tracked_points):
        predicted = kf.predict()
        tp[:] = predicted.A1
        cv2.circle(frame, (int(tp[0]), int(tp[1])), 5, (0, 255, 0), -1)  # Dessiner les positions prédites

    # Affichage de l'image
    cv2.imshow('Suivi de visages avec Kalman', frame)

    # Quitter avec la touche "q"
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

webcam.release()
cv2.destroyAllWindows()
