import cv2
import mediapipe as mp

# Initialiser MediaPipe Hands
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

# Paramètres pour la détection de mains
hands = mp_hands.Hands(static_image_mode=False, max_num_hands=1, min_detection_confidence=0.5)

# Fonction pour compter les doigts levés
def count_raised_fingers(hand_landmarks):
    # Index des articulations à vérifier pour savoir si le doigt est levé
    finger_tips = [mp_hands.HandLandmark.INDEX_FINGER_TIP,
                   mp_hands.HandLandmark.MIDDLE_FINGER_TIP,
                   mp_hands.HandLandmark.RING_FINGER_TIP,
                   mp_hands.HandLandmark.PINKY_TIP]
    raised_fingers = 0

    # Vérifier chaque doigt
    for tip in finger_tips:
        # Si la pointe du doigt est au-dessus de l'articulation du doigt, le doigt est levé
        if hand_landmarks.landmark[tip].y < hand_landmarks.landmark[tip - 2].y:
            raised_fingers += 1

    # Pouce
    thumb_tip = mp_hands.HandLandmark.THUMB_TIP
    thumb_ip = mp_hands.HandLandmark.THUMB_IP  # Articulation intermédiaire du pouce
    if hand_landmarks.landmark[thumb_tip].x > hand_landmarks.landmark[thumb_ip].x:  # Pour une main gauche
        raised_fingers += 1

    return raised_fingers

# Accéder à la caméra
cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Convertir l'image en RGB pour MediaPipe
    image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(image_rgb)

    # Si une main est détectée
    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            # Dessiner les articulations de la main
            mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

            # Compter les doigts levés
            fingers_count = count_raised_fingers(hand_landmarks)

            # Afficher le nombre de doigts levés
            cv2.putText(frame, f"Nombre de doigts leves: {fingers_count}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    # Afficher le flux vidéo
    cv2.imshow("Compteur de doigts avec estimation de pose", frame)

    # Sortie avec 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Libérer la capture et fermer les fenêtres
cap.release()
cv2.destroyAllWindows()
