import cv2 as cv
from picamera2 import Picamera2
import numpy as np
import time

# Initialisation de la caméra
picam2 = Picamera2()
config = picam2.create_video_configuration(main={"size": (840, 720)})
picam2.configure(config)
picam2.start()

frame_count = 0
total_processing_time = 0  # Temps total de traitement des frames
start_time = time.time()  # Pour calculer les FPS

plateau_detecte = False
plateau_coords = None  # Stocker les coordonnées (x, y) et rayon du plateau

while True:
    frame_start = time.time()  # Début du traitement de la frame

    # Capture et traitement de l'image
    frame = picam2.capture_array()
    grayFrame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    grayFrame = cv.GaussianBlur(grayFrame, (15, 15), 0)

    # Recherche du plateau
    if not plateau_detecte:
        plateauCircles = cv.HoughCircles(
            grayFrame, cv.HOUGH_GRADIENT, 2, np.shape(frame)[0],
            param1=50, param2=100, minRadius=100, maxRadius=300
        )
        if plateauCircles is not None:
            plateauCircles = np.uint16(np.around(plateauCircles))
            x, y, r = plateauCircles[0, 0]
            plateau_coords = (x, y, r)
            plateau_detecte = True
            print(f"Plateau détecté : coordonnées ({x}, {y}), rayon {r}")
            cv.circle(frame, (x, y), r, (255, 0, 0), 3)  # Dessiner le plateau

    # Recherche des cercles (par exemple, des balles) si le plateau est détecté
    if plateau_detecte:
        x_plateau, y_plateau, r_plateau = plateau_coords
        circles = cv.HoughCircles(
            grayFrame, cv.HOUGH_GRADIENT, 2, 20,
            param1=20, param2=40, minRadius=5, maxRadius=50
        )
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for circle in circles[0, :]:
                x, y, r = circle
                # Vérifier si le cercle est à l'intérieur du plateau
                if np.sqrt((x - x_plateau)**2 + (y - y_plateau)**2) < r_plateau:
                    cv.circle(frame, (x, y), r, (0, 255, 0), 3)  # Dessiner la balle
                    cv.circle(frame, (x, y), 5, (0, 0, 255), -1)  # Centre de la balle

    # Calcul des FPS
    frame_count += 1
    elapsed_time = time.time() - start_time
    fps = frame_count / elapsed_time

    # Affichage des FPS
    fps_text = f"FPS: {fps:.2f}"
    cv.putText(frame, fps_text, (10, 30), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv.LINE_AA)

    # Affichage de la frame
    cv.imshow("Circles", frame)

    # Temps de traitement pour cette frame
    frame_processing_time = time.time() - frame_start
    total_processing_time += frame_processing_time

    # Quitter la boucle avec 'q'
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

# Afficher les statistiques
average_time_per_frame = total_processing_time / frame_count
print(f"Temps moyen par frame : {average_time_per_frame:.4f} secondes")
print(f"FPS moyen (calculé) : {1 / average_time_per_frame:.2f}")

# Arrêter la caméra et fermer les fenêtres
picam2.stop()
cv.destroyAllWindows()
