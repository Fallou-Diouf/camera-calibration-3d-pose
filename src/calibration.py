import cv2
import numpy as np
import glob
import os

# ==========================================
# 1. PARAMÈTRES DU DAMIER & CAPTEUR
# ==========================================
CHECKERBOARD = (9, 6)   # 9x6 coins internes (pour damier 10x7)
SQUARE_SIZE = 0.0085    # Votre mesure : 0.85 cm = 0.0085 m

SENSOR_WIDTH_MM = 4.8 
SENSOR_HEIGHT_MM = 3.6

# Préparation des points 3D réels (X, Y, Z=0)
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp = objp * SQUARE_SIZE

objpoints = [] 
imgpoints = [] 

# ==========================================
# 2. CHARGEMENT SÉCURISÉ DES IMAGES
# ==========================================
script_dir = os.path.dirname(os.path.abspath(__file__))
root_dir = os.path.dirname(script_dir)
image_dir = os.path.join(root_dir, "data", "images_calibration") 

extensions = ['*.png', '*.PNG', '*.jpg', '*.JPG', '*.jpeg', '*.JPEG']
images = []
for ext in extensions:
    images.extend(glob.glob(os.path.join(image_dir, ext)))

print(f"Nombre d'images trouvées dans {image_dir} : {len(images)}")

if len(images) == 0:
    print("\n[ERREUR] Aucune image trouvée ! Vérifie que tes photos sont bien dans le dossier 'images_calibration'.")
    exit()

image_size = None
valid_images = 0

# Critère d'arrêt corrigé pour OpenCV 4/5
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

for fname in images:
    img = cv2.imread(fname)
    
    if img is None:
        print(f"[ATTENTION] Impossible de lire le fichier : {fname}")
        continue

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    image_size = gray.shape[::-1]

    # Détection des coins du damier
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if ret:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)
        valid_images += 1
        print(f"[OK] Coins détectés avec succès : {os.path.basename(fname)}")
    else:
        print(f"[ECHEC] Damier non détecté sur : {os.path.basename(fname)}")

print(f"\nTotal : {valid_images}/{len(images)} images validées pour la calibration.")

if valid_images < 5:
    print("[ERREUR] Il faut au moins 5 images valides pour faire la calibration.")
    exit()

# ==========================================
# 3. CALIBRATION & EXTRACTION
# ==========================================
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, image_size, None, None
)

print("\n================ RÉSULTATS DE CALIBRATION ================")
print(f"Erreur de rétroprojection RMS : {ret:.4f} pixels")
print("\nMatrice Intrinsèque (K) :\n", mtx)
print("\nCoefficients de Distorsion [k1, k2, p1, p2, k3] :\n", dist.ravel())

# Extraction physique via calibrationMatrixValues
fovx, fovy, focalLength, principalPoint, aspectRatio = cv2.calibrationMatrixValues(
    cameraMatrix=mtx,
    imageSize=image_size,
    apertureWidth=SENSOR_WIDTH_MM,
    apertureHeight=SENSOR_HEIGHT_MM
)

focal_meters = focalLength / 1000.0

print("\n======= EXTRACTION PHYSIQUE (calibrationMatrixValues) =======")
print(f"Longueur focale estimée : {focalLength:.2f} mm ({focal_meters:.6f} mètres)")
print(f"Champ de vision X (FOV X) : {fovx:.2f}°")
print(f"Champ de vision Y (FOV Y) : {fovy:.2f}°")
print(f"Point Principal (Ox, Oy) : {principalPoint} pixels")
print(f"Rapport d'aspect (Aspect Ratio) : {aspectRatio:.4f}")

# ==========================================
# 4. ESTIMATION DE POSE 3D ET PROJECTION D'AXES
# ==========================================
axis = np.float32([[3*SQUARE_SIZE,0,0], [0,3*SQUARE_SIZE,0], [0,0,-3*SQUARE_SIZE]]).reshape(-1,3)

img_test = cv2.imread(images[0])
gray_test = cv2.cvtColor(img_test, cv2.COLOR_BGR2GRAY)
ret_test, corners_test = cv2.findChessboardCorners(gray_test, CHECKERBOARD, None)

if ret_test:
    corners2_test = cv2.cornerSubPix(gray_test, corners_test, (11, 11), (-1, -1), criteria)
    
    _, rvec, tvec = cv2.solvePnP(objp, corners2_test, mtx, dist)
    imgpts, _ = cv2.projectPoints(axis, rvec, tvec, mtx, dist)
    
    corner = tuple(corners2_test[0].ravel().astype(int))
    img_test = cv2.line(img_test, corner, tuple(imgpts[0].ravel().astype(int)), (0, 0, 255), 4)  # Rouge (X)
    img_test = cv2.line(img_test, corner, tuple(imgpts[1].ravel().astype(int)), (0, 255, 0), 4)  # Vert (Y)
    img_test = cv2.line(img_test, corner, tuple(imgpts[2].ravel().astype(int)), (255, 0, 0), 4)  # Bleu (Z)
    
    output_path = os.path.join(root_dir, "data", "output" 'pose_3d_resultat.jpg')
    cv2.imwrite(output_path, img_test)
    print(f"\nVisualisation enregistrée sous : {output_path}")