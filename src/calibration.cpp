#include <iostream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/utils/filesystem.hpp>

int main() {
    // ==========================================
    // 1. PARAMÈTRES DU DAMIER & CAPTEUR
    // ==========================================
    const cv::Size CHECKERBOARD(9, 6);   // 9x6 coins internes (pour damier 10x7)
    const float SQUARE_SIZE = 0.0085f;  // 8.5 mm = 0.0085 m

    const double SENSOR_WIDTH_MM = 4.8;
    const double SENSOR_HEIGHT_MM = 3.6;

    // Préparation des points 3D réels (X, Y, Z=0)
    std::vector<cv::Point3f> objp;
    for (int i = 0; i < CHECKERBOARD.height; ++i) {
        for (int j = 0; j < CHECKERBOARD.width; ++j) {
            objp.push_back(cv::Point3f(j * SQUARE_SIZE, i * SQUARE_SIZE, 0.0f));
        }
    }

    std::vector<std::vector<cv::Point3f>> objpoints;
    std::vector<std::vector<cv::Point2f>> imgpoints;

    // ==========================================
    // 2. CHARGEMENT ET DÉTECTION DES COINS
    // ==========================================
    std::vector<cv::String> image_paths;
    
    // Recherche de toutes les extensions dans data/images_calibration
    std::vector<std::string> patterns = {
        "../data/images_calibration/*.jpg", "../data/images_calibration/*.JPG",
        "../data/images_calibration/*.png", "../data/images_calibration/*.PNG",
        "../data/images_calibration/*.jpeg", "../data/images_calibration/*.JPEG"
    };

    for (const auto& pattern : patterns) {
        std::vector<cv::String> current_paths;
        cv::glob(pattern, current_paths, false);
        image_paths.insert(image_paths.end(), current_paths.begin(), current_paths.end());
    }

    std::cout << "Nombre d'images trouvees dans ../data/images_calibration : " 
              << image_paths.size() << std::endl;

    if (image_paths.empty()) {
        std::cerr << "[ERREUR] Aucune image trouvee dans ../data/images_calibration/" << std::endl;
        return -1;
    }

    cv::Size image_size;
    int valid_images = 0;
    cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001);

    for (const auto& path : image_paths) {
        cv::Mat img = cv::imread(path);
        if (img.empty()) continue;

        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        image_size = gray.size();

        std::vector<cv::Point2f> corners;
        bool ret = cv::findChessboardCorners(gray, CHECKERBOARD, corners);

        if (ret) {
            objpoints.push_back(objp);
            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), criteria);
            imgpoints.push_back(corners);
            valid_images++;

            // Visualisation en direct
            cv::Mat img_drawn = img.clone();
            cv::drawChessboardCorners(img_drawn, CHECKERBOARD, corners, ret);
            cv::Mat img_resized;
            cv::resize(img_drawn, img_resized, cv::Size(), 0.5, 0.5);
            cv::imshow("Detection du Damier (C++)", img_resized);
            cv::waitKey(150);
        }
    }
    cv::destroyAllWindows();

    std::cout << "\nTotal : " << valid_images << "/" << image_paths.size() 
              << " images validees pour la calibration." << std::endl;

    if (valid_images < 5) {
        std::cerr << "[ERREUR] Nombre d'images valides insuffisant (< 5) !" << std::endl;
        return -1;
    }

    // ==========================================
    // 3. CALIBRATION & EXTRACTION
    // ==========================================
    cv::Mat mtx, dist;
    std::vector<cv::Mat> rvecs, tvecs;

    double rms = cv::calibrateCamera(objpoints, imgpoints, image_size, mtx, dist, rvecs, tvecs);

    std::cout << "\n================ RESULTATS DE CALIBRATION (C++) ================" << std::endl;
    std::cout << "Erreur de retroprojection RMS : " << rms << " pixels" << std::endl;
    std::cout << "\nMatrice Intrinseque (K) :\n" << mtx << std::endl;
    std::cout << "\nCoefficients de Distorsion :\n" << dist << std::endl;

    // Extraction physique via calibrationMatrixValues
    double fovx, fovy, focalLength, aspectRatio;
    cv::Point2d principalPoint;

    cv::calibrationMatrixValues(mtx, image_size, SENSOR_WIDTH_MM, SENSOR_HEIGHT_MM,
                                fovx, fovy, focalLength, principalPoint, aspectRatio);

    double focal_meters = focalLength / 1000.0;

    std::cout << "\n======= EXTRACTION PHYSIQUE (calibrationMatrixValues) =======" << std::endl;
    std::cout << "Longueur focale estimee : " << focalLength << " mm (" << focal_meters << " metres)" << std::endl;
    std::cout << "Champ de vision X (FOV X) : " << fovx << " degres" << std::endl;
    std::cout << "Champ de vision Y (FOV Y) : " << fovy << " degres" << std::endl;
    std::cout << "Point Principal (Ox, Oy) : " << principalPoint << " pixels" << std::endl;
    std::cout << "Rapport d'aspect (Aspect Ratio) : " << aspectRatio << std::endl;

    // ==========================================
    // 4. ESTIMATION DE POSE 3D ET PROJECTION D'AXES
    // ==========================================
    std::vector<cv::Point3f> axis;
    axis.push_back(cv::Point3f(3 * SQUARE_SIZE, 0, 0));
    axis.push_back(cv::Point3f(0, 3 * SQUARE_SIZE, 0));
    axis.push_back(cv::Point3f(0, 0, -3 * SQUARE_SIZE));

    cv::Mat img_test = cv::imread(image_paths[0]);
    cv::Mat gray_test;
    cv::cvtColor(img_test, gray_test, cv::COLOR_BGR2GRAY);

    std::vector<cv::Point2f> corners_test;
    bool ret_test = cv::findChessboardCorners(gray_test, CHECKERBOARD, corners_test);

    if (ret_test) {
        cv::cornerSubPix(gray_test, corners_test, cv::Size(11, 11), cv::Size(-1, -1), criteria);

        cv::Mat rvec, tvec;
        cv::solvePnP(objp, corners_test, mtx, dist, rvec, tvec);

        std::vector<cv::Point2f> imgpts;
        cv::projectPoints(axis, rvec, tvec, mtx, dist, imgpts);

        cv::Point origin = corners_test[0];
        cv::line(img_test, origin, imgpts[0], cv::Scalar(0, 0, 255), 4); // Rouge (X)
        cv::line(img_test, origin, imgpts[1], cv::Scalar(0, 255, 0), 4); // Vert (Y)
        cv::line(img_test, origin, imgpts[2], cv::Scalar(255, 0, 0), 4); // Bleu (Z)

        // Création automatique du dossier data/output s'il n'existe pas
        cv::utils::fs::createDirectories("../data/output");
        
        std::string output_path = "../data/output/pose_3d_cpp_resultat.jpg";
        cv::imwrite(output_path, img_test);
        std::cout << "\nVisualisation 3D (C++) enregistree sous : " << output_path << std::endl;
    }

    return 0;
}