cmake_minimum_required(VERSION 3.10)
project(CameraCalibrationCPP CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Recherche de la bibliothèque OpenCV sur le système
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

# Création de l'exécutable
add_executable(calibration_cpp src/calibration.cpp)

# Liaison des bibliothèques OpenCV
target_link_libraries(calibration_cpp ${OpenCV_LIBS})