cmake_minimum_required(VERSION 3.20)
project(CamCalibration)
set(OpenCV_DIR "/home/tyy/opencv-4.5.4/build")
FIND_PACKAGE(OpenCV 4.5.4 REQUIRED)

set(CMAKE_CXX_STANDARD 14)

add_executable(CamCalibration main.cpp Calibration.h Calibration.cpp)
TARGET_LINK_LIBRARIES(CamCalibration ${OpenCV_LIBS})