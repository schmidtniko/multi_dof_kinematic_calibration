cmake_minimum_required (VERSION 2.6)
project(ptuCalibration)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}
                    ${CMAKE_CURRENT_SOURCE_DIR}/../cameraSurveyingWithAprilTags/cameraSurveyingWithAprilTags/include/
                    ${CMAKE_CURRENT_SOURCE_DIR}/../cameraSurveyingWithAprilTags/fast_mit_apriltags/
                    ${Boost_INCLUDE_DIRS})


add_library(${PROJECT_NAME}
            src/PtuImageCapture.cpp
            src/PtuCalibrationProject.cpp
            src/PtuCalibration.cpp)

target_link_libraries(${PROJECT_NAME}
                        camSurv
                        apriltags
                        ${OpenCV_LIBS}
                        ${Boost_LIBRARIES})


set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD_REQUIRED ON)
