cmake_minimum_required (VERSION 2.6)
project(ptuCalibration)

IF(CMAKE_COMPILER_IS_GNUCC)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()

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
