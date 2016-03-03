cmake_minimum_required (VERSION 3.0)
project(multi_dof_kinematic_calibration)

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")
endif()

add_executable(${PROJECT_NAME}
               src/PtuImageCapture.cpp
               src/PtuCalibrationProject.cpp
               src/main.cpp)

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD_REQUIRED ON)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include
                    ../visual_marker_mapping/visual_marker_mapping/include/
                    ${Boost_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME}
                      visual_marker_mapping_lib
                      apriltags
                      ${OpenCV_LIBS}
                      ${Boost_LIBRARIES})

