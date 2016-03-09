cmake_minimum_required (VERSION 3.0)
project(multi_dof_kinematic_calibration)

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")
elseif (CMAKE_CXX_COMPILER_ID MATCHES "Clang") # Works for Clang and AppleClang
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")
endif()

add_executable(${PROJECT_NAME}
               src/CalibrationDataIO.cpp
               src/Calibrator.cpp
               src/main.cpp
               include/multi_dof_kinematic_calibration/CeresUtil.h
               include/multi_dof_kinematic_calibration/Calibrator.h
               include/multi_dof_kinematic_calibration/CalibrationDataIO.h)

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

