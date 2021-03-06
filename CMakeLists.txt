cmake_minimum_required (VERSION 3.0)
project(multi_dof_kinematic_calibration_project)

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")
elseif (CMAKE_CXX_COMPILER_ID MATCHES "Clang") # Works for Clang and AppleClang
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")
endif()

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
find_package(OpenCV REQUIRED)

# Boost
find_package(Boost COMPONENTS filesystem system program_options REQUIRED) 
if(WIN32)
add_definitions(-D_USE_MATH_DEFINES)
add_definitions(-DBOOST_ALL_DYN_LINK)
set(Boost_USE_STATIC_LIBS OFF )
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
endif(WIN32)

find_package(Ceres REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/visual_marker_mapping/visual_marker_mapping/include)
add_subdirectory(visual_marker_mapping)

# bin dir for executable targets
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_library(multi_dof_kinematic_calibration_lib
               src/CalibrationDataIO.cpp
               src/Calibrator.cpp
               include/multi_dof_kinematic_calibration/CeresUtil.h
               include/multi_dof_kinematic_calibration/Calibrator.h
               include/multi_dof_kinematic_calibration/CalibrationDataIO.h
               include/multi_dof_kinematic_calibration/DebugVis.h)
           
set_property(TARGET multi_dof_kinematic_calibration_lib PROPERTY CXX_STANDARD 11)
set_property(TARGET multi_dof_kinematic_calibration_lib PROPERTY CXX_STANDARD_REQUIRED ON)

target_include_directories(multi_dof_kinematic_calibration_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
                    ${Boost_INCLUDE_DIRS})
                
target_link_libraries(multi_dof_kinematic_calibration_lib PUBLIC
                      ${OpenCV_LIBS}
                      ${Boost_LIBRARIES}
                      ${GLOG_LIBRARY}
                      visual_marker_mapping_lib
 )

                

add_executable(${PROJECT_NAME} src/main.cpp)

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD_REQUIRED ON)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
                    ${Boost_INCLUDE_DIRS})
                
target_link_libraries(${PROJECT_NAME} multi_dof_kinematic_calibration_lib visual_marker_mapping_lib)



configure_file(util/setup.sh setup.sh COPYONLY)
configure_file(visualize_results.py bin/visualize_results.py COPYONLY)

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    target_compile_definitions(${PROJECT_NAME} PUBLIC EIGEN_DONT_ALIGN_STATICALLY)
    target_compile_definitions(${PROJECT_NAME} PUBLIC _USE_MATH_DEFINES)
endif()
