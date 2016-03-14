#ifndef MULTI_DOF_KINEMATIC_CALIBRATION_CALIBRATIONFRAMEIO_h
#define MULTI_DOF_KINEMATIC_CALIBRATION_CALIBRATIONFRAMEIO_h

#include "visual_marker_mapping/CameraModel.h"
#include "visual_marker_mapping/TagDetector.h"
#include "visual_marker_mapping/TagReconstructor.h"

#include <vector>
#include <functional>
#include <opencv2/opencv.hpp>
#include <memory>

namespace multi_dof_kinematic_calibration
{
struct Scan3D
{
    Eigen::Matrix<double, 3, Eigen::Dynamic> points;
};

struct JointInfo
{
    std::string name;
    std::string type;
    double ticks_to_rad; // ticks * ticks_to_rad = rad
    double angular_noise_std_dev;

    std::string parent;

    Eigen::Matrix<double, 7, 1> joint_to_parent_guess;

    bool fixed;
};
struct CalibrationFrame
{
    int location_id; // -1 for no location

    std::vector<int> joint_config; // jointName --> {ticks}

    std::map<int, std::map<std::uint32_t, Eigen::Vector2d> > cam_id_to_observations;
	std::map<int, std::shared_ptr<Scan3D> > sensor_id_to_laser_scan_3d;
};


struct CalibrationData
{
    CalibrationData(const std::string& filePath);

    std::map<int, visual_marker_mapping::CameraModel> cameraModelById;
    std::vector<int> laser_sensor_ids;
	
	std::map<int, std::string> sensor_id_to_type;

    std::vector<CalibrationFrame> calib_frames;
    std::vector<JointInfo> joints;

    std::map<std::string, size_t> name_to_joint;

    std::map<int, std::string> sensor_id_to_parent_joint;

    std::map<std::uint32_t, Eigen::Vector3d> reconstructed_map_points;
	std::map<int, visual_marker_mapping::ReconstructedTag> reconstructed_tags;
};
}

#endif
