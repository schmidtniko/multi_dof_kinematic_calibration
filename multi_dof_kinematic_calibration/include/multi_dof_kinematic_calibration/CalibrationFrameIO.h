#ifndef MULTI_DOF_KINEMATIC_CALIBRATION_CALIBRATIONFRAMEIO_h
#define MULTI_DOF_KINEMATIC_CALIBRATION_CALIBRATIONFRAMEIO_h

#include "visual_marker_mapping/CameraModel.h"
#include "visual_marker_mapping/TagDetector.h"

#include <vector>
#include <functional>
#include <opencv2/opencv.hpp>

namespace multi_dof_kinematic_calibration
{
struct JointInfo
{
    std::string name;
    double ticks_to_rad; // ticks * ticks_to_rad = rad
    double angular_noise_std_dev;
};
struct CalibrationFrame
{
    int camera_id = -1;
    std::string image_path;
    std::vector<int> joint_config; // jointName --> {ticks}
	
	std::vector<visual_marker_mapping::TagObservation> marker_observations;
};


struct CalibrationData
{
    CalibrationData() {}
    CalibrationData(const std::string& filePath);

    std::map<int, visual_marker_mapping::CameraModel> cameraModelById;

    std::vector<CalibrationFrame> calib_frames;
    std::vector<JointInfo> joints;
};
}

#endif
