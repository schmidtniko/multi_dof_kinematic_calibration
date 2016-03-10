#ifndef MULTI_DOF_KINEMATIC_CALIBRATION_CALIBRATOR_H
#define MULTI_DOF_KINEMATIC_CALIBRATION_CALIBRATOR_H

#include "visual_marker_mapping/Camera.h"
#include "visual_marker_mapping/TagReconstructor.h"
#include "visual_marker_mapping/TagDetector.h"
#include "CalibrationDataIO.h"

namespace multi_dof_kinematic_calibration
{
struct Calibrator
{
	Calibrator(CalibrationData calib_data);
	
    void calibrate();

    void optimizeJoint(size_t jointIndex);

    CalibrationData calib_data;

    bool computeRelativeCameraPoseFromImg(size_t camera_id, size_t calibration_frame_id,
        const Eigen::Matrix3d& K, const Eigen::Matrix<double, 5, 1>& distCoefficients,
        Eigen::Quaterniond& q, Eigen::Vector3d& t);

    void exportCalibrationResults(const std::string& filePath) const;


    /////////

    // frame, camera -> camera model
    std::map<std::pair<size_t, size_t>, Eigen::Matrix<double,7,1> > reconstructedPoses;

    //////////

//    struct DebugVis
//    {
//        visual_marker_mapping::Camera cam;
//        int type = 0;
//    };

//    std::vector<DebugVis> debugVis;


    struct JointData
    {
        Eigen::Matrix<double, 7, 1> joint_to_parent_pose;
        // std::vector<double> joint_positions;
        double ticks_to_rad;
    };

    Eigen::Matrix<double, 7, 1> cameraPose;

    std::vector<JointData> jointData;
};
}

#endif
