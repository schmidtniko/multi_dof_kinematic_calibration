#ifndef PTUCALIBRATIONPROJECT_H_

#include "visual_marker_mapping/Camera.h"
#include "visual_marker_mapping/TagReconstructor.h"
#include "visual_marker_mapping/TagDetector.h"
#include "multi_dof_kinematic_calibration/PtuImageCapture.h"

struct PtuCalibrationProject
{
    void processFolder(const std::string& folder);

    // Reconstruction
    std::map<int, visual_marker_mapping::ReconstructedTag> reconstructedTags;

    // Pan Tilt data
    PtuImageCapture ptuData;

    // Pan Tilt Image Detections
    visual_marker_mapping::DetectionResult ptuDetectionResult;

    bool computeRelativeCameraPoseFromImg(int imageId, const Eigen::Matrix3d& K,
        const Eigen::Matrix<double, 5, 1>& distCoefficients, Eigen::Quaterniond& q,
        Eigen::Vector3d& t) const;


    /////////

    std::map<int, visual_marker_mapping::Camera> reconstructedPoses;

    //////////

    struct DebugVis
    {
        visual_marker_mapping::Camera cam;
    };

    std::vector<DebugVis> debugVis;
};

#endif
