#ifndef PTUCALIBRATIONPROJECT_H_

#include "visual_marker_mapping/Camera.h"
#include "visual_marker_mapping/TagReconstructor.h"
#include "visual_marker_mapping/TagDetector.h"
#include "multi_dof_kinematic_calibration/PtuImageCapture.h"

struct PtuCalibrationProject
{
    void processFolder(const std::string& folder);

    // Reconstruction
    std::map<int, camSurv::ReconstructedTag> reconstructedTags;

    // Pan Tilt data
    PtuImageCapture ptuData;

    // Pan Tilt Image Detections
    camSurv::DetectionResult ptuDetectionResult;

    bool computeRelativeCameraPoseFromImg(int imageId, const Eigen::Matrix3d& K,
        const Eigen::Matrix<double, 5, 1>& distCoefficients, Eigen::Quaterniond& q,
        Eigen::Vector3d& t) const;


    /////////

    std::map<int, camSurv::Camera> reconstructedPoses;

    //////////

    struct DebugVis
    {
        camSurv::Camera cam;
    };

    std::vector<DebugVis> debugVis;
};

#endif
