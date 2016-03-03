#ifndef MULTI_DOF_KINEMATIC_CALIBRATION_PTUIMAGECAPTURE_H
#define MULTI_DOF_KINEMATIC_CALIBRATION_PTUIMAGECAPTURE_H

#include "visual_marker_mapping/CameraModel.h"

#include <vector>
#include <functional>
#include <opencv2/opencv.hpp>


// struct PanTiltImageInfo
//{
//    int cameraId = -1;
//    std::string imagePath;
//    double panAngle = 0.;
//    double tiltAngle = 0.;
//    int panTicks = 0;
//    int tiltTicks = 0;
//};

namespace multi_dof_kinematic_calibration
{
struct JointImageInfo
{
    int locationId;
    int cameraId = -1;
    std::string imagePath = "";
    std::vector<int> jointConfiguration; // jointName --> {ticks}
};


struct PtuImageCapture
{
    void startCapture(
        const std::function<std::map<int, cv::Mat>(double, double, int&, int&)>& captureFunction,
        const std::string& rootdirPath, bool preferTilt);

    void exportPanTiltImages(const std::string& filePath);

    void importPanTiltImages(const std::string& filePath);


    // TODO SM remove then cleanup ptuimage capture
    double panStartAngle;
    double panEndAngle;

    double tiltStartAngle;
    double tiltEndAngle;

    int numStepsPan;
    int numStepsTilt;

    // contains for every camera the corresponding camera model
    std::map<int, visual_marker_mapping::CameraModel> cameraModelById;

    std::vector<JointImageInfo> ptuImagePoses;

    std::vector<std::string> jointNames;

protected:
    void executeCapture(const std::string& rootDirPath,
        const std::function<std::map<int, cv::Mat>(double, double, int&, int&)>& captureFunction,
        double panAngle, double tiltAngle, int counter);
};
}

#endif
