#ifndef PTUIMAGECAPTURE_H_
#define PTUIMAGECAPTURE_H_

#include "visual_marker_mapping/CameraModel.h"

#include <vector>
#include <functional>
#include <opencv2/opencv.hpp>


struct PanTiltImageInfo
{
    int cameraId = -1;
    std::string imagePath;
    double panAngle = 0.;
    double tiltAngle = 0.;
    int panTicks = 0;
    int tiltTicks = 0;
};


struct PtuImageCapture
{
    void startCapture(
        const std::function<std::map<int, cv::Mat>(double, double, int&, int&)>& captureFunction,
        const std::string& rootdirPath, bool preferTilt);

    void exportPanTiltImages(const std::string& filePath);

    void importPanTiltImages(const std::string& filePath);

    double panStartAngle;
    double panEndAngle;

    double tiltStartAngle;
    double tiltEndAngle;

    int numStepsPan;
    int numStepsTilt;

    // contains for every camera the corresponding camera model
    std::map<int, camSurv::CameraModel> cameraModelById;

    std::vector<PanTiltImageInfo> ptuImagePoses;

protected:
    void executeCapture(const std::string& rootDirPath,
        const std::function<std::map<int, cv::Mat>(double, double, int&, int&)>& captureFunction,
        double panAngle, double tiltAngle, int counter);
};
#endif
