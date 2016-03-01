/*
 *  PtuCalibration.h
 *
 *  Created on: 30.01.16
 *      Author: Stephan Manthe
 */

#ifndef PTUCALIBRATION_H_
#define PTUCALIBRATION_H_

#include "cameraSurveyingWithAprilTags/CameraModel.h"
#include "cameraSurveyingWithAprilTags/TagReconstructor.h"
#include "cameraSurveyingWithAprilTags/TagDetector.h"
#include <Eigen/Core>


class PtuCalibration
{
    public:
        std::map<int, camSurv::CameraModel> 
            reconstructCameras(const std::map<std::string, camSurv::TagImg>& tagImgName2tagImg,
                               const std::map<int, camSurv::ReconstructedTag>& reconstructedTags);
            
        Eigen::Matrix3d K; 
        Eigen::Matrix<double, 5, 1> distCoefficents;

    protected:
        std::map<int, camSurv::CameraModel> reconstructedCameras;
};

#endif
