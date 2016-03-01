/*
 *  PtuCalibration.cpp
 *
 *  Created on: 30.01.16
 *      Author: Stephan Manthe
 */

#include "ptuCalibration/PtuCalibration.h"
#include <opencv2/opencv.hpp>
#include "cameraSurveyingWithAprilTags/eigenCVConversions.h"
#include <opencv2/core/eigen.hpp>

std::map<int, camSurv::CameraModel> 
PtuCalibration::reconstructCameras(const std::map<std::string, camSurv::TagImg>& tagImgName2tagImg,
                                   const std::map<int, camSurv::ReconstructedTag>& reconstructedTags)
{
//    if(tagImgName2tagImg.empty())
//        throw std::runtime_error("Tag images are empty.");
//
//    if(reconstructedTags.empty())
//        throw std::runtime_error("Reconstructed tags are empty.");
//
    std::map<int, camSurv::CameraModel> reconstructedCams; 
//    int minNumObservations = 4; 
//
//    for (auto tagImgPair : tagImgName2tagImg)
//    {
//        const camSurv::TagImg& tagImg = tagImgPair.second;
//        
//        if(tagImg.observations.size() < minNumObservations)
//            continue;
//
//        std::vector<Eigen::Vector3d> objectPoints;
//        std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > observations;
//        for (auto obsPair : tagImg.observations)
//        {
//            const camSurv::TagObservation& obs = obsPair.second;
//            std::vector<cv::Point2d> observationsTmp;
//
//            observations.insert(observations.end(),
//                                obs.corners.begin(),
//                                obs.corners.end());
//
//            const camSurv::ReconstructedTag& recTag = reconstructedTags.find(obs.id)->second;
//            
//            const std::vector<Eigen::Vector3d> points3DTmp = recTag.computeMarkerCorners3D();
//            objectPoints.insert(objectPoints.end(),
//                            points3DTmp.begin(),
//                            points3DTmp.end());
//        }
//
//        Eigen::Matrix3d R;
//        Eigen::Vector3d t;
//        utilities::solvePnPEigen(objectPoints,
//                                 observations,
//                                 K,
//                                 distCoefficents,
//                                 R,
//                                 t);
//
//
//    
//        Eigen::Matrix4d extMat = Eigen::Matrix4d::Identity();
//        extMat.block<3,3>(0,0) = R;
//        extMat.block<3,1>(0,3) = t;
//
//        Eigen::Matrix4d calibMatrix4d = Eigen::Matrix4d::Identity();
//        calibMatrix4d.block<3,3>(0,0) = K;
//        camSurv::CameraModel camModel(tagImg.id, calibMatrix4d, extMat);
//
//        reconstructedCams[tagImg.id] = camModel;
//    }
//    
    return reconstructedCams;
}
