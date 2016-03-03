#include "multi_dof_kinematic_calibration/CalibrationFrameIO.h"

#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <Eigen/Core>

#include "visual_marker_mapping/CameraUtilities.h"

namespace multi_dof_kinematic_calibration
{
//----------------------------------------------------------------------------
PtuImageCapture::PtuImageCapture(const std::string& filePath)
{
    namespace pt = boost::property_tree;
    pt::ptree rootNode;
    pt::read_json(filePath, rootNode);

    //    panStartAngle = rootNode.get<double>("meta_information.pan_start_angle");
    //    panEndAngle = rootNode.get<double>("meta_information.pan_end_angle");
    //
    //    tiltStartAngle = rootNode.get<double>("meta_information.tilt_start_angle");
    //    tiltEndAngle = rootNode.get<double>("meta_information.tilt_end_angle");
    //
    //    numStepsPan = rootNode.get<int>("meta_information.num_steps_pan");
    //    numStepsTilt = rootNode.get<int>("meta_information.num_steps_tilt");

    jointNames.clear();
    for (const auto& name : rootNode.get_child("joint_names"))
    {
        const auto& jointName = name.second.get_value<std::string>();
        jointNames.push_back(jointName);
    }

    // read camera models
    cameraModelById.clear();
    for (const auto& cameraModelPair : rootNode.get_child("camera_models"))
    {
        pt::ptree cameraModelPt = cameraModelPair.second;
        int cameraId = cameraModelPt.get<int>("camera_model_id");
        visual_marker_mapping::CameraModel camModel
            = visual_marker_mapping::propertyTreeToCameraModel(cameraModelPt);
        //        std::cout << "K = " << camModel.getK() << std::endl;
        //        std::cout << "d = " << camModel.distortionCoefficients << std::endl;
        cameraModelById[cameraId] = camModel;
    }

    ptuImagePoses.clear();
    for (const auto& ptuPoseNode : rootNode.get_child("joint_configurations"))
    {
        JointImageInfo ptuInfo;
        ptuInfo.imagePath = ptuPoseNode.second.get<std::string>("image_path");
        ptuInfo.cameraId = ptuPoseNode.second.get<int>("camera_model_id");
        ptuInfo.locationId = ptuPoseNode.second.get<int>("location_id");

        for (const auto& jointName : jointNames)
        {
            pt::ptree jointPt = ptuPoseNode.second.get_child(jointName);
            double angle = jointPt.get<double>("angle");
            int ticks = jointPt.get<int>("ticks");
            ptuInfo.jointConfiguration.push_back(ticks);
        }
        ptuImagePoses.push_back(ptuInfo);
    }
}
//----------------------------------------------------------------------------
}