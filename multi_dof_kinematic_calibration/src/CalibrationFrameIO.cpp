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

    for (const auto& name : rootNode.get_child("joint_names"))
    {
        jointNames.push_back(name.second.get_value<std::string>());
    }

    for (const auto& ptuPoseNode : rootNode.get_child("joint_configurations"))
    {
        JointImageInfo ptuInfo;
        ptuInfo.imagePath = ptuPoseNode.second.get<std::string>("image_path");
        ptuInfo.cameraId = ptuPoseNode.second.get<int>("camera_model_id");

        for (const auto& jointName : jointNames)
        {
            pt::ptree jointPt = ptuPoseNode.second.get_child(jointName);
            const int ticks = jointPt.get<int>("ticks");
            ptuInfo.jointConfiguration.push_back(ticks);
        }
        ptuImagePoses.push_back(ptuInfo);
    }
}
//----------------------------------------------------------------------------
}
