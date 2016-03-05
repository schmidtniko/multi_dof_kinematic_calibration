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

	for (const auto& jointNode : rootNode.get_child("kinematic_chain"))
	{
		JointInfo inf;
		inf.name=jointNode.second.get<std::string>("name");
		inf.ticks_to_rad=jointNode.second.get<double>("ticks_to_rad");
		inf.angular_noise_std_dev=jointNode.second.get<double>("angular_noise_std_dev");
		joints.emplace_back(std::move(inf));
	}

    for (const auto& ptuPoseNode : rootNode.get_child("joint_configurations"))
    {
        JointImageInfo ptuInfo;
        ptuInfo.imagePath = ptuPoseNode.second.get<std::string>("image_path");
        ptuInfo.cameraId = ptuPoseNode.second.get<int>("camera_model_id");

        for (const JointInfo& jointInfo : joints)
        {
            pt::ptree jointPt = ptuPoseNode.second.get_child(jointInfo.name);
            const int ticks = jointPt.get<int>("ticks");
            ptuInfo.jointConfiguration.push_back(ticks);
        }
        ptuImagePoses.push_back(ptuInfo);
    }
}
//----------------------------------------------------------------------------
}
