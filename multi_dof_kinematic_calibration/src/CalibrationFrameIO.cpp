#include "multi_dof_kinematic_calibration/CalibrationFrameIO.h"
#include "visual_marker_mapping/DetectionIO.h"

#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <Eigen/Core>

#include "visual_marker_mapping/CameraUtilities.h"

namespace multi_dof_kinematic_calibration
{
//----------------------------------------------------------------------------
CalibrationData::CalibrationData(const std::string& filePath)
{
    namespace pt = boost::property_tree;
    pt::ptree rootNode;
    pt::read_json(filePath, rootNode);

    for (const auto& jointNode : rootNode.get_child("kinematic_chain"))
    {
        JointInfo inf;
        inf.name = jointNode.second.get<std::string>("name");
        inf.ticks_to_rad = jointNode.second.get<double>("ticks_to_rad");
        inf.angular_noise_std_dev = jointNode.second.get<double>("angular_noise_std_dev");
        joints.emplace_back(std::move(inf));
    }

    std::map<int, visual_marker_mapping::DetectionResult> detectionResultsByCamId;
    for (const auto& cameraNode : rootNode.get_child("cameras"))
    {
        const int camera_id = cameraNode.second.get<int>("camera_id");
        boost::filesystem::path camera_path = cameraNode.second.get<std::string>("camera_path");

        if (camera_path.is_relative())
            camera_path = boost::filesystem::path(filePath).parent_path() / camera_path;

        // Read cam intrinsics

        const std::string cam_intrin_filename = (camera_path / "camera_intrinsics.json").string();

        std::cout << "Trying to load camera intrinsics for camera " << camera_id << " from "
                  << cam_intrin_filename << " ..." << std::endl;

        boost::property_tree::ptree cameraTree;
        boost::property_tree::json_parser::read_json(cam_intrin_filename, cameraTree);
        cameraModelById.emplace(
            camera_id, visual_marker_mapping::propertyTreeToCameraModel(cameraTree));

        // Read marker detections

        const std::string marker_det_filename = (camera_path / "marker_detections.json").string();

        std::cout << "Trying to load marker detections for camera " << camera_id << " from "
                  << marker_det_filename << " ..." << std::endl;
        detectionResultsByCamId.emplace(
            camera_id, visual_marker_mapping::readDetectionResult(marker_det_filename));
        std::cout << "Read marker detections for camera " << camera_id << "!" << std::endl;
    }

    for (const auto& ptuPoseNode : rootNode.get_child("joint_configurations"))
    {
        CalibrationFrame ptuInfo;
        boost::filesystem::path image_path = ptuPoseNode.second.get<std::string>("image_path");
        if (image_path.is_relative())
            image_path = boost::filesystem::path(filePath).parent_path() / image_path;

        ptuInfo.image_path = image_path.string();
        ptuInfo.camera_id = ptuPoseNode.second.get<int>("camera_id");

        for (const JointInfo& jointInfo : joints)
        {
            pt::ptree jointPt = ptuPoseNode.second.get_child(jointInfo.name);
            const int ticks = jointPt.get<int>("ticks");
            ptuInfo.joint_config.push_back(ticks);
        }


        // find marker observations for this calibration frame
        int detectedImageId = -1;
        for (size_t j = 0; j < detectionResultsByCamId[ptuInfo.camera_id].images.size(); j++)
        {
            // this is a hack and should really be ==...
            if (image_path.filename()
                == detectionResultsByCamId[ptuInfo.camera_id].images[j].filename)
            {
                detectedImageId = static_cast<int>(j);
                break;
            }
        }
        for (const auto& tagObs : detectionResultsByCamId[ptuInfo.camera_id].tagObservations)
        {
            if (tagObs.imageId != detectedImageId)
                continue;
            ptuInfo.marker_observations.push_back(tagObs);
            ptuInfo.marker_observations.back().imageId = -1;
        }


        calib_frames.emplace_back(std::move(ptuInfo));
    }
}
//----------------------------------------------------------------------------
}
