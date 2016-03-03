#include "multi_dof_kinematic_calibration/PtuImageCapture.h"

#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <Eigen/Core>

#include "visual_marker_mapping/CameraUtilities.h"

namespace multi_dof_kinematic_calibration
{
void PtuImageCapture::startCapture(
    const std::function<std::map<int, cv::Mat>(double, double, int&, int&)>& captureFunction,
    const std::string& rootdirPath, bool preferTilt)
{
    if (!boost::filesystem::is_directory(rootdirPath))
        throw std::runtime_error("The root director does not exist.");

    ptuImagePoses.clear();

    if (panStartAngle > panEndAngle) std::swap(panStartAngle, panEndAngle);

    if (tiltStartAngle > tiltEndAngle) std::swap(tiltStartAngle, tiltEndAngle);

    const double stepWidthPan = (fabs(panEndAngle) + fabs(panStartAngle)) / numStepsPan;
    const double stepWidthTilt = (fabs(tiltEndAngle) + fabs(tiltStartAngle)) / numStepsTilt;

    int counter = 0;

    if (preferTilt)
    {
        // calculate the start position for the pan angle
        double panAngle = panStartAngle;
        while (panAngle <= panEndAngle)
        {
            // calculate the start position for the tilt angle
            double tiltAngle = tiltStartAngle;
            while (tiltAngle <= tiltEndAngle)
            {
                executeCapture(rootdirPath, captureFunction, panAngle, tiltAngle, counter);

                tiltAngle += stepWidthTilt;
                counter++;
            }

            panAngle += stepWidthPan;
        }
    }
    else
    {
        double tiltAngle = tiltStartAngle;
        while (tiltAngle <= tiltEndAngle)
        {
            double panAngle = panStartAngle;
            while (panAngle <= panEndAngle)
            {
                executeCapture(rootdirPath, captureFunction, panAngle, tiltAngle, counter);


                panAngle += stepWidthPan;
                counter++;
            }

            tiltAngle += stepWidthTilt;
        }
    }
}
//----------------------------------------------------------------------------
void PtuImageCapture::executeCapture(const std::string& rootDirPath,
    const std::function<std::map<int, cv::Mat>(double, double, int&, int&)>& captureFunction,
    double panAngle, double tiltAngle, int counter)
{
#if 0 // TODO(smanthe)
    // call the capture callback
    PanTiltImageInfo ptuInfo;
    ptuInfo.panAngle = panAngle;
    ptuInfo.tiltAngle = tiltAngle;
    
    std::map<int, cv::Mat> imageCameraPairs = captureFunction(panAngle,
                                                              tiltAngle,
                                                              ptuInfo.panTicks,
                                                              ptuInfo.tiltTicks);
     
    if (imageCameraPairs.size() == 0)
        std::cerr << "No images captured." << std::endl;
    else
    {
        for (const auto& image : imageCameraPairs)
        {
            const int cameraId = image.first;
    
            std::string folderPath = rootDirPath + boost::filesystem::path::preferred_separator;
            std::string relativePath = "cam" + std::to_string(cameraId) + boost::filesystem::path::preferred_separator;
            
            folderPath += relativePath; 
            if (!boost::filesystem::exists(folderPath))
            {
                std::cout << "Creating path: " << folderPath << std::endl;
                boost::filesystem::create_directories(folderPath);
            }
    
            const std::string imageName = "img_" + std::to_string(counter) + ".png";
            const std::string filePath = folderPath + imageName;
            cv::imwrite(filePath, image.second);
    
            ptuInfo.cameraId = cameraId;
            ptuInfo.imagePath = relativePath + imageName;
            ptuImagePoses.push_back(ptuInfo);
        }
    }
#endif
}
//----------------------------------------------------------------------------
void PtuImageCapture::importPanTiltImages(const std::string& filePath)
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
void PtuImageCapture::exportPanTiltImages(const std::string& filePath)
{
    namespace pt = boost::property_tree;
    pt::ptree rootNode;

    //    pt::ptree metaInfoNode;
    //    metaInfoNode.put("pan_start_angle", panStartAngle);
    //    metaInfoNode.put("pan_end_angle", panEndAngle);
    //
    //    metaInfoNode.put("tilt_start_angle", tiltStartAngle);
    //    metaInfoNode.put("tilt_end_angle", tiltEndAngle);
    //
    //    metaInfoNode.put("num_steps_pan", numStepsPan);
    //    metaInfoNode.put("num_steps_tilt", numStepsTilt);
    //
    //    rootNode.add_child("meta_information", metaInfoNode);


    // write camera names
    pt::ptree cameraModelsPt;
    for (const auto& camModelPair : cameraModelById)
    {
        const int modelId = camModelPair.first;
        const visual_marker_mapping::CameraModel& camModel = camModelPair.second;

        pt::ptree cameraModelPt = visual_marker_mapping::cameraModelToPropertyTree(camModel);
        cameraModelPt.put("model_id", modelId);
        cameraModelsPt.push_back(std::make_pair("", cameraModelPt));
    }

    rootNode.add_child("camera_models", cameraModelsPt);

    pt::ptree ptuInfoPt;
    for (const auto& panTiltImage : ptuImagePoses)
    {
        pt::ptree ptuInfoNode;
        ptuInfoNode.put("image_path", panTiltImage.imagePath);
        ptuInfoNode.put("model_id", panTiltImage.cameraId);
        ptuInfoNode.put("location_id", panTiltImage.locationId);

        // for (const auto& jointState : panTiltImage.jointConfiguration)
        for (int i = 0; i < panTiltImage.jointConfiguration.size(); i++)
        {
            // const auto& jointSetting = angleTicks.second;
            pt::ptree angleTicksPt;
            // angleTicksPt.put("angle", jointState.second);
            angleTicksPt.put("ticks", panTiltImage.jointConfiguration[i]);

            // const auto& jointName = angleTicks.first;
            ptuInfoNode.add_child(jointNames[i], angleTicksPt);
        }

        ptuInfoPt.push_back(std::make_pair("", ptuInfoNode));
    }

    rootNode.add_child("joint_configurations", ptuInfoPt);
    pt::write_json(filePath, rootNode);
}
}