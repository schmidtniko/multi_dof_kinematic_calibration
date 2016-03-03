#include "multi_dof_kinematic_calibration/PtuImageCapture.h"

#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <Eigen/Core>

#include "visual_marker_mapping/CameraUtilities.h"

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
    // call the capture callback
    PanTiltImageInfo ptuInfo;
    ptuInfo.panAngle = panAngle;
    ptuInfo.tiltAngle = tiltAngle;

    std::map<int, cv::Mat> imageCameraPairs
        = captureFunction(panAngle, tiltAngle, ptuInfo.panTicks, ptuInfo.tiltTicks);

    if (imageCameraPairs.size() == 0)
        std::cerr << "No images captured." << std::endl;
    else
    {
        for (auto image : imageCameraPairs)
        {
            const int cameraId = image.first;

            std::string folderPath = rootDirPath + boost::filesystem::path::preferred_separator;
            std::string relativePath
                = "cam" + std::to_string(cameraId) + boost::filesystem::path::preferred_separator;

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
}
//----------------------------------------------------------------------------
void PtuImageCapture::importPanTiltImages(const std::string& filePath)
{
    namespace pt = boost::property_tree;
    pt::ptree rootNode;
    pt::read_json(filePath, rootNode);

    panStartAngle = rootNode.get<double>("pan_start_angle");
    panEndAngle = rootNode.get<double>("pan_end_angle");

    tiltStartAngle = rootNode.get<double>("tilt_start_angle");
    tiltEndAngle = rootNode.get<double>("tilt_end_angle");

    numStepsPan = rootNode.get<int>("num_steps_pan");
    numStepsTilt = rootNode.get<int>("num_steps_tilt");


    // read camera models
    cameraModelById.clear();
    for (auto cameraModelPair : rootNode.get_child("camera_models"))
    {
        pt::ptree cameraModelPt = cameraModelPair.second;
        int cameraId = cameraModelPt.get<int>("model_id");
        visual_marker_mapping::CameraModel camModel = visual_marker_mapping::propertyTreeToCameraModel(cameraModelPt);
        cameraModelById[cameraId] = camModel;
    }

    ptuImagePoses.clear();
    for (auto ptuPoseNode : rootNode.get_child("ptu_positions"))
    {
        PanTiltImageInfo ptuInfo;
        ptuInfo.panAngle = ptuPoseNode.second.get<double>("pan_angle");
        ptuInfo.panTicks = ptuPoseNode.second.get<double>("pan_ticks");
        ptuInfo.tiltAngle = ptuPoseNode.second.get<double>("tilt_angle");
        ptuInfo.tiltTicks = ptuPoseNode.second.get<double>("tilt_ticks");
        ptuInfo.imagePath = ptuPoseNode.second.get<std::string>("image_path");
        ptuInfo.cameraId = ptuPoseNode.second.get<int>("camera_id");
        ptuImagePoses.push_back(ptuInfo);
    }
}
//----------------------------------------------------------------------------
void PtuImageCapture::exportPanTiltImages(const std::string& filePath)
{
    namespace pt = boost::property_tree;
    pt::ptree rootNode;

    rootNode.put("pan_start_angle", panStartAngle);
    rootNode.put("pan_end_angle", panEndAngle);

    rootNode.put("tilt_start_angle", tiltStartAngle);
    rootNode.put("tilt_end_angle", tiltEndAngle);

    rootNode.put("num_steps_pan", numStepsPan);
    rootNode.put("num_steps_tilt", numStepsTilt);

    // write camera names
    pt::ptree cameraModelsPt;
    for (auto camModelPair : cameraModelById)
    {
        const int modelId = camModelPair.first;
        const visual_marker_mapping::CameraModel& camModel = camModelPair.second;

        pt::ptree cameraModelPt = visual_marker_mapping::cameraModelToPropertyTree(camModel);
        cameraModelPt.put("model_id", modelId);
        cameraModelsPt.push_back(std::make_pair("", cameraModelPt));
    }

    rootNode.add_child("camera_models", cameraModelsPt);

    pt::ptree ptuInfoPt;
    for (auto panTiltImage : ptuImagePoses)
    {
        pt::ptree ptuInfoNode;
        ptuInfoNode.put("image_path", panTiltImage.imagePath);
        ptuInfoNode.put("camera_id", panTiltImage.cameraId);

        ptuInfoNode.put("pan_angle", panTiltImage.panAngle);
        ptuInfoNode.put("pan_ticks", panTiltImage.panTicks);

        ptuInfoNode.put("tilt_angle", panTiltImage.tiltAngle);
        ptuInfoNode.put("tilt_ticks", panTiltImage.tiltTicks);

        ptuInfoPt.push_back(std::make_pair("", ptuInfoNode));
    }

    rootNode.add_child("ptu_positions", ptuInfoPt);
    pt::write_json(filePath, rootNode);
}
