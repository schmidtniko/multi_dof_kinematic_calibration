#include <iostream>
#include <string>

#include "ptuImageCapture/PtuImageCapture.h"
#include "cameraSurveyingWithAprilTags/TagReconstructor.h"
#include "cameraSurveyingWithAprilTags/TagDetector.h"
int main()
{

    std::string rootPath
        = "/Users/stephanmanthe/ownCloud/Uni/Master/Forschungsarbeit/ptuImages/lisa/firstTest/";

#if 0
    std::string filePathPanTiltData = rootPath + "panTiltData.json"
    PtuImageCapture ptuImageData;
    ptuImageData.importPanTiltImages(filePath);
    
    std::string folderPath = ptuImageData.rootdirPath;
    for(std::string cameraName : ptuImageData.usedCameras)
    {
        std::vector<std::string> filePaths;
        for(auto img : ptuImageData.capturedImages)
        {
            std::string filePath = folderPath + "/" + cameraName +"/"+  img.imageNames[cameraName];
            filePaths.push_back(filePath);
        }

        double tagWidth, tagHeight;
        tagWidth =  0.116;
        tagHeight = 0.117;

        int visWidth, visHeight;
        visWidth = 1500;
        visHeight = 1000;

        // TODO celanup, separate visualization, extraction and detection
        std::string exportFilename = cameraName + "_detectedTags.json";
        camSurv::TagDetector detector(folderPath,
                                      "",
                                      "detections/",
                                      exportFilename,
                                      visHeight,
                                      visWidth,
                                      tagWidth,
                                      tagHeight);

        detector.detectTags(filePaths, false);
    }
#else
    std::string observationsPath = rootPath + "/irImage_detectedTags.json";
    std::string reconstructionPath = rootPath + "/reconstructedObjects.json";


#endif


    return 0;
}
