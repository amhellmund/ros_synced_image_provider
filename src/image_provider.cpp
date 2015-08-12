#include <sstream>
#include <cv_bridge/cv_bridge.h>
#include <boost/regex.hpp>
#include <boost/tuple/tuple.hpp>
#include <XmlRpc.h>
#include <XmlRpcException.h>

#include "image_provider.hpp"

namespace bfs = boost::filesystem;


ImageProvider::ImageProvider (ros::NodeHandle& nodeHandle, const ImageProviderConfiguration::ConstPtr& configuration)
        : nodeHandle(nodeHandle), imgTransport(nodeHandle), configuration(configuration), numberOfImages(0), imagePos(configuration->startPos) {
    // setup the file pattern to be used for all files
    boost::regex re(configuration->filePattern);
    // iterate the configurations and retrieve matching files
    runtimeData.reserve(configuration->cameraConfigs.size());
    if (configuration->cameraConfigs.size() > 0) {
        for (CameraConfigEntry::ConstPtrList::const_iterator it = configuration->cameraConfigs.begin(); it != configuration->cameraConfigs.end(); ++it) {
            bfs::path imageDir ((*it)->directoryName);
            if (!imageDir.is_absolute()) { imageDir = bfs::path(configuration->sourceDir) / imageDir; }
            if (!bfs::exists(imageDir)) {
                throw ImageProviderException(std::string("Image directory '") + imageDir.string() + std::string("' does not exist"));
            }
            if (!bfs::is_directory(imageDir)) {
                throw ImageProviderException(std::string("Image directory '") + imageDir.string() + std::string("' is not a directory"));
            }
            FileListPtr fileList = boost::make_shared<FileListPtr::element_type>();
            for (bfs::directory_iterator dirIterator (imageDir); dirIterator != bfs::directory_iterator(); ++dirIterator) {
                bfs::path fileName = dirIterator->path().filename();
                if (boost::regex_match(fileName.string(), re)) {
                    fileList->push_back(fileName);
                }
            }
            std::sort(fileList->begin(), fileList->end());
            ImageProviderRuntimeData rtData;
            rtData.cameraConfiguration = *it;
            rtData.directory = imageDir;
            rtData.fileList = fileList;
            runtimeData.push_back(rtData);
        }
        // validate the file lists
        FlagErrorReturnValue retVal = validateFileLists(runtimeData);
        if (boost::get<0>(retVal)) {
            for (ImageProviderRuntimeData::List::iterator it = runtimeData.begin(); it != runtimeData.end(); ++it) {
                it->pubImage = boost::make_shared<image_transport::Publisher>();
                *it->pubImage = imgTransport.advertise(configuration->topicPrefix + "/" + it->cameraConfiguration->cameraName, it->cameraConfiguration->bufferSize);
            }
            numberOfImages = runtimeData[0].fileList->size();
        }
        else {
            throw ImageProviderException("Validation of file lists: " + boost::get<1>(retVal));
        }
    }
    else {
        throw ImageProviderException("No camera configuration available");
    }
}

FlagErrorReturnValue ImageProvider::validateFileLists (const ImageProviderRuntimeData::List& runtimeData) {
    FlagErrorReturnValue returnValue (true, std::string());
    if (runtimeData.size() > 0) {
        // check the list sizes
        const size_t baseSize = runtimeData[0].fileList->size();
        for (int i = 1; i < runtimeData.size(); i++) {
            if (runtimeData[i].fileList->size() != baseSize) {
                std::stringstream ss;
                ss << "File list size validation failed: entry = " << i
                   << ", expected = " << baseSize
                   << ", got = " << runtimeData[i].fileList->size();
                boost::get<0>(returnValue) = false;
                boost::get<1>(returnValue) = ss.str();
                break;
            }
        }
        if (boost::get<0>(returnValue)) {
            // check the file names
            for (uint64_t i = 0; i < baseSize; i++) {
                bfs::path fileName = runtimeData[0].fileList->at(i);
                for (uint64_t j = 1; j < runtimeData.size(); j++) {
                    if (fileName != runtimeData[j].fileList->at(i)) {
                        std::stringstream ss;
                        ss << "File name validation failed: list position = " << i << ", entry = " << j
                           << ", expected = " << fileName
                           << ", got = " << runtimeData[j].fileList->at(i).string();
                        boost::get<0>(returnValue) = false;
                        boost::get<1>(returnValue) = ss.str();
                        break;
                    }
                }
            }
        }
    }
    return returnValue;
}

void ImageProvider::run () {
    if (runtimeData.size() > 0) {
        run(numberOfImages);
    }
}

void ImageProvider::run (const uint64_t numImages) {
    if (runtimeData.size() > 0) {
        const size_t fileSize = runtimeData[0].fileList->size();
        if (imagePos < fileSize) {
            ros::Rate r(configuration->rate);
            unsigned long imageCounter = 0;
            do {
                unsigned long loopCounter = 0;
                unsigned long imagePosLoop = imagePos;
                while (ros::ok() && imagePosLoop < fileSize && loopCounter < numImages) {
                    ros::Time time = ros::Time::now();
                    for (ImageProviderRuntimeData::List::const_iterator it = runtimeData.begin(); it != runtimeData.end(); ++it) {
                        bfs::path imageFileName = it->directory / it->fileList->at(imagePosLoop);
                        cv_bridge::CvImage cvImg;
                        cvImg.image = cv::imread(imageFileName.string(), getCVLoadTypeFromImageType(it->cameraConfiguration->imageType));
                        if (cvImg.image.data != NULL) {
                            cvImg.header.stamp = time;
                            cvImg.header.seq = imageCounter;
                            cvImg.encoding = getImageEncodingFromImageType(it->cameraConfiguration->imageType);
                            ROS_DEBUG_STREAM("Publishing image: " << imageFileName.string());
                            it->pubImage->publish(cvImg.toImageMsg());
                        }
                        else {
                            throw ImageProviderException(std::string("Failed to read image: " + imageFileName.string()));
                        }
                    }
                    r.sleep();
                    imagePosLoop++;
                    loopCounter++;
                    imageCounter++;
                }
            } while (configuration->loop && ros::ok());
        }
        else {
            throw ImageProviderException(std::string("The initialized image position (") + boost::lexical_cast<std::string>(imagePos)
                                       + std::string(") is larger than the total number of images (")
                                       + boost::lexical_cast<std::string>(fileSize) + std::string(")"));
        }
    }
}

ImageProvider::~ImageProvider () {
    for (ImageProviderRuntimeData::List::iterator it = runtimeData.begin(); it != runtimeData.end(); ++it) {
        it->pubImage->shutdown();
    }
}

/**
 * \brief Load a single camera configuration entry.
 * @param configNodeHandle The ROS node handle to read the configuration from.
 * @param cameraName The camera name to get the parameters for.
 * @return A single camera configuration entry.
 */
CameraConfigEntry::Ptr getCameraConfigurationEntry (ros::NodeHandle configNodeHandle, const std::string& cameraName) {
    CameraConfigEntry::Ptr configEntry = boost::make_shared<CameraConfigEntry>();
    configEntry->cameraName = cameraName;
    if (!configNodeHandle.getParam("cameras/" + cameraName + "/directory", configEntry->directoryName)) {
        throw ImageProviderException(std::string("Parameter 'directory' not found for camera: ") + cameraName);
    }
    std::string imageType;
    if (!configNodeHandle.getParam("cameras/" + cameraName + "/imagetype", imageType)) {
        throw ImageProviderException(std::string("Parameter 'imagetype' not found dfor camera: ") + cameraName);
    }
    if (!configNodeHandle.getParam("cameras/" + cameraName + "/buffersize", configEntry->bufferSize)) {
        configEntry->bufferSize = 1;
    }
    configEntry->imageType = getImageTypeFromString(imageType);
    return configEntry;
}

/**
 * \brief Load the configuration from file.
 * @param configurationPrefix The entry node inside the configuration YAML file.
 * @return The complete image provider configuration.
 */
ImageProviderConfiguration::ConstPtr getImageProviderConfiguration () {
    ImageProviderConfiguration::Ptr configuration = boost::make_shared<ImageProviderConfiguration>();
    // private parameters
    ros::NodeHandle privateNodeHandle ("~");
    std::string configurationPrefix;
    privateNodeHandle.param("cfgprefix", configurationPrefix, std::string("configuration"));
    privateNodeHandle.param("startpos", configuration->startPos, 0);
    privateNodeHandle.param("rate", configuration->rate, 1);
    privateNodeHandle.param("loop", configuration->loop, false);
    privateNodeHandle.param("topicprefix", configuration->topicPrefix, std::string("cameras"));
    ros::NodeHandle configNodeHandle (privateNodeHandle, configurationPrefix);
    // configuration parameters (supposed to be loaded via YAML file)
    if (!configNodeHandle.getParam("srcdir", configuration->sourceDir)) {
        throw ImageProviderException("Parameter 'srcdir' not found");
    }
    if (!configNodeHandle.getParam("filepattern", configuration->filePattern)) {
        throw ImageProviderException("Parameter 'filepattern' not found");
    }
    try {
        XmlRpc::XmlRpcValue cameraNames;
        if (configNodeHandle.getParam("cameras", cameraNames)) {
            std::map<std::string, XmlRpc::XmlRpcValue>::iterator nameIterator;
            for (nameIterator = cameraNames.begin(); nameIterator != cameraNames.end(); ++nameIterator) {
                std::string cameraName = nameIterator->first;
                ROS_INFO_STREAM("Reading camera configuration for camera: " << cameraName);
                CameraConfigEntry::ConstPtr cameraConfig = getCameraConfigurationEntry(configNodeHandle, cameraName);
                configuration->cameraConfigs.push_back(cameraConfig);
            }
        }
    }
    catch (XmlRpc::XmlRpcException& e) {
        throw ImageProviderException(std::string("Failed to retrieve camera names: ") + std::string(e.getMessage()));
    }
    return configuration;
}


int main (int argc, char *argv[]) {
    ros::init(argc, argv, "synchronized_image_provider");
    ros::NodeHandle nodeHandle;
    try {
        ImageProviderConfiguration::ConstPtr configuration = getImageProviderConfiguration();
        ImageProvider imageProvider (nodeHandle, configuration);
        imageProvider.run();
    }
    catch (ImageProviderException& e) {
        ROS_ERROR_STREAM("Failed to read provider configuration: " << e.what());
        exit(EXIT_FAILURE);
    }
    exit(EXIT_SUCCESS);
}

