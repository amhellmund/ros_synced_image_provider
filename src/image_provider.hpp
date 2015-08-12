#pragma once

#include <exception>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/tuple/tuple.hpp>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

/**
 * Type definitions.
 */
typedef boost::shared_ptr< std::vector<boost::filesystem::path> > FileListPtr;
typedef boost::tuple<bool, std::string> FlagErrorReturnValue;

/**
 * \brief Custom exception class.
 */
class ImageProviderException : public std::exception {
public:
    /**
     * \brief Constructor.
     * @param msg The exception message.
     */
    ImageProviderException(const std::string msg) : msg(msg) {}

    /**
     * \brief Exception identification.
     * @return The exception message.
     */
    virtual const char *what()  throw() { return msg.c_str(); }

    ~ImageProviderException() throw() {}

private:
    std::string msg;                            /**< Exception message. */
};


/**
 * Enumerations of supported image types.
 */
enum ImageType {
    IMGTYPE_MONO8 = 0,              /**< Grayscale images (8-bits). */
    IMGTYPE_RGB8 = 1,               /**< RGB images (8-bits per channel) */
    IMGTYPE_BGR8 = 2                /**< BGR images (8-bits per channel) */
};

static inline ImageType getImageTypeFromString (const std::string& imageType) {
    std::string s (imageType);
    boost::algorithm::to_lower(s);
    if (s == "mono8") { return IMGTYPE_MONO8; }
    if (s == "rgb8") { return IMGTYPE_RGB8; }
    if (s == "bgr8") { return IMGTYPE_BGR8; }
    throw ImageProviderException(std::string("Invalid image type provided: ") + boost::lexical_cast<std::string>(imageType));
}

static inline int getCVLoadTypeFromImageType (const ImageType imageType) {
    switch (imageType) {
    case IMGTYPE_MONO8:
        return CV_LOAD_IMAGE_GRAYSCALE;
    case IMGTYPE_RGB8:
    case IMGTYPE_BGR8:
        return CV_LOAD_IMAGE_COLOR;
    default:
        throw ImageProviderException(std::string("Invalid image type specified: ") + boost::lexical_cast<std::string>(imageType));
    }
}

static inline std::string getImageEncodingFromImageType (const ImageType imageType) {
    switch (imageType) {
    case IMGTYPE_MONO8:
        return sensor_msgs::image_encodings::MONO8;
    case IMGTYPE_RGB8:
        return sensor_msgs::image_encodings::RGB8;
    case IMGTYPE_BGR8:
        return sensor_msgs::image_encodings::BGR8;
    default:
        throw ImageProviderException(std::string("Invalid image type specified:") + boost::lexical_cast<std::string>(imageType));
    }
}


/**
 * \brief Single Camera configuration entry in multi-camera setup.
 */
struct CameraConfigEntry {
    typedef std::vector<CameraConfigEntry> List;
    typedef boost::shared_ptr<CameraConfigEntry> Ptr;
    typedef boost::shared_ptr<const CameraConfigEntry> ConstPtr;
    typedef std::vector<ConstPtr> ConstPtrList;

    std::string cameraName;                     /**< The camera name. */
    ImageType imageType;                        /**< The type of the images to be loaded. */
    std::string directoryName;                  /**< The directory name where the images reside in.
                                                     This is relative to the sourceDir in the provider configuration (see below) */
    int bufferSize;                             /**< The size of the image transport buffer. */
};


/**
 * \brief Image provider configuration
    */
struct ImageProviderConfiguration {
    typedef boost::shared_ptr<ImageProviderConfiguration> Ptr;
    typedef boost::shared_ptr<const ImageProviderConfiguration> ConstPtr;

    std::string sourceDir;                              /**< The base dir for the multi-camera setup. */
    std::string filePattern;                            /**< The file pattern for the images. */
    std::string topicPrefix;                            /**< The ROS topic prefix. */
    int rate;                                           /**< The rate at which images are provided. */
    int startPos;                                       /**< The start position inside the image sequence. */
    bool loop;							                /**< Repeat the sequence at the end. */
    CameraConfigEntry::ConstPtrList cameraConfigs;      /**< A list of camera configurations. */
};


/**
 *
 * \brief Image provider runtime data.
 *
 * There is a one-to-one correspondence between camera configurations and runtime data. The runtime data is
 * loaded from the configuration data.
 */
struct ImageProviderRuntimeData {
    typedef std::vector<ImageProviderRuntimeData> List;

    boost::shared_ptr<image_transport::Publisher> pubImage;     /**< The ROS image publisher. */
    boost::filesystem::path directory;                          /**< The base directory where the file reside in. */
    FileListPtr fileList;                                       /**< The list of files to provide. */
    CameraConfigEntry::ConstPtr cameraConfiguration;            /**< The camera configuration. */
};


/**
 * \brief Main class for providing images into the ROS system.
 */
class ImageProvider {
public:
    /**
     * \brief Constructor.
     * @param nodeHandle The ROS node handle.
     * @param configuration The configuration of the image provider.
     */
    ImageProvider (ros::NodeHandle& nodeHandle, const ImageProviderConfiguration::ConstPtr& configuration);

    /**
     * \brief Resets the image provider.
     */
    void reset () { imagePos = 0; }

    /**
     * \brief Set the current image position.
     *
     * @param pos The current image position.
     */
    void set (uint64_t pos) { imagePos = pos; }

    /**
     * \brief Execute the image provider.
     *
     * This function is the main loop of the image provider, that is, it reads the images from
     * disk and publishes them synchronously into the ROS system.
     */
    void run ();

    /**
     * \brief Execute the image provider.
     *
     * Executes the main loop for a given number of times.
     * @param numImages The number of images to provide given the rate.
     *
     */
    void run (const uint64_t numImages);

    /**
     * \brief Interrupt the image provider.
     *
     * Interrupts the image provider.
     */
    void interrupt ();

    /**
     * \brief Checks if all images haven been provided
     *
     * @return True if all images haven been provided.
     */
    bool isFinished () { return imagePos >= numberOfImages; }

    /**
     * \brief Destructor.
     */
    ~ImageProvider ();

private:
    /**
     * \brief Validates if the loaded file lists are correct.
     * @param runtimeData The runtime data loaded.
     * @return Flag that indicates if the runtime data is valid.
     */
    FlagErrorReturnValue validateFileLists (const ImageProviderRuntimeData::List& runtimeData);

    ros::NodeHandle nodeHandle;             	        /**< The ROS node handle. */
    image_transport::ImageTransport imgTransport;       /**< The image transport handle. */
    ImageProviderRuntimeData::List runtimeData;         /**< The image provider runtime data. */
    ImageProviderConfiguration::ConstPtr configuration; /**< The image provider configuration. */
    unsigned long numberOfImages;					    /**< The total number of images to be provided (excluding repititions). */
    unsigned long imagePos;							    /**< The (internal) state of the image provider (position inside list). */
};
