#include "spinnaker_sdk_camera_driver/ImageEventHandler.h"

ImageEventHandler::ImageEventHandler(CameraPtr pCam, bool if_color, bool to_ros, bool save,
        const shared_ptr<tbb::concurrent_queue<Mat>>& ros_queue_ptr, 
        const shared_ptr<tbb::concurrent_queue<cvMatContainer*>>& save_queue_ptr)
{ 
    // Retrieve device serial number
    INodeMap & nodeMap = pCam->GetTLDeviceNodeMap();
    m_deviceSerialNumber = "";
    CStringPtr ptrDeviceSerialNumber = nodeMap.GetNode("m_deviceSerialNumber");
    if (IsAvailable(ptrDeviceSerialNumber) && IsReadable(ptrDeviceSerialNumber))
    {
            m_deviceSerialNumber = ptrDeviceSerialNumber->GetValue();
    }
    // Initialize image counter to 0
    m_imageCnt = 0;
    
    this->COLOR_ = if_color;
    this->SAVE_ = save;
    this->TO_ROS_ = to_ros;
    this->SAVE_queue = save_queue_ptr;
    this->ROS_queue = ros_queue_ptr;
}

ImageEventHandler::~ImageEventHandler() {}

// This method defines an image event. In it, the image that triggered the 
// event is converted and saved before incrementing the count. Please see 
// Acquisition_CSharp example for more in-depth comments on the acquisition 
// of images.
void ImageEventHandler::OnImageEvent(ImagePtr image)
{
    // Save a maximum of 10 images
    
    ROS_DEBUG_STREAM("Image event occurred...");
    // Check image retrieval status
    if (image->IsIncomplete())
    {
        ROS_WARN_STREAM("Image incomplete with image status " << image->GetImageStatus() << "...");
    }
    else
    {
        // Print image information
        ROS_DEBUG_STREAM("Grabbed image " << m_imageCnt << ", width = " << image->GetWidth() << ", height = " << image->GetHeight());
        // from chunkdata to get timestamp
        ChunkData chunkData = image->GetChunkData();
        int64_t timestamp = chunkData.GetTimestamp();
        // Convert image
        ImagePtr convertedImage;
        if (COLOR_)
            convertedImage = image->Convert(PixelFormat_BGR8, HQ_LINEAR); 
        else
            convertedImage = image->Convert(PixelFormat_Mono8, HQ_LINEAR); 
        // Increment image counter
        save2queue(convertedImage, timestamp, m_imageCnt);
    }
    m_imageCnt++;
    
}
// Getter for image counter
int ImageEventHandler::getImageCount()
{
        return m_imageCnt;
}


void ImageEventHandler::save2queue(ImagePtr convertedImage, int64_t time_stamp,  int framecount)
{	
    unsigned int XPadding = convertedImage->GetXPadding();
    unsigned int YPadding = convertedImage->GetYPadding();
    unsigned int rowsize = convertedImage->GetWidth();
    unsigned int colsize = convertedImage->GetHeight();

    //image data contains padding. When allocating Mat container size, you need to account for the X,Y image data padding.
    Mat img;
    if (COLOR_)
        img = Mat(colsize + YPadding, rowsize + XPadding, CV_8UC3, convertedImage->GetData(), convertedImage->GetStride());
    else
        img = Mat(colsize + YPadding, rowsize + XPadding, CV_8UC1, convertedImage->GetData(), convertedImage->GetStride());
    if(SAVE_){
        cvMatContainer* temp_container = new cvMatContainer(img, time_stamp, framecount, true);
        SAVE_queue->push(temp_container);
    }
    if(TO_ROS_ )//&& (m_imageCnt%2)==0 )
        ROS_queue->push(img.clone());

    
}