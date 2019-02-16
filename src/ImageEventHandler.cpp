#include "spinnaker_sdk_camera_driver/ImageEventHandler.h"

ImageEventHandler::ImageEventHandler(CameraPtr pCam) 
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
    // Release reference to camera
    pCam = NULL;
}
ImageEventHandler::~ImageEventHandler() {}

// This method defines an image event. In it, the image that triggered the 
// event is converted and saved before incrementing the count. Please see 
// Acquisition_CSharp example for more in-depth comments on the acquisition 
// of images.
void ImageEventHandler::OnImageEvent(ImagePtr image)
{
    // Save a maximum of 10 images
    if (m_imageCnt < mk_numImages)
    {
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
            // Convert image to mono 8
            ImagePtr convertedImage = image->Convert(PixelFormat_Mono8, HQ_LINEAR);
            // Increment image counter
            m_imageCnt++;
        }
    }
}
// Getter for image counter
int ImageEventHandler::getImageCount()
{
        return m_imageCnt;
}
// Getter for maximum images
int ImageEventHandler::getMaxImages()
{
        return mk_numImages;
}
