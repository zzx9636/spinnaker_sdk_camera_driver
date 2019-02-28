#ifndef IMG_EVENT_HEADER
#define IMG_EVENT_HEADER

#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <iostream>
#include <sstream>
#include <ros/console.h>
#include <string.h>
#include <tbb/concurrent_queue.h>
#include <memory>
#include "camera.h"
#include <mutex>


using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;
using namespace cv;
//using namespace acquisition;

class ImageEventHandler : public ImageEvent
{
        public:
                
                // The constructor retrieves the serial number and initializes the image 
                // counter to 0.
                ImageEventHandler(CameraPtr pCam, bool if_color, bool to_ros , bool save,
                        const shared_ptr<tbb::concurrent_queue<Mat>>& ros_queue_ptr, 
                        const shared_ptr<tbb::concurrent_queue<Mat>>& save_queue_ptr);

                ImageEventHandler(acquisition::Camera& cam, bool if_color, bool to_ros , bool save, const shared_ptr<tbb::concurrent_queue<Mat>>& ros_queue_ptr, const shared_ptr<tbb::concurrent_queue<Mat>>& save_queue_ptr);

                ~ImageEventHandler();
                // This method defines an image event. In it, the image that triggered the 
                // event is converted and saved before incrementing the count. Please see 
                // Acquisition_CSharp example for more in-depth comments on the acquisition 
                // of images.
                void OnImageEvent(ImagePtr image);
        
                // Getter for image counter
                int getImageCount();
        
        
        private:
                void save2queue(ImagePtr convertedImage);
                static const unsigned int mk_numImages = 10000000;
                unsigned int m_imageCnt;
                string m_deviceSerialNumber;
                shared_ptr<tbb::concurrent_queue<Mat>> SAVE_queue;
                shared_ptr<tbb::concurrent_queue<Mat>> ROS_queue;
                
                bool COLOR_;
                bool SAVE_;
                bool TO_ROS_;
                
                
};

#endif