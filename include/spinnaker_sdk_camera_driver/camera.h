#ifndef CAMERA_HEADER
#define CAMERA_HEADER


#include "std_include.h"
#include "serialization.h"
#include "spinnaker_sdk_camera_driver/ImageEventHandler.h"
#include <boost/archive/binary_oarchive.hpp>
#include <boost/filesystem.hpp>
#include <string>
#include <tbb/concurrent_queue.h>

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace cv;
using namespace std;


namespace acquisition {

    class Camera {

    public:

        ~Camera();
        Camera(CameraPtr);

        void init();
        void deinit();
        void begin_acquisition();
        void end_acquisition();

        void setEnumValue(string, string);
        void setIntValue(string, int);
        void setFloatValue(string, float);
        void setBoolValue(string, bool);
        void setTLIntValue(string setting, int val);

        void setISPEnable();
        void setFREnable();
        void setPixelFormat(gcstring formatPic);
        void exposureTest();
        void setResolutionPixels(int width, int height);
        void setBufferSize(int numBuf);
        void adcBitDepth(gcstring bitDep);
        Mat convert_to_mat(ImagePtr);
        INodeMap & GetTLDeviceNodeMap();
        void RegisterEvent(bool Color_, bool Export2ROS_, bool Save_, \
            const shared_ptr<tbb::concurrent_queue<Mat>> & ROS_queue, \
            const shared_ptr<tbb::concurrent_queue<cvMatContainer*>> & Save_queue);
        void ResetEvent();

        void ConfigureChunkData();

        string get_id() { return string(pCam_->GetUniqueID()); }
        void set_color(bool flag) { COLOR_ = flag; }
        void set_cam_type(string type) { TYPE_ = type; }
        
    private:

        ImageEventHandler* imageEventHandler=NULL;
        
        CameraPtr pCam_;
        int64_t timestamp_;
        int frameID_;
        int lastFrameID_;

        bool COLOR_;
        bool MASTER_;
        string TYPE_ = "BFS"; // BFS for BlackFly S, and BF for BlackFly
        uint64_t GET_NEXT_IMAGE_TIMEOUT_;

    };

}

#endif
