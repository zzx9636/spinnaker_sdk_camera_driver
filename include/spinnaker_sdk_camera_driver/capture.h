#ifndef CAPTURE_HEADER
#define CAPTURE_HEADER

#include "std_include.h"
#include "serialization.h"
#include "camera.h"

#include <boost/archive/binary_oarchive.hpp>
#include <boost/filesystem.hpp>

#include "std_msgs/String.h"
#include "spinnaker_sdk_camera_driver/SpinnakerImageNames.h"
#include "spinnaker_sdk_camera_driver/ImageEventHandler.h"
#include <sstream>
#include <tbb/concurrent_queue.h>
#include <memory>
#include <thread>


using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace cv;
using namespace std;

namespace acquisition {
    
    class Capture {

    public:
    
        ~Capture();
        Capture();

        void load_cameras();
        
        void init_array();
        void init_cameras(bool);
        void start_acquisition();
        void end_acquisition();
        void deinit_cameras();
        void run();
        void read_parameters();
        std::string todays_date();
    
    private:

        void set_frame_rate(CameraPtr, float);
        void create_cam_directories();
        void saving_thread();
        void save_frames();

        void ROS_pub_thread();
        void export_to_ROS();
        float mem_usage();

        void ConfigureImageEvents(CameraPtr pCam);
        //void handler_wait4image(ImageEventHandler*& imageEventHandler);
        void ResetImageEvents(CameraPtr pCam, ImageEventHandler*& imageEventHandler);
    
        SystemPtr system_;    
        CameraList camList_;
        vector<acquisition::Camera> cams;
        vector<string> cam_ids_;
        vector<string> cam_names_;
        vector<string> cam_types_;
        string master_cam_id_;
        unsigned int numCameras_;
        vector<CameraPtr> pCams_;
        vector<ImagePtr> pResultImages_;
        vector<Mat> frames_;
        vector<string> time_stamps_;
        vector< vector<Mat> > mem_frames_;
        vector<vector<double>> intrinsic_coeff_vec_;
        vector<vector<double>> distortion_coeff_vec_;
        vector<vector<double>> rect_coeff_vec_;
        vector<vector<double>> proj_coeff_vec_;
        vector<string> imageNames;
        vector<ImageEventHandler*> handler_ptr_vec_;
        vector<shared_ptr<tbb::concurrent_queue<Mat>>> Save_queue_vec_;
        vector<shared_ptr<tbb::concurrent_queue<Mat>>> ROS_queue_vec_;   
        string path_;
        string todays_date_;

        time_t time_now_;
        double grab_time_, save_time_, toMat_time_, save_mat_time_, export_to_ROS_time_, achieved_time_;

        int nframes_;
        float init_delay_;
        int skip_num_;
        float master_fps_;
        int binning_;
        bool color_;
        string dump_img_;
        string ext_;
        float exposure_time_;
        // int decimation_;

        int soft_framerate_; // Software (ROS) frame rate
        
        int MASTER_CAM_;
        int CAM_; // active cam during live

        bool FIXED_NUM_FRAMES_;
        bool TIME_BENCHMARK_;
        bool MASTER_TIMESTAMP_FOR_ALL_;
        bool SAVE_;
        bool CAM_DIRS_CREATED_;
        bool GRID_VIEW_;
        bool EXPORT_TO_ROS_;
        bool PUBLISH_CAM_INFO_;

        // grid view related variables
        bool GRID_CREATED_;
        Mat grid_;

        // ros variables
        ros::NodeHandle nh_;
        ros::NodeHandle nh_pvt_;

        ros::Publisher acquisition_pub;
        vector<ros::Publisher> camera_image_pubs;
        vector<ros::Publisher> camera_info_pubs;
		
        vector<sensor_msgs::ImagePtr> img_msgs;
        vector<sensor_msgs::CameraInfo> cam_info_msgs;
        spinnaker_sdk_camera_driver::SpinnakerImageNames mesg;
        boost::mutex queue_mutex_;  
    };

}

#endif
