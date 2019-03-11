#include "spinnaker_sdk_camera_driver/capture.h"

acquisition::Capture::~Capture(){

    // destructor
    end_acquisition();
    deinit_cameras();

    // pCam = nullptr;
    
    ROS_INFO_STREAM("Clearing camList...");
    camList_.Clear();

    ROS_INFO_STREAM("Releasing camera pointers...");
    for (int i=0; i<cams.size(); i++)
        cams[i].~Camera();
    
    ROS_INFO_STREAM("Releasing system instance...");
    system_->ReleaseInstance();

    ROS_INFO_STREAM("Shut Down the ROS...");
    ros::shutdown();
}

acquisition::Capture::Capture():nh_(),nh_pvt_ ("~") {

    // struct sigaction sigIntHandler;

    // sigIntHandler.sa_handler = handler;
    // sigemptyset(&sigIntHandler.sa_mask);
    // sigIntHandler.sa_flags = 0;

    // sigaction(SIGINT, &sigIntHandler, NULL);

    // int mem;
    // ifstream usb_mem("/sys/module/usbcore/parameters/usbfs_memory_mb");
    // if (usb_mem) {
    //     usb_mem >> mem;
    //     if (mem >= 1000)
    //         ROS_INFO_STREAM("[ OK ] USB memory: "<<mem<<" MB");
    //     else{
    //         ROS_FATAL_STREAM("  USB memory on system too low ("<<mem<<" MB)! Must be at least 1000 MB. Run: \nsudo sh -c \"echo 1000 > /sys/module/usbcore/parameters/usbfs_memory_mb\"\n Terminating...");
    //         ros::shutdown();
    //     }
    // } else {
    //     ROS_FATAL_STREAM("Could not check USB memory on system! Terminating...");
    //     ros::shutdown();
    // }

    // default values for the parameters are set here. Should be removed eventually!!
    exposure_time_ = 0 ; // default as 0 = auto exposure
    gain_ = 0;
    balance_ = 0;
    gamma_ = 0;


    ext_ = ".tiff";
    EXPORT_TO_ROS_ = false;
    PUBLISH_CAM_INFO_ = false;
    SAVE_ = false;

    binning_ = 1;
    todays_date_ = todays_date();
    
    grab_time_ = 0;
    save_time_ = 0;
    toMat_time_ = 0;
    save_mat_time_ = 0;
    export_to_ROS_time_ = 0; 
    achieved_time_ = 0;
        
    // decimation_ = 1;
    CAM_ = 0;

    //read_settings(config_file);
    read_parameters();
    
    // Retrieve singleton reference to system object
    ROS_INFO_STREAM("Creating system instance...");
    system_ = System::GetInstance();

    load_cameras();
 
    //initializing the ros publisher
    //acquisition_pub = nh_.advertise<spinnaker_sdk_camera_driver::SpinnakerImageNames>("camera", 1000);
}

void acquisition::Capture::load_cameras() {

    // Retrieve list of cameras from the system
    ROS_INFO_STREAM("Retreiving list of cameras...");
    camList_ = system_->GetCameras();
    
    numCameras_ = camList_.GetSize();
    ROS_ASSERT_MSG(numCameras_,"No cameras found!");
    ROS_INFO_STREAM("Numer of cameras found: " << numCameras_);
    ROS_INFO_STREAM(" Cameras connected: " << numCameras_);

    for (int i=0; i<numCameras_; i++) {
        acquisition::Camera cam(camList_.GetByIndex(i));
        ROS_INFO_STREAM("  - "<<cam.get_id());
    }

    bool master_set = false;
    int cam_counter = 0;
    for (int j=0; j<cam_ids_.size(); j++) {
        bool current_cam_found=false;
        for (int i=0; i<numCameras_; i++) {
        
            acquisition::Camera cam(camList_.GetByIndex(i));
                
            if (cam.get_id().compare(cam_ids_[j]) == 0) {
                ROS_INFO_STREAM("  Find "<<cam.get_id());
                current_cam_found=true;
    
                cam.set_cam_type(cam_types_[j]);        
                cams.push_back(cam);
                
                camera_image_pubs.push_back(nh_.advertise<sensor_msgs::Image>("RGB_camera/"+cam_names_[j]+"/image_raw", 1));
                camera_info_pubs.push_back(nh_.advertise<sensor_msgs::CameraInfo>("RGB_camera/"+cam_names_[j]+"/camera_info", 1));
                img_msgs.push_back(sensor_msgs::ImagePtr());
                
                if (PUBLISH_CAM_INFO_){
                    sensor_msgs::CameraInfo ci_msg;
                    int image_width = 0;
                    int image_height = 0;
                    std::string distortion_model = ""; 
                    nh_pvt_.getParam("image_height", image_height);
                    nh_pvt_.getParam("image_width", image_width);
                    nh_pvt_.getParam("distortion_model", distortion_model);

                    ci_msg.header.frame_id = "cam_"+to_string(j)+"_optical_frame";
                    ci_msg.width = image_width;
                    ci_msg.height = image_height;
                    ci_msg.distortion_model = distortion_model;
                    ci_msg.binning_x = binning_;
                    ci_msg.binning_y = binning_;
                    
                    ci_msg.D = distortion_coeff_vec_[j];
                    for (int count = 0; count<intrinsic_coeff_vec_[j].size();count++)
                        ci_msg.K[count] = intrinsic_coeff_vec_[j][count];
                    
                    if (!rect_coeff_vec_.empty())
                        ci_msg.R = {rect_coeff_vec_[j][0], rect_coeff_vec_[j][1], rect_coeff_vec_[j][2],
                                    rect_coeff_vec_[j][3], rect_coeff_vec_[j][4], rect_coeff_vec_[j][5],
                                    rect_coeff_vec_[j][6], rect_coeff_vec_[j][7], rect_coeff_vec_[j][8]};
                    if (!proj_coeff_vec_.empty())
                        ci_msg.P = {proj_coeff_vec_[j][0], proj_coeff_vec_[j][1], proj_coeff_vec_[j][2], proj_coeff_vec_[j][3],
                                    proj_coeff_vec_[j][4], proj_coeff_vec_[j][5], proj_coeff_vec_[j][6], proj_coeff_vec_[j][7],
                                    proj_coeff_vec_[j][8], proj_coeff_vec_[j][9], proj_coeff_vec_[j][10], proj_coeff_vec_[j][11]};

                    cam_info_msgs.push_back(ci_msg);
                }
                cam_counter++;
            }
        }
        if (!current_cam_found) ROS_WARN_STREAM("   Camera "<<cam_ids_[j]<<" not detected!!!");
    }
    numCameras_ = cams.size();
    ROS_ASSERT_MSG(cams.size(),"None of the connected cameras are in the config list!");
}

void acquisition::Capture::read_parameters() {

    ROS_INFO_STREAM("*** PARAMETER SETTINGS ***");
    ROS_INFO_STREAM("** Date = "<<todays_date_);
    
    if (nh_pvt_.getParam("save_path", path_)){
    if(path_.front() =='~'){
        const char *homedir;
        if ((homedir = getenv("HOME")) == NULL)
            homedir = getpwuid(getuid())->pw_dir;
        std::string hd(homedir);
        path_.replace(0,1,hd);
    }
    ROS_INFO_STREAM("  Save path set via parameter to: " << path_);
    }
    else {
    boost::filesystem::path canonicalPath = boost::filesystem::canonical(".", boost::filesystem::current_path());
    path_ = canonicalPath.string();
       
    ROS_WARN_STREAM("  Save path not provided, data will be saved to: " << path_);
    }

    if (path_.back() != '/')
        path_ = path_ + '/';
        
    struct stat sb;
    ROS_ASSERT_MSG(stat(path_.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode),"Specified Path Doesn't Exist!!!");

    ROS_INFO("  Camera IDs:");
    
    std::vector<int> cam_id_vec;
    ROS_ASSERT_MSG(nh_pvt_.getParam("cam_ids", cam_id_vec),"Camera ID must be provided!");
    int num_ids = cam_id_vec.size();
    for (int i=0; i < num_ids; i++){
        cam_ids_.push_back(to_string(cam_id_vec[i]));
        ROS_INFO_STREAM("    " << to_string(cam_id_vec[i]));
    }

    std::vector<int> cam_type_vec;
    if (nh_pvt_.getParam("cam_types", cam_type_vec)){
        ROS_INFO_STREAM("  Camera types:");
        ROS_ASSERT_MSG(num_ids == cam_type_vec.size(),"If cam_aliases are provided, they should be the same number as cam_ids and should correspond in order!");
        for (int i=0; i<cam_type_vec.size(); i++) {
            if (cam_type_vec[i]){
                ROS_INFO_STREAM("    " << cam_ids_[i] << " >> BlackFly S");
                cam_types_.push_back("BFS");
            }
            else{
                ROS_INFO_STREAM("    " << cam_ids_[i] << " >> BlackFly"); 
                cam_types_.push_back("BF");
            } 
        }
    } else {
        ROS_INFO_STREAM("  No camera types provided. Assume all camera are BlackFly S.");
        for (int i=0; i<cam_ids_.size(); i++)
            cam_types_.push_back("BFS");
    }


    std::vector<string> cam_alias_vec;
    if (nh_pvt_.getParam("cam_aliases", cam_names_)){
        ROS_INFO_STREAM("  Camera Aliases:");
        ROS_ASSERT_MSG(num_ids == cam_names_.size(),"If cam_aliases are provided, they should be the same number as cam_ids and should correspond in order!");
        for (int i=0; i<cam_names_.size(); i++) {
            ROS_INFO_STREAM("    " << cam_ids_[i] << " >> " << cam_names_[i]);
        }
    } else {
        ROS_INFO_STREAM("  No camera aliases provided. Camera IDs will be used as names.");
        for (int i=0; i<cam_ids_.size(); i++)
            cam_names_.push_back(cam_ids_[i]);
    }

    if (nh_pvt_.getParam("color", color_)) 
        ROS_INFO("  color set to: %s",color_?"true":"false");
        else ROS_WARN("  'color' Parameter not set, using default behavior color=%s",color_?"true":"false");

    if (nh_pvt_.getParam("to_ros", EXPORT_TO_ROS_)) 
        ROS_INFO("  Exporting images to ROS: %s",EXPORT_TO_ROS_?"true":"false");
        else ROS_WARN("  'to_ros' Parameter not set, using default behavior to_ros=%s",EXPORT_TO_ROS_?"true":"false");

    if (nh_pvt_.getParam("exp", exposure_time_)){
        if (exposure_time_ >0) ROS_INFO("  Exposure set to: %.1f",exposure_time_);
        else ROS_INFO("  'exp'=%0.f, Setting autoexposure",exposure_time_);
    } else ROS_WARN("  'exp' Parameter not set, using default behavior: Automatic Exposure ");

    if (nh_pvt_.getParam("gain", gain_)){
        if (gain_ >0) ROS_INFO("  Gain set to: %.1f",gain_);
        else ROS_INFO("  'gain'=%0.f, Setting auto gain",gain_);
    } else ROS_WARN("  'gain' Parameter not set, using default behavior: Automatic Gain Control ");

    if (nh_pvt_.getParam("balance", balance_)){
        if (balance_ >0) ROS_INFO("  White Balance set to: %.1f",balance_);
        else ROS_INFO("  'balance'=%0.f, Setting auto white balance",balance_);
    } else ROS_WARN("  'balance' Parameter not set, using default behavior: Automatic White Balance ");

    if (nh_pvt_.getParam("gamma", gamma_)){
        if (gamma_ >0) ROS_INFO("  Gamma ratio set to: %.1f", gamma_);
        else ROS_INFO("  'gamma'=%0.f, disable gamma correction", gamma_);
    } else ROS_WARN("  'gamma' Parameter not set, using default behavior: Disable Gamma Correction ");

    if (nh_pvt_.getParam("binning", binning_)){
        if (binning_ >0) ROS_INFO("  Binning set to: %d",binning_);
        else {
            binning_=1;
            ROS_INFO("  'binning'=%d invalid, Using defauly binning=",binning_);
        }
    } else ROS_WARN("  'binning' Parameter not set, using default behavior: Binning = %d",binning_);

    if (nh_pvt_.getParam("save", SAVE_)) 
        ROS_INFO("  Saving images set to: %d",SAVE_);
        else ROS_WARN("  'save' Parameter not set, using default behavior save=%d",SAVE_);

    if (SAVE_){
        if (nh_pvt_.getParam("save_type", ext_)){
            ROS_INFO_STREAM("    save_type set as: "<<ext_);
            ext_="."+ext_;
        }else ROS_WARN("    'save_type' Parameter not set, using default behavior save=%d",SAVE_);
    }

    bool intrinsics_list_provided = false;
    XmlRpc::XmlRpcValue intrinsics_list;
    if (nh_pvt_.getParam("intrinsic_coeffs", intrinsics_list)) {
        ROS_INFO("  Camera Intrinsic Paramters:");
        ROS_ASSERT_MSG(intrinsics_list.size() == num_ids,"If intrinsic_coeffs are provided, they should be the same number as cam_ids and should correspond in order!");
        for (int i=0; i<intrinsics_list.size(); i++){
            std::vector<double> intrinsics;
            String intrinsics_str="";
            for (int j=0; j<intrinsics_list[i].size(); j++){
                ROS_ASSERT_MSG(intrinsics_list[i][j].getType()== XmlRpc::XmlRpcValue::TypeDouble,"Make sure all numbers are entered as doubles eg. 0.0 or 1.1");
                intrinsics.push_back(static_cast<double>(intrinsics_list[i][j]));
                intrinsics_str = intrinsics_str +to_string(intrinsics[j])+" ";
            }

            intrinsic_coeff_vec_.push_back(intrinsics);
            ROS_INFO_STREAM("   "<< intrinsics_str );
            intrinsics_list_provided=true;
        }
    }
    bool distort_list_provided = false;
    XmlRpc::XmlRpcValue distort_list;

    if (nh_pvt_.getParam("distortion_coeffs", distort_list)) {
        ROS_INFO("  Camera Distortion Paramters:");
        ROS_ASSERT_MSG(distort_list.size() == num_ids,"If intrinsic_coeffs are provided, they should be the same number as cam_ids and should correspond in order!");
        for (int i=0; i<distort_list.size(); i++){
            std::vector<double> distort;
            String distort_str="";
            for (int j=0; j<distort_list[i].size(); j++){
                ROS_ASSERT_MSG(distort_list[i][j].getType()== XmlRpc::XmlRpcValue::TypeDouble,"Make sure all numbers are entered as doubles eg. 0.0 or 1.1");
                distort.push_back(static_cast<double>(distort_list[i][j]));
                distort_str = distort_str +to_string(distort[j])+" ";
            }
            distortion_coeff_vec_.push_back(distort);
            ROS_INFO_STREAM("   "<< distort_str );
            distort_list_provided = true;
        }
    }
    
    XmlRpc::XmlRpcValue rect_list;

    if (nh_pvt_.getParam("rectification_coeffs", rect_list)) {
        ROS_INFO("  Camera Rectification Paramters:");
        ROS_ASSERT_MSG(rect_list.size() == num_ids,"If rectification_coeffs are provided, they should be the same number as cam_ids and should correspond in order!");
        for (int i=0; i<rect_list.size(); i++){
            std::vector<double> rect;
            String rect_str="";
            for (int j=0; j<rect_list[i].size(); j++){
                ROS_ASSERT_MSG(rect_list[i][j].getType()== XmlRpc::XmlRpcValue::TypeDouble,"Make sure all numbers are entered as doubles eg. 0.0 or 1.1");
                rect.push_back(static_cast<double>(rect_list[i][j]));
                rect_str = rect_str +to_string(rect[j])+" ";
            }
            rect_coeff_vec_.push_back(rect);
            ROS_INFO_STREAM("   "<< rect_str );
        }
    }
    
    XmlRpc::XmlRpcValue proj_list;

    if (nh_pvt_.getParam("projection_coeffs", proj_list)) {
        ROS_INFO("  Camera Projection Paramters:");
        ROS_ASSERT_MSG(proj_list.size() == num_ids,"If projection_coeffs are provided, they should be the same number as cam_ids and should correspond in order!");
        for (int i=0; i<proj_list.size(); i++){
            std::vector<double> proj;
            String proj_str="";
            for (int j=0; j<proj_list[i].size(); j++){
                ROS_ASSERT_MSG(proj_list[i][j].getType()== XmlRpc::XmlRpcValue::TypeDouble,"Make sure all numbers are entered as doubles eg. 0.0 or 1.1");
                proj.push_back(static_cast<double>(proj_list[i][j]));
                proj_str = proj_str +to_string(proj[j])+" ";
            }
            proj_coeff_vec_.push_back(proj);
            ROS_INFO_STREAM("   "<< proj_str );
        }
    }

    PUBLISH_CAM_INFO_ = intrinsics_list_provided && distort_list_provided;
    if (PUBLISH_CAM_INFO_)
        ROS_INFO("  Camera coeffs provided, camera info messges will be published.");
        else
            ROS_INFO("  Camera coeffs not provided correctly, camera info messges will not be published.");
}

void acquisition::Capture::init_cameras() {
    ROS_INFO_STREAM("Initializing cameras...");
    // Set cameras 1 to 4 to continuous
    for (int i = 0 ; i < numCameras_; i++) {                
        ROS_DEBUG_STREAM("Initializing camera " << cam_ids_[i] << "...");
        try {         
            cams[i].init();

            cams[i].setEnumValue("AcquisitionMode", "Continuous");

            // set color mode for camera
            cams[i].set_color(color_);
            if (color_)
                cams[i].setEnumValue("PixelFormat", "BGR8");
            else
                cams[i].setEnumValue("PixelFormat", "Mono8");
            ROS_INFO("Set Output Color"); 

            // output frame size
            cams[i].setIntValue("BinningHorizontal", binning_);
            cams[i].setIntValue("BinningVertical", binning_);
            ROS_INFO("Set Binning"); 

            // set exposure mode
            cams[i].setEnumValue("ExposureMode", "Timed");
            if (exposure_time_ > 0) { 
                cams[i].setEnumValue("ExposureAuto", "Off");
                cams[i].setFloatValue("ExposureTime", exposure_time_);
            } else {
                cams[i].setEnumValue("ExposureAuto", "Continuous");
            }
            ROS_INFO("Set Exposure");

             // set gain mode
            if (gain_ > 0) { 
                cams[i].setEnumValue("GainAuto", "Off");
                cams[i].setFloatValue("Gain", gain_);
            } else {
                cams[i].setEnumValue("GainAuto", "Continuous");
            }
            ROS_INFO("Set Gain");
            if(color_){
                // set WB mode
                if (balance_ > 0) { 
                    cams[i].setEnumValue("BalanceWhiteAuto", "Off");
                    cams[i].setEnumValue("BalanceRatioSelector", "Red");
                    cams[i].setFloatValue("BalanceRatio", balance_);
                } else {
                    cams[i].setEnumValue("BalanceWhiteAuto", "Continuous");
                }
                ROS_INFO("Set White Balance");
            }

            // set gamma correction
            if(gamma_>0)
            {
                cams[i].setBoolValue("GammaEnable", true);
                cams[i].setFloatValue("Gamma", gamma_);
            }else{
                cams[i].setBoolValue("GammaEnable", false);
            }
    
            // set to be triggerd by GPIO
            cams[i].setEnumValue("AcquisitionMode", "Continuous");
            cams[i].setEnumValue("TriggerMode", "On");
            cams[i].setEnumValue("TriggerSource", "Line0");
            ROS_INFO("Set Hardware Trigger");

            // enable PTP
            cams[i].setBoolValue("GevIEEE1588", true);
            cams[i].setEnumValue("GevIEEE1588Mode", "SlaveOnly");
            ROS_INFO("Set PTP enabled");

            ConfigureImageEvents(i);
            ROS_INFO("Image Event Configured");
        }
        catch (Spinnaker::Exception &e) {
            string error_msg = e.what();
            ROS_FATAL_STREAM("Error: " << error_msg);
            if (error_msg.find("Unable to set PixelFormat to BGR8") >= 0)
              ROS_WARN("Most likely cause for this error is if your camera can't support color and your are trying to set it to color mode");
            ros::shutdown();
        }

    }
    ROS_INFO_STREAM("All cameras initialized.");
}

void acquisition::Capture::start_acquisition() {
    Catch_Stop.setupSignalHandlers();
    for (int i = numCameras_-1; i>=0; i--)
        cams[i].begin_acquisition();    
}

void acquisition::Capture::end_acquisition() {

    for (int i = 0; i < numCameras_; i++)
        cams[i].end_acquisition();
    
}

void acquisition::Capture::deinit_cameras() {

    ROS_INFO_STREAM("Deinitializing cameras...");
    for (int i = numCameras_-1 ; i >=0 ; i--) {
        ROS_DEBUG_STREAM("Camera "<<i<<": Deinit...");
        cams[i].ResetEvent();
        cams[i].deinit();
    }
    ROS_INFO_STREAM("All cameras deinitialized."); 

}

void acquisition::Capture::create_cam_directories() {

    ROS_INFO_STREAM("Creating camera directories...");
    ostringstream base_folder;
    base_folder<<path_<<"RGB_cam_"<<todays_date_<<"/";
     ROS_INFO_STREAM("Create folder"<<base_folder.str());
    if (mkdir(base_folder.str().c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) < 0) {
            ROS_WARN_STREAM("Failed to create directory "<<base_folder.str()<<"! Data will be written into pre existing directory if it exists...");
    }
    for (int i=0; i<numCameras_; i++) {
        ostringstream ss;
        ss<<base_folder.str()<<cam_names_[i];
        ROS_INFO_STREAM("Create folder"<<ss.str());
        if (mkdir(ss.str().c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) < 0) {
            ROS_WARN_STREAM("Failed to create directory "<<ss.str()<<"! Data will be written into pre existing directory if it exists...");
        }
        shared_ptr<ofstream> logfile(new std::ofstream);
        ostringstream LogName;
        LogName << ss.str() << "/" << "log_"<< todays_date_ <<".txt";
        logfile->open(LogName.str());
        *logfile<<cam_names_[i]<<"_timestamp\t"<<cam_names_[i]<<"_framecount\n";
        logfile_vec_.push_back(logfile);
    }
    CAM_DIRS_CREATED_ = true;
}

void acquisition::Capture::saving_thread()
{
    if (!CAM_DIRS_CREATED_)
        create_cam_directories();
    while(!Catch_Stop.gotExitSignal())
        save_frames();

    for(auto & logfile : logfile_vec_)
        logfile->close();
}

void acquisition::Capture::ROS_pub_thread()
{
    ROS_INFO_STREAM("***** Publish ROS Node ************");
    for(int i=0; i<numCameras_; i++)
    {
        ROS_queue_vec_[i]->clear();
    }
    while(!Catch_Stop.gotExitSignal())
        export_to_ROS();
}

void acquisition::Capture::save_frames()
{
    string timestamp;
    double t = ros::Time::now().toSec();
    try{
        for (unsigned int i = 0; i < numCameras_; i++) {
            // TRY to pop from the top of the queue
            if(Save_queue_vec_[i]->try_pop(frames_)){
                
                timestamp = to_string(frames_->getTimeCam());
                ostringstream filename;
                filename<< path_ << cam_names_[i] << "/" << timestamp << ext_;
                ROS_DEBUG_STREAM("Saving image at " << filename.str());
                frames_->saveImg(filename.str());
                *(logfile_vec_[i])<< frames_->getTimeCam()<<"\t"<< frames_->getFrameNum()<<"\n";
                delete frames_;
                frames_ = NULL;
            }
        }
        save_mat_time_ = ros::Time::now().toSec() - t;
    }
    catch (Spinnaker::Exception &e)
    {
        ROS_FATAL_STREAM("Error: " << e.what());
        ros::shutdown();
    }
}


void acquisition::Capture::export_to_ROS() {
    double t = ros::Time::now().toSec();
    std_msgs::Header img_msg_header;
    img_msg_header.stamp = ros::Time::now();
    Mat top_frame;
    try{
        for(int i=0; i<numCameras_; i++){
            if(ROS_queue_vec_[i]->try_pop(top_frame)){
                img_msg_header.frame_id = "cam_"+to_string(i)+"_optical_frame";
                if(color_)
                    img_msgs[i]=cv_bridge::CvImage(img_msg_header, "bgr8", top_frame ).toImageMsg();
                else
                    img_msgs[i]=cv_bridge::CvImage(img_msg_header, "mono8", top_frame).toImageMsg();
                camera_image_pubs[i].publish(img_msgs[i]);

                if (PUBLISH_CAM_INFO_){
                    cam_info_msgs[i].header.stamp = img_msg_header.stamp;
                    camera_info_pubs[i].publish(cam_info_msgs[i]);
                }
                export_to_ROS_time_ = ros::Time::now().toSec()-t;
                ROS_DEBUG_STREAM("Export to ros take "<<export_to_ROS_time_<<" sec.");
            }
        }
    }
    catch (Spinnaker::Exception &e)
    {
        ROS_FATAL_STREAM("Error: " << e.what());
        ros::shutdown();
    }
}


float acquisition::Capture::mem_usage() {
    std::string token;
    std::ifstream file("/proc/meminfo");
    unsigned long int total, free;
    while (file >> token) {
        if (token == "MemTotal:")
            if (!(file >> total))
                ROS_FATAL_STREAM("Could not poll total memory!");
        if (token == "MemAvailable:")
            if (!(file >> free)) {
                ROS_FATAL_STREAM("Could not poll free memory!");
                break;
            }
        // ignore rest of the line
        file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
    return 1-float(free)/float(total);
}


void acquisition::Capture::run() {
    ROS_INFO("*** ACQUISITION ***"); 
    start_acquisition();
    vector<thread> thread_vec_;
   if(EXPORT_TO_ROS_)
        thread_vec_.push_back(thread(& acquisition::Capture::ROS_pub_thread, this)); 
   if(SAVE_)
        thread_vec_.push_back(thread(& acquisition::Capture::saving_thread, this));

    for( auto& itr : thread_vec_)
        itr.join();
}

std::string acquisition::Capture::todays_date()
{
    char out[80];
    std::time_t t=std::time(NULL);
    std::strftime(out, sizeof(out), "%Y_%m_%d_%H_%M_%S", std::localtime(&t));
    std::string td(out);
    return td;
}

void acquisition::Capture::ConfigureImageEvents(int idx)
{
    // Create image event
    shared_ptr<tbb::concurrent_queue<cvMatContainer*>> save_queue_ptr(new tbb::concurrent_queue<cvMatContainer*>);
    shared_ptr<tbb::concurrent_queue<Mat>> ros_queue_ptr(new tbb::concurrent_queue<Mat>);
    cams[idx].RegisterEvent( color_, EXPORT_TO_ROS_ , \
                            SAVE_, ros_queue_ptr, save_queue_ptr);
    Save_queue_vec_.push_back(save_queue_ptr);
    ROS_queue_vec_.push_back(ros_queue_ptr);
}


