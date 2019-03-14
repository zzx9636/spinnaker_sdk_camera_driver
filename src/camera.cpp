#include "spinnaker_sdk_camera_driver/camera.h"

acquisition::Camera::~Camera() {

    pCam_ = NULL;
    timestamp_ = 0;

}

acquisition::Camera::Camera(CameraPtr pCam) {

    pCam_ = pCam;

    if (pCam_->IsInitialized()) {
        ROS_WARN_STREAM("Camera already initialized. Deinitializing...");
        pCam_->EndAcquisition();
        pCam_->DeInit();
    }

    lastFrameID_ = -1;
    frameID_ = -1;
    MASTER_ = false;
    timestamp_ = 0;
    GET_NEXT_IMAGE_TIMEOUT_ = 2000;
    
}

void acquisition::Camera::init() {

    pCam_->Init();
    ConfigureChunkData();
    
}

void acquisition::Camera::deinit() {

    pCam_->DeInit();

}

Mat acquisition::Camera::convert_to_mat(ImagePtr pImage) {

    ImagePtr convertedImage;
    if (COLOR_)
        convertedImage = pImage->Convert(PixelFormat_BGR8); //, NEAREST_NEIGHBOR);
    else
		convertedImage = pImage->Convert(PixelFormat_Mono8); //, NEAREST_NEIGHBOR);
		
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
    
    return img.clone();
    // return img;
    
}

void acquisition::Camera::begin_acquisition() {

    ROS_DEBUG_STREAM("Begin Acquisition...");
    pCam_->BeginAcquisition();
    
}

void acquisition::Camera::end_acquisition() {

    if (pCam_->GetNumImagesInUse())
        ROS_WARN_STREAM("Some images still currently in use! Use image->Release() before deinitializing.");
        
    ROS_DEBUG_STREAM("End Acquisition...");
    pCam_->EndAcquisition();    
    
}

void acquisition::Camera::setEnumValue(string setting, string value) {

    INodeMap & nodeMap = pCam_->GetNodeMap();
    
    // Retrieve enumeration node from nodemap
    CEnumerationPtr ptr = nodeMap.GetNode(setting.c_str());
    if (!IsAvailable(ptr) || !IsWritable(ptr))
        ROS_FATAL_STREAM("Unable to set " << setting << " to " << value << " (enum retrieval). Aborting...");

    // Retrieve entry node from enumeration node
    CEnumEntryPtr ptrValue = ptr->GetEntryByName(value.c_str());
    if (!IsAvailable(ptrValue) || !IsReadable(ptrValue))
        ROS_FATAL_STREAM("Unable to set " << setting << " to " << value << " (entry retrieval). Aborting...");
		
    // retrieve value from entry node
    int64_t valueToSet = ptrValue->GetValue();
		
    // Set value from entry node as new value of enumeration node
    ptr->SetIntValue(valueToSet);    

    ROS_DEBUG_STREAM(setting << " set to " << value);
    
}

void acquisition::Camera::setIntValue(string setting, int val) {

    INodeMap & nodeMap = pCam_->GetNodeMap();
    
    CIntegerPtr ptr = nodeMap.GetNode(setting.c_str());
    if (!IsAvailable(ptr) || !IsWritable(ptr)) {
        ROS_FATAL_STREAM("Unable to set " << setting << " to " << val << " (ptr retrieval). Aborting...");
    }
    ptr->SetValue(val);

    ROS_DEBUG_STREAM(setting << " set to " << val);
    
}

void acquisition::Camera::setFloatValue(string setting, float val) {

    INodeMap & nodeMap = pCam_->GetNodeMap();
    
    CFloatPtr ptr = nodeMap.GetNode(setting.c_str());
    if (!IsAvailable(ptr) || !IsWritable(ptr)) {
        ROS_FATAL_STREAM("Unable to set " << setting << " to " << val << " (ptr retrieval). Aborting...");
    }
    ptr->SetValue(val);

    ROS_DEBUG_STREAM(setting << " set to " << val);
    
}

void acquisition::Camera::setBoolValue(string setting, bool val) {

    INodeMap & nodeMap = pCam_->GetNodeMap();
    
    CBooleanPtr ptr = nodeMap.GetNode(setting.c_str());
    if (!IsAvailable(ptr) || !IsWritable(ptr)) {
        ROS_FATAL_STREAM("Unable to set " << setting << " to " << val << " (ptr retrieval). Aborting...");
    }
    if (val) ptr->SetValue("True");
		else ptr->SetValue("False");

    ROS_DEBUG_STREAM(setting << " set to " << val);
    
}

void acquisition::Camera::setTLIntValue(string setting, int val) {

    INodeMap & nodeMap = pCam_->GetTLDeviceNodeMap();;
    
    CBooleanPtr ptr = nodeMap.GetNode(setting.c_str());
    if (!IsAvailable(ptr) || !IsWritable(ptr)) {
        ROS_FATAL_STREAM("Unable to set " << setting << " to " << val << " (ptr retrieval). Aborting...");
    }
    ptr->SetValue(val);
    ROS_DEBUG_STREAM(setting << " set to " << val);
}


void acquisition::Camera::setResolutionPixels(int width, int height) {
    CIntegerPtr ptrHeight=pCam_->GetNodeMap().GetNode("Height");
    CIntegerPtr ptrWidth=pCam_->GetNodeMap().GetNode("Width");
    if (!IsAvailable(ptrWidth) || !IsWritable(ptrWidth)){
        ROS_FATAL_STREAM("Unable to set width" << "). Aborting...");
        return ; 
    }
    int64_t widthMax = ptrWidth->GetMax();
    if(widthMax<width)
        width=widthMax;
    ptrWidth->SetValue(width);
    ROS_DEBUG_STREAM("Set Width"<<width);

    if (!IsAvailable(ptrHeight) || !IsWritable(ptrHeight)){
        ROS_FATAL_STREAM("Unable to set height" << "). Aborting...");
        return ; 
    }
    int64_t heightMax = ptrHeight->GetMax();
    if(heightMax<height)
        height=heightMax;

    ROS_DEBUG_STREAM("Set Height"<<height);
    ptrHeight->SetValue(height);                                                                                                                                 
}

void acquisition::Camera::adcBitDepth(gcstring bitDep) {
    CEnumerationPtr ptrADC = pCam_->GetNodeMap().GetNode("AdcBitDepth");
    if (!IsAvailable(ptrADC) || !IsWritable(ptrADC)){
        ROS_FATAL_STREAM("Unable to set ADC Bit " <<  "). Aborting...");
        return ;
    }

    CEnumEntryPtr ptrADCA = ptrADC->GetEntryByName(bitDep);
    int currDepth=ptrADCA->GetValue();
    ptrADC->SetIntValue(currDepth);
    ROS_DEBUG_STREAM("BPS: "<<ptrADC->GetIntValue());

}

void acquisition::Camera::setBufferSize(int numBuf) {

    INodeMap & sNodeMap = pCam_->GetTLStreamNodeMap();
    CIntegerPtr StreamNode = sNodeMap.GetNode("StreamDefaultBufferCount");
    int64_t bufferCount = StreamNode->GetValue();
    if (!IsAvailable(StreamNode) || !IsWritable(StreamNode)){
        ROS_FATAL_STREAM("Unable to set StreamMode " << "). Aborting...");
        return;
    }
    StreamNode->SetValue(numBuf);
    ROS_DEBUG_STREAM("Set Buf "<<numBuf<<endl);
}

void acquisition::Camera::setISPEnable() {
    CBooleanPtr ptrISPEn=pCam_->GetNodeMap().GetNode("IspEnable");
    if (!IsAvailable(ptrISPEn) || !IsWritable(ptrISPEn)){
        ROS_FATAL_STREAM("Unable to set ISP Enable (node retrieval; camera " << "). Aborting...");
        return;
    }
    ptrISPEn->SetValue("True");
}

void acquisition::Camera::setFREnable() {
    CBooleanPtr ptrAcquisitionFrameRateEnable=pCam_->GetNodeMap().GetNode("AcquisitionFrameRateEnable");
    if (!IsAvailable(ptrAcquisitionFrameRateEnable) || !IsWritable(ptrAcquisitionFrameRateEnable)){
        ROS_FATAL_STREAM("Unable to set frameRateEnable (node retrieval; camera " << "). Aborting...");
        return;
    }

    ptrAcquisitionFrameRateEnable->SetValue("True");
}

void acquisition::Camera::setPixelFormat(gcstring formatPic) {
    CEnumerationPtr ptrPixelFormat = pCam_->GetNodeMap().GetNode(formatPic);
    if ( !IsWritable(ptrPixelFormat)){
        ROS_FATAL_STREAM("Unable to set Pixel Format " <<  "). Aborting...");
        return ;
    }
    CEnumEntryPtr ptrPixelEnt = ptrPixelFormat->GetEntryByName("RGB8Packed");
    if (!IsAvailable(ptrPixelEnt) || !IsReadable(ptrPixelEnt)){
        ROS_FATAL_STREAM("Unable to set RGBPoint"  << "). Aborting...");
        return ;
    }                                                                                                                                        
    int64_t colorNum = ptrPixelEnt->GetValue();
                                                                                                                                                
    ptrPixelFormat->SetIntValue(colorNum);
    ROS_DEBUG_STREAM( "Camera " << " set pixel format");
}

void acquisition::Camera::exposureTest() {
    CFloatPtr ptrExpTest=pCam_->GetNodeMap().GetNode("ExposureTime");
    if (!IsAvailable(ptrExpTest) || !IsReadable(ptrExpTest)){
        ROS_FATAL_STREAM("Unable to set exposure " << "). Aborting..." << endl << endl);
        return ;
    }
    float expTime=ptrExpTest->GetValue();
    ROS_DEBUG_STREAM("Exposure Time: "<<expTime<<endl);

}

INodeMap & acquisition::Camera::GetTLDeviceNodeMap()
{
    return pCam_->GetTLDeviceNodeMap();
}

void acquisition::Camera::RegisterEvent(bool Color_, bool Export2ROS_, bool Save_, \
            const shared_ptr<tbb::concurrent_queue<Mat>> & ROS_queue, \
            const shared_ptr<tbb::concurrent_queue<cvMatContainer*>> & Save_queue)
{
    if(!pCam_->IsInitialized())
        ROS_WARN("Camera not initialized");
    try{
        imageEventHandler = new ImageEventHandler(pCam_, Color_, Export2ROS_ , \
                        Save_, ROS_queue, Save_queue);  
        pCam_->RegisterEvent(*imageEventHandler);
    }catch(Spinnaker::Exception &e){
        ROS_FATAL_STREAM("Error: " << e.what());
        ros::shutdown();
    }   
}

void acquisition::Camera::ResetEvent()
{
    try{
        //
        // Unregister image event handler
        //
        // *** NOTES ***
        // It is important to unregister all image events from all cameras
        // they are registered to.
        //
        pCam_->UnregisterEvent(*imageEventHandler);
        // Delete image event (because it is a pointer)
        delete imageEventHandler;
        imageEventHandler = NULL;
        ROS_INFO_STREAM("Image events unregistered...");
    }catch (Spinnaker::Exception &e)
    {
        ROS_FATAL_STREAM("Error: " << e.what());
        ros::shutdown();
    }
}

void acquisition::Camera::ConfigureChunkData()
{
    INodeMap & nodeMap = pCam_->GetNodeMap();
    ROS_INFO("*** CONFIGURING CHUNK DATA ***" );
    try
    {
            // Activate chunk mode
            CBooleanPtr ptrChunkModeActive = nodeMap.GetNode("ChunkModeActive");
            if (!IsAvailable(ptrChunkModeActive) || !IsWritable(ptrChunkModeActive))
            {
                    ROS_WARN("Unable to activate chunk mode. Aborting...");
                    return;
            }
            ptrChunkModeActive->SetValue(true);
            ROS_INFO("Chunk mode activated");
            // Enable all types of chunk data
           
            NodeList_t entries;
            // Retrieve the selector node
            CEnumerationPtr ptrChunkSelector = nodeMap.GetNode("ChunkSelector");
            if (!IsAvailable(ptrChunkSelector) || !IsReadable(ptrChunkSelector))
            {
                    ROS_WARN("Unable to retrieve chunk selector. Aborting...");
                    return;
            }
            // Retrieve entries
            ptrChunkSelector->GetEntries(entries);
            ROS_INFO("Enabling entries...");
            for (int i = 0; i < entries.size(); i++)
            {
                    // Select entry to be enabled
                    CEnumEntryPtr ptrChunkSelectorEntry = entries.at(i);
                    // Go to next node if problem occurs
                    if (!IsAvailable(ptrChunkSelectorEntry) || !IsReadable(ptrChunkSelectorEntry))
                    {
                            continue;
                    }
                    ptrChunkSelector->SetIntValue(ptrChunkSelectorEntry->GetValue());
                    cout << "\t" << ptrChunkSelectorEntry->GetSymbolic() << ": ";
                    // Retrieve corresponding boolean
                    CBooleanPtr ptrChunkEnable = nodeMap.GetNode("ChunkEnable");
                    // Enable the boolean, thus enabling the corresponding chunk data
                    if (!IsAvailable(ptrChunkEnable))
                    {
                            cout << "Node not available" << endl;
                    }
                    else if (ptrChunkEnable->GetValue())
                    {
                            cout << "Enabled" << endl;
                    }
                    else if (IsWritable(ptrChunkEnable))
                    {
                            ptrChunkEnable->SetValue(true);
                            cout << "Enabled" << endl;
                    }
                    else
                    {
                            cout << "Node not writable" << endl;
                    }
            }
    }
    catch (Spinnaker::Exception &e)
    {
            ROS_WARN_STREAM("Error: " << e.what() );
            return;
    }
    return;
}