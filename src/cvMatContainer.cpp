#include <spinnaker_sdk_camera_driver/cvMatContainer.h>

cvMatContainer::cvMatContainer(cv::Mat & input, int64_t time_stamp_camera, int frame, bool copy=false)
{
    if (copy)
        this->img = input.clone();
    this->time_stamp_camera = time_stamp_camera;
    this->frame_camera = frame;
}

cvMatContainer::cvMatContainer(cv::Mat & input, bool copy=false)
{
    if (copy)
        this->img = input.clone();
    this->time_stamp_camera = 0;
    this->frame_camera = 0;
}

void cvMatContainer::assignTimeCam(int time_stamp)
{
    this->time_stamp_camera = time_stamp;
}

int64_t cvMatContainer::getTimeCam() const
{
    return this->time_stamp_camera;
}


cv::Mat cvMatContainer::getImg() const
{
    return this->img;
}

int cvMatContainer::getFrameNum() const
{
    return this->frame_camera;
}

bool cvMatContainer::saveImg(string path) const
{
    return imwrite(path, this->img);
}
