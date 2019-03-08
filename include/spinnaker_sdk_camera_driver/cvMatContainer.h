#ifndef MAT_CONTAIN
#define MAT_CONTAIN

#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <time.h>

using namespace std;

class cvMatContainer
{
    public:
        //constructor
        cvMatContainer(cv::Mat & input, int64_t time_stamp_camera, int frame, bool copy);

        cvMatContainer(cv::Mat & input, bool copy);

        void assignTimeCam(int time_stamp);
        int64_t getTimeCam() const; 

        int getFrameNum() const;

        cv::Mat getImg() const;

        bool saveImg(string path) const;

    private:

        cv::Mat img;
        int frame_camera;
        int64_t time_stamp_camera;
};


#endif
 