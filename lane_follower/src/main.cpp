#include <iostream>
#include "LaneController.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main()
{
    cv::Mat frame;
    VideoCapture cap("../res/car_rgb_xtion.avi");
    //frame = cv::imread("../res/test2.jpg");
    cap.read(frame);
    LaneController controller(frame.cols, frame.rows);
    while (cap.read(frame))
    {
        controller.run(frame);
        //cap >> frame;
        //imshow("frame",frame);
        //waitKey(1);
    }
    cv::destroyAllWindows();
    return 0;
}

