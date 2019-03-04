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
        //cap >> frame;
        //imshow("frame",frame);
        //waitKey(1);
        std::vector<Point2d> coords = controller.lane_segment(frame);
        if (coords.size() >= 6)
        {
            std::vector<double> solution = controller.run_mpc(coords);
            controller.draw_mpc(solution);
        }
        else
        {
            controller.classic_lane_follow(frame);
        }
    }
    return 0;
}

