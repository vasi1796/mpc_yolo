#include <iostream>
#include "LaneController.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main()
{
    cv::Mat frame;
    frame = cv::imread("../res/test2.jpg");
    LaneController controller(frame.cols,frame.rows);

    std::vector<Point2i> coords = controller.lane_segment(frame);
    controller.run_mpc(coords);
    return 0;
}

