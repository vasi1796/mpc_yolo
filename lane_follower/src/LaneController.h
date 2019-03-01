//
// Created by vasy1 on 2/27/2019.
//

#ifndef LANE_FOLLOWER_LANECONTROLLER_H
#define LANE_FOLLOWER_LANECONTROLLER_H

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "MPC.h"

using namespace cv;

class LaneController
{
public:
    LaneController(int width, int height);

    std::vector<Point2i> lane_segment(const Mat &frame);

    std::vector<double> run_mpc(std::vector<Point2i> coords);

    void draw_mpc(std::vector<double> mpc_result);

    void classic_lane_follow(Mat &frame);

private:
    double track_lane(Mat &frame);

    double get_steering(double off_center);

    Mat m_frameMpc;
    const int m_cropYOorigin = 170;
    const int m_laneThreshold = 75;
    const int m_minArea = 1500;
    const Scalar m_green_range_start = Scalar(30, 0, 100);
    const Scalar m_green_range_end = Scalar(85, 255, 255);
    int m_width;
    int m_height;
    int m_slicedH;
    int m_slicedW;
    Point2f m_perspectiveSrc[4];
    Point2f m_perspectiveDst[4];

    //Get the Perspective Matrix.
    Mat m_perspectiveMatrix;
    Mat m_inv_perspectiveMatrix;
};

#endif //LANE_FOLLOWER_LANECONTROLLER_H
