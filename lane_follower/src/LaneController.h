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
            LaneController();
            std::vector<Point2i> lane_segment(const Mat &frame);
    void run_mpc(std::vector<Point2i> coords);

        private:
    const int m_width = 640;
    const int m_height = 480;
    const int m_cropYOorigin = 170;

    const int m_laneThreshold = 75;
    const int m_minArea = 3000;

    const int m_slicedH = m_height - m_cropYOorigin;;
    const int m_slicedW = m_width;

    Point2f m_perspectiveSrc[4];
    Point2f m_perspectiveDst[4];
    //Get the Perspective Matrix.
    Mat m_perspectiveMatrix;
    Mat m_inv_perspectiveMatrix;
        };

#endif //LANE_FOLLOWER_LANECONTROLLER_H
