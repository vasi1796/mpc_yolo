#include "LaneController.h"

#include "utilities.hpp"
//
// Created by vasy1 on 2/27/2019.
//
LaneController::LaneController(int width, int height):
        m_width(width),
        m_height(height),
        m_slicedH(m_height - m_cropYOorigin),
        m_slicedW(m_width),
        m_perspectiveSrc{Point2f(m_slicedW / 2.f - 50, 20), Point2f(150, m_slicedH),
                         Point2f(m_slicedW - 20, m_slicedH), Point2f(m_slicedW / 2.f + 80, 20)},
        m_perspectiveDst{Point2f(150, 0), Point2f(150, m_slicedH), Point2f(m_slicedW, m_slicedH),
                         Point2f(m_slicedW, 0)}
                         {
    m_perspectiveMatrix = getPerspectiveTransform(m_perspectiveSrc, m_perspectiveDst);
    m_inv_perspectiveMatrix = getPerspectiveTransform(m_perspectiveDst, m_perspectiveSrc);
}

void LaneController::run_mpc(std::vector<Point2i> coords)
{
    std::vector<Eigen::Map<Eigen::VectorXd>> mpc_coords_rot = utilities::rotate_coords(coords);

    MPC mpc;
    //find coeffs of ref
    auto coeffs = utilities::polyfit(mpc_coords_rot[0], mpc_coords_rot[1], 3);

    //find cross track error
    double cte = utilities::polyeval(coeffs, 0); // px = 0, py = 0

    //find angle diff
    double epsi = -atan(coeffs[1]); // p

    Eigen::VectorXd state(6);

    //there is a problem with the speed input
    state<<0,0,0,0.1,cte,epsi;
    std::vector<double> vars = mpc.Solve(state, coeffs);
    std::cout<<"d "<<vars[0] <<" a "<<vars[1]<<std::endl;
}

std::vector<Point2i> LaneController::lane_segment(const Mat &frame)
{

    const int num_slices = 6;
    //height of slice
    const int slice_size = m_slicedH / num_slices;

    double lane_center_x;
    double lane_center_y;
    double max_area = -1;
    std::vector<Point2i> center_points;
    std::vector<Mat> slices;
    std::vector<std::vector<Point>> slice_contours;
    std::vector<std::vector<Point>> contours;

    Mat concat_slices;
    Mat cropped;
    Mat distorted_bw;
    Mat undistorted;
    Mat maskImage;
    //crop top of image of unrelevant information
    frame(Rect(0, m_cropYOorigin, m_slicedW, m_slicedH)).copyTo(cropped);
    Mat distorted = cropped.clone();

    //warp area of interest
    warpPerspective(cropped, distorted, m_perspectiveMatrix, Size(m_slicedW, m_slicedH));
    //create black mask image for contour overlay drawing
    maskImage.create(distorted.size().height, distorted.size().width, CV_8UC3);
    maskImage = Scalar(0, 0, 0);

    cvtColor(distorted, distorted_bw, COLOR_RGB2GRAY);
    threshold(distorted_bw, distorted_bw, m_laneThreshold, 255,
              THRESH_BINARY_INV); // sau THRESH_BINARY, si verifici care arie e mare sau offset centers
//  alternativ color masking (hsv, inRange)

    Mat erodeElmt = getStructuringElement(MORPH_RECT, Size(4, 4));    // reflectii etc
    Mat dilateElmt = getStructuringElement(MORPH_RECT, Size(5, 5));
    erode(distorted_bw, distorted_bw, erodeElmt);
    dilate(distorted_bw, distorted_bw, dilateElmt);
    //morphologyEx(distorted_bw, distorted_bw, MORPH_OPEN, erodeElmt);
    //imshow("thresh_no_refl",distorted_bw);
    //waitKey(0);

    //construct slices
    for (int i = 0; i < num_slices; i++)
    {
        int part = slice_size * i;
        // Setup a rectangle to define your region of interest
        cv::Rect myROI(0, part, m_slicedW, slice_size);
        Mat slice_mat = distorted_bw(myROI);
        findContours(slice_mat, slice_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
        drawContours(slice_mat, slice_contours, -1, Scalar(0, 255, 0));
        slices.push_back(slice_mat);
    }
    vconcat(slices, concat_slices);
    imshow("concat_slices", concat_slices);
    waitKey(0);

    //find slices contour and center of mass
    findContours(concat_slices, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    drawContours(maskImage, contours, -1, Scalar(0, 255, 0));

    for (const auto &contour : contours)
    {
        double area = contourArea(contour);

        if (area > m_minArea && area > max_area)
        {
            Moments mu;
            mu = moments(contour, false);
            lane_center_x = mu.m10 / mu.m00;
            lane_center_y = mu.m01 / mu.m00;
            Point2d center(lane_center_x, lane_center_y);
            //save points with y inverted because of origin system
            center_points.emplace_back(lane_center_x,m_slicedH-lane_center_y);
            circle(maskImage, center, 5, Scalar(0, 255, 0), -1);
        }
    }
    imshow("contour_points", maskImage);
    waitKey(0);
    //unwarp area of interest and add on cropped image
    warpPerspective(maskImage, undistorted, m_inv_perspectiveMatrix, Size(m_slicedW, m_slicedH));
    addWeighted(cropped, 1.0, undistorted, 1.0, 0, cropped);
    imshow("final", cropped);
    waitKey(0);
    return center_points;
}

