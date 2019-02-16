#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "utilities.hpp"
#include "MPC.h"

using namespace cv;

std::vector<Point2i> lane_segment(int lane_threshold, int min_area, int sliced_h, int sliced_w, int crop_y_origin,
                                  const Mat &inv_perspectiveMatrix, const Mat &perspectiveMatrix, const Mat &frame);

void run_mpc(const std::vector<Eigen::Map<Eigen::VectorXd>> &mpc_coords_rot);

int main()
{
    Mat frame;
    frame = cv::imread("../res/test2.jpg");
    const int width = frame.cols;
    const int height = frame.rows;
    const int crop_y_origin = 170;

    const int lane_threshold = 75;
    const int min_area = 3000;

    const int sliced_h = height - crop_y_origin;
    const int sliced_w = width;

    Point2f perspectiveSrc[] = {Point2f(sliced_w / 2.f - 50, 20), Point2f(150, sliced_h),
                                Point2f(sliced_w - 20, sliced_h), Point2f(sliced_w / 2.f + 80, 20)};
    Point2f perspectiveDst[] = {Point2f(150, 0), Point2f(150, sliced_h), Point2f(sliced_w, sliced_h),
                                Point2f(sliced_w, 0)};

    //Get the Perspective Matrix.
    Mat perspectiveMatrix = getPerspectiveTransform(perspectiveSrc, perspectiveDst);
    Mat inv_perspectiveMatrix = getPerspectiveTransform(perspectiveDst, perspectiveSrc);

    std::vector<Point2i> coords = lane_segment(lane_threshold, min_area, sliced_h, sliced_w, crop_y_origin,
                                                   inv_perspectiveMatrix, perspectiveMatrix, frame);

    std::vector<Eigen::Map<Eigen::VectorXd>> mpc_coords_rot = rotate_coords(coords);
    run_mpc(mpc_coords_rot);

    return 0;
}

void run_mpc(const std::vector<Eigen::Map<Eigen::VectorXd>> &mpc_coords_rot)
{
    MPC mpc;
    //find coeffs of ref
    auto coeffs = polyfit(mpc_coords_rot[0], mpc_coords_rot[1], 3);

    //find cross track error
    double cte = polyeval(coeffs, 0); // px = 0, py = 0

    //find angle diff
    double epsi = -atan(coeffs[1]); // p

    Eigen::VectorXd state(6);

    //there is a problem with the speed input
    state<<0,0,0,0.1,cte,epsi;
    std::vector<double> vars = mpc.Solve(state, coeffs);
    std::cout<<"d "<<vars[0] <<" a "<<vars[1]<<std::endl;
}

std::vector<Point2i> lane_segment(int lane_threshold, int min_area, int sliced_h, int sliced_w, int crop_y_origin,
                                  const Mat &inv_perspectiveMatrix, const Mat &perspectiveMatrix, const Mat &frame)
{

    const int num_slices = 6;
    //height of slice
    const int slice_size = sliced_h / num_slices;

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
    frame(Rect(0, crop_y_origin, sliced_w, sliced_h)).copyTo(cropped);
    Mat distorted = cropped.clone();

    //warp area of interest
    warpPerspective(cropped, distorted, perspectiveMatrix, Size(sliced_w, sliced_h));
    //create black mask image for contour overlay drawing
    maskImage.create(distorted.size().height, distorted.size().width, CV_8UC3);
    maskImage = Scalar(0, 0, 0);

    cvtColor(distorted, distorted_bw, COLOR_RGB2GRAY);
    threshold(distorted_bw, distorted_bw, lane_threshold, 255,
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
        cv::Rect myROI(0, part, sliced_w, slice_size);
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

        if (area > min_area && area > max_area)
        {
            std::cout << "area size: " << area << std::endl;

            max_area = area;
            Moments mu;
            mu = moments(contour, false);
            lane_center_x = mu.m10 / mu.m00;
            lane_center_y = mu.m01 / mu.m00;
            Point2d center(lane_center_x, lane_center_y);
            //save points with y inverted because of origin system
            center_points.emplace_back(lane_center_x,sliced_h-lane_center_y);
            circle(maskImage, center, 5, Scalar(0, 255, 0), -1);
        }
    }
    imshow("contour_points", maskImage);
    waitKey(0);
    //unwarp area of interest and add on cropped image
    warpPerspective(maskImage, undistorted, inv_perspectiveMatrix, Size(sliced_w, sliced_h));
    addWeighted(cropped, 1.0, undistorted, 1.0, 0, cropped);
    imshow("final", cropped);
    waitKey(0);
    return center_points;
}
