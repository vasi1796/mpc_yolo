#include "LaneController.h"

#include "utilities.hpp"

//
// Created by vasy1 on 2/27/2019.
//
LaneController::LaneController(int width, int height) :
        m_width(width),
        m_height(height),
        m_slicedH(m_height - m_cropYOorigin),
        m_slicedW(m_width),
        m_perspectiveSrc{Point2f(m_slicedW / 2.f - 50, 20), Point2f(150, m_slicedH),
                         Point2f(m_slicedW - 20, m_slicedH), Point2f(m_slicedW / 2.f + 80, 20)},
        m_perspectiveDst{Point2f(150, 0), Point2f(150, m_slicedH), Point2f(m_slicedW, m_slicedH),
                         Point2f(m_slicedW, 0)},
                         m_state(INIT)
{
    m_perspectiveMatrix = getPerspectiveTransform(m_perspectiveSrc, m_perspectiveDst);
    m_inv_perspectiveMatrix = getPerspectiveTransform(m_perspectiveDst, m_perspectiveSrc);
    m_server.run();
    m_detector.setDictionary("ARUCO_MIP_36h12", 0.f);
    //m_cameraParams.readFromXMLFile("/home/car/cameraCalib.yml");
}

void LaneController::run(Mat &frame)
{
    if (!frame.empty())
    {
        switch(m_state)
        {
            case INIT:
            {
                std::vector<Point2d> coords = this->lane_segment(frame);
                if(!coords.empty())
                {
                    m_state = DRIVE;
                }

                //check parking condition
                if(m_server.park.load())
                {
                    std::vector<aruco::Marker> markers = m_detector.detect(frame, m_cameraParams, m_markerSize);
                    if(!markers.empty())
                    {
                        m_state = PARK;
                    }
                }
                break;
            }
            case DRIVE:
            {
                //check stop condition
                if (m_server.stop.load())
                {
                    m_state = STOP;
                    break;
                }

                //check parking condition
                if(m_server.park.load())
                {
                    std::vector<aruco::Marker> markers = m_detector.detect(frame, m_cameraParams, m_markerSize);
                    if(!markers.empty())
                    {
                        m_state = PARK;
                        break;
                    }
                }

                drive(frame);
                break;
            }
            case PARK:
            {
                std::vector<aruco::Marker> markers = m_detector.detect(frame, m_cameraParams, m_markerSize);
                if(!markers.empty())
                {
                    m_parkMarker = markers[0];
                    double markerArea = m_parkMarker.getArea();
                    m_drawingFrame = frame.clone();
                    m_parkMarker.draw(m_drawingFrame, Scalar(255, 0, 0), 2);
                    const int camera_offset = 30;
                    if (markerArea < 25000)
                    {
                        double sign_center = m_parkMarker.getCenter().x;
                        double off_center;
                        double steering;

                        off_center = (((m_width / 2) - sign_center) * -1) - camera_offset;
                        steering = get_steering(off_center);
                    }
                    else
                    {
                        m_state = INIT;
                        m_server.park = false;
                    }
                }
                else
                {
                    m_state = INIT;
                }
                break;
            }
            case STOP:
            {
                if (!m_server.stop.load())
                {
                    m_state = INIT;
                }
                drive(frame);
                break;
            }
        };
    }

}

void LaneController::drive(Mat &frame)
{
    m_drawingFrame = frame.clone();
    vector<Point2d> coords = lane_segment(frame);
    if (coords.size() >= 6)
    {
        vector<double> solution = run_mpc(coords);
        draw_mpc(solution);
    }
    else
    {
        classic_lane_follow(frame);
    }
    imshow("window", m_drawingFrame);
    waitKey(1);
}

std::vector<double> LaneController::run_mpc(std::vector<Point2d> coords)
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
    state << 0, 0, 0, 0.1, cte, epsi;
    std::vector<double> vars = mpc.Solve(state, coeffs);
    std::cout<<"d "<<vars[0] <<" a "<<vars[1]<<std::endl;
    return vars;
}

std::vector<Point2d> LaneController::lane_segment(const Mat &frame)
{

    const int num_slices = 6;
    //height of slice
    const int slice_size = m_slicedH / num_slices;

    double lane_center_x;
    double lane_center_y;

    std::vector<Point2d> center_points;
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

//    cvtColor(distorted, distorted_bw, COLOR_RGB2GRAY);
//    threshold(distorted_bw, distorted_bw, m_laneThreshold, 255,
//              THRESH_BINARY_INV); // sau THRESH_BINARY, si verifici care arie e mare sau offset centers
//  alternativ color masking (hsv, inRange)
    equalize_illuminance(distorted);

    Mat distorted_hsv;
    cvtColor(distorted, distorted_hsv, cv::COLOR_BGR2HSV);
    inRange(distorted_hsv, m_green_range_start, m_green_range_end, distorted_bw);

    Mat erodeElmt = getStructuringElement(MORPH_RECT, Size(4, 4));    // reflectii etc
    Mat dilateElmt = getStructuringElement(MORPH_RECT, Size(5, 5));
    erode(distorted_bw, distorted_bw, erodeElmt);
    dilate(distorted_bw, distorted_bw, dilateElmt);

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
    //imshow("concat_slices", concat_slices);
    //waitKey(1);

    //find slices contour and center of mass
    findContours(concat_slices, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    drawContours(maskImage, contours, -1, Scalar(255, 0, 255));

    for (const auto &contour : contours)
    {
        double area = contourArea(contour);

        if (area > m_minArea)
        {
            Moments mu;
            mu = moments(contour, false);
            lane_center_x = mu.m10 / mu.m00;
            lane_center_y = mu.m01 / mu.m00;
            Point2d center(lane_center_x, lane_center_y);
            //save points with y inverted because of origin system
            center_points.emplace_back((lane_center_x-m_slicedW/2.0-40)/2, m_slicedH - lane_center_y);
            circle(maskImage, center, 5, Scalar(255, 0, 255), -1);
        }
    }
    //imshow("contour_points", maskImage);
    //waitKey(1);
    //unwarp area of interest and add on cropped image
    warpPerspective(maskImage, undistorted, m_inv_perspectiveMatrix, Size(m_slicedW, m_slicedH));
    addWeighted(cropped, 1.0, undistorted, 1.0, 0, cropped);
    m_drawingFrame = cropped.clone();
    cv::Mat top;
    frame(Rect(0, 0, m_slicedW, m_cropYOorigin)).copyTo(top);
    vconcat(std::vector<cv::Mat>({top,m_drawingFrame}),m_drawingFrame);

    return center_points;
}

void LaneController::equalize_illuminance(Mat &distorted)
{
    Mat equalized;
    cvtColor(distorted, equalized, CV_BGR2YUV);
    vector<Mat> channels;
    split(equalized, channels);
    equalizeHist(channels[0], channels[0]);
    merge(channels, equalized);
    cvtColor(equalized, distorted, CV_YUV2BGR);
}

void LaneController::draw_mpc(std::vector<double> mpc_result)
{
    const double sin_psi = sin(utilities::deg2rad(90));
    const double cos_psi = cos(utilities::deg2rad(90));
    for (int index = 2; index < mpc_result.size() - 1; index += 2)
    {
        int x = int(
                ((mpc_result[index] * cos_psi - mpc_result[index + 1] * sin_psi) * 800 + 11) / 20 * m_drawingFrame.cols);
        int y = int(m_drawingFrame.rows -
                    (mpc_result[index] * sin_psi + mpc_result[index + 1] * cos_psi) * 50 / 22 * m_drawingFrame.rows);
        //std::cout<<mpc_result[index] * cos_psi - mpc_result[index + 1] * sin_psi<<std::endl;
        circle(m_drawingFrame, Point2i(x, y), 3, Scalar(255, 0, 0), -1);
    }
}

void LaneController::classic_lane_follow(Mat &frame)
{
    double lane_center = track_lane(frame);
    double off_center;
    double steering;

    const int camera_offset = 30;

    if (lane_center == -1)
    {
        cout << "lane not detected" << endl;
    } else
    {
        off_center = (((m_width / 2.0) - lane_center) * -1) - camera_offset;
        steering = get_steering(off_center);
    }
}

double LaneController::track_lane(Mat &frame)
{

    Mat sliced;
    Mat sliced_bw;
    float lane_center = -1;
    const int slice_height = 100;
    const int slice_y = 300;
    const int valid_area = 10000;

    frame(Rect(0, slice_y, m_width, slice_height)).copyTo(sliced);

    equalize_illuminance(sliced);
    Mat sliced_hsv;
    cvtColor(sliced, sliced_hsv, cv::COLOR_BGR2HSV);
    inRange(sliced_hsv, m_green_range_start, m_green_range_end, sliced_bw);


    Mat erodeElmt = getStructuringElement(MORPH_RECT, Size(4, 4));    // reflectii etc
    Mat dilateElmt = getStructuringElement(MORPH_RECT, Size(5, 5));
    erode(sliced_bw, sliced_bw, erodeElmt);
    dilate(sliced_bw, sliced_bw, dilateElmt);
    //morphologyEx(sliced_bw, sliced_bw, MORPH_OPEN, erodeElmt);

    vector<vector<Point>> contours;

    findContours(sliced_bw, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, slice_y));


    for (vector<Point> &vec : contours)
    {
        vector<Point> hull;
        convexHull(Mat(vec), hull, true);
        vec = hull;
    }


    for (const auto &contour : contours)
    {
        double area = contourArea(contour);

        if (area > valid_area)
        {
            //cout << "area size: " << area << endl;
            drawContours(m_drawingFrame, std::vector<vector<Point>>(1, contour), -1, Scalar(0, 0, 255));
            Moments mu;
            mu = moments(contour, false);
            lane_center = mu.m10 / mu.m00;
            Point2f center(lane_center, slice_y + (slice_height / 2));
            circle(m_drawingFrame, center, 5, Scalar(0, 0, 255), -1);
        }
    }

    return lane_center;

}


double LaneController::get_steering(double off_center) {

    const int min_edge_distance = 50;
    const double max_steering = 0.55;
    double tmax = m_width / 2.0 - min_edge_distance;
    bool right = true;

    if (off_center < 0) right = false;
    off_center = abs(off_center);

    if (off_center > tmax) return right ? max_steering : -max_steering;
    return (off_center * max_steering / tmax) * (right ? 1 : -1);
}