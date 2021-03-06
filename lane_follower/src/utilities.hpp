//
// Created by vasy1 on 2/14/2019.
//
#include <Eigen/Core>
#include <Eigen/QR>
#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>

namespace utilities
{
    static constexpr double pi() { return M_PI; }

    inline double deg2rad(double x) { return x * pi() / 180; }

    inline double rad2deg(double x) { return x * 180 / pi(); }

// Evaluate a polynomial.
    static double polyeval(Eigen::VectorXd coeffs, double x)
    {
        double result = 0.0;
        for (int i = 0; i < coeffs.size(); i++)
        {
            result += coeffs[i] * pow(x, i);
        }
        return result;
    }

    static Eigen::VectorXd polyfit(const Eigen::VectorXd &xvals, const Eigen::VectorXd &yvals,
                            int order)
    {
        assert(xvals.size() == yvals.size());
        assert(order >= 1 && order <= xvals.size() - 1);
        Eigen::MatrixXd A(xvals.size(), order + 1);

        for (int i = 0; i < xvals.size(); i++)
        {
            A(i, 0) = 1.0;
        }

        for (int j = 0; j < xvals.size(); j++)
        {
            for (int i = 0; i < order; i++)
            {
                A(j, i + 1) = A(j, i) * xvals(j);
            }
        }

        auto Q = A.householderQr();
        auto result = Q.solve(yvals);
        return result;
    }

    static std::vector<Eigen::Map<Eigen::VectorXd>> rotate_coords(const std::vector<cv::Point2d> &coords)
    {
        //adjust points in ref to be in front of car, apply angle transformation
        const double sin_psi = sin(deg2rad(-90));
        const double cos_psi = cos(deg2rad(-90));
        std::vector<double> waypoints_x;
        std::vector<double> waypoints_y;
        for (auto &coord : coords)
        {
            waypoints_x.push_back(coord.x * cos_psi - coord.y * sin_psi);
            waypoints_y.push_back(coord.x * sin_psi + coord.y * cos_psi);
        }

        double *ptrx = &waypoints_x[0];
        double *ptry = &waypoints_y[0];
        Eigen::Map<Eigen::VectorXd> waypoints_x_eig(ptrx, 6);
        Eigen::Map<Eigen::VectorXd> waypoints_y_eig(ptry, 6);
        return {waypoints_x_eig, waypoints_y_eig};
    }
}




