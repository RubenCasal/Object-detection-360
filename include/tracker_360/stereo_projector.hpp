// stereo_projector.hpp
#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>

class StereoProjector {
public:
    StereoProjector(int pano_width, int pano_height, cv::Size proj_size, cv::Size2f fov_deg);

    void project(const cv::Mat& pano_img, std::vector<cv::Mat>& projections);

private:
    void compute_map(int yaw_deg, int pitch_deg, cv::Mat& map_x, cv::Mat& map_y);
    void map_to_sphere(const cv::Mat& x, const cv::Mat& y, const cv::Mat& z,
                       float radius, float yaw_rad, float pitch_rad,
                       cv::Mat& theta, cv::Mat& phi);

    int pano_width_;
    int pano_height_;
    cv::Size proj_size_;
    cv::Size2f fov_deg_;

    std::vector<cv::Mat> map_xs_;
    std::vector<cv::Mat> map_ys_;
};

// stereo_projector.cpp
#include "stereo_projector.hpp"

StereoProjector::StereoProjector(int pano_width, int pano_height, cv::Size proj_size, cv::Size2f fov_deg)
    : pano_width_(pano_width), pano_height_(pano_height), proj_size_(proj_size), fov_deg_(fov_deg) {

    for (int i = 0; i < 4; ++i) {
        cv::Mat map_x(proj_size_, CV_32F);
        cv::Mat map_y(proj_size_, CV_32F);
        compute_map(i * 90, 90, map_x, map_y);
        map_xs_.push_back(map_x);
        map_ys_.push_back(map_y);
    }
}

void StereoProjector::project(const cv::Mat& pano_img, std::vector<cv::Mat>& projections) {
    projections.clear();
    for (int i = 0; i < 4; ++i) {
        cv::Mat out;
        cv::remap(pano_img, out, map_xs_[i], map_ys_[i], cv::INTER_LINEAR, cv::BORDER_WRAP);
        projections.push_back(out);
    }
}

void StereoProjector::map_to_sphere(const cv::Mat& x, const cv::Mat& y, const cv::Mat& z,
                                    float radius, float yaw_rad, float pitch_rad,
                                    cv::Mat& theta, cv::Mat& phi) {
    cv::Mat denom = x.mul(x) + y.mul(y) + 4.0 * (radius * radius);

    cv::Mat x_circ = (4.0 * radius * radius) * x / denom;
    cv::Mat y_circ = (4.0 * radius * radius) * y / denom;
    cv::Mat z_circ = -radius + (8.0 * std::pow(radius, 3)) / denom;

    cv::Mat norm;
    cv::sqrt(x_circ.mul(x_circ) + y_circ.mul(y_circ) + z_circ.mul(z_circ), norm);

    theta = z_circ / norm;
    cv::acos(theta, theta);

    phi = cv::Mat(x.size(), CV_32F);
    for (int i = 0; i < x.rows; ++i) {
        for (int j = 0; j < x.cols; ++j) {
            phi.at<float>(i, j) = std::atan2(y_circ.at<float>(i, j), x_circ.at<float>(i, j));
        }
    }

    // Apply pitch rotation
    for (int i = 0; i < x.rows; ++i) {
        for (int j = 0; j < x.cols; ++j) {
            float t = theta.at<float>(i, j);
            float p = phi.at<float>(i, j);

            float sin_t = std::sin(t);
            float cos_t = std::cos(t);
            float sin_p = std::sin(p);
            float cos_p = std::cos(p);
            float sin_pitch = std::sin(pitch_rad);
            float cos_pitch = std::cos(pitch_rad);

            float theta_prime = std::acos(sin_t * sin_p * sin_pitch + cos_t * cos_pitch);
            float phi_prime = std::atan2(sin_t * sin_p * cos_pitch - cos_t * sin_pitch,
                                         sin_t * cos_p);

            phi_prime += yaw_rad;
            if (phi_prime < 0) phi_prime += 2 * CV_PI;
            if (phi_prime >= 2 * CV_PI) phi_prime -= 2 * CV_PI;

            theta.at<float>(i, j) = theta_prime;
            phi.at<float>(i, j) = phi_prime;
        }
    }
}
