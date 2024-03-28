#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <vector>
#include <fstream>

#include <filesystem>

int sad_window = 6;
int min_disparity = 0;
int ndisparities = sad_window*16;
int blocksize = 11;
// int P1 = 8 * blocksize;
// int P2 = 32 * blocksize;
int P1 = 8 * 3 * pow(sad_window, 2);
int P2 = 32 * 3 * pow(sad_window, 2);

cv::Ptr<cv::StereoBM> stereoBM = cv::StereoBM::create(ndisparities, blocksize);
cv::Ptr<cv::StereoSGBM> stereoSGBM = cv::StereoSGBM::create(min_disparity, ndisparities, blocksize, P1, P2, 0, 0, 0, 0, 0, cv::StereoSGBM::MODE_SGBM_3WAY);

cv::Ptr<cv::Feature2D> orb = cv::ORB::create(200, 1.2f, 8);
