#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <vector>
#include <fstream>

#include <filesystem>

int blocksize = 15;
int P1 = blocksize * 8;
int P2 = blocksize * 32;

int min_disparity = 0;
int ndisparities = 32;

cv::Ptr<cv::StereoBM> stereoBM = cv::StereoBM::create(ndisparities, blocksize);
cv::Ptr<cv::StereoSGBM> stereoSGBM = cv::StereoSGBM::create(min_disparity, ndisparities, blocksize, P1, P2);
