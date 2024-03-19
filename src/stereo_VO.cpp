#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <vector>
#include <fstream>
#include <Eigen/Dense>

#include <filesystem> // C++17 이상에서 지원되는 표준 라이브러리

using namespace std;
using namespace cv;
namespace fs = std::filesystem;


int main(int argc, char* argv[]) {

    // KITTI dataset 경로 설정
    string dataset_path = argv[1];

    string left_dir = dataset_path + "image_0/";
    string right_dir = dataset_path + "image_1/";
    
    // 이미지 개수 계산
    int num_images = 0;
    for (const auto& entry : fs::directory_iterator(left_dir)){
        if (entry.is_regular_file()) {
            num_images++;
        }
    }

    // Calibration file 불러오기
    string calib = dataset_path + "calib.txt";
    ifstream file(calib);
    
    if (!file.is_open()){
        cerr << "Unable to open file" << endl;
        return -1;
    }

    string line;
    vector<string> intrinsic;
    // vector<string> extrinsic;
    Mat extrinsicMat = Mat(3, 4, CV_64F);

    if(file.is_open()){
        while(getline(file, line)){
            stringstream ss(line);
            string value, v;
            while (ss >> value){
                if (value == "P0:"){
                    while (ss >> v){intrinsic.push_back(v);}
                }
                else if (value == "Tr:"){
                    int idx = 0;
                    while (ss >> v){
                        int col = idx % 4;
                        int row = idx / 4;
                        // cout << "row : " << row << " col : " << col << " val : " << v << endl;
                        extrinsicMat.at<double>(row, col) = stod(v); // stoi 대신 stod 사용
                        idx++;}
                }
            }
        }
        file.close();
    }

    double fx = stod(intrinsic[0]); double fy = stod(intrinsic[5]);
    double cx = stod(intrinsic[2]); double cy = stod(intrinsic[6]);
    
    // std::cout << "Intrinsic parameters : " << fx << " " << fy << " " << cx << " " << cy << endl;
    // std::cout << "Extrinsic Matrix:\n" << extrinsicMat << std::endl;

    // 첫 이미지 불러오기



    // for문으로 전체 처리
    for (int Frame = 1; Frame < num_images; Frame++){

    }




    return 0;
}
