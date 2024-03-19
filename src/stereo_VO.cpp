#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <vector>
#include <fstream>

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
    Mat intrinsicMat = Mat(3, 4, CV_64F);
    Mat extrinsicMat = Mat(3, 4, CV_64F);

    if(file.is_open()){
        while(getline(file, line)){
            stringstream ss(line);
            string value, v;
            while (ss >> value){
                if (value == "P0:"){
                    int idx = 0;
                    while (ss >> v){
                        int col = idx % 4;
                        int row = idx / 4;
                        // cout << "row : " << row << " col : " << col << " val : " << v << endl;
                        intrinsicMat.at<double>(row, col) = stod(v); // stoi 대신 stod 사용
                        idx++;}
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

    double f_x = intrinsicMat.at<double>(0,0);
    double f_y = intrinsicMat.at<double>(1,1);
    double c_x = intrinsicMat.at<double>(0,2);
    double c_y = intrinsicMat.at<double>(1,2);
    
    // std::cout << "Intrinsic parameters : " << f_x << " " << f_y << " " << c_x << " " << c_y << endl;
    // std::cout << "Extrinsic Matrix:\n" << extrinsicMat << std::endl;


    // 첫 이미지 불러오기
    Mat first_l = imread(left_dir + "000000.png");
    Mat first_r = imread(right_dir + "000000.png");

    if (first_l.empty() || first_r.empty()) {
        cerr << "Failed to load images!" << endl;
        return -1;
    }

    string filename_format = "%06d.png";
    Mat curr_l, curr_r;
    Mat pred_l, pred_r;

    // for문으로 전체 처리
    for (int Frame = 1; Frame < num_images; Frame++){
        // pred_lr을 이전 timestamp 프레임으로 저장
        if (Frame == 1){
            pred_l = first_l.clone();
            pred_r = first_r.clone();
        }
        else{
            pred_l = curr_l.clone();
            pred_r = curr_r.clone();
        }

        char filename[100];
        sprintf(filename, filename_format.c_str(), Frame);
        string curr_filename_l = left_dir + filename;
        string curr_filename_r = right_dir + filename;

        curr_l = imread(curr_filename_l);
        curr_r = imread(curr_filename_r);

        if (curr_l.empty() || curr_r.empty()) {
            cerr << "Failed to load images!" << endl;
            return -1;
        }

        imshow("curr_l", curr_l);
        imshow("curr_r", curr_r);
        waitKey(0);

    }




    return 0;
}
