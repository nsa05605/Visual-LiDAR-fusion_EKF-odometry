#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <vector>
#include <fstream>

#include <filesystem>

using namespace std;
using namespace cv;
namespace fs = std::filesystem;


cv::Mat getCalib(string dataset_path, string N_Camera){

    // Load calibration data
    string calib = dataset_path + "calib.txt";
    ifstream file(calib);

    if (!file.is_open()){
        cerr << "Unable to open file" << endl;
    }

    string line;
    Mat calibMat = Mat(3, 4, CV_64F);

    if(file.is_open()){
        while(getline(file, line)){
            stringstream ss(line);
            string value, v;
            while (ss >> value){
                if (value == N_Camera){
                    int idx = 0;
                    while (ss >> v){
                        int col = idx % 4;
                        int row = idx / 4;
                        // cout << "row : " << row << " col : " << col << " val : " << v << endl;
                        calibMat.at<double>(row, col) = stod(v); // stoi 대신 stod 사용
                        idx++;}
                }
            }
        }
        file.close();
    }

    return calibMat;
}


// stereo image를 입력 받아서 rectification 수행하는 함수
cv::Mat stereo_rectify(Mat img_l, Mat img_r){



}

cv::Mat calculate_disparity(Mat img_l, Mat img_r){

    // StereoBM
    int ndisparities = 64;
    int blocksize = 15;

    // create stereo image
    Mat img_disparity_16s, img_disparity_8u;

    cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create(ndisparities, blocksize);
    stereo->compute(img_l, img_r, img_disparity_16s);

    img_disparity_16s.convertTo(img_disparity_8u, CV_8UC1);

    return img_disparity_8u;

}


int main(int argc, char* argv[]) {

    // set path of KITTI dataset
    // {$PATH}/EKF_Localization/dataset/KITTI/00/
    string dataset_path = argv[1];

    string left_dir = dataset_path + "image_0/";
    string right_dir = dataset_path + "image_1/";
    

    // Calculate num of images
    int num_images = 0;
    for (const auto& entry : fs::directory_iterator(left_dir)){
        if (entry.is_regular_file()) {
            num_images++;
        }
    }

    // Load calibration data
    Mat calib_l = getCalib(dataset_path, "P0:");
    Mat calib_r = getCalib(dataset_path, "P1:");

    double f_x = calib_l.at<double>(0,0);
    double f_y = calib_l.at<double>(1,1);
    double c_x = calib_l.at<double>(0,2);
    double c_y = calib_l.at<double>(1,2);


    // Load first image pair
    Mat first_l = imread(left_dir + "000000.png", CV_8UC1);
    Mat first_r = imread(right_dir + "000000.png", CV_8UC1);

    if (first_l.empty() || first_r.empty()) {
        cerr << "Failed to load images!" << endl;
        return -1;
    }

    
    // 필요한 작업을 생각해보면
    // 1. stereo rectification
    //  -> https://alida.tistory.com/59
    //  -> https://velog.io/@cjh1995-ros/1.-%EC%A0%80%EA%B0%80-%EC%B9%B4%EB%A9%94%EB%9D%BC%EC%97%90%EC%84%9C-%EA%B3%A0%EA%B0%80%EC%B9%B4%EB%A9%94%EB%9D%BC-%EA%B8%B0%EB%8A%A5-%EA%B5%AC%ED%98%84
    
    // 2. calculate disparity
    // 3. 3D point recover?
    // 4. feature matching
    // 5. calculate R, t



    // Stereo rectification & disparity
    Mat R1, R2, P1, P2, Q;
    Size imageSize = first_l.size();

    Mat intrinsicMat = calib_l(cv::Rect(0,0,3,3));
    Mat dist = Mat::zeros(1, 5, CV_64F);
    Mat R = Mat::eye(3,3, CV_64F);
    Mat T = calib_r.col(3) * 0.001;
    // cout << intrinsicMat << endl;

    stereoRectify(intrinsicMat, dist, intrinsicMat, dist, imageSize, R, T, R1, R2, P1, P2, Q);

    cout << imageSize << endl;
    cout << "R1 : " << R1 << endl;
    cout << "R2 : " << R2 << endl;
    cout << "P1 : " << P1 << endl;
    cout << "P2 : " << P2 << endl;
    cout << "Q : " << Q << endl;

    Mat map1x, map1y, map2x, map2y;
    Mat imgU1, imgU2, imgRectify, img1Teste, img2Teste;
    initUndistortRectifyMap(intrinsicMat, dist, R1, P1, first_l.size(), CV_32FC1, map1x, map1y);
    initUndistortRectifyMap(intrinsicMat, dist, R2, P2, first_r.size(), CV_32FC1, map2x, map2y);

    img1Teste = first_l.clone();
    img2Teste = first_r.clone(); 
    
    cvtColor(img1Teste, img1Teste, cv::COLOR_BGR2RGB);
    cvtColor(img2Teste, img2Teste, cv::COLOR_BGR2RGB);
    
    remap(img1Teste, imgU1, map1x, map1y, INTER_LINEAR, BORDER_CONSTANT, Scalar());
    remap(img2Teste, imgU2, map2x, map2y, INTER_LINEAR, BORDER_CONSTANT, Scalar());
    
    //imshow("image1", img1Teste);
    //imshow("image2", img2Teste);
    
    //To display the rectified images
    // imgRectify = Mat::zeros(imgU1.rows, imgU1.cols*2+10, imgU1.type());

    // imgU1.copyTo(imgRectify(Range::all(), Range(0, imgU2.cols)));
    // imgU2.copyTo(imgRectify(Range::all(), Range(imgU2.cols+10, imgU2.cols*2+10)));

    // //If it is too large to fit on the screen, scale down by 2, it should fit.
    // // if(imgRectify.cols > 1920){
    // //     resize(imgRectify, imgRectify, Size(imgRectify.cols/2, imgRectify.rows/2));
    // // }
    
    // //To draw the lines in the rectified image
    // for(int j = 0; j < imgRectify.rows; j += 16){
    //     Point p1 = Point(0,j);
    //     Point p2 = Point(imgRectify.cols*2,j);
    //     line(imgRectify, p1, p2, CV_RGB(255,0,0));
    // }

    // imshow("Rectified", imgRectify);

    cvtColor(imgU1, imgU1, cv::COLOR_BGR2GRAY);
    cvtColor(imgU2, imgU2, cv::COLOR_BGR2GRAY);
    Mat disparity = calculate_disparity(imgU1, imgU2);
    imshow("disp", disparity);
    waitKey();

    // for문으로 전체 처리

    string filename_format = "%06d.png";
    Mat curr_l, curr_r;
    Mat pred_l, pred_r;

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

        curr_l = imread(curr_filename_l, CV_8UC1);
        curr_r = imread(curr_filename_r, CV_8UC1);

        if (curr_l.empty() || curr_r.empty()) {
            cerr << "Failed to load images!" << endl;
            return -1;
        }


        // Mat disparity = calculate_disparity(curr_l, curr_r);
        // imshow("disp", disparity);

        initUndistortRectifyMap(intrinsicMat, dist, R1, P1, first_l.size(), CV_32FC1, map1x, map1y);
        initUndistortRectifyMap(intrinsicMat, dist, R2, P2, first_r.size(), CV_32FC1, map2x, map2y);

        img1Teste = curr_l.clone();
        img2Teste = curr_r.clone(); 
        
        cvtColor(img1Teste, img1Teste, cv::COLOR_BGR2RGB);
        cvtColor(img2Teste, img2Teste, cv::COLOR_BGR2RGB);
        
        remap(img1Teste, imgU1, map1x, map1y, INTER_LINEAR, BORDER_CONSTANT, Scalar());
        remap(img2Teste, imgU2, map2x, map2y, INTER_LINEAR, BORDER_CONSTANT, Scalar());
    

        cvtColor(imgU1, imgU1, cv::COLOR_BGR2GRAY);
        cvtColor(imgU2, imgU2, cv::COLOR_BGR2GRAY);
        Mat disparity = calculate_disparity(imgU1, imgU2);
        imshow("disp", disparity);


        imshow("curr_l", curr_l);
        imshow("curr_r", curr_r);
        waitKey(0);

    }




    return 0;
}
