#include <stereo_VO.h>
// #include <Open3D/Open3D.h>

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
    cv::Mat calibMat = Mat(3, 4, CV_64F);

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

// stereo image를 입력 받아서 disparity를 계산하는 과정
void calculate_disparity(cv::Mat img_l, cv::Mat img_r, Mat &disparity){

    cv::Mat img_disparity_16s;
    // stereoBM->compute(img_l, img_r, img_disparity_16s);
    stereoSGBM->compute(img_l, img_r, img_disparity_16s);

    img_disparity_16s.convertTo(disparity, CV_32F);
    disparity = disparity / 16.0f;
}


void calculate_depth(cv::Mat disparity, const double focal, const double baseline, cv::Mat &depth){

    cv::Mat mask = (disparity == 0) | (disparity == -1);
    disparity.setTo(0.1, mask);

    depth = (focal * baseline) / disparity;
}


void ORBextract(cv::Mat img, std::vector<KeyPoint> &keys, cv::Mat &desc, const int nfeatures){
    const float W = 200;

    const int minBorderX = 16;
    const int minBorderY = minBorderX;
    const int maxBorderX = img.cols - 16;
    const int maxBorderY = img.rows - 16;

    keys.reserve(nfeatures * 5);

    const float width = maxBorderX - minBorderX;
    const float height = maxBorderY - minBorderY;
    
    const int nCols = width / W;
    const int nRows = height / (W/2);
    const int wCell = ceil(width/nCols);
    const int hCell = ceil(height/nRows);

    for (int i = 0; i < nRows; i++)
    {
        const float iniY = minBorderY + i*hCell;
        float maxY = iniY + hCell+6;
        if(iniY>=maxBorderY-3)
            continue;
        if(maxY>maxBorderY)
            maxY = maxBorderY;

        for (int j = 0; j < nCols; j++)
        {
            const float iniX = minBorderX + j*wCell;
            float maxX = iniX+wCell+6;
            if(iniX>=maxBorderX-6)
                continue;
            if(maxX>maxBorderX)
                maxX = maxBorderX;

            vector<cv::KeyPoint> vKeysCell;
            
            orb->detect(img.rowRange(iniY,maxY).colRange(iniX,maxX), vKeysCell, Mat());

            if(!vKeysCell.empty())
            {
                for(vector<cv::KeyPoint>::iterator vit=vKeysCell.begin(); vit!=vKeysCell.end();vit++)
                {
                    (*vit).pt.x+=j*wCell;
                    (*vit).pt.y+=i*hCell;
                    keys.push_back(*vit);
                }            
            }
        }
    }
    // cout << keys.size() << endl;    // 1258

    orb->compute(img, keys, desc);
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
    cv::Mat proj_l = getCalib(dataset_path, "P0:");
    cv::Mat proj_r = getCalib(dataset_path, "P1:");

    cv::Mat intrinsic, Rot, trans;
    
    // Projection matrix를 분해해서 intrinsic, extrinsic(R, t) 계산
    cv::decomposeProjectionMatrix(proj_r, intrinsic, Rot, trans);
    trans /= trans.at<double>(0,3);

    double f_x = intrinsic.at<double>(0,0);
    double f_y = intrinsic.at<double>(1,1);
    double c_x = intrinsic.at<double>(0,2);
    double c_y = intrinsic.at<double>(1,2);
    double baseline = trans.at<double>(0,0);
    // cout << baseline << endl;    // 0.537166

    Mat Q = Mat::eye(4, 4, CV_64F);
    Q.at<double>(0, 3) = -c_x;
    Q.at<double>(1, 3) = -c_y;
    Q.at<double>(2, 3) = f_x;
    Q.at<double>(3, 3) = 0.0;
    Q.at<double>(3, 2) = -1 / baseline;

    // Load first image pair
    cv::Mat pred_l = imread(left_dir + "000000.png", CV_8UC1);
    cv::Mat pred_r = imread(right_dir + "000000.png", CV_8UC1);
    cv::Mat curr_l = imread(left_dir + "000001.png", CV_8UC1);
    cv::Mat curr_r = imread(right_dir + "000001.png", CV_8UC1);

    if (pred_l.empty() || pred_r.empty() || curr_l.empty() || curr_r.empty()) {
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
    cv::Mat disparity_p, disparity_c, depth_p, depth_c;

    calculate_disparity(pred_l, pred_r, disparity_p);
    calculate_disparity(curr_l, curr_r, disparity_c);

    calculate_depth(disparity_p, f_x, baseline, depth_p);
    calculate_depth(disparity_c, f_x, baseline, depth_c);


    // double minD, maxD;
    // cv::Point minLoc, maxLoc;
    // cv::minMaxLoc(depth_p, &minD, &maxD, &minLoc, &maxLoc);
    // std::cout << "Min value: " << minD << " at location: " << minLoc << std::endl;
    // std::cout << "Max value: " << maxD << " at location: " << maxLoc << std::endl;


    /////////////////////////////////////
    // ORB feature 추출 후 매칭해서 R, t 계산
    
    // ORB feature 추출
    // 이미지를 16등분? 정도 내서 각 부분에서 특징 뽑고 매칭하는 코드로 새로 짜보자
    std::vector<cv::KeyPoint> key_curr, key_pred, key_local;
    cv::Mat desc_curr, desc_pred, desc_local;

    cv::Ptr<cv::DescriptorMatcher> Matcher_orb = cv::BFMatcher::create(cv::NORM_HAMMING);
    std::vector<cv::DMatch> matches;



    int nfeatures = 2000;
    std::vector<cv::KeyPoint> vToDistributeKeys_curr, vToDistributeKeys_pred;

    ORBextract(pred_l, vToDistributeKeys_pred, desc_pred, nfeatures);
    ORBextract(curr_l, vToDistributeKeys_curr, desc_curr, nfeatures);

    // ORB feature 매칭
    Matcher_orb->match(desc_pred, desc_curr, matches);

    sort(matches.begin(), matches.end());
    const int match_size = matches.size();
    std::vector<cv::DMatch> good_matches(matches.begin(), matches.begin() + (int)(match_size * 0.2f));

    cv::Mat Result;
    cv::drawMatches(pred_l, vToDistributeKeys_pred, curr_l, vToDistributeKeys_curr, good_matches, Result, cv::Scalar::all(-1), cv::Scalar(-1), vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    // Draws the found matches of keypoints from two images.

    imshow("Matching Result_ORB", Result);
    waitKey(0);
    
    // // 좋은 매칭 포인트로 변환 행렬 계산
    // std::vector<Point2f> points_t, points_tplus1;
    // for (size_t i = 0; i < good_matches.size(); i++) {
    //     points_t.push_back(vToDistributeKeys_pred[good_matches[i].queryIdx].pt);
    //     points_tplus1.push_back(vToDistributeKeys_curr[good_matches[i].trainIdx].pt);
    // }
    // // 디스패리티를 이용하여 3D 포인트 계산
    // std::vector<Point3f> points_3d;
    // for (int i = 0; i < points_t.size(); i++) {
    //     float depth_value = depth_c.at<float>(points_t[i]); // 해당 점의 디스패리티 값
    //     Point3f point_3d(points_t[i].x, points_t[i].y, depth_value);
    //     points_3d.push_back(point_3d);
    // }
    // // 변환 행렬 계산
    // Mat fundamental_matrix = findFundamentalMat(points_t, points_tplus1, FM_RANSAC);

    // Mat essential_matrix = findEssentialMat(points_t, points_tplus1, f_x, c_x);

    // Mat R, t;
    // recoverPose(essential_matrix, points_t, points_tplus1, R, t);

    // print(R);
    // print(t);


    // // for문으로 전체 처리

    // string filename_format = "%06d.png";

    // for (int Frame = 2; Frame < num_images; Frame++){
    //     // pred_lr을 이전 timestamp 프레임으로 저장
    //     if (Frame == 2){
    //         pred_l = curr_l.clone();
    //         pred_r = curr_r.clone();
    //     }
    //     else{
    //         pred_l = curr_l.clone();
    //         pred_r = curr_r.clone();
    //     }

    //     char filename[100];
    //     sprintf(filename, filename_format.c_str(), Frame);
    //     string curr_filename_l = left_dir + filename;
    //     string curr_filename_r = right_dir + filename;

    //     curr_l = imread(curr_filename_l, CV_8UC1);
    //     curr_r = imread(curr_filename_r, CV_8UC1);

    //     if (curr_l.empty() || curr_r.empty()) {
    //         cerr << "Failed to load images!" << endl;
    //         return -1;
    //     }

    //     calculate_disparity(curr_l, curr_r, disparity_c, depth_c, Q);
    //     imshow("curr_disp", disparity_c);
    //     // imshow("curr_l", curr_l);
    //     // imshow("curr_r", curr_r);
    //     waitKey(0);

    // }




    return 0;
}
