#include <stereo_VO.h>
// #include <Open3D/Open3D.h>

using namespace std;
using namespace cv;
namespace fs = std::filesystem;

cv::Mat getCalib(string &dataset_path, string N_Camera){

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
void calculate_disparity(cv::Mat &img_l, cv::Mat &img_r, Mat &disparity){

    cv::Mat img_disparity_16s;
    // stereoBM->compute(img_l, img_r, img_disparity_16s);
    stereoSGBM->compute(img_l, img_r, img_disparity_16s);

    img_disparity_16s.convertTo(disparity, CV_32F);
    disparity = disparity / 16.0f;
}


void calculate_depth(cv::Mat &disparity, const double focal, const double baseline, cv::Mat &depth){

    cv::Mat mask = (disparity == 0) | (disparity == -1);
    disparity.setTo(0.1, mask);

    depth = (focal * baseline) / disparity;
}


void ORBextract(cv::Mat &img, std::vector<KeyPoint> &keys, cv::Mat &desc, const int nfeatures){
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

    // KITTI dataset 경로 설정해주기
    // {$PATH}/EKF_Localization/dataset/KITTI/00/
    string dataset_path = argv[1];

    string left_dir = dataset_path + "image_0/";
    string right_dir = dataset_path + "image_1/";
    

    // 총 이미지 개수 카운트
    int num_images = 0;
    for (const auto& entry : fs::directory_iterator(left_dir)){
        if (entry.is_regular_file()) {
            num_images++;
        }
    }


    // Calibration 데이터 불러오기
    // KITTI에서는 projection matrix(P0, P1) 형식으로 제공해줌. 그래서 이를 intrinsic, extrinsic(여기서는 stereo 간의 변환 관계)를 따로 추출해야 함.
    cv::Mat proj_l = getCalib(dataset_path, "P0:");
    cv::Mat proj_r = getCalib(dataset_path, "P1:");

    cv::Mat intrinsic, Rot, trans;
    cv::Mat dist = cv::Mat::zeros(1, 5, CV_64F);
    cv::decomposeProjectionMatrix(proj_r, intrinsic, Rot, trans);   // projection matrix를 intrinsic, extrinsic(R,t)로 분해
    trans /= trans.at<double>(0,3);

    double f_x = intrinsic.at<double>(0,0);
    double f_y = intrinsic.at<double>(1,1);
    double c_x = intrinsic.at<double>(0,2);
    double c_y = intrinsic.at<double>(1,2);
    double baseline = trans.at<double>(0,0);
    // cout << baseline << endl;    // 0.537166


    // 매칭에 사용할 이미지 쌍 불러오기.
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
    // 3. 3D point recover
    // 4. feature matching
    // 5. calculate R, t


    // Stereo rectification 및 disparity 계산
    // 추가적으로 intrinsic과 baseline 정보를 활용해서 depth estimation까지 진행
    calculate_disparity(pred_l, pred_r, disparity_p);
    calculate_depth(disparity_p, f_x, baseline, depth);


    /////////////////////////////////////
    // ORB feature 추출 후 매칭해서 R, t 계산
    // 단순하게 이미지 전체에서 특징을 추출하는 것보다 이미지에서 균일하게 추출하기 위해, 200x200 패치로 나눠서 추출함. 이는 ORB-SLAM에서 사용하는 방법과 유사하게 진행함.
    
    
    std::vector<cv::KeyPoint> vToDistributeKeys_curr, vToDistributeKeys_pred;

    ORBextract(pred_l, vToDistributeKeys_pred, desc_pred, nfeatures);
    ORBextract(curr_l, vToDistributeKeys_curr, desc_curr, nfeatures);

    // ORB feature 매칭
    // 단순하게 상위 20%의 매칭만 inlier(=good_matches)로 사용함
    Matcher_orb->match(desc_pred, desc_curr, matches);
    sort(matches.begin(), matches.end());
    const int match_size = matches.size();
    std::vector<cv::DMatch> good_matches(matches.begin(), matches.begin() + (int)(match_size * 0.2f));
    

    // t 시점의 특징을 기존에 계산한 depth를 사용해서 3차원 특징으로 복원(recover)하고, t+1 시점의 특징을 2창원 특징으로 사용해서 PnP 알고리즘에 대입
    // PnP 알고리즘은 OpenCV의 solvePnPRansac()을 사용함
    std::vector<Point2f> points_t, points_tplus1;
    for (size_t i = 0; i < good_matches.size(); i++) {
        float depth_value = depth.at<float>(vToDistributeKeys_pred[good_matches[i].queryIdx].pt);
        if (depth_value > 100) {continue;} // depth 값이 너무 크면 이상치(outlier)로 판단하여 제외함
        points_t.push_back(vToDistributeKeys_pred[good_matches[i].queryIdx].pt);
        points_tplus1.push_back(vToDistributeKeys_curr[good_matches[i].trainIdx].pt);
    }

    // t 시점의 특징을 depth 정보를 활용해서 3차원 특징으로 복원
    std::vector<Point3f> points_3d;
    for (int i = 0; i < points_t.size(); i++) {
        float depth_value = depth.at<float>(points_t[i]);
        float x = depth_value * (points_t[i].x - c_x) / f_x;
        float y = depth_value * (points_t[i].y - c_y) / f_y;
        Point3f point_3d(x, y, depth_value);
        points_3d.push_back(point_3d);
    }

    // 앞에서 pred_image의 3차원 특징(points_3d)을 구했고, 그와 매칭된 curr_image의 2차원 특징(points_tplus1)을 구했으니,
    // 이제 SolvePnPRansac 사용해서 Rotation, Translation 계산
    // solvePnPRansac은 회전을 rodrigues 형식으로 제공해주기 때문에, 이를 rotation matrix 형식으로 변환하는 과정을 거쳐야 함.
    cv::Mat est_R, est_T, R_matrix;
    cv::solvePnPRansac(points_3d, points_tplus1, intrinsic, dist, est_R, est_T, false, 200);
    cv::Rodrigues(est_R, R_matrix);

    // 위에서 구한 R, t를 4x4 형태의 transformation matrix로 변환
    cv::Mat Transform = cv::Mat::eye(4, 4, CV_64F);
    R_matrix.copyTo(Transform(cv::Rect(0, 0, 3, 3)));
    est_T.copyTo(Transform(cv::Rect(3, 0, 1, 3)));

    // cout << Transform << endl;
    // [0.9999921656919954, 0.002470933226444729, 0.003092417116621545, 0.01435153757869215;
    // -0.002476919057093277, 0.9999950635660482, 0.001933317232098841, 0.01763167311365072;
    // -0.003087624753322628, -0.001940961752784845, 0.9999933495983145, -0.6557104889323022;
    // 0, 0, 0, 1]

    // cv::namedWindow("Road facing camera", WINDOW_AUTOSIZE);// Create a window for display.
    cv::namedWindow("Trajectory", WINDOW_AUTOSIZE);// Create a window for display.

    cv::Mat traj = cv::Mat::zeros(800, 800, CV_8UC3);

    // for문으로 전체 처리

    string filename_format = "%06d.png";

    for (int Frame = 2; Frame <= num_images; Frame++){
        // pred_lr을 이전 timestamp 프레임으로 저장
        pred_l = curr_l.clone();
        pred_r = curr_r.clone();

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

        calculate_disparity(pred_l, pred_r, disparity_p);
        calculate_depth(disparity_p, f_x, baseline, depth);

        std::vector<cv::KeyPoint> vToDistributeKeys_curr, vToDistributeKeys_pred;

        // 1. 패치로 나눠서 특징 추출 후 매칭하는 방법
        // ORBextract(pred_l, vToDistributeKeys_pred, desc_pred, nfeatures);
        // ORBextract(curr_l, vToDistributeKeys_curr, desc_curr, nfeatures);
        // 2. 패치로 나누지 않고 특징 추출하는 방법
        orb->detectAndCompute(pred_l, cv::Mat(), vToDistributeKeys_pred, desc_pred);
        orb->detectAndCompute(curr_l, cv::Mat(), vToDistributeKeys_curr, desc_curr);

        Matcher_orb->match(desc_pred, desc_curr, matches);
        sort(matches.begin(), matches.end());
        const int match_size = matches.size();
        std::vector<cv::DMatch> good_matches(matches.begin(), matches.begin() + (int)(match_size * 0.15f));
        std::vector<cv::DMatch> filtered_matches;

        std::vector<Point2f> points_t, points_tplus1;
        double avg_depth = 0;
        for (size_t i = 0; i < good_matches.size(); i++) {
            float depth_value = depth.at<float>(vToDistributeKeys_pred[good_matches[i].queryIdx].pt);
            if (depth_value > 100 || depth_value <= 0) {continue;} // depth 값이 너무 크면 이상치(outlier)로 판단하여 제외함
            if (good_matches[i].distance < 0.8) {continue;}
            points_t.push_back(vToDistributeKeys_pred[good_matches[i].queryIdx].pt);
            points_tplus1.push_back(vToDistributeKeys_curr[good_matches[i].trainIdx].pt);
            filtered_matches.push_back(good_matches[i]);
            avg_depth += depth_value;
        }

        std::vector<Point3f> points_3d;
        for (int i = 0; i < points_t.size(); i++) {
            float depth_value = depth.at<float>(points_t[i]);
            float x = depth_value * (points_t[i].x - c_x) / f_x;
            float y = depth_value * (points_t[i].y - c_y) / f_y;
            Point3f point_3d(x, y, depth_value);
            points_3d.push_back(point_3d);
        }


        cv::Mat est_R, est_T, R_matrix;
        cv::solvePnPRansac(points_3d, points_tplus1, intrinsic, dist, est_R, est_T, false, 500);
        cv::Rodrigues(est_R, R_matrix);

        cv::Mat curr_transform = cv::Mat::eye(4, 4, CV_64F);
        R_matrix.copyTo(curr_transform(cv::Rect(0, 0, 3, 3)));
        est_T.copyTo(curr_transform(cv::Rect(3, 0, 1, 3)));

        Transform = Transform * curr_transform;
        cout << "Frame : " << Frame << '\n' << Transform << endl;

        int x = int(Transform.at<double>(0,3)) + 400;
        int y = int(Transform.at<double>(2,3)) + 700;
        circle(traj, Point(x, y), 1, CV_RGB(255, 0, 0), 2);

        rectangle(traj, Point(10, 30), Point(550, 50), CV_RGB(0, 0, 0), cv::FILLED);

        // 매칭점 그리기
        cv::Mat img_matches, img_filtered_matches;
        // cv::drawMatches(pred_l, vToDistributeKeys_pred, curr_l, vToDistributeKeys_curr, good_matches, img_matches);
        cv::drawMatches(pred_l, vToDistributeKeys_pred, curr_l, vToDistributeKeys_curr, filtered_matches, img_filtered_matches);

        // 결과 이미지 출력
        // cv::imshow("Good Matches", img_matches);
        cv::imshow("Filtered Matches", img_filtered_matches);
        // imshow("Road facing camera", curr_l);
        imshow("Trajectory", traj);
        cout << "Before : " << good_matches.size() << " | After : " << filtered_matches.size() << '\n';
        cout << "Avg_detph : " << avg_depth / filtered_matches.size() << '\n';
        waitKey(1);
    }

    return 0;
}