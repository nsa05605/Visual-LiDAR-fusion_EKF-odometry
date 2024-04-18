# Implementation of visual-LiDAR odometry system using Extended Kalman Filter (EKF)

- 참여 : Jihoon Jung, Donghyun Lee, Beomsu Cho
- 역할
  - Stereo odometry : Jihoon Jung
  - LiDAR odometry : Beomsu Cho
  - Kalman filter : Donghyun Lee


### 데이터셋 다운로드

KITTI 데이터셋을 사용합니다.
다음의 경로를 통하여 수동으로 다운로드 받을 수 있습니다.
https://drive.google.com/file/d/1jedKIILg7Dcd93Odyrlzjz8LAXN_2e-5/view?usp=drive_link


### Requirements

unzip 패키지가 필요합니다.

```bash
sudo apt install unzip
```

python의 패키지 gdown이 필요합니다. (sudo 필수)

```bash
sudo pip install gdown
```

### 실행

다음의 명령으로 데이터셋을 다운로드 할 수 있습니다.

```bash
chmod +x download_dataset.sh
./download_dataset.sh
```



## 라이다 데이터 시각화

### Requirements

다음의 파이썬 패키지가 필요합니다.
- numpy
- open3d
- tqdm

```bash
pip install numpy open3d tqdm
```

### 실행

다음의 명령으로 라이다 데이터를 시각화할 수 있습니다.

```bash
python3 ./src/visualize_lidar.py
```


## Stereo Odometry

### 목표
: KITTI dataset을 사용한 stereo localization

**Input**  
  - stereo images
  - calibration data(intrinsic, extrinsic)

**Output**  
  - Localization results(=trajectory)

### 진행 상황

**2024/04/14**
  - Stereo odometry 초기 구현 완료
  - [구현 영상](https://www.notion.so/robotailab/e13c6fea80aa4be3b64b2a434bb7b0eb)