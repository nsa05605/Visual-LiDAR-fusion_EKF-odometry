#!/bin/bash

# 정보 입력
dataset_path="./dataset"
file_id="1jedKIILg7Dcd93Odyrlzjz8LAXN_2e-5"
file_name="KITTI.zip"

# =====================================================

# 입력 정보로 필요한 변수 설정
file_path="${dataset_path}/${file_name}"

# 데이터셋 경로 폴더 생성
echo "(0/3) 데이터셋 폴더 생성 : ${dataset_path}"
mkdir -p ${dataset_path}

# 데이터셋 압축 파일 다운로드
echo "(1/3) 데이터셋 파일 다운로드 : ${file_path}"
if [ ! -f "$file_path" ]; then
    gdown https://drive.google.com/uc?id=${file_id} -O ${file_path}
fi

# 파일 압축 해제
echo "(2/3) 데이터셋 파일 압축 해제"
unzip ${file_path} -d ./dataset

# 종료
echo "(3/3) 데이터셋 다운로드 완료"
