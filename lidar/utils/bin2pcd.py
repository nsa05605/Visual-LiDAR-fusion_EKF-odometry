import os
import open3d as o3d
from tqdm.auto import tqdm
import numpy as np

def bin2pcd(bin_path, pcd_path):
    '''
    bin_path : binary 파일들이 보관된 폴더
    pcd_path : 생성될 pcd 파일들을 보관할 폴더
    '''
    
    # pcd 파일들을 저장할 폴더 생성
    os.makedirs(pcd_path, exist_ok=True)
    
    
    # 포인트 클라우드 클래스 생성
    pcd = o3d.geometry.PointCloud()
    
    # 변환시킬 파일들의 목록을 불러온다.
    files = sorted(os.listdir(bin_path))
    
    # 파일 목록들을 순차적으로 변환
    for file in tqdm(files):
        # bin 파일의 데이터를 불러온다.
        file_path = os.path.join(bin_path, file)
        points = np.fromfile(file_path, dtype=np.float32).reshape(-1, 4)[:, :3]
    
        # 포인트 클라우드 클래스에 할당
        pcd.points = o3d.utility.Vector3dVector(points)
        
        # pcd 파일 명 지정
        pcd_file = os.path.splitext(file)[0] + ".pcd"
        pcd_file = os.path.join(pcd_path, pcd_file)
        
        # pcd 파일 저장
        o3d.io.write_point_cloud(pcd_file, pcd)
