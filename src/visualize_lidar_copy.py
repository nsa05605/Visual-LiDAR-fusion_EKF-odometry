import numpy as np
import open3d as o3d
import os
from tqdm.auto import tqdm

def icp_transformation(pcd1, pcd2):
    threshold = 0.02
    reg_p2p = o3d.pipelines.registration.registration_icp(
        pcd1, pcd2, threshold, np.eye(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint()
    )
    
    return reg_p2p

def bin2pcd(bin_path, pcd_path):

    os.makedirs(pcd_path, exist_ok=True)
    
    pcd = o3d.geometry.PointCloud()
    
    files = sorted(os.listdir(bin_path))
    for file in tqdm(files):
        file_path = os.path.join(bin_path, file)
        points = np.fromfile(file_path, dtype=np.float32).reshape(-1, 4)[:, :3]
    
        pcd.points = o3d.utility.Vector3dVector(points)
        
        pcd_file = os.path.splitext(file)[0] + ".pcd"
        pcd_file = os.path.join(pcd_path, pcd_file)
        
        o3d.io.write_point_cloud(pcd_file, pcd)
    

bin_path = "./dataset/KITTI/00/velodyne"
pcd_path = "./dataset/KITTI/00/velodyne_pcd"

# 만약 .pcd 확장자의 데이터가 존재하지 않을 경우 변환
if not os.path.exists(pcd_path):
    bin2pcd(bin_path, pcd_path)

if __name__ == "__main__":
    files = sorted(os.listdir(pcd_path))

    pcd = o3d.io.read_point_cloud(pcd_path+"/000000.pcd")
    pcd_before = o3d.io.read_point_cloud(pcd_path+"/000000.pcd")
    
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(pcd)
    
    for file in files:
        file_path = os.path.join(pcd_path, file)
        
        pcd_before = pcd.points
        pcd.points = o3d.io.read_point_cloud(file_path).points
        
        reg_p2p = icp_transformation(pcd_before, pcd)
        pcd.transform(reg_p2p.transformation)
        
        vis.update_geometry(pcd)
        vis.poll_events()
        vis.update_renderer()
    
    vis.destroy_window()
