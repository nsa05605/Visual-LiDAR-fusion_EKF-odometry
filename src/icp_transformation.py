import numpy as np
import open3d as o3d
import os
from tqdm.auto import tqdm

bin_path = "./dataset/KITTI/00/velodyne"
pcd_path = "./dataset/KITTI/00/velodyne_pcd"

pcd1 = o3d.io.read_point_cloud(pcd_path+"/000000.pcd")
pcd2 = o3d.io.read_point_cloud(pcd_path+"/000001.pcd")

threshold = 0.02
reg_p2p = o3d.pipelines.registration.registration_icp(
    pcd1, pcd2, threshold, np.eye(4),
    o3d.pipelines.registration.TransformationEstimationPointToPoint()
)

print("Transformation is:")
print(reg_p2p.transformation)

# 회전 및 이동 추출
rotation_matrix = reg_p2p.transformation[:3, :3]
translation_vector = reg_p2p.transformation[:3, 3]
print("Rotation matrix:")
print(rotation_matrix)
print("Translation vector:")
print(translation_vector)
    