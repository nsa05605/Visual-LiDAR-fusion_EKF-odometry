import open3d as o3d
import numpy as np

def transformation(pcd1, pcd2, threshold=100):
    transformation = o3d.pipelines.registration.registration_icp(
        pcd1, pcd2, threshold, np.eye(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint()
    ).transformation
    
    return transformation