import numpy as np
import open3d as o3d
import os

def find_transformation(source, target, trans_init):
    threshold = 0.2
    transformation = o3d.pipelines.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane()
    ).transformation
    
    return transformation

def get_pcd_from_numpy(pcd_np):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pcd_np[:, :3])
    return pcd

class loaderKITTI:
    
    def __init__(self, dataset_path):
        self.folder_path = dataset_path
        self.pcd_list = os.listdir(self.folder_path)
        self.pcd_list.sort()
    
    def length(self):
        return len(self.pcd_list)
    
    def get_item(self, ind):
        path = os.path.join(self.folder_path, self.pcd_list[ind])
        pcd = o3d.io.read_point_cloud(path)
        return pcd
      

class OdometryEstimator:
    
    def __init__(self):
        self.threshold = 0.2
        self.last_position = np.eye(4)
        
    def transformation(self, source, target):
        # source.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        # target.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        
        transformation = o3d.pipelines.registration.registration_icp(
            source, target, self.threshold, self.last_position,
            o3d.pipelines.registration.TransformationEstimationPointToPoint()
            # o3d.pipelines.registration.TransformationEstimationPointToPlane()
        ).transformation
        
        # self.last_position = transformation @ self.last_position

        return transformation
    
    def append_pcd(self, pcd):
        pass
    
    def self_edge_correspondences(self, sharp_points):
        pass
    
    def find_surface_correspondences(self, flat_points, pcd):
        pass
    
    

# if __name__ == "__main__":
#     dataset_path = "./dataset/KITTI/00/velodyne"
#     loader = loaderKITTI(dataset_path)
#     odometry = OdometryEstimator()
    
#     global_transform = np.eye(4)
#     pcds = []
    
#     for i in range(loader.length()):
#         pass

import copy

def remove_outliers(pcd):
    pcd, inliers = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    return pcd

def only_objects(pcd):
    plane, inliers = pcd.segment_plane(distance_threshold=0.3, ransac_n=3, num_iterations=100)
    objects = pcd.select_by_index(inliers, invert=True)
    return objects

if __name__ == "__main__":
    dataset_path = "./dataset/KITTI/00/velodyne_pcd"
    loader = loaderKITTI(dataset_path)
    odometry = OdometryEstimator()
    
    source = loader.get_item(0).voxel_down_sample(voxel_size=0.2)
    target = loader.get_item(1).voxel_down_sample(voxel_size=0.2)
    
    source.paint_uniform_color([1,0,0])
    target.paint_uniform_color([0,0,1])
    
    
    # 이상치 제거
    source = remove_outliers(source)
    target = remove_outliers(target)
    
    # 도로 제거
    source = only_objects(source)
    target = only_objects(target)
    
    
    
    # ICP 매칭
    ori = copy.deepcopy(source).paint_uniform_color([0,1,0])
    for _ in range(10):
        transformation = odometry.transformation(source, target)
        print(transformation)
        source.transform(transformation)
        o3d.visualization.draw_geometries([source, target, ori])
    