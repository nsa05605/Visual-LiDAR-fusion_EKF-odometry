import numpy as np
import open3d as o3d
import os
import copy

class loaderKITTI:
    
    def __init__(self, dataset_path):
        self.folder_path = dataset_path
        self.pcd_list = os.listdir(self.folder_path)
        self.pcd_list.sort()
        
        self.down_sampling_flag = False
        self.delete_road_flag = False
        self.remove_outlier_flag = False
    
    def length(self):
        return len(self.pcd_list)
    
    def get_item(self, ind):
        path = os.path.join(self.folder_path, self.pcd_list[ind])
        pcd = o3d.io.read_point_cloud(path)
        
        if self.down_sampling_flag:
            pcd = pcd.voxel_down_sample(voxel_size=0.2)
        
        if self.remove_outlier_flag:
            pcd = self.remove_outliers(pcd)
        
        if self.delete_road_flag:
            pcd = self.only_objects(pcd)
            
        return pcd
    
    def only_objects(self, pcd):
        plane, inliers = pcd.segment_plane(distance_threshold=0.3, ransac_n=3, num_iterations=100)
        objects = pcd.select_by_index(inliers, invert=True)
        return objects
    
    def remove_outliers(self, pcd):
        pcd, inliers = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        return pcd

    def down_sampling(self, voxel_size=0.2):
        self.voxel_down = True
    
    def delete_road(self):
        self.delete_road_flag = True
    
    def remove_outlier(self):
        self.remove_outlier_flag = True
      

class OdometryEstimator:
    
    def __init__(self):
        self.threshold = 1.0
        self.last_position = np.eye(4)
        
    def transformation(self, source, target):
        # source.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        # target.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        
        transformation = o3d.pipelines.registration.registration_icp(
            source, target, self.threshold, np.eye(4),
            o3d.pipelines.registration.TransformationEstimationPointToPoint()
            # o3d.pipelines.registration.TransformationEstimationPointToPlane()
        ).transformation
        
        self.last_position = self.last_position @ transformation

        return transformation
    
    def append_pcd(self, pcd):
        pass
    
    def self_edge_correspondences(self, sharp_points):
        pass
    
    def find_surface_correspondences(self, flat_points, pcd):
        pass

class Car:
    def __init__(self):
        self.car = o3d.geometry.TriangleMesh.create_coordinate_frame()
        self.last_position = np.eye(4)
        self.position_list = [self.last_position]
    
    def get_car(self, clone=False):
        return copy.deepcopy(self.car) if clone else self.car

    def transform(self, transformation):
        self.car.transform(transformation)
        self.last_position = self.last_position @ transformation
        self.position_list.append(self.last_position)


if __name__ == "__main__":
    dataset_path = "./dataset/KITTI/00/velodyne_pcd"
    loader = loaderKITTI(dataset_path)
    loader.down_sampling(voxel_size=0.2)
    loader.delete_road()
    loader.remove_outlier()
    
    source = loader.get_item(0)
    target = loader.get_item(0)
    
    odometry = OdometryEstimator()
    car = Car()
    
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(target)
    vis.add_geometry(car.get_car())
    
    for i in range(loader.length()):
        source.points = target.points
        target.points = loader.get_item(i).points
        
        transformation = odometry.transformation(source, target)
        
        car.transform(transformation)
        vis.add_geometry(car.get_car(clone=True))
        target.transform(odometry.last_position)
        
        vis.update_geometry(target)
        # vis.update_geometry(car.get_car())
        vis.poll_events()
        vis.update_renderer()
    
    vis.destroy_window()
        
        
    