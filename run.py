import open3d as o3d
from tqdm.auto import tqdm
import numpy as np


from lidar.dataset import KITTI
from lidar.utils import transformation


bin_path = "./dataset/KITTI/00/velodyne"
pcd_path = "./dataset/KITTI/00/velodyne_pcd"

voxel_size = 0.2
        
def main():
    dataset = KITTI(bin_path, pcd_path, voxel_size)
    
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    
    pre_pcd_vis, pre_pcd = dataset[0]
    vis.add_geometry(pre_pcd_vis, reset_bounding_box=True)
    
    car = np.eye(4)
    
    for cur_pcd_vis, cur_pcd in tqdm(dataset):
        transform_matrix = transformation(cur_pcd, pre_pcd)
        car = car @ transform_matrix
        
        cur_pcd_vis.transform(car)
        
        vis.add_geometry(cur_pcd_vis, reset_bounding_box=False)
        vis.remove_geometry(pre_pcd_vis, reset_bounding_box=False)
        
        
        # Ending 부분
        vis.poll_events()
        vis.update_renderer()
        
        pre_pcd = cur_pcd
        pre_pcd_vis = cur_pcd_vis
    
    vis.destroy_winodw()
    

if __name__ == '__main__':
    main()