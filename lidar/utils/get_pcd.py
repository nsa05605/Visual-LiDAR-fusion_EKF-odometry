import open3d as o3d

def get_pcd_from_file(file):
    return o3d.io.read_point_cloud(file)

def get_pcd_voxel_down(pcd, voxel_size):
    return pcd.voxel_down_sample(voxel_size=voxel_size)

def remove_outliers(pcd):
    pcd, inliers = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    return pcd

def only_objects(pcd):
    plane, inliers = pcd.segment_plane(distance_threshold=0.3, ransac_n=3, num_iterations=100)
    objects = pcd.select_by_index(inliers, invert=True)
    return objects

def get_pcd(file, voxel_size=0.2):
    pcd_ori = get_pcd_from_file(file)
    pcd = get_pcd_voxel_down(pcd_ori, voxel_size)
    pcd = remove_outliers(pcd)
    pcd = only_objects(pcd)
    return pcd_ori, pcd