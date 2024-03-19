import numpy as np
import open3d as o3d

# 가상의 포인트 클라우드 데이터 생성
# 실제 데이터를 사용하는 경우, 여기서 데이터를 불러오는 코드로 대체
points = np.random.rand(10000, 3)  # 10000개의 포인트를 가진 3차원 numpy 배열

# Open3D의 포인트 클라우드 객체 생성
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)

# 포인트 클라우드 시각화
o3d.visualization.draw_geometries([pcd])
