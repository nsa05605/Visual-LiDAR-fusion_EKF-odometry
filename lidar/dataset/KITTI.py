import os
import copy

from ..utils import bin2pcd, get_pcd

class KITTI:
    
    def __init__(self, bin_path, pcd_path, voxel_size=0.2):
        self.bin_path = bin_path
        self.pcd_path = pcd_path
        self.voxel_size = voxel_size
        
        # 만약 .pcd 확장자의 데이터를 저장한 폴더가 존재하지 않을 경우에만 변환
        if not os.path.exists(pcd_path):
            print(".bin을 .pcd 파일로 변환 중...")
            bin2pcd(bin_path, pcd_path)
        
        self.files = sorted(os.listdir(pcd_path))
    
    def __len__(self):
        return len(self.files)
    
    def __iter__(self):
        iter = copy.copy(self)
        iter.current = 0
        return iter

    def __getitem__(self, idx):
        if idx < len(self):
            file = self.files[idx]
            file = os.path.join(self.pcd_path, file)
            pcd_ori, pcd = get_pcd(file, self.voxel_size)
            return pcd_ori, pcd
        else:
            raise IndexError
    
    def __next__(self):
        if not hasattr(self, 'current'):
            self.current = 0
        
        if self.current < len(self):
            ret = self[self.current]
            self.current += 1
            return ret
        else:
            raise StopIteration



    