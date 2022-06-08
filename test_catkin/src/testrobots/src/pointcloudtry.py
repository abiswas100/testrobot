import numpy as np 
import open3d as o3d

pcd = o3d.io.read_point_cloud("/home/tran/testrobot/test_catkin/PointCloud.pcd")
out_arr = np.asarray(pcd.points)  
# print("output array from input list : ", out_arr)

print(out_arr.shape)

# print("Downsample the point cloud with a voxel of 0.05")
downpcd = pcd.voxel_down_sample(voxel_size=0.1)
out_arr = np.asarray(downpcd.points)  
print("output array from input list : ", out_arr.shape)
o3d.visualization.draw_geometries([downpcd])

# box = o3d.geometry.create_mesh_box()

# o3d.visualization.draw_geometries([pcd])