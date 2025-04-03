import open3d as o3d
import numpy as np
import open3d as o3d

# # 读取 PCD 文件
# pcd = o3d.io.read_point_cloud("example.pcd")

# # 保存为 PLY 文件
# o3d.io.write_point_cloud("example.ply", pcd)

# 读取 PLY 文件
pcd = o3d.io.read_point_cloud("example.ply")

# 定义绕 Y 轴顺时针旋转的变换矩阵
angley = 0  # 旋转角度（弧度）
R = np.array([[np.cos(angley), 0, np.sin(angley)],
              [0, 1, 0],
              [-np.sin(angley), 0, np.cos(angley)]])

# 应用旋转变换
pcd.rotate(R)

# # 定义绕 X 轴顺时针旋转的变换矩阵
# anglex = 0.013  # 旋转角度（弧度）
# R = np.array([[1, 0, 0],
#               [0, np.cos(anglex), -np.sin(anglex)],
#               [0, np.sin(anglex), np.cos(anglex)]])

# # 应用旋转变换
# pcd.rotate(R)


# 保存为 PCD 文件
o3d.io.write_point_cloud("src/third_party/FAST_LIO/PCD/example.pcd", pcd)

print("PLY 转换为 PCD 完成，文件已保存为 example.pcd")
