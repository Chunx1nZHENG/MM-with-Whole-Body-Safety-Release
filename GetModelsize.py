import rospkg
import os
from stl import mesh

# 获取ROS包的路径
rospack = rospkg.RosPack()
package_path = rospack.get_path('swerve_description')



# 计算模型的尺寸：最大点和最小点之差
def get_size(the_mesh):
    min_x, max_x = min(the_mesh.x.flatten()), max(the_mesh.x.flatten())
    min_y, max_y = min(the_mesh.y.flatten()), max(the_mesh.y.flatten())
    min_z, max_z = min(the_mesh.z.flatten()), max(the_mesh.z.flatten())

    length = max_x - min_x
    width = max_y - min_y
    height = max_z - min_z

    return length, width, height

stl_path = os.path.join(package_path, 'meshes', 'holonomic', 'base_link.STL')
the_mesh = mesh.Mesh.from_file(stl_path)
(length, width, height) = get_size(the_mesh)
print(f"Size of base_link :Length: {length}, Width: {width}, Height: {height}")

# 构建STL文件的路径
stl_path = os.path.join(package_path, 'meshes', 'jaco', 'bicep_link.STL')
the_mesh = mesh.Mesh.from_file(stl_path)
(length, width, height) = get_size(the_mesh)
print(f"Size of bicep_link :Length: {length}, Width: {width}, Height: {height}")

# 构建STL文件的路径
stl_path = os.path.join(package_path, 'meshes', 'jaco', 'base_link.STL')
the_mesh = mesh.Mesh.from_file(stl_path)
(length, width, height) = get_size(the_mesh)
print(f"Size of x_arm_link :Length: {length}, Width: {width}, Height: {height}")

stl_path = os.path.join(package_path, 'meshes', 'jaco', 'forearm_link.STL')
the_mesh = mesh.Mesh.from_file(stl_path)
(length, width, height) = get_size(the_mesh)
print(f"Size of forearm_link :Length: {length}, Width: {width}, Height: {height}")

stl_path = os.path.join(package_path, 'meshes', 'jaco', 'spherical_wrist_2_link.STL')
the_mesh = mesh.Mesh.from_file(stl_path)
(length, width, height) = get_size(the_mesh)
print(f"Size of spherical_wrist_2_link :Length: {length}, Width: {width}, Height: {height}")

