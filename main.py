from scipy.spatial import cKDTree
import open3d as o3d
import numpy as np
from sys import argv

filename = argv[1]

def PointCloud(filename: str):
    pcd = o3d.io.read_point_cloud(filename, format="xyz")
    points = np.asarray(pcd.points)
    return pcd
def Mesh(filename: str):
    mesh = o3d.io.read_triangle_mesh(filename)
    vertices = mesh.vertices
    return mesh


mesh = Mesh(filename + ".obj")
points_mesh = np.asarray(mesh.vertices)

cloud = PointCloud(filename + ".xyz")
points_cloud = np.asarray(cloud.points)

tree = cKDTree(points_mesh)

distances, indices = tree.query(points_cloud)

max_distance = np.max(distances)
average_distance = np.mean(distances)

print("Максимальное расстояние между ближайшими точками двух облаков: ", max_distance)
print("Среднее расстояние между ближайшими точками двух облаков: ", average_distance)

min_x = np.min(points_mesh[:,0])
max_x = np.max(points_mesh[:,0])

min_y = np.min(points_mesh[:,1])
max_y = np.max(points_mesh[:,1])

min_z = np.min(points_mesh[:,2])
max_z = np.max(points_mesh[:,2])

a = max_x - min_x
b = max_y - min_y
c = max_z - min_z

l = (a * b * c)**(1 / 3)

metric_max = max_distance / l
metric_average = average_distance / l

print("Метрика 1: ", metric_max)
print("Метрика 2: ", metric_average)
o3d.visualization.draw_geometries([cloud])
o3d.visualization.draw_geometries([mesh])