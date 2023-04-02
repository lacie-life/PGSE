import matplotlib.pyplot as plt
import cv2, collections, numpy, math, pandas, scipy.integrate
import skeletor as sk
import open3d as o3d
import numpy as np

# TODO: STUPID CONVERSION
if __name__ == "__main__":
    print("Hello")

    # Conver to mesh
    pcd = o3d.io.read_point_cloud("/home/lacie/Github/PhenomenalApp/test/corn_cropped_2.ply")
    pcd.estimate_normals()

    # estimate radius for rolling ball
    distances = pcd.compute_nearest_neighbor_distance()
    avg_dist = np.mean(distances)
    radius = 1.5 * avg_dist

    radii = [0.005, 0.01, 0.02, 0.04]

    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        pcd, o3d.utility.DoubleVector([radius, radius * 2]))

    # mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
    #     pcd, o3d.utility.DoubleVector(radii))

    # alpha = 0.03
    # print(f"alpha={alpha:.3f}")
    # mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
    # mesh.compute_vertex_normals()
    # o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)

    o3d.visualization.draw_geometries([mesh])
    o3d.io.write_triangle_mesh("mesh_2.ply", mesh)
