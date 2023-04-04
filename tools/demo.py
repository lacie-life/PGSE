import matplotlib.pyplot as plt
import cv2, collections, numpy, math, pandas, scipy.integrate
import skeletor as sk
import open3d as o3d
import numpy as np
import pc_skeletor
from pc_skeletor import skeletor
from pc_skeletor.download import Dataset


# TODO: STUPID CONVERSION
if __name__ == "__main__":
    print("Hello")

    mesh = o3d.io.read_point_cloud("/home/lacie/Github/PGSE/tools/pc_skeletor/data/02_sceleton.ply")
    o3d.visualization.draw_geometries([mesh])


