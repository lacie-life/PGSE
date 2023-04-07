import open3d as o3d
import numpy as np

from pc_skeletor import SLBC, LBC

if __name__ == "__main__":

    # Laplacian-based Contraction
    lbc = LBC(point_cloud="/home/lacie/Github/PGSE/data/ver2.ply",
              init_contraction=2,
              init_attraction=0.5,
              down_sample=0.03)
    lbc.extract_skeleton()
    # lbc.extract_topology()
    lbc.visualize()
    lbc.show_graph(lbc.skeleton_graph, fig_size=(30, 30))
    lbc.show_graph(lbc.topology_graph)
    lbc.save('./output')
    # lbc.animate(init_rot=np.asarray([[1, 0, 0], [0, 0, 1], [0, 1, 0]]), steps=500, output='./output')
    # lbc.animate_contracted_pcd(init_rot=np.asarray([[1, 0, 0], [0, 0, 1], [0, 1, 0]]), steps=300, output='./output')
    lbc.animate_topology(init_rot=np.asarray([[1, 0, 0], [0, 0, 1], [0, 1, 0]]), steps=300, output='./output')




