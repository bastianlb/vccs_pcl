import open3d as o3d
import open3d.visualization.gui as gui


if __name__ == "__main__":
    pcd = o3d.io.read_point_cloud("./vccs_pcl.pcd")
    app = gui.Application.instance
    app.initialize()
    vis = o3d.visualization.O3DVisualizer("Open3D - 3D Text", 1024, 768)
    vis.show_settings = True
    vis.add_geometry("pcd", pcd)

    app.add_window(vis)
    app.run()



