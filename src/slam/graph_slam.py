import pyvista as pv
import numpy as np
import test_point_clouds as tpc
import icp as icp

point_cloud = pv.read("src/slam/test_data/bunny.ply").points
point_cloud = tpc.downsample_point_cloud(point_cloud, .01)
point_cloud_transformed = tpc.apply_translation_and_rotation(point_cloud, [0, .3, 0], tpc.rotation_matrix_from_euler_angles(np.radians(30), np.radians(0), np.radians(0)))
# transformed_point_cloud = tpc.add_noise_to_point_cloud(transformed_point_cloud, 0.005)

pd_point_cloud = pv.PolyData(point_cloud)
pd_point_cloud_transformed = pv.PolyData(point_cloud_transformed)

running = True
def quit(): 
    global running
    running = False

def run():
    global point_cloud_transformed, point_cloud
    point_cloud_transformed, transformation_matrix = icp.icp(point_cloud_transformed, point_cloud, 1)
    pd_point_cloud_transformed.points = point_cloud_transformed

plotter = pv.Plotter()
plotter.add_points(pd_point_cloud, color="blue")
plotter.add_points(pd_point_cloud_transformed, color="red")
plotter.add_key_event("q", quit)
plotter.add_key_event("r", run)
plotter.show(interactive=False, interactive_update=True, before_close_callback=quit)

while running:
    plotter.update()
