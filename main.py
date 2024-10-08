import sapien
from sapien import Pose
import numpy as np
import environment


def main():
    scene = sapien.Scene()
    scene.set_timestep(1 / 100.0)

    viewer = scene.create_viewer()
    viewer.set_camera_xyz(x=-12, y=0, z=15)
    viewer.set_camera_rpy(r=0, p=-np.arctan2(2, 2), y=0)
    viewer.window.set_camera_parameters(near=0.05, far=100, fovy=1)
    
    #environment.create_wall_grid(scene=scene, grid_size=4, spacing=4)

    while not viewer.closed:
        scene.step()
        scene.update_render()

if __name__ == "__main__":
    main()