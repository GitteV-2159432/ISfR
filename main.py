import sapien
from sapien import Pose
import numpy as np
import cv2
import test_environment
import test_driver  
import lidar

def points_to_image(points, img_size=(800, 800), scale=10):
    """Project 3D points to a 2D plane for visualization."""
    img = np.zeros(img_size + (3,), dtype=np.uint8)

    for point in points:
        x, y, z = point

        img_x = int(img_size[0] / 2 + y * scale)
        img_y = int(img_size[1] / 2 - x * scale)

        if 0 <= img_x < img_size[0] and 0 <= img_y < img_size[1]: 
            cv2.circle(img, (img_x, img_y), radius=2, color=(0, 255, 0), thickness=-1)
        else:
            print("Scale too large")

    return img
def main():
    scene = sapien.Scene()
    scene.set_timestep(0.1)
    
    scene.set_ambient_light([0.5, 0.5, 0.5])
    scene.add_directional_light([0, 1, -1], [0.5, 0.5, 0.5], shadow=True)

    viewer = scene.create_viewer()
    ground_material = scene.create_physical_material(0.4, 0.2, 0)
    ground = scene.add_ground(0, True, ground_material)

    viewer.set_camera_xyz(x=0, y=0, z=0)
    viewer.set_camera_rpy(r=0, p=-np.arctan2(2, 2), y=0)
    viewer.window.set_camera_parameters(near=0.05, far=100, fovy=1)
    scene_config = sapien.SceneConfig()
    

    
    driver = test_driver.Driver(scene, viewer)  
    
    while not viewer.closed:
       
        scene.step()     
        scene.update_render()  
   
        viewer.render()  
        driver.update()
if __name__ == "__main__":
    main()
