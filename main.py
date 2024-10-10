import sapien
from sapien import Pose
import numpy as np
import cv2
import test_driver
import paho.mqtt.client as mqtt
import time


BROKER = "localhost"  
PORT = 1883
VELOCITY_TOPIC = "robot/desired_velocity"  


mqtt_client = mqtt.Client()

def on_connect(client, userdata, flags, rc):
    print(f"Connected to MQTT broker with result code: {rc}")


mqtt_client.on_connect = on_connect
mqtt_client.connect(BROKER, PORT, 60)
mqtt_client.loop_start()



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


mqtt_client.loop_stop()
