import pyvista as pv
import numpy as np
import point_cloud_helper as pch
import icp as icp

bunny_target1 = np.load("src/slam/test_data/bunny.npy")
bunny_target2 = np.load("src/slam/test_data/bunny.npy")
bunny = np.load("src/slam/test_data/bunny.npy")
dog = np.load("src/slam/test_data/dog.npy")

bunny_target1 = pch.downsample_point_cloud(bunny_target1, 0.15)
bunny_target2 = pch.downsample_point_cloud(bunny_target2, 0.15)
bunny = pch.downsample_point_cloud(bunny, 0.15)
dog = pch.downsample_point_cloud(dog, 0.03)

dog = pch.apply_scale(dog, .0025)

bunny_target2 = pch.apply_translation(bunny_target2, [.3, 0, 0])
bunny = pch.apply_translation(bunny, [0, .3, 0])
dog = pch.apply_translation(dog, [.3, .5, 0])

pd_bunny_target1 = pv.PolyData(bunny_target1)
pd_bunny_target2 = pv.PolyData(bunny_target2)
pd_bunny = pv.PolyData(bunny)
pd_dog = pv.PolyData(dog)

exit = False
def quit(): 
    global exit
    exit = True

running = False
def run():
    global running
    running = True
    
plotter = pv.Plotter()
plotter.add_points(pd_bunny_target1, color="blue")
plotter.add_points(pd_bunny_target2, color="blue")
plotter.add_points(pd_bunny, color="red")
plotter.add_points(pd_dog, color="red")
plotter.add_key_event("q", quit)
plotter.add_key_event("r", run)
text_error1 = plotter.add_text("Total error (bunny):", position="upper_left", color="black", font_size=9)
text_error2 = plotter.add_text("\nTotal error (dog):", position="upper_left", color="black", font_size=9)
plotter.show(interactive=False, interactive_update=True, before_close_callback=quit)

while exit is False:
    if running:
        bunny, transformation_matrix, mean_error = icp.icp(bunny, bunny_target1, 1)
        pd_bunny.points = bunny
        text_error1.set_text(text=f"mean Error (bunny): {round(mean_error, 6)}", position="upper_left")

        dog, transformation_matrix, mean_error = icp.icp(dog, bunny_target2, 1)
        pd_dog.points = dog
        text_error2.set_text(text=f"\nmean Error (dog): {round(mean_error, 6)}", position="upper_left")
        pass

    plotter.update()
