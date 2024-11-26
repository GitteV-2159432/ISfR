import numpy as np
import pyvista as pv
from typing import List, Tuple

def _polar2cartesian(polar_points: List[Tuple[float, float, float]]) -> np.ndarray:
    polar_points = np.array(polar_points)
    angles_horizontal, angles_vertical, distances = polar_points[:, 0], polar_points[:, 1], polar_points[:, 2]

    x = np.cos(angles_vertical) * np.cos(angles_horizontal) * distances
    y = np.cos(angles_vertical) * np.sin(angles_horizontal) * distances
    z = np.sin(angles_vertical) * distances

    return np.vstack((x, y, z)).T

def _draw_polar_coordinate_system(plotter: pv.Plotter):   
    circle = pv.Circle(radius=.1, resolution=20)
    plotter.add_mesh(circle, color=[1.0, 0.0, 0.0], opacity=.5)
    for radius in range(0, 31, 2):
        circle = pv.Circle(radius=radius, resolution=20)
        plotter.add_mesh(circle, color=[radius / 50.0] * 3, opacity=.1)
        plotter.add_actor(pv.Label(f'{radius}m', (0, radius, 0), size=10))

class PolarCoordinatesPlot():
    def __init__(self, title: str="Polar Plotter", color=[0.0, 1.0, 0.0]) -> None:
        self.pd_lidar = pv.PolyData(np.zeros((150, 3)))
        self.plotter = pv.Plotter()
        self.plotter.add_points(self.pd_lidar, color=color)
        self.plotter.show(auto_close=False, interactive_update=True, interactive=True)

        _draw_polar_coordinate_system(self.plotter)
        self.plotter.title = title
        self.plotter.camera.position = (0, 0, 50)
        self.plotter.camera.focal_point = (0, 0, 0)
        self.plotter.camera.up = (1.0, 0.0, 0.0)

    def update(self, points_polar: List[Tuple[float, float, float]]) -> None:
        points_cartesian = _polar2cartesian(points_polar)
        self.pd_lidar.points = points_cartesian
        self.plotter.update()