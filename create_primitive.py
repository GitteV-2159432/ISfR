import sapien
import numpy as np

def box(scene: sapien.Scene, pose: sapien.Pose, half_size, color=None, name="", is_kinematic=False) -> sapien.Entity:
    half_size = np.array(half_size)
    builder: sapien.ActorBuilder = scene.create_actor_builder()
    builder.add_box_collision(half_size=half_size)
    builder.add_box_visual(half_size=half_size, material=color)
    if is_kinematic:
        box = builder.build_kinematic(name=name)
    else:
        box: sapien.Entity = builder.build(name=name)
    box.set_pose(pose)
    return box


def sphere(scene: sapien.Scene, pose: sapien.Pose, radius, color=None, name="", is_kinematic=False) -> sapien.Entity:
    builder = scene.create_actor_builder()
    builder.add_sphere_collision(radius=radius)
    builder.add_sphere_visual(radius=radius, material=color)
    if is_kinematic:
        sphere = builder.build_kinematic(name=name)
    else:
        sphere: sapien.Entity = builder.build(name=name)
    sphere.set_pose(pose)
    return sphere


def capsule(scene: sapien.Scene, pose: sapien.Pose, radius, half_length, color=None, name="", is_kinematic=False) -> sapien.Entity:
    builder = scene.create_actor_builder()
    builder.add_capsule_collision(radius=radius, half_length=half_length)
    builder.add_capsule_visual(radius=radius, half_length=half_length, material=color)
    if is_kinematic:
        capsule = builder.build_kinematic(name=name)
    else:
        capsule: sapien.Entity = builder.build(name=name)
    capsule.set_pose(pose)
    return capsule