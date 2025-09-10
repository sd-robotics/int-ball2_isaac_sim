from pxr import Gf, Sdf, UsdPhysics
from omni.usd import get_context
import omni.client
import omni.kit.async_engine
import omni.timeline
import omni.kit.viewport.utility as vu
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.api.robots import Robot
from isaacsim.core.utils.stage import (
    add_reference_to_stage, create_new_stage_async)
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.core.api import World
from isaacsim.sensors.physics import IMUSensor

import asyncio
import os
import numpy as np

# import rclpy
# from rclpy.executors import MultiThreadedExecutor
# from imu_publisher import setup_imu_ros_publishers, imu_sensor_publisher


async def create_scene(env_path: str, robot_path: str):
    World.clear_instance()
    # Start a fresh stage
    await create_new_stage_async()
    my_world = World(
        physics_dt=1.0 / 60.0,
        rendering_dt=1.0 / 60.0,
        stage_units_in_meters=1.0,
        backend="torch",
        device="cpu"
    )
    await my_world.initialize_simulation_context_async()
    await omni.kit.app.get_app().next_update_async()

    stage = get_context().get_stage()

    # Set gravity before adding the robot
    set_gravity(
        stage=stage,
        direction=(0.0, 0.0, 0.0),
        magnitude=0.0
    )
    
    # import robot
    robot_name = "INTBALL2"
    intball_prim_path = f'/World/{robot_name}'
    # load the robot USD file
    add_reference_to_stage(robot_path, intball_prim_path)
    robot = Robot(
        prim_path=intball_prim_path,
        name=robot_name,
        position=(10.88492, -3.53022, 4.07888),  # This pose is base on the position of kibou_body in the KIBOU.usd
        orientation=(-0.001975, 0.707104, -0.707104, 0.001975), # (w,x,y,z)
        scale=(0.01, 0.01, 0.01),
    )

    print(f"__{robot_name} is onboard!__")
    await omni.kit.app.get_app().next_update_async()

    arti_view = SingleArticulation(intball_prim_path)
    my_world.scene.add(arti_view)
    await my_world.reset_async(soft=False)

    await omni.kit.app.get_app().next_update_async()
    add_reference_to_stage(env_path, '/World/ISS_frame')
    
    # Set camera to ISS camera as default
    vp_api = vu.get_active_viewport()
    vp_api.camera_path = '/World/ISS_frame/ISS_Camera'


    print("__WELCOME TO KIBOU ISS!__")


def set_gravity(stage, direction=(0.0, 0.0, -1.0), magnitude=0.0):
    """
    Set the gravity vector and magnitude for the physics scene.
    """
    physics_path = Sdf.Path("/physicsScene")

    # Define (or get) the physics scene
    physics_scene = UsdPhysics.Scene.Get(stage, physics_path)
    if not physics_scene:
        physics_scene = UsdPhysics.Scene.Define(stage, physics_path)

    gravity_vec = Gf.Vec3f(*direction)
    physics_scene.CreateGravityDirectionAttr().Set(gravity_vec)
    physics_scene.CreateGravityMagnitudeAttr().Set(magnitude)

    print(f"Gravity set to direction {gravity_vec} with magnitude {magnitude}")
