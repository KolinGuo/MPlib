import numpy as np
import sapien.core as sapien
from demo_setup import DemoSetup

import mplib


class PlanningDemo(DemoSetup):
    def __init__(self):
        super().__init__()
        self.setup_scene()
        self.load_robot()
        self.setup_planner()

        # Set initial joint positions
        init_qpos = [
            0,
            0.19634954084936207,
            0.0,
            -2.617993877991494,
            0.0,
            2.941592653589793,
            0.7853981633974483,
            0,
            0,
        ]
        self.robot.set_qpos(init_qpos)

        # table top
        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.4, 0.4, 0.025])
        builder.add_box_visual(half_size=[0.4, 0.4, 0.025])
        self.table = builder.build_kinematic(name="table")
        self.table.set_pose(sapien.Pose([0.56, 0, -0.025]))

        # boxes
        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.02, 0.02, 0.06])
        render_material = self.renderer.create_material()
        render_material.base_color = (1, 0, 0, 1.0)
        builder.add_box_visual(half_size=[0.02, 0.02, 0.06], material=render_material)
        self.red_cube = builder.build(name="red_cube")
        self.red_cube.set_pose(sapien.Pose([0.7, 0, 0.06]))

        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.04, 0.04, 0.005])
        render_material = self.renderer.create_material()
        render_material.base_color = (0, 1, 0, 1.0)
        builder.add_box_visual(half_size=[0.04, 0.04, 0.005], material=render_material)
        self.green_cube = builder.build(name="green_cube")
        self.green_cube.set_pose(sapien.Pose([0.4, 0.3, 0.005]))

        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.05, 0.2, 0.1])
        render_material = self.renderer.create_material()
        render_material.base_color = (0, 0, 1, 1.0)
        builder.add_box_visual(half_size=[0.05, 0.2, 0.1], material=render_material)
        self.blue_cube = builder.build(name="blue_cube")
        self.blue_cube.set_pose(sapien.Pose([0.55, 0, 0.1]))

    def add_point_cloud(self):
        import trimesh

        box = trimesh.creation.box([0.1, 0.4, 0.2])
        points, _ = trimesh.sample.sample_surface(box, 1000)
        points += [0.55, 0, 0.1]
        self.planner.update_point_cloud(points)

    def demo(self, with_screw=True, use_point_cloud=True, use_attach=True):
        pickup_pose = [0.7, 0, 0.12, 0, 1, 0, 0]
        delivery_pose = [0.4, 0.3, 0.13, 0, 1, 0, 0]

        if use_point_cloud:
            self.add_point_cloud()

        pickup_pose[2] += 0.2
        # no attach since nothing picked up yet
        self.move_to_pose(pickup_pose, with_screw)
        self.open_gripper()
        pickup_pose[2] -= 0.12
        # no attach since nothing picked up yet
        self.move_to_pose(pickup_pose, with_screw)
        self.close_gripper()
        # Set planner robot qpos to allow auto-detect touch_links
        self.planner.robot.set_qpos(self.robot.get_qpos(), True)

        if use_attach:
            self.planner.update_attached_box(
                [0.04, 0.04, 0.12], [0, 0, 0.14, 1, 0, 0, 0]
            )

        pickup_pose[2] += 0.12
        self.move_to_pose(pickup_pose, with_screw)
        delivery_pose[2] += 0.2
        self.move_to_pose(delivery_pose, with_screw)
        delivery_pose[2] -= 0.12
        self.move_to_pose(delivery_pose, with_screw)
        self.open_gripper()
        delivery_pose[2] += 0.12
        if use_attach:
            ret = self.planner.detach_object(
                f"robot_{self.planner.move_group_link_id}_box", also_remove=True
            )
            assert ret, "object is not attached"
        self.move_to_pose(delivery_pose, with_screw)


if __name__ == "__main__":
    demo = PlanningDemo()
    demo.demo(False, True, True)
