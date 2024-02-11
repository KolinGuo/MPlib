from __future__ import annotations

import numpy as np
from sapien import Entity, Pose, Scene
from sapien.physx import (
    PhysxArticulation,
    PhysxArticulationLinkComponent,
    PhysxCollisionShapeBox,
    PhysxCollisionShapeCapsule,
    PhysxCollisionShapeConvexMesh,
    PhysxCollisionShapeCylinder,
    PhysxCollisionShapePlane,
    PhysxCollisionShapeSphere,
    PhysxCollisionShapeTriangleMesh,
    PhysxRigidBaseComponent,
)
from transforms3d.euler import euler2quat

from ..pymp.articulation import ArticulatedModel
from ..pymp.fcl import Box, Capsule, CollisionObject, Convex, Cylinder
from ..pymp.planning_world import PlanningWorld
from .srdf_exporter import export_srdf
from .urdf_exporter import export_kinematic_chain_urdf


def convert_sapien_col_shape(
    component: PhysxRigidBaseComponent,
) -> list[CollisionObject]:
    """Converts a SAPIEN physx.PhysxRigidBaseComponent to an FCL CollisionObject
    Returns a list of collision_obj at their current poses.

    If the component is an articulation link, the returned collision_obj is at
    the shape's local_pose.
    Otherwise, the returned collision_obj is at the entity's global pose
    """
    shapes = component.collision_shapes
    if len(shapes) == 0:
        return []

    # NOTE: MPlib currently only supports 1 collision shape per object
    # TODO: multiple collision shapes
    assert len(shapes) == 1, (
        f"Should only have 1 collision shape, got {len(shapes)} shapes for "
        f"entity '{component.entity.name}'"
    )

    shape = shapes[0]
    if isinstance(component, PhysxArticulationLinkComponent):  # articulation link
        pose = shape.local_pose
    else:
        pose = component.entity.pose * shape.local_pose

    if isinstance(shape, PhysxCollisionShapeBox):
        collision_geom = Box(side=shape.half_size * 2)
    elif isinstance(shape, PhysxCollisionShapeCapsule):
        collision_geom = Capsule(radius=shape.radius, lz=shape.half_length * 2)
        # NOTE: physx Capsule has x-axis along capsule height
        # FCL Capsule has z-axis along capsule height
        pose = pose * Pose(q=euler2quat(0, np.pi / 2, 0))
    elif isinstance(shape, PhysxCollisionShapeConvexMesh):
        assert np.allclose(
            shape.scale, 1.0
        ), f"Not unit scale {shape.scale}, need to rescale vertices?"
        collision_geom = Convex(vertices=shape.vertices, faces=shape.triangles)
    elif isinstance(shape, PhysxCollisionShapeCylinder):
        collision_geom = Cylinder(radius=shape.radius, lz=shape.half_length * 2)
        # NOTE: physx Cylinder has x-axis along cylinder height
        # FCL Cylinder has z-axis along cylinder height
        pose = pose * Pose(q=euler2quat(0, np.pi / 2, 0))
    elif isinstance(shape, PhysxCollisionShapePlane):
        raise NotImplementedError(
            "Support for Plane collision is not implemented yet in mplib, "
            "need fcl::Plane"
        )
    elif isinstance(shape, PhysxCollisionShapeSphere):
        raise NotImplementedError(
            "Support for Sphere collision is not implemented yet in mplib, "
            "need fcl::Sphere"
        )
    elif isinstance(shape, PhysxCollisionShapeTriangleMesh):
        # NOTE: see mplib.pymp.fcl.Triangle
        raise NotImplementedError(
            "Support for TriangleMesh collision is not implemented yet."
        )
    else:
        raise TypeError(f"Unknown shape type: {type(shape)}")
    return [CollisionObject(collision_geom, pose.p, pose.q)]


class SapienPlanningWorld(PlanningWorld):
    def __init__(self, sim_scene: Scene, planned_articulation_names: list[str] = []):
        """
        Creates an mplib.pymp.planning_world.PlanningWorld from a sapien.Scene.

        :param planned_articulation_names: name of planned articulations.
        """
        super().__init__([], [])
        self._sim_scene = sim_scene

        articulations: list[PhysxArticulation] = sim_scene.get_all_articulations()
        actors: list[Entity] = sim_scene.get_all_actors()

        for articulation in articulations:
            urdf_str = export_kinematic_chain_urdf(articulation)
            srdf_str = export_srdf(articulation)

            # Get all links with collision shapes at local_pose
            collision_links = []  # [(link_name, [CollisionObject, ...]), ...]
            for link in articulation.links:
                col_objs = convert_sapien_col_shape(link)
                if len(col_objs) > 0:
                    collision_links.append((link.name, col_objs))

            articulated_model = ArticulatedModel.create_from_urdf_string(
                urdf_str,
                srdf_str,
                collision_links=collision_links,
                gravity=[0, 0, -9.81],
                joint_names=[j.name for j in articulation.active_joints],
                link_names=[l.name for l in articulation.links],
                verbose=False,
            )
            articulated_model.set_qpos(articulation.qpos)  # set_qpos to update poses

            self.add_articulation(articulation.name, articulated_model)

        for articulation_name in planned_articulation_names:
            self.set_articulation_planned(articulation_name, True)

        for entity in actors:
            component = entity.find_component_by_type(PhysxRigidBaseComponent)
            assert (
                component is not None
            ), f"No PhysxRigidBaseComponent found in {entity.name}: {entity.components=}"
            assert not isinstance(
                component, PhysxArticulationLinkComponent
            ), f"Component should not be PhysxArticulationLinkComponent: {component=}"

            # Convert collision shapes at current global pose
            col_objs = convert_sapien_col_shape(component)
            # TODO: multiple collision shapes
            assert len(col_objs) == 1, (
                f"Should only have 1 collision object, got {len(col_objs)} shapes for "
                f"entity '{entity.name}'"
            )

            self.add_normal_object(entity.name, col_objs[0])

    def update_from_simulation(self, update_attached_object: bool = True) -> None:
        """Updates planning_world articulations/objects pose with current Scene state

        :param update_attached_object: whether to update the attached pose of
                                    all attached objects
        """
        for articulation in self._sim_scene.get_all_articulations():
            # set_qpos to update poses
            self.get_articulation(articulation.name).set_qpos(articulation.qpos)

        for entity in self._sim_scene.get_all_actors():
            component = entity.find_component_by_type(PhysxRigidBaseComponent)
            assert (
                component is not None
            ), f"No PhysxRigidBaseComponent found in {entity.name}: {entity.components=}"
            assert not isinstance(
                component, PhysxArticulationLinkComponent
            ), f"Component should not be PhysxArticulationLinkComponent: {component=}"

            shapes = component.collision_shapes
            # TODO: multiple collision shapes
            assert len(shapes) == 1, (
                f"Should only have 1 collision shape, got {len(shapes)} shapes for "
                f"entity '{entity.name}'"
            )
            shape = shapes[0]

            pose: Pose = entity.pose * shape.local_pose
            # NOTE: Convert poses for Capsule/Cylinder
            if isinstance(
                shape, (PhysxCollisionShapeCapsule, PhysxCollisionShapeCylinder)
            ):
                pose = pose * Pose(q=euler2quat(0, np.pi / 2, 0))

            # handle attached object
            if self.is_normal_object_attached(entity.name):
                attached_body = self.get_attached_object(entity.name)
                if update_attached_object:
                    parent_posevec = (
                        attached_body.get_attached_articulation()
                        .get_pinocchio_model()
                        .get_link_pose(attached_body.get_attached_link_id())
                    )
                    parent_pose = Pose(parent_posevec[:3], parent_posevec[3:])
                    pose = parent_pose.inv() * pose  # new attached pose
                    attached_body.set_pose(np.hstack((pose.p, pose.q)))
                attached_body.update_pose()
            else:
                self.get_normal_object(entity.name).set_transformation(
                    np.hstack((pose.p, pose.q))
                )
