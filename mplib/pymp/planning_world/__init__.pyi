import typing

import numpy

import mplib.pymp.articulation
import mplib.pymp.fcl
import mplib.pymp.planning_world

_Shape = typing.Tuple[int, ...]

__all__ = ["PlanningWorld", "WorldCollisionResult"]

class PlanningWorld:
    def __init__(
        self,
        articulations: list[mplib.pymp.articulation.ArticulatedModel],
        articulation_names: list[str],
        normal_objects: list[mplib.pymp.fcl.CollisionObject],
        normal_object_names: list[str],
        plan_articulation_id: int = 0,
    ) -> None: ...
    def add_articulation(
        self, model: mplib.pymp.articulation.ArticulatedModel, name: str
    ) -> None: ...
    def add_articulations(
        self, models: list[mplib.pymp.articulation.ArticulatedModel], names: list[str]
    ) -> None: ...
    def add_normal_object(
        self, collision_object: mplib.pymp.fcl.CollisionObject, name: str
    ) -> None: ...
    def add_normal_objects(
        self, collision_objects: list[mplib.pymp.fcl.CollisionObject], names: list[str]
    ) -> None: ...
    def collide(self) -> bool: ...
    @staticmethod
    def collide_full(*args, **kwargs) -> typing.Any: ...
    @staticmethod
    def collide_with_others(*args, **kwargs) -> typing.Any: ...
    def get_articulations(self) -> list[mplib.pymp.articulation.ArticulatedModel]: ...
    def get_normal_objects(self) -> list[mplib.pymp.fcl.CollisionObject]: ...
    def print_attached_tool_pose(self) -> None: ...
    def remove_attach(self) -> None: ...
    @staticmethod
    def self_collide(*args, **kwargs) -> typing.Any: ...
    def set_qpos(
        self, index: int, qpos: numpy.ndarray[numpy.float64, _Shape[m, 1]]
    ) -> None: ...
    def set_qpos_all(
        self, qpos: numpy.ndarray[numpy.float64, _Shape[m, 1]]
    ) -> None: ...
    def set_use_attach(self, use: bool = False) -> None: ...
    def set_use_point_cloud(self, use: bool = False) -> None: ...
    def update_attached_box(
        self,
        size: numpy.ndarray[numpy.float64, _Shape[3, 1]],
        link_id: int,
        pose: numpy.ndarray[numpy.float64, _Shape[7, 1]],
    ) -> None: ...
    def update_attached_mesh(
        self,
        mesh_path: str,
        link_id: int,
        pose: numpy.ndarray[numpy.float64, _Shape[7, 1]],
    ) -> None: ...
    def update_attached_sphere(
        self,
        radius: float,
        link_id: int,
        pose: numpy.ndarray[numpy.float64, _Shape[7, 1]],
    ) -> None: ...
    def update_attached_tool(
        self,
        p_geom: mplib.pymp.fcl.CollisionGeometry,
        link_id: int,
        pose: numpy.ndarray[numpy.float64, _Shape[7, 1]],
    ) -> None: ...
    def update_point_cloud(
        self,
        vertices: numpy.ndarray[numpy.float64, _Shape[m, 3]],
        resolution: float = 0.01,
    ) -> None: ...
    pass

class WorldCollisionResult:
    @property
    def collision_type(self) -> str:
        """
        :type: str
        """

    @property
    def link_name1(self) -> str:
        """
        :type: str
        """

    @property
    def link_name2(self) -> str:
        """
        :type: str
        """

    @property
    def object_name1(self) -> str:
        """
        :type: str
        """

    @property
    def object_name2(self) -> str:
        """
        :type: str
        """

    @property
    def res(self) -> mplib.pymp.fcl.CollisionResult:
        """
        :type: mplib.pymp.fcl.CollisionResult
        """
    pass
