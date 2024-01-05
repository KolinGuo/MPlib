import typing

import numpy

import mplib.pymp.kdl

_Shape = typing.Tuple[int, ...]

__all__ = ["KDLModel"]

class KDLModel:
    def __init__(
        self,
        urdf_filename: str,
        joint_names: list[str],
        link_names: list[str],
        verbose: bool,
    ) -> None: ...
    def chain_IK_LMA(
        self,
        index: int,
        q_init: numpy.ndarray[numpy.float64, _Shape[m, 1]],
        goal_pose: numpy.ndarray[numpy.float64, _Shape[7, 1]],
    ) -> tuple[numpy.ndarray[numpy.float64, _Shape[m, 1]], int]: ...
    def chain_IK_NR(
        self,
        index: int,
        q_init: numpy.ndarray[numpy.float64, _Shape[m, 1]],
        goal_pose: numpy.ndarray[numpy.float64, _Shape[7, 1]],
    ) -> tuple[numpy.ndarray[numpy.float64, _Shape[m, 1]], int]: ...
    def chain_IK_NR_JL(
        self,
        index: int,
        q_init: numpy.ndarray[numpy.float64, _Shape[m, 1]],
        goal_pose: numpy.ndarray[numpy.float64, _Shape[7, 1]],
        q_min: numpy.ndarray[numpy.float64, _Shape[m, 1]],
        q_max: numpy.ndarray[numpy.float64, _Shape[m, 1]],
    ) -> tuple[numpy.ndarray[numpy.float64, _Shape[m, 1]], int]: ...
    def get_tree_root_name(self) -> str: ...
    def tree_IK_NR_JL(
        self,
        endpoints: list[str],
        q_init: numpy.ndarray[numpy.float64, _Shape[m, 1]],
        goal_poses: list[numpy.ndarray[numpy.float64, _Shape[7, 1]]],
        q_min: numpy.ndarray[numpy.float64, _Shape[m, 1]],
        q_max: numpy.ndarray[numpy.float64, _Shape[m, 1]],
    ) -> tuple[numpy.ndarray[numpy.float64, _Shape[m, 1]], int]: ...
    pass
