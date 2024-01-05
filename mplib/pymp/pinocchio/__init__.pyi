import typing

import numpy

import mplib.pymp.pinocchio

_Shape = typing.Tuple[int, ...]

__all__ = ["PinocchioModel"]

class PinocchioModel:
    def __init__(
        self,
        urdf_filename: str,
        gravity: numpy.ndarray[numpy.float64, _Shape[3, 1]] = array([0.0, 0.0, -9.81]),
        verbose: bool = True,
    ) -> None: ...
    def compute_IK_CLIK(
        self,
        index: int,
        pose: numpy.ndarray[numpy.float64, _Shape[7, 1]],
        q_init: numpy.ndarray[numpy.float64, _Shape[m, 1]],
        mask: list[bool] = [],
        eps: float = 1e-05,
        maxIter: int = 1000,
        dt: float = 0.1,
        damp: float = 1e-12,
    ) -> tuple[
        numpy.ndarray[numpy.float64, _Shape[m, 1]],
        bool,
        numpy.ndarray[numpy.float64, _Shape[6, 1]],
    ]: ...
    def compute_IK_CLIK_JL(
        self,
        index: int,
        pose: numpy.ndarray[numpy.float64, _Shape[7, 1]],
        q_init: numpy.ndarray[numpy.float64, _Shape[m, 1]],
        q_min: numpy.ndarray[numpy.float64, _Shape[m, 1]],
        q_max: numpy.ndarray[numpy.float64, _Shape[m, 1]],
        eps: float = 1e-05,
        maxIter: int = 1000,
        dt: float = 0.1,
        damp: float = 1e-12,
    ) -> tuple[
        numpy.ndarray[numpy.float64, _Shape[m, 1]],
        bool,
        numpy.ndarray[numpy.float64, _Shape[6, 1]],
    ]: ...
    def compute_forward_kinematics(
        self, qpos: numpy.ndarray[numpy.float64, _Shape[m, 1]]
    ) -> None: ...
    def compute_full_jacobian(
        self, qpos: numpy.ndarray[numpy.float64, _Shape[m, 1]]
    ) -> None: ...
    def compute_single_link_local_jacobian(
        self, qpos: numpy.ndarray[numpy.float64, _Shape[m, 1]], index: int
    ) -> numpy.ndarray[numpy.float64, _Shape[6, n]]: ...
    def get_chain_joint_index(self, arg0: str) -> list[int]: ...
    def get_chain_joint_name(self, arg0: str) -> list[str]: ...
    def get_joint_dim(self, index: int, user: bool = True) -> int: ...
    def get_joint_dims(
        self, user: bool = True
    ) -> numpy.ndarray[numpy.int32, _Shape[m, 1]]: ...
    def get_joint_ids(
        self, user: bool = True
    ) -> numpy.ndarray[numpy.int32, _Shape[m, 1]]: ...
    def get_joint_limits(
        self, user: bool = True
    ) -> list[numpy.ndarray[numpy.float64, _Shape[m, n]]]: ...
    def get_joint_names(self, user: bool = True) -> list[str]: ...
    def get_joint_types(self, user: bool = True) -> list[str]: ...
    def get_leaf_links(self) -> list[str]: ...
    def get_link_jacobian(
        self, index: int, local: bool = False
    ) -> numpy.ndarray[numpy.float64, _Shape[6, n]]: ...
    def get_link_names(self, user: bool = True) -> list[str]: ...
    def get_link_pose(
        self, index: int
    ) -> numpy.ndarray[numpy.float64, _Shape[7, 1]]: ...
    def get_parents(
        self, user: bool = True
    ) -> numpy.ndarray[numpy.int32, _Shape[m, 1]]: ...
    def get_random_configuration(
        self,
    ) -> numpy.ndarray[numpy.float64, _Shape[m, 1]]: ...
    def print_frames(self) -> None: ...
    def set_joint_order(self, names: list[str]) -> None: ...
    def set_link_order(self, names: list[str]) -> None: ...
    pass
