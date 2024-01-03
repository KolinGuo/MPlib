import os
from typing import Sequence, Tuple, Union

import numpy as np
import toppra as ta
import toppra.algorithm as algo
import toppra.constraint as constraint
from transforms3d.quaternions import quat2mat

from .pymp import articulation, ompl, planning_world


class Planner:
    """Motion planner."""

    # TODO(jigu): default joint vel and acc limits
    # TODO(jigu): how does user link names and joint names are exactly used?

    def __init__(
        self,
        urdf: str,
        user_link_names: Sequence[str],
        user_joint_names: Sequence[str],
        move_group: str,
        joint_vel_limits: Union[Sequence[float], np.ndarray],
        joint_acc_limits: Union[Sequence[float], np.ndarray],
        srdf: str = "",
        package_keyword_replacement: str = "",
    ):
        r"""Motion planner for robots.

        Args:
            urdf: Unified Robot Description Format file.
            user_link_names: names of links, the order
            user_joint_names: names of the joints to plan
            move_group: target link to move, usually the end-effector.
            joint_vel_limits: maximum joint velocities for time parameterization,
                which should have the same length as
            joint_acc_limits: maximum joint accelerations for time parameterization,
                which should have the same length as
            srdf: Semantic Robot Description Format file.
        References:
            http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/urdf_srdf/urdf_srdf_tutorial.html

        """
        self.urdf = urdf
        if srdf == "" and os.path.exists(urdf.replace(".urdf", ".srdf")):
            srdf = urdf.replace(".urdf", ".srdf")
            print("No SRDF file provided. Try to load %s." % srdf)

        self.srdf = srdf
        self.user_link_names = user_link_names
        self.user_joint_names = user_joint_names

        self.joint_name_2_idx = {}
        for i, joint in enumerate(self.user_joint_names):
            self.joint_name_2_idx[joint] = i
        self.link_name_2_idx = {}
        for i, link in enumerate(self.user_link_names):
            self.link_name_2_idx[link] = i

        # replace package:// keyword if exists
        urdf = self.replace_package_keyword(package_keyword_replacement)

        self.robot = articulation.ArticulatedModel(
            urdf,
            srdf,
            [0, 0, -9.81],
            self.user_joint_names,
            self.user_link_names,
            verbose=False,
            convex=True,
        )
        self.pinocchio_model = self.robot.get_pinocchio_model()

        self.planning_world = planning_world.PlanningWorld(
            [self.robot], ["robot"], [], []
        )
        self.acm = self.planning_world.get_allowed_collision_matrix()

        if srdf == "":
            self.generate_collision_pair()
            self.robot.update_SRDF(self.srdf)

        assert move_group in self.user_link_names
        self.move_group = move_group
        self.robot.set_move_group(self.move_group)
        self.move_group_joint_indices = self.robot.get_move_group_joint_indices()

        self.joint_types = self.pinocchio_model.get_joint_types()
        self.joint_limits = np.concatenate(self.pinocchio_model.get_joint_limits())
        self.planner = ompl.OMPLPlanner(world=self.planning_world)
        self.joint_vel_limits = joint_vel_limits
        self.joint_acc_limits = joint_acc_limits
        self.move_group_link_id = self.link_name_2_idx[self.move_group]
        assert len(self.joint_vel_limits) == len(self.move_group_joint_indices), len(
            self.move_group_joint_indices
        )
        assert len(self.joint_acc_limits) == len(self.move_group_joint_indices)

        # Mask for joints that have equivalent values (revolute joints with range > 2pi)
        self.equiv_joint_mask = [
            t.startswith("JointModelR") for t in self.joint_types
        ] & (self.joint_limits[:, 1] - self.joint_limits[:, 0] > 2 * np.pi)

    def replace_package_keyword(self, package_keyword_replacement):
        rtn_urdf = self.urdf
        with open(self.urdf, "r") as in_f:
            content = in_f.read()
            if "package://" in content:
                rtn_urdf = self.urdf.replace(".urdf", "_package_keyword_replaced.urdf")
                content = content.replace("package://", package_keyword_replacement)
                if not os.path.exists(rtn_urdf):
                    with open(rtn_urdf, "w") as out_f:
                        out_f.write(content)
        return rtn_urdf

    def generate_collision_pair(self, sample_time=1000000, echo_freq=100000):
        print(
            "Since no SRDF file is provided, we will first detect link pairs "
            "that will always collide. This may take several minutes."
        )

        n_link = len(self.user_link_names)
        cnt = np.zeros((n_link, n_link), dtype=np.int32)
        for i in range(sample_time):
            qpos = self.pinocchio_model.get_random_configuration()
            self.robot.set_qpos(qpos, True)
            collisions = self.planning_world.collide_full()
            for collision in collisions:
                u = self.link_name_2_idx[collision.link_name1]
                v = self.link_name_2_idx[collision.link_name2]
                cnt[u][v] += 1
            if i % echo_freq == 0:
                print("Finish %.1f%%!" % (i * 100 / sample_time))

        import xml.etree.ElementTree as ET
        from xml.dom import minidom

        root = ET.Element("robot")
        robot_name = self.urdf.split("/")[-1].split(".")[0]
        root.set("name", robot_name)
        self.srdf = self.urdf.replace(".urdf", ".srdf")

        for i in range(n_link):
            for j in range(n_link):
                if cnt[i][j] == sample_time:
                    link1 = self.user_link_names[i]
                    link2 = self.user_link_names[j]
                    print(
                        "Ignore collision pair: (%s, %s), reason:  always collide"
                        % (link1, link2)
                    )
                    collision = ET.SubElement(root, "disable_collisions")
                    collision.set("link1", link1)
                    collision.set("link2", link2)
                    collision.set("reason", "Default")
        srdffile = open(self.srdf, "w")
        srdffile.write(
            minidom.parseString(ET.tostring(root)).toprettyxml(indent="    ")
        )
        srdffile.close()
        print("Saving the SRDF file to %s" % self.srdf)

    def distance_6D(self, p1, q1, p2, q2):
        return np.linalg.norm(p1 - p2) + min(
            np.linalg.norm(q1 - q2), np.linalg.norm(q1 + q2)
        )

    def check_joint_limit(self, qpos: np.ndarray) -> bool:
        """Check joint limits, always clip q towards lower limits for revolute joints
        i.e., q will be in range [qlimit[0], qlimit[0] + 2*pi)

        :param qpos: joint positions, qpos will be clipped if within_limits.
                     If not within_limits, qpos might not be fully clipped.
        :return within_limits: whether the joint positions are within joint limits.
        """
        for i, (q, joint_type, qlimit) in enumerate(
            zip(qpos, self.joint_types, self.joint_limits)
        ):
            if joint_type.startswith("JointModelR"):  # revolute joint
                if np.abs(q - qlimit[0]) < 1e-3:
                    continue
                q -= 2 * np.pi * np.floor((q - qlimit[0]) / (2 * np.pi))
                qpos[i] = q  # clip qpos
                if q > qlimit[1] + 1e-3:
                    return False
            else:
                if q < qlimit[0] - 1e-3 or q > qlimit[1] + 1e-3:
                    return False
        return True

    def check_for_collision(
        self,
        collision_function,
        state: np.ndarray = None,
    ) -> list:
        """
        :param state: all planned articulations qpos state
        """
        planned_articulations = self.planning_world.get_planned_articulations()
        assert (
            len(planned_articulations) == 1
        ), "Only support 1 planned articulation now"
        articulation = planned_articulations[0]

        # first save the current qpos
        old_qpos = articulation.get_qpos()

        # handle no user input
        if state is None:
            state = old_qpos
        # if the user does not specify the end-effector joints, append them to the qpos
        if len(state) == len(self.move_group_joint_indices):
            tmp = old_qpos.copy()
            tmp[: len(state)] = state
            state = tmp
        # set robot to new qpos
        articulation.set_qpos(state, True)
        # check for self-collision
        collisions = collision_function()
        # reset qpos
        articulation.set_qpos(old_qpos, True)
        return collisions

    def check_for_self_collision(
        self,
        qpos: np.ndarray = None,
    ) -> list:
        """Check if the robot is in self-collision.

        Args:
            articulation: robot model. if none will be self.robot
            qpos: robot configuration. if none will be the current pose

        Returns:
            A list of collisions.
        """
        return self.check_for_collision(self.planning_world.self_collide, qpos)

    def check_for_env_collision(
        self,
        qpos: np.ndarray = None,
    ) -> list:
        """Check if the robot is in collision with the environment

        Args:
            articulation: robot model. if none will be self.robot
            qpos: robot configuration. if none will be the current pose

        Returns:
            A list of collisions.
        """
        return self.check_for_collision(self.planning_world.collide_with_others, qpos)

    def IK(
        self,
        goal_pose: np.ndarray,
        start_qpos: np.ndarray,
        mask: np.ndarray = [],
        *,
        n_init_qpos: int = 20,
        threshold: float = 0.001,
        return_closest: bool = False,
        verbose: bool = False,
    ):
        """Compute inverse kinematics

        :param goal_pose: goal pose (xyz, wxyz), (7,) np.floating np.ndarray.
        :param start_qpos: starting qpos, (ndof,) np.floating np.ndarray.
        :param mask: qpos mask to disable planning, (ndof,) bool np.ndarray.
        :param n_init_qpos: number of init qpos to sample.
        :param threshold: distance_6D threshold for marking sampled IK as success.
                          distance_6D is position error norm + quaternion error norm.
        :param return_closest: whether to return the qpos that is closest to start_qpos,
                               considering equivalent joint values.
        :param verbose: whether to print collision info if any collision exists.
        :return status: IK status, "Success" if succeeded.
        :return q_goals: list of sampled IK qpos, (ndof,) np.floating np.ndarray.
                         IK is successful if q_goals is not None.
                         If return_closest, q_goals is np.ndarray if successful
                         and None if not successful.
        """
        # TODO: verbose: print collision info
        move_link_idx = self.link_name_2_idx[self.move_group]
        move_joint_idx = self.move_group_joint_indices
        self.robot.set_qpos(start_qpos, True)

        min_dis = 1e9
        q_goals = []
        qpos = start_qpos
        for _ in range(n_init_qpos):
            ik_qpos, ik_success, ik_error = self.pinocchio_model.compute_IK_CLIK(
                move_link_idx, goal_pose, qpos, mask
            )
            # NOTE: check_joint_limit() will clip qpos towards lower limits
            success = ik_success and self.check_joint_limit(ik_qpos)

            if success:
                # check collision
                self.planning_world.set_qpos_all(ik_qpos[move_joint_idx])
                if len(self.planning_world.collide_full()) > 0:
                    success = False

            if success:
                self.pinocchio_model.compute_forward_kinematics(ik_qpos)
                new_pose = self.pinocchio_model.get_link_pose(move_link_idx)
                tmp_dis = self.distance_6D(
                    goal_pose[:3], goal_pose[3:], new_pose[:3], new_pose[3:]
                )
                if tmp_dis < min_dis:
                    min_dis = tmp_dis
                if tmp_dis < threshold:
                    for q_goal in q_goals:
                        if (
                            np.linalg.norm(
                                q_goal[move_joint_idx] - ik_qpos[move_joint_idx]
                            ) < 0.1
                        ):
                            break  # not unique ik_qpos
                    else:
                        q_goals.append(ik_qpos)

            qpos = self.pinocchio_model.get_random_configuration()
            qpos[mask] = start_qpos[mask]  # use start_qpos for disabled joints

        if len(q_goals) > 0:
            status = "Success"
        elif min_dis != 1e9:
            status = f"IK Failed! Distance {min_dis} is greater than {threshold=}."
            return status, None
        else:
            status = "IK Failed! Cannot find valid solution."
            return status, None

        if return_closest:
            q_goals = np.asarray(q_goals)  # [N, ndof]
            start_qpos = np.asarray(start_qpos)[None]  # [1, ndof]

            # Consider equivalent joint values
            q1 = q_goals[:, self.equiv_joint_mask]  # [N, n_equiv_joint]
            q2 = q1 + 2 * np.pi  # equivalent joints
            start_q = start_qpos[:, self.equiv_joint_mask]  # [1, n_equiv_joint]

            # Mask where q2 is valid and closer to start_q
            q2_closer_mask = (
                q2 < self.joint_limits[:, 1][None, self.equiv_joint_mask]
            ) & (np.abs(q1 - start_q) > np.abs(q2 - start_q))  # [N, n_equiv_joint]
            # Convert q_goals to equivalent joint values closest to start_qpos
            q_goals[:, self.equiv_joint_mask] = np.where(q2_closer_mask, q2, q1)

            q_goals = q_goals[np.linalg.norm(q_goals - start_qpos, axis=1).argmin()]
        return status, q_goals

    def TOPP(self, path, step=0.1, verbose=False):
        N_samples = path.shape[0]
        dof = path.shape[1]
        assert dof == len(self.joint_vel_limits)
        assert dof == len(self.joint_acc_limits)
        ss = np.linspace(0, 1, N_samples)
        path = ta.SplineInterpolator(ss, path)
        pc_vel = constraint.JointVelocityConstraint(self.joint_vel_limits)
        pc_acc = constraint.JointAccelerationConstraint(self.joint_acc_limits)
        instance = algo.TOPPRA(
            [pc_vel, pc_acc], path, parametrizer="ParametrizeConstAccel"
        )
        jnt_traj = instance.compute_trajectory()
        ts_sample = np.linspace(0, jnt_traj.duration, int(jnt_traj.duration / step))
        qs_sample = jnt_traj(ts_sample)
        qds_sample = jnt_traj(ts_sample, 1)
        qdds_sample = jnt_traj(ts_sample, 2)
        return ts_sample, qs_sample, qds_sample, qdds_sample, jnt_traj.duration

    def update_point_cloud(self, pc, resolution=1e-3, name="scene_pcd"):
        self.planning_world.add_point_cloud(name, pc, resolution)

    def remove_point_cloud(self, name="scene_pcd"):
        self.planning_world.remove_normal_object(name)

    def update_attach_object(
        self,
        fcl_collision_geometry,
        pose,
        name="attached_geom",
        art_name="robot",
        link_id=-1,
    ):
        if link_id == -1:
            link_id = self.move_group_link_id
        self.planning_world.attach_object(
            name, fcl_collision_geometry, art_name, link_id, pose
        )

    def update_attached_sphere(self, radius, pose, art_name="robot", link_id=-1):
        if link_id == -1:
            link_id = self.move_group_link_id
        self.planning_world.attach_sphere(radius, art_name, link_id, pose)

    def update_attached_box(self, size, pose, art_name="robot", link_id=-1):
        if link_id == -1:
            link_id = self.move_group_link_id
        self.planning_world.attach_box(size, art_name, link_id, pose)

    def update_attached_mesh(self, mesh_path, pose, art_name="robot", link_id=-1):
        if link_id == -1:
            link_id = self.move_group_link_id
        self.planning_world.attach_mesh(mesh_path, art_name, link_id, pose)

    def detach_object(self, name="attached_geom", also_remove=False) -> bool:
        return self.planning_world.detach_object(name, also_remove)

    def plan(
        self,
        goal_pose: np.ndarray,
        current_qpos: np.ndarray,
        mask: np.ndarray = [],
        *,
        planner_name: str = "RRTConnect",
        time_step: float = 0.1,
        rrt_range: float = 0.1,
        planning_time: float = 1,
        pathlen_obj_weight: float = 10.0,
        pathlen_obj_only: bool = False,
        fix_joint_limits: bool = True,
        verbose: bool = False,
    ) -> dict[str, str | np.ndarray | np.float64]:
        """Plan path with RRTConnect

        :param goal_pose: goal pose (xyz, wxyz), (7,) np.floating np.ndarray.
        :param current_qpos: current qpos, (ndof,) np.floating np.ndarray.
        :param mask: qpos mask to disable planning, (ndof,) bool np.ndarray.
        :param planner_name: name of planner to use. ["RRTConnect", "PRMstar",
                             "LazyPRMstar", "RRTstar", "RRTsharp", "RRTXstatic",
                             "InformedRRTstar"]
        :param time_step: time interval between the generated waypoints.
                          The larger the value, the sparser the output waypoints.
        :param rrt_range: the incremental distance in the RRTConnect algorithm,
                          The larger the value, the sparser the sampled waypoints
                          (before time parameterization).
        :param planning_time: time limit for RRTConnect algorithm, in seconds.
        :param pathlen_obj_weight: weight of path length objective for RRTstar.
        :param pathlen_obj_only: Only plan with shorted path length objective.
        :param fix_joint_limits: whether to clip the current joint positions
                                 if they are out of the joint limits.
        :param use_point_cloud: whether to avoid collisions
                                between the robot and the point cloud.
        :param use_attach: whether to avoid collisions
                           between the attached tool and the point cloud.
                           Requires use_point_cloud to be True.
        :param verbose: whether to display some internal outputs.
        :return result: A dictionary containing:
                        * status: ik_status if IK failed, "Success" if RRT succeeded.
                        If successful, the following key/value will be included:
                        * time: Time step of each waypoint, (n_step,) np.float64
                        * position: qpos of each waypoint, (n_step, ndof) np.float64
                        * velocity: qvel of each waypoint, (n_step, ndof) np.float64
                        * acceleration: qacc of each waypoint, (n_step, ndof) np.float64
                        * duration: optimal duration of the generated path, np.float64
                        Note that ndof is n_active_dof
        """
        if fix_joint_limits:
            current_qpos = np.clip(
                current_qpos, self.joint_limits[:, 0], self.joint_limits[:, 1]
            )

        self.robot.set_qpos(current_qpos, True)
        collisions = self.planning_world.collide_full()
        if len(collisions) > 0:
            print("Invalid start state!")
            for collision in collisions:
                print(f"{collision.link_name1} and {collision.link_name2} collide!")

        move_joint_idx = self.move_group_joint_indices
        ik_status, goal_qpos = self.IK(goal_pose, current_qpos, mask)
        if ik_status != "Success":
            return {"status": ik_status}

        if verbose:
            print("IK results:")
            for i in range(len(goal_qpos)):
                print(goal_qpos[i])

        goal_qpos_ = []
        for i in range(len(goal_qpos)):
            goal_qpos_.append(goal_qpos[i][move_joint_idx])
        self.robot.set_qpos(current_qpos, True)

        status, path = self.planner.plan(
            current_qpos[move_joint_idx],
            goal_qpos_,
            planner_name=planner_name,
            time=planning_time,
            range=rrt_range,
            pathlen_obj_weight=pathlen_obj_weight,
            pathlen_obj_only=pathlen_obj_only,
            verbose=verbose,
        )

        if status == "Exact solution":
            if verbose:
                ta.setup_logging("INFO")
            else:
                ta.setup_logging("WARNING")

            times, pos, vel, acc, duration = self.TOPP(path, time_step)
            return {
                "status": "Success",
                "time": times,
                "position": pos,
                "velocity": vel,
                "acceleration": acc,
                "duration": duration,
            }
        else:
            return {
                "status": f"{planner_name} failed. {status}",
                "path": path,
            }

    def plan_screw(
        self,
        goal_pose: np.ndarray,
        current_qpos: np.ndarray,
        *,
        time_step: float = 0.1,
        qpos_step: float = 0.1,
        verbose: bool = False,
    ) -> dict[str, str | np.ndarray | np.float64]:
        """Plan path with straight-line screw motion

        :param goal_pose: goal pose (xyz, wxyz), (7,) np.floating np.ndarray.
        :param current_qpos: current qpos, (ndof,) np.floating np.ndarray.
        :param time_step: time interval between the generated waypoints.
                          The larger the value, the sparser the output waypoints.
        :param qpos_step: the incremental distance of the joint positions
                          during the path generation (before time paramtertization).
        :param use_point_cloud: whether to avoid collisions
                                between the robot and the point cloud.
        :param use_attach: whether to avoid collisions
                           between the attached tool and the point cloud.
                           Requires use_point_cloud to be True.
        :param verbose: whether to display some internal outputs.
        :return result: A dictionary containing:
                        * status: "Success" if succeeded.
                        If successful, the following key/value will be included:
                        * time: Time step of each waypoint, (n_step,) np.float64
                        * position: qpos of each waypoint, (n_step, ndof) np.float64
                        * velocity: qvel of each waypoint, (n_step, ndof) np.float64
                        * acceleration: qacc of each waypoint, (n_step, ndof) np.float64
                        * duration: optimal duration of the generated path, np.float64
                        Note that ndof is n_active_dof
        """
        current_qpos = np.copy(current_qpos)
        self.robot.set_qpos(current_qpos, True)

        def pose7D2mat(pose):
            mat = np.eye(4)
            mat[0:3, 3] = pose[:3]
            mat[0:3, 0:3] = quat2mat(pose[3:])
            return mat

        def skew(vec):
            return np.array(
                [
                    [0, -vec[2], vec[1]],
                    [vec[2], 0, -vec[0]],
                    [-vec[1], vec[0], 0],
                ]
            )

        def pose2exp_coordinate(pose: np.ndarray) -> Tuple[np.ndarray, float]:
            def rot2so3(rotation: np.ndarray):
                assert rotation.shape == (3, 3)
                if np.isclose(rotation.trace(), 3):
                    return np.zeros(3), 1
                if np.isclose(rotation.trace(), -1):
                    return np.zeros(3), -1e6
                theta = np.arccos((rotation.trace() - 1) / 2)
                omega = (
                    1
                    / 2
                    / np.sin(theta)
                    * np.array(
                        [
                            rotation[2, 1] - rotation[1, 2],
                            rotation[0, 2] - rotation[2, 0],
                            rotation[1, 0] - rotation[0, 1],
                        ]
                    ).T
                )
                return omega, theta

            omega, theta = rot2so3(pose[:3, :3])
            if theta < -1e5:
                return omega, theta
            ss = skew(omega)
            inv_left_jacobian = (
                np.eye(3) / theta
                - 0.5 * ss
                + (1.0 / theta - 0.5 / np.tan(theta / 2)) * ss @ ss
            )
            v = inv_left_jacobian @ pose[:3, 3]
            return np.concatenate([v, omega]), theta

        self.pinocchio_model.compute_forward_kinematics(current_qpos)
        ee_index = self.link_name_2_idx[self.move_group]
        current_p = pose7D2mat(self.pinocchio_model.get_link_pose(ee_index))
        target_p = pose7D2mat(goal_pose)
        relative_transform = target_p @ np.linalg.inv(current_p)

        omega, theta = pose2exp_coordinate(relative_transform)

        if theta < -1e4:
            return {"status": "screw plan failed."}
        omega = omega.reshape((-1, 1)) * theta

        move_joint_idx = self.move_group_joint_indices
        path = [np.copy(current_qpos[move_joint_idx])]

        while True:
            self.pinocchio_model.compute_full_jacobian(current_qpos)
            J = self.pinocchio_model.get_link_jacobian(ee_index, local=False)
            delta_q = np.linalg.pinv(J) @ omega
            delta_q *= qpos_step / (np.linalg.norm(delta_q))
            delta_twist = J @ delta_q

            flag = False
            if np.linalg.norm(delta_twist) > np.linalg.norm(omega):
                ratio = np.linalg.norm(omega) / np.linalg.norm(delta_twist)
                delta_q = delta_q * ratio
                delta_twist = delta_twist * ratio
                flag = True

            current_qpos += delta_q.reshape(-1)
            omega -= delta_twist

            def check_joint_limit(q):
                n = len(q)
                for i in range(n):
                    if (
                        q[i] < self.joint_limits[i][0] - 1e-3
                        or q[i] > self.joint_limits[i][1] + 1e-3
                    ):
                        return False
                return True

            within_joint_limit = check_joint_limit(current_qpos)
            self.planning_world.set_qpos_all(current_qpos[move_joint_idx])
            collide = self.planning_world.collide()

            if np.linalg.norm(delta_twist) < 1e-4 or collide or not within_joint_limit:
                return {"status": "screw plan failed"}

            path.append(np.copy(current_qpos[move_joint_idx]))

            if flag:
                if verbose:
                    ta.setup_logging("INFO")
                else:
                    ta.setup_logging("WARNING")
                times, pos, vel, acc, duration = self.TOPP(np.vstack(path), time_step)
                return {
                    "status": "Success",
                    "time": times,
                    "position": pos,
                    "velocity": vel,
                    "acceleration": acc,
                    "duration": duration,
                }
