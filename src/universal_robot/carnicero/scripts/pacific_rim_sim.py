#!/usr/bin/env python
"""Admittance controller configured for the Gazebo UR simulation.

This script reuses the logic from ``pacific_rim.py`` but adjusts the
controlled frame to ``tool0`` which corresponds to the end-effector in
simulation.  It expects the elbow and wrist positions to be provided on
``/right_arm/elbow`` and ``/right_arm/wrist`` (they can be generated with
``right_arm_constant_publisher.py``).
"""
import rospy
from urdf_parser_py.urdf import URDF
import PyKDL
from kdl_parser_py.urdf import treeFromUrdfModel
import os
import importlib.util

# Try to import the base controller from ``pacific_rim.py``.  In some
# setups the package path may not expose ``admittance_control`` directly,
# so fall back to loading the script explicitly from disk.
try:  # pragma: no cover - only executed in ROS runtime
    from pacific_rim import admittance_control
except Exception:  # noqa: BLE001 - broad to handle missing module/attr
    script_path = os.path.join(os.path.dirname(__file__), "pacific_rim.py")
    spec = importlib.util.spec_from_file_location("pacific_rim", script_path)
    pacific_rim = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(pacific_rim)  # type: ignore[attr-defined]
    admittance_control = pacific_rim.admittance_control


class admittance_control_sim(admittance_control):
    """Admittance controller adapted for Gazebo.

    The original controller uses ``wrist_3_link`` as the tool frame.
    In simulation the link ``tool0`` is usually available and represents
    the same physical location, so we recompute the kinematic chain with
    this frame.
    """

    def __init__(self):
        super(admittance_control_sim, self).__init__()
        # Override the frame used for control and update KDL solvers
        self.tool_frame = "tool0"
        robot_description = rospy.get_param("/robot_description")
        robot = URDF.from_xml_string(robot_description)
        ok, tree = treeFromUrdfModel(robot)
        chain = tree.getChain(self.base_frame, self.tool_frame)
        self.kdl_jnt_to_jac_solver = PyKDL.ChainJntToJacSolver(chain)
        self.kdl_fk_solver = PyKDL.ChainFkSolverPos_recursive(chain)

    def init_offset_and_latch(self):
        """Wait for the TF tree before running base initialisation."""
        if not self.tf_buffer.can_transform(
            self.base_frame,
            self.tool_frame,
            rospy.Time(0),
            rospy.Duration(5.0),
        ):
            raise rospy.ROSException(
                "Transform %s -> %s not available" % (self.base_frame, self.tool_frame)
            )
        super(admittance_control_sim, self).init_offset_and_latch()


def main():
    try:
        ctrl = admittance_control_sim()
        ctrl.switch_controllers(['joint_group_vel_controller'], ['scaled_pos_joint_traj_controller'])
        rospy.loginfo("Init offset + latch...")
        ctrl.init_offset_and_latch()
        rospy.loginfo("Go.")
        ctrl.run()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        return


if __name__ == "__main__":
    main()
