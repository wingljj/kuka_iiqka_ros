#!/usr/bin/env python3
# src/kuka_mujoco_sim/viewer_node.py
"""Standalone MuJoCo viewer, decoupled from the physics/RSI loop.

Why a separate process: the authoritative sim (sim_node) drives the RSI UDP
link, and the KRC-side hw_interface latches a fault after 5 missed 8ms reads
(~40ms). A `viewer.sync()` call in that loop can stall tens of ms on a render
frame and starve RSI -> fault -> DEGRADED -> no compliance motion. So the
viewer lives here instead: it loads the SAME MJCF, subscribes to sim_node's
``flange_pose`` (PoseStamped, SI units), places the flange free joint at that
pose, runs mj_forward (kinematics only -- no dynamics, so it can never diverge
from or fight the authoritative sim), and renders at its own pace. If this
process stalls, only the picture stutters; RSI keeps its full 250Hz budget.

This mirrors the flange/payload/tip and (via the same model) the wall. The
green mocap target is not driven here (it is a mocap body with no free joint);
the flange pose already reflects where compliance moved the arm, which is what
you want to watch.

KNOWN LIMITATION (measured, not theoretical): a separate process removes the
in-loop stall, but the viewer still competes with sim_node for CPU/GPU on the
same machine. On a soft-real-time host that jitter can push sim_node's SRI
frame interval past the controller's 12 ms wrench_timeout (measured ~15 ms
peaks under gui), which trips the controller's stale-wrench zero-output path
and suppresses compliance motion. Net effect on some machines: with the viewer
open the arm may NOT move even though the loop is otherwise healthy
(system_state=SERVOING). This is the soft-real-time-vs-hard-real-time gap noted
in the followups, NOT a defect in the force-compliance fix. The AUTHORITATIVE
verification is the headless e2e (rosrun kuka_mujoco_sim
e2e_compliance_smoke.py), which is stable. Use this viewer for a qualitative
look on a capable machine; do not treat "it didn't move with gui" as a
regression -- confirm headless first. See README "让它真正动起来 / GUI 局限".
"""
import os

import rospy
from geometry_msgs.msg import PoseStamped

from .mujoco_world import MujocoWorld


class ViewerNode:
    def __init__(self):
        model = rospy.get_param('~model_path', self._default_model())
        mount = rospy.get_param('~mount_abc', [0.0, 0.0, 0.0])
        payload_mass = rospy.get_param('~payload_mass', 1.0)
        wall = rospy.get_param('~wall_enabled', True)
        # Build the same world so geometry/colors match the sim exactly.
        self.world = MujocoWorld(model, mount_abc_deg=tuple(mount),
                                 payload_mass=payload_mass, wall_enabled=wall)
        self._latest = None  # (pos_m, quat_wxyz)
        rospy.Subscriber('flange_pose', PoseStamped, self._on_pose,
                         queue_size=1)

    def _default_model(self):
        here = os.path.dirname(os.path.abspath(__file__))
        return os.path.normpath(
            os.path.join(here, '..', '..', 'models', 'kuka_tcp_scene.xml'))

    def _on_pose(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        self._latest = ([p.x, p.y, p.z], [q.w, q.x, q.y, q.z])

    def spin(self):
        import mujoco.viewer
        with mujoco.viewer.launch_passive(self.world.model,
                                          self.world.data) as viewer:
            rate = rospy.Rate(30)  # render pace; independent of the 250Hz sim
            while not rospy.is_shutdown() and viewer.is_running():
                if self._latest is not None:
                    pos_m, quat = self._latest
                    self.world.set_flange_pose_m(pos_m, quat)
                viewer.sync()
                rate.sleep()


def main():
    rospy.init_node('kuka_mujoco_viewer')
    node = ViewerNode()
    try:
        node.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
