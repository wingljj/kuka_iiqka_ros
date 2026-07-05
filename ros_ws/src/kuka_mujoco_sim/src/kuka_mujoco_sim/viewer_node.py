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

import mujoco
import rospy
from geometry_msgs.msg import PoseStamped, Wrench
from sensor_msgs.msg import JointState

from .mujoco_world import MujocoWorld
from . import frame_conventions as fc


class ViewerNode:
    def __init__(self):
        model = rospy.get_param('~model_path', self._default_model())
        mount = rospy.get_param('~mount_abc', [0.0, 0.0, 0.0])
        payload_mass = rospy.get_param('~payload_mass', 1.0)
        wall = rospy.get_param('~wall_enabled', True)
        # Build the same world so geometry/colors match the sim exactly.
        self.world = MujocoWorld(model, mount_abc_deg=tuple(mount),
                                 payload_mass=payload_mass, wall_enabled=wall)
        self._latest = None       # (pos_m, quat_wxyz) -- for the log readout
        self._latest_q = None     # 6 arm joint angles [rad] -- for rendering
        rospy.Subscriber('flange_pose', PoseStamped, self._on_pose,
                         queue_size=1)
        # Render the arm from the sim's actual joint angles. A flange-pose-only
        # viewer leaves the arm frozen at zero because mj_forward does not solve
        # the weld for joints (that is an mj_step result the viewer never runs).
        rospy.Subscriber('joint_states', JointState, self._on_joints,
                         queue_size=1)
        # Publish drag force so sim_node can apply it to the physics loop.
        self._drag_pub = rospy.Publisher(
            '/kuka_mujoco_sim/drag_force', Wrench, queue_size=1)
        self._last_log_t = 0.0

    def _default_model(self):
        here = os.path.dirname(os.path.abspath(__file__))
        return os.path.normpath(
            os.path.join(here, '..', '..', 'models', 'kuka_kr20_scene.xml'))

    def _on_pose(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        self._latest = ([p.x, p.y, p.z], [q.w, q.x, q.y, q.z])

    def _on_joints(self, msg):
        if len(msg.position) >= 6:
            self._latest_q = list(msg.position[:6])

    def spin(self):
        import mujoco.viewer
        with mujoco.viewer.launch_passive(self.world.model,
                                          self.world.data) as viewer:
            # Show perturbation force arrows in the 3D window.
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_PERTFORCE] = True
            rate = rospy.Rate(30)  # render pace; independent of the 250Hz sim
            _prev_active = 0
            while not rospy.is_shutdown() and viewer.is_running():
                # Render from the sim's actual joint angles so every arm link
                # is where the physics put it. Falls back to the flange-pose
                # marker update if joint data has not arrived yet.
                if self._latest_q is not None:
                    self.world.set_joint_angles(self._latest_q)
                elif self._latest is not None:
                    pos_m, quat = self._latest
                    self.world.set_flange_pose_m(pos_m, quat)

                # Apply any active perturbation to viewer-side data so the
                # force arrow renders correctly, then read back the resulting
                # xfrc_applied and forward it to sim_node via drag_force.
                # Only publish when the user is actively dragging (perturb.active)
                # or on the one trailing frame after release (_prev_active, to
                # send an explicit zero so sim_node clears its latched force).
                # Never publish unprompted zeros: that would overwrite forces
                # injected by other publishers (e.g. the e2e smoke test).
                perturb = viewer.perturb
                mujoco.mjv_applyPerturbForce(
                    self.world.model, self.world.data, perturb)
                # DIAG: report perturb state + xfrc on the SELECTED body and
                # the payload body, so we can see where the drag force lands.
                if perturb.active or perturb.active2:
                    sel = int(perturb.select)
                    xsel = self.world.data.xfrc_applied[sel]
                    xpay = self.world.data.xfrc_applied[self.world._payload_bid]
                    rospy.loginfo_throttle(
                        0.3,
                        'DIAG active=%d active2=%d select=%d '
                        'xfrc[sel]=[%.2f %.2f %.2f] xfrc[payload]=[%.2f %.2f %.2f]'
                        % (perturb.active, perturb.active2, sel,
                           xsel[0], xsel[1], xsel[2],
                           xpay[0], xpay[1], xpay[2]))
                if perturb.active or perturb.active2 or _prev_active:
                    w = Wrench()
                    # Read xfrc from whatever body the user actually grabbed,
                    # falling back to the payload body when nothing is selected.
                    sel = int(perturb.select)
                    bid = sel if sel > 0 else self.world._payload_bid
                    xfrc = self.world.data.xfrc_applied[bid]
                    w.force.x, w.force.y, w.force.z = (
                        float(xfrc[0]), float(xfrc[1]), float(xfrc[2]))
                    w.torque.x, w.torque.y, w.torque.z = (
                        float(xfrc[3]), float(xfrc[4]), float(xfrc[5]))
                    self._drag_pub.publish(w)
                _prev_active = perturb.active or perturb.active2

                # Log Cartesian pose at 2 Hz so it's readable in the terminal.
                now = rospy.get_time()
                if now - self._last_log_t >= 0.5 and self._latest is not None:
                    pos = self._latest[0]
                    quat = self._latest[1]
                    a, b, c = fc.quat_to_abc_deg(quat)
                    rospy.loginfo_throttle(
                        0.5,
                        'flange  X=%.1f mm  Y=%.1f mm  Z=%.1f mm'
                        '  A=%.2f°  B=%.2f°  C=%.2f°' % (
                            pos[0] * fc.MM_PER_M,
                            pos[1] * fc.MM_PER_M,
                            pos[2] * fc.MM_PER_M,
                            a, b, c))
                    self._last_log_t = now

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
