#!/usr/bin/env python3
# src/kuka_mujoco_sim/sim_node.py
"""MuJoCo sim node: closes the RSI + SRI loop against the real drivers."""
import os
import socket
import rospy

from geometry_msgs.msg import Wrench, PointStamped, PoseStamped

from .mujoco_world import MujocoWorld
from .rsi_endpoint import RsiEndpoint
from .sri_endpoint import SriEndpoint


class SimNode:
    def __init__(self):
        model = rospy.get_param('~model_path', self._default_model())
        mount = rospy.get_param('~mount_abc', [0.0, 0.0, 0.0])
        payload_mass = rospy.get_param('~payload_mass', 1.0)
        wall = rospy.get_param('~wall_enabled', True)
        rsi_ip = rospy.get_param('~rsi_target_ip', '127.0.0.1')
        rsi_port = rospy.get_param('~rsi_target_port', 49152)
        sri_port = rospy.get_param('~sri_listen_port', 4008)

        self.world = MujocoWorld(model, mount_abc_deg=tuple(mount),
                                 payload_mass=payload_mass, wall_enabled=wall)
        self.rsi = RsiEndpoint(target_ip=rsi_ip, target_port=rsi_port)
        self.rsi.set_initial_target(self.world.home_pose6)
        self.sri = SriEndpoint(listen_port=sri_port, require_start=True)
        self.sri.start()

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.05)
        # Optional programmatic hand-drag: publish a geometry_msgs/Wrench on
        # ~drag_force (world frame, N/Nm) to inject a persistent external load
        # on the payload. Used by the end-to-end compliance smoke to prove the
        # force->controller->motion chain without a human at the GUI. Latched
        # into _drag and applied every physics step until changed.
        self._drag = None
        rospy.Subscriber('~drag_force', Wrench, self._on_drag, queue_size=1)
        # Publish the flange (TCP) position each loop so motion is observable
        # headlessly (no GUI). Units are mm, matching the RSI pose6 convention;
        # this is what the end-to-end compliance smoke watches to confirm the
        # green mocap target actually moved under an applied force.
        self._pose_pub = rospy.Publisher('~flange_position', PointStamped,
                                          queue_size=1)
        # Full SI pose (m + quaternion) for the STANDALONE viewer node. The
        # viewer runs in its own process and only renders; it never touches the
        # RSI/SRI sockets. This keeps rendering stalls out of this loop's time
        # budget -- an in-process viewer.sync() here would starve the RSI UDP
        # link (KRC hw_interface faults after 5 missed 8ms reads) and latch a
        # fault -> DEGRADED -> no compliance motion. See viewer_node.py.
        self._flange_pose_pub = rospy.Publisher('~flange_pose', PoseStamped,
                                                queue_size=1)

    def _default_model(self):
        here = os.path.dirname(os.path.abspath(__file__))
        return os.path.normpath(
            os.path.join(here, '..', '..', 'models', 'kuka_tcp_scene.xml'))

    def _on_drag(self, msg):
        self._drag = (msg.force.x, msg.force.y, msg.force.z,
                      msg.torque.x, msg.torque.y, msg.torque.z)

    def spin(self):
        rate = rospy.Rate(250)
        while not rospy.is_shutdown():
            actual = self.world.get_actual_pose()
            self.rsi.send_state(self.sock, actual)
            now = rospy.Time.now()
            msg = PointStamped()
            msg.header.stamp = now
            msg.point.x, msg.point.y, msg.point.z = actual[0], actual[1], actual[2]
            self._pose_pub.publish(msg)
            try:
                data, _ = self.sock.recvfrom(4096)
                self.rsi.apply_reply(data)
            except socket.timeout:
                pass
            self.world.set_target_pose(self.rsi.pose_target6)
            if self._drag is not None:
                self.world.apply_external_force(*self._drag)
            self.world.step(1)
            self.sri.set_wrench(self.world.get_sensor_wrench())
            self.sri.send_frame_if_streaming()
            # Publish full flange pose for the standalone viewer (cheap: no
            # render here, so RSI timing is untouched).
            pos_m, quat = self.world.get_flange_pose_m()
            pmsg = PoseStamped()
            pmsg.header.stamp = now
            pmsg.pose.position.x, pmsg.pose.position.y, pmsg.pose.position.z = (
                float(pos_m[0]), float(pos_m[1]), float(pos_m[2]))
            pmsg.pose.orientation.w = float(quat[0])
            pmsg.pose.orientation.x = float(quat[1])
            pmsg.pose.orientation.y = float(quat[2])
            pmsg.pose.orientation.z = float(quat[3])
            self._flange_pose_pub.publish(pmsg)
            rate.sleep()

    def shutdown(self):
        self.sri.stop()
        self.sock.close()


def main():
    rospy.init_node('kuka_mujoco_sim')
    node = SimNode()
    rospy.on_shutdown(node.shutdown)
    try:
        node.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
