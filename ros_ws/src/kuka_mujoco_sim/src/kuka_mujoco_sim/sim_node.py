#!/usr/bin/env python3
# src/kuka_mujoco_sim/sim_node.py
"""MuJoCo sim node: closes the RSI + SRI loop against the real drivers."""
import os
import socket
import rospy

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
        self._gui = rospy.get_param('~gui', False)

        self.world = MujocoWorld(model, mount_abc_deg=tuple(mount),
                                 payload_mass=payload_mass, wall_enabled=wall)
        self.rsi = RsiEndpoint(target_ip=rsi_ip, target_port=rsi_port)
        self.rsi.set_initial_target(self.world.home_pose6)
        self.sri = SriEndpoint(listen_port=sri_port, require_start=True)
        self.sri.start()

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.05)
        self._viewer = None
        if self._gui:
            import mujoco.viewer
            self._viewer = mujoco.viewer.launch_passive(
                self.world.model, self.world.data)

    def _default_model(self):
        here = os.path.dirname(os.path.abspath(__file__))
        return os.path.normpath(
            os.path.join(here, '..', '..', 'models', 'kuka_tcp_scene.xml'))

    def spin(self):
        rate = rospy.Rate(250)
        while not rospy.is_shutdown():
            actual = self.world.get_actual_pose()
            self.rsi.send_state(self.sock, actual)
            try:
                data, _ = self.sock.recvfrom(4096)
                self.rsi.apply_reply(data)
            except socket.timeout:
                pass
            self.world.set_target_pose(self.rsi.pose_target6)
            self.world.step(1)
            self.sri.set_wrench(self.world.get_sensor_wrench())
            self.sri.send_frame_if_streaming()
            if self._viewer is not None:
                self._viewer.sync()
            rate.sleep()

    def shutdown(self):
        self.sri.stop()
        self.sock.close()
        if self._viewer is not None:
            self._viewer.close()


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
