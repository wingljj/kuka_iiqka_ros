# src/kuka_mujoco_sim/mujoco_world.py
"""MuJoCo wrapper: elastic pose tracking + contact + FT sensor readout.

The mocap 'target' body is driven to the corrected TCP pose; the 'flange'
free body tracks it through a weld equality (elastic). Contact with the wall
pushes the flange off target -> compliance displacement. The FT sensor reads
contact reaction + payload gravity projected into the sensor frame.

The force/torque sensor site sits on the ``payload`` sub-body so its
``cfrc_int`` isolates the interaction transmitted across the payload<-flange
connection: payload gravity + any contact reaction picked up by the tip. This
keeps the free-space gravity magnitude equal to the payload weight (not the
flange+payload weight).
"""
import numpy as np
import mujoco

from . import frame_conventions as fc


class MujocoWorld:
    DT = 0.004

    def __init__(self, model_path, mount_abc_deg=(0, 0, 0), payload_mass=None,
                 payload_com_m=None, wall_enabled=True):
        self.model = self._load_spec(model_path, mount_abc_deg, payload_mass,
                                     payload_com_m, wall_enabled)
        self.data = mujoco.MjData(self.model)
        self._mocap_id = self.model.body('target').mocapid[0]
        self._flange_bid = self.model.body('flange').id
        self._force_adr = self.model.sensor('ft_force').adr[0]
        self._torque_adr = self.model.sensor('ft_torque').adr[0]
        self._site_id = self.model.site('ft_site').id
        mujoco.mj_forward(self.model, self.data)
        # home pose = initial flange pose in mm/deg
        self.home_pose6 = self._flange_pose6()
        self._wall_normal_world = np.array([-1.0, 0.0, 0.0])  # wall faces -X

    def _load_spec(self, path, mount_abc_deg, payload_mass, payload_com_m,
                   wall_enabled):
        with open(path, 'r') as fh:
            xml = fh.read()
        model = mujoco.MjModel.from_xml_string(xml)
        # Apply runtime overrides on the compiled model.
        if payload_mass is not None:
            bid = model.body('payload').id
            model.body_mass[bid] = payload_mass
        if payload_com_m is not None:
            bid = model.body('payload').id
            model.body_ipos[bid] = np.asarray(payload_com_m, dtype=float)
        if mount_abc_deg != (0, 0, 0):
            sid = model.site('ft_site').id
            quat = fc.abc_deg_to_quat(*mount_abc_deg)
            model.site_quat[sid] = quat
            # The compiler sets site_sameframe=1 because ft_site has an
            # identity pose relative to its parent body; that flag makes the
            # kinematics reuse the body frame and ignore a post-compile
            # site_quat edit, so the mount rotation would silently no-op.
            # Clear it so the edited orientation actually drives site_xmat.
            model.site_sameframe[sid] = 0
        if not wall_enabled:
            gid = model.geom('wall_geom').id
            model.geom_contype[gid] = 0
            model.geom_conaffinity[gid] = 0
        return model

    def _flange_pose6(self):
        bid = self._flange_bid
        pos_m = self.data.xpos[bid].copy()
        quat = self.data.xquat[bid].copy()  # [w,x,y,z]
        a, b, c = fc.quat_to_abc_deg(quat)
        return (pos_m[0] * fc.MM_PER_M, pos_m[1] * fc.MM_PER_M,
                pos_m[2] * fc.MM_PER_M, a, b, c)

    def set_target_pose(self, pose6_mm_deg):
        x, y, z, a, b, c = pose6_mm_deg
        self.data.mocap_pos[self._mocap_id] = np.array(
            [x, y, z]) / fc.MM_PER_M
        self.data.mocap_quat[self._mocap_id] = fc.abc_deg_to_quat(a, b, c)

    def step(self, n=1):
        for _ in range(n):
            mujoco.mj_step(self.model, self.data)

    def get_actual_pose(self):
        return self._flange_pose6()

    def get_sensor_wrench(self):
        # MuJoCo force/torque sensors report the interaction across the site
        # body's connection to its parent, expressed in the SITE frame. The
        # reported force is what the parent exerts on the child; for a payload
        # hanging under gravity that reaction points opposite to gravity, i.e.
        # +Z(site) when upright. We negate so the returned wrench is the load
        # the sensor "feels" (the force the payload+contact exert on the
        # flange), matching the physical FT-sensor convention. The gravity
        # magnitude test is sign-insensitive; the orientation test is a
        # difference; both remain valid under this negation.
        f = -self.data.sensordata[self._force_adr:self._force_adr + 3].copy()
        t = -self.data.sensordata[self._torque_adr:self._torque_adr + 3].copy()
        return (f[0], f[1], f[2], t[0], t[1], t[2])

    def push_into_wall(self, depth_mm):
        """Test helper: move target from current flange pose toward the wall
        normal by depth_mm so contact builds up."""
        p = self._flange_pose6()
        d = self._wall_normal_world * (-depth_mm)  # into +X (toward wall)
        self.set_target_pose((p[0] + d[0], p[1] + d[1], p[2] + d[2],
                              p[3], p[4], p[5]))
