#!/usr/bin/env python3
"""End-to-end force-compliance smoke: prove the force -> controller -> motion
chain actually moves the arm (plan Task 26).

This is the ONE check that exercises the full closed loop the way the user
does by hand: bring up mujoco_sim.launch, start servo in FORCE_COMPLIANCE,
inject an external drag past the deadband, and assert the flange (TCP) moves.
It is the automation of "启动伺服 -> 施力 -> 运动" so we stop verifying it by eye.

Run it inside a sourced workspace:
    source devel/setup.bash
    rosrun kuka_mujoco_sim e2e_compliance_smoke.py          # self-launches stack
    # or against an already-running stack:
    rosrun kuka_mujoco_sim e2e_compliance_smoke.py --external

Honest exit codes: 0 = PASS (moved), 1 = FAIL (came up but did not move),
2 = NOT-RUN (stack/env could not be brought up -- e.g. no ROS master, MuJoCo
missing, or READY never reached). NOT-RUN is not a pass; it says so plainly.

Config: mount=[0,0,0] + payload=2.0 kg isolates the main-chain closure the
plan asked to nail first (gravity fully on -Z, compensated by the launch
bridge; drag on +X). The mount!=0 alignment is covered separately by the
offline test/test_gravity_equivalence.py.
"""
import subprocess
import sys
import time

DRAG_N = 40.0            # +X external push, well past the ~5 N settled deadband
REST_S = 4.0            # let adaptive deadband settle + auto_retare at rest
DRAG_S = 5.0            # hold the drag (velocity compliance ~ mm/s * s)
MOVE_THRESHOLD_MM = 5.0  # flange must travel at least this far along +X
READY_TIMEOUT_S = 90.0   # generous: EKI + SRI + RSI + READY in sim


def _log(msg):
    print("[e2e] " + msg, flush=True)


def main():
    external = "--external" in sys.argv
    launch = None
    try:
        import rospy
        from geometry_msgs.msg import Wrench, PointStamped
        from soft_robot_msgs.srv import StartServo
        from soft_robot_msgs.msg import ModeState
    except Exception as e:  # pragma: no cover - env guard
        _log("NOT-RUN: ROS / message imports unavailable: %r" % e)
        return 2

    if not external:
        _log("launching mujoco_sim.launch (headless, payload=2.0, mount=[0,0,0])")
        launch = subprocess.Popen(
            ["roslaunch", "kuka_mujoco_sim", "mujoco_sim.launch",
             "gui:=false", "payload_mass:=2.0", "mount_abc:=[0.0, 0.0, 0.0]",
             "wall_enabled:=false"],
            stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)

    try:
        # Wait for the master + node graph, then init.
        deadline = time.time() + 30
        while time.time() < deadline:
            try:
                rospy.init_node("e2e_compliance_smoke", anonymous=True,
                                disable_signals=True)
                break
            except Exception:
                time.sleep(1)
        else:
            _log("NOT-RUN: could not init_node (no master)")
            return 2

        # --- state observers ---
        latest = {"state": None, "pos": None}

        def on_state(m):
            latest["state"] = m.system_state

        def on_pos(m):
            latest["pos"] = m.point.x

        rospy.Subscriber("/soft_robot/mode_state", ModeState, on_state)
        rospy.Subscriber("/kuka_mujoco_sim/flange_position", PointStamped, on_pos)
        drag_pub = rospy.Publisher("/kuka_mujoco_sim/drag_force", Wrench,
                                   queue_size=1)

        # --- start servo (retry until READY reached) ---
        _log("waiting for /soft_robot/start_servo ...")
        try:
            rospy.wait_for_service("/soft_robot/start_servo",
                                   timeout=READY_TIMEOUT_S)
        except rospy.ROSException:
            _log("NOT-RUN: start_servo service never appeared")
            return 2
        start = rospy.ServiceProxy("/soft_robot/start_servo", StartServo)

        servoing = False
        deadline = time.time() + READY_TIMEOUT_S
        last_msg = ""
        while time.time() < deadline and not rospy.is_shutdown():
            resp = start(mode=2, profile=0)  # 2 = MODE_FORCE_COMPLIANCE
            if resp.success:
                servoing = True
                break
            if resp.message != last_msg:
                last_msg = resp.message
                _log("start_servo not ready yet: %s" % resp.message)
            time.sleep(2)
        if not servoing:
            _log("NOT-RUN: never reached READY to start servo (last: %s)"
                 % last_msg)
            return 2
        _log("servo started (FORCE_COMPLIANCE)")

        # Confirm the gate is actually engaged (SERVOING = gate on, wrench
        # fresh, not degraded).
        deadline = time.time() + 10
        while time.time() < deadline and latest["state"] != 3:
            time.sleep(0.2)
        if latest["state"] != 3:
            _log("FAIL: system_state never reached SERVOING (got %r)"
                 % latest["state"])
            return 1
        _log("system_state = SERVOING (3): gate engaged")

        # --- rest so the adaptive deadband settles to ~5 N (not the drag) ---
        _log("resting %.1fs to settle deadband + auto-retare ..." % REST_S)
        time.sleep(REST_S)
        deadline = time.time() + 5
        while time.time() < deadline and latest["pos"] is None:
            time.sleep(0.2)
        if latest["pos"] is None:
            _log("NOT-RUN: no flange_position published")
            return 2
        baseline = latest["pos"]
        _log("baseline flange X = %.2f mm" % baseline)

        # --- apply the drag AFTER the deadband settled ---
        _log("applying +X %.0f N drag for %.1fs ..." % (DRAG_N, DRAG_S))
        w = Wrench()
        w.force.x = DRAG_N
        rate = rospy.Rate(50)
        t_end = time.time() + DRAG_S
        while time.time() < t_end and not rospy.is_shutdown():
            drag_pub.publish(w)
            rate.sleep()
        # release
        drag_pub.publish(Wrench())
        time.sleep(0.5)

        final = latest["pos"]
        dx = final - baseline
        _log("final flange X = %.2f mm (dx = %+.2f mm)" % (final, dx))

        if dx > MOVE_THRESHOLD_MM:
            _log("PASS: flange moved %+.2f mm along the applied force "
                 "(threshold %.1f mm). Force -> controller -> motion chain "
                 "is closed." % (dx, MOVE_THRESHOLD_MM))
            return 0
        _log("FAIL: flange moved only %+.2f mm (< %.1f mm). The loop came up "
             "but did not comply." % (dx, MOVE_THRESHOLD_MM))
        return 1
    finally:
        if launch is not None:
            _log("shutting down stack ...")
            launch.terminate()
            try:
                launch.wait(timeout=15)
            except subprocess.TimeoutExpired:
                launch.kill()


if __name__ == "__main__":
    sys.exit(main())
