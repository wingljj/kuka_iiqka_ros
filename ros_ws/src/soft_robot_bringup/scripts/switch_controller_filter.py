#!/usr/bin/env python3
"""Lenient switch_controller proxy (Task 8 deviation, recorded).

ManagerRuntime::startServo (and the calibration entry) always lists the
sibling controller in stop_controllers, but from READY neither controller
is running and the controller_manager under STRICT refuses to stop a
controller that is not running, so every first start failed in the real
closed loop (Task 7's lambda-mock tests cannot see STRICT semantics).
Existing packages are frozen for this task, so the bringup remaps the
manager's /controller_manager/switch_controller to this proxy, which
drops no-op entries (stop of a non-running controller, start of an
already-running one) and forwards the remainder unchanged (STRICT).
Debt: fold this filtering into the manager itself and delete this node.
"""
import rospy
from controller_manager_msgs.srv import (ListControllers, SwitchController,
                                         SwitchControllerRequest,
                                         SwitchControllerResponse)

REAL_SWITCH = '/controller_manager/switch_controller'
REAL_LIST = '/controller_manager/list_controllers'
FILTERED = '/controller_manager/switch_controller_filtered'


def main():
    rospy.init_node('switch_controller_filter')
    switch = rospy.ServiceProxy(REAL_SWITCH, SwitchController)
    lst = rospy.ServiceProxy(REAL_LIST, ListControllers)

    def handle(req):
        try:
            running = set(c.name for c in lst().controller
                          if c.state == 'running')
        except rospy.ServiceException as e:
            rospy.logerr('switch_controller_filter: list failed: %s', e)
            return SwitchControllerResponse(ok=False)
        fwd = SwitchControllerRequest()
        fwd.start_controllers = [c for c in req.start_controllers
                                 if c not in running]
        fwd.stop_controllers = [c for c in req.stop_controllers
                                if c in running]
        fwd.strictness = req.strictness
        fwd.start_asap = req.start_asap
        fwd.timeout = req.timeout
        if not fwd.start_controllers and not fwd.stop_controllers:
            return SwitchControllerResponse(ok=True)
        try:
            return switch(fwd)
        except rospy.ServiceException as e:
            rospy.logerr('switch_controller_filter: switch failed: %s', e)
            return SwitchControllerResponse(ok=False)

    rospy.Service(FILTERED, SwitchController, handle)
    rospy.loginfo('switch_controller_filter: up (%s -> %s)',
                  FILTERED, REAL_SWITCH)
    rospy.spin()


if __name__ == '__main__':
    main()
