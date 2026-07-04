// Scripted rosbridge probe: drives the REAL websocket (node
// --experimental-websocket) against a running sim stack + web.launch.
// Exercises the three transport patterns the UI uses — subscribe,
// service call, action goal/feedback/result — and prints PASS/FAIL
// lines. Exit 0 only if every check passed. Bounded: hard 240 s cap
// (a full sim calibration measured 137 s — nine EKI PTP round-trips
// pace the MOVING phases, so the plan's original 90 s cap cannot fit).
//
// Usage:
//   roslaunch soft_robot_bringup sim.launch        (terminal 1)
//   roslaunch soft_robot_web_interface web.launch  (terminal 2)
//   node --experimental-websocket test/integration/probe_rosbridge.mjs
//
// ROSBRIDGE_PORT env overrides the default 9090 (pass the same value as
// web.launch rosbridge_port:= when 9090 is taken on the dev machine).
import { RosClient } from '../../www/js/ros_client.js';
import { ActionClient, GOAL_STATUS } from '../../www/js/action_client.js';

const checks = [];
function check(name, ok, detail = '') {
  checks.push(ok);
  console.log((ok ? 'PASS' : 'FAIL') + ' ' + name +
              (detail ? ' — ' + detail : ''));
}
function waitFor(pred, ms) {
  return new Promise((resolve) => {
    const t0 = Date.now();
    const h = setInterval(() => {
      if (pred() || Date.now() - t0 > ms) {
        clearInterval(h);
        resolve(pred());
      }
    }, 100);
  });
}

setTimeout(() => { console.log('FAIL global 240 s cap'); process.exit(2); },
           240000).unref();

const port = process.env.ROSBRIDGE_PORT || '9090';
const ros = new RosClient('ws://127.0.0.1:' + port, (u) => new WebSocket(u));
let wsUp = false;
ros.onStatus = (up) => { wsUp = up; };
ros.connect();
check('rosbridge socket', await waitFor(() => wsUp, 5000));

// 1. subscribe: manager_state arrives and reaches READY (sim stack).
let mgr = null;
ros.subscribe('/soft_robot/manager_state', 'soft_robot_msgs/ManagerState',
              (m) => { mgr = m; });
check('manager_state received', await waitFor(() => mgr !== null, 5000));
check('manager READY (state 2)',
      await waitFor(() => mgr && mgr.system_state === 2, 20000),
      mgr ? 'state=' + mgr.system_state : 'no msg');

// 2. wrench stream through the UI's own throttle settings.
let wrenches = 0;
ros.subscribe('/sri_ft/wrench_raw', 'geometry_msgs/WrenchStamped',
              () => { wrenches += 1; }, { throttleMs: 50, queueLength: 1 });
check('throttled wrench stream', await waitFor(() => wrenches >= 10, 5000),
      wrenches + ' msgs');

// 3. service round-trips: gate rejection then accepted start/stop.
//    (zero_sensor-in-READY runs AFTER the calibration below: zeroing
//    biases the driver stream to 0, and the fit check needs the raw
//    fz=5 mock signal. The SERVOING attempt is rejected, so it does
//    not zero anything.)
const start = await ros.callService('/soft_robot/start_servo',
                                    { mode: 2, profile: 0 });
check('start_servo accepted', start.success === true, start.message);
check('SERVOING (state 3)',
      await waitFor(() => mgr && mgr.system_state === 3, 5000));
const zeroInServo = await ros.callService('/soft_robot/zero_sensor', {});
check('zero_sensor REJECTED while SERVOING', zeroInServo.success === false,
      zeroInServo.message);
const stop = await ros.callService('/soft_robot/stop_servo', {});
check('stop_servo accepted', stop.success === true, stop.message);
check('back to READY',
      await waitFor(() => mgr && mgr.system_state === 2, 10000));

// 4. action channel: full calibration against the sim mocks
//    (sim.launch cuts samples_per_pose to 20; expected fit for the
//    constant fz=5 mock: gravity ~ 0, bias ~ (0,0,5), r2_force = 1 —
//    derivation in the bringup README / sim.launch comments).
const cal = new ActionClient(ros, '/soft_robot/calibrate_payload',
                             'soft_robot_msgs/CalibratePayload');
let feedbacks = 0;
let calDone = null;
cal.onFeedback = () => { feedbacks += 1; };
cal.onResult = (status, res) => { calDone = { status, res }; };
check('calibration goal sent', cal.sendGoal({}) === true);
check('calibration finished', await waitFor(() => calDone !== null, 180000));
if (calDone) {
  check('calibration SUCCEEDED',
        calDone.status === GOAL_STATUS.SUCCEEDED,
        'status=' + calDone.status + ' msg=' + calDone.res.message);
  check('feedback flowed', feedbacks >= 5, feedbacks + ' frames');
  check('fit sane (|G|<1, bias_fz≈5, r2_force>0.999)',
        Math.abs(calDone.res.gravity_n) < 1.0 &&
        Math.abs(calDone.res.bias_fz - 5.0) < 0.5 &&
        calDone.res.r2_force > 0.999,
        'G=' + calDone.res.gravity_n.toFixed(3) +
        ' bias_fz=' + calDone.res.bias_fz.toFixed(3) +
        ' r2=' + calDone.res.r2_force.toFixed(5));
}

// 5. zero_sensor accepted in READY (moved after the calibration — see
//    the note above section 3).
const zeroInReady = await ros.callService('/soft_robot/zero_sensor', {});
check('zero_sensor in READY ok', zeroInReady.success === true,
      zeroInReady.message);

ros.close();
const failed = checks.filter((c) => !c).length;
console.log(checks.length + ' checks, ' + failed + ' failed');
process.exit(failed === 0 ? 0 : 1);
