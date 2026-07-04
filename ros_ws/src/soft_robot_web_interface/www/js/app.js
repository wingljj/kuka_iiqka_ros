// DOM wiring shell: everything testable lives in the imported modules.
// Dangerous operations go exclusively through the manager services and
// the calibrate_payload action (spec 13; decision 7).
import { RosClient } from './ros_client.js';
import { ActionClient, GOAL_STATUS } from './action_client.js';
import {
  STATE_NAMES, MODE_NAMES, PROFILE_NAMES,
  linkBadges, buttonGates, presentState, controllersHint, calPhaseLabel,
} from './state_model.js';
import { TraceBuffer, drawChart } from './wrench_chart.js';

const $ = (id) => document.getElementById(id);
// rosbridge port defaults to 9090; override with ?ws=<port> when the
// default collides with another service (e.g. a local proxy) and
// web.launch was started with rosbridge_port:=<port>.
const wsPort = new URLSearchParams(location.search).get('ws') || '9090';
const wsUrl = 'ws://' + location.hostname + ':' + wsPort;
const ros = new RosClient(wsUrl, (url) => new WebSocket(url));

let lastMgr = null;
let lastEki = null;
let lastStopAtMs = null;

function setBadge(el, ok) {
  el.classList.toggle('on', ok);
  el.classList.toggle('off', !ok);
}

function cmdResult(text) { $('cmd-result').textContent = text; }

function render() {
  if (!lastMgr) return;
  const m = lastMgr;
  const banner = presentState(m, lastStopAtMs, Date.now());
  const header = document.querySelector('header');
  header.className = banner.level;
  $('banner-state').textContent = STATE_NAMES[m.system_state] ?? '?';
  $('banner-note').textContent = banner.text === STATE_NAMES[m.system_state]
    ? '' : banner.text;

  const badges = linkBadges(m, lastEki);
  setBadge($('badge-eki'), badges.eki.ok);
  setBadge($('badge-rsi'), badges.rsi.ok);
  setBadge($('badge-sri'), badges.sri.ok);
  $('kv-ekinote').textContent = badges.eki.note || '—';

  $('kv-state').textContent =
    m.system_state + ' (' + (STATE_NAMES[m.system_state] ?? '?') + ')';
  $('kv-mode').textContent = (MODE_NAMES[m.mode] ?? m.mode) + ' / ' +
    (PROFILE_NAMES[m.profile] ?? m.profile);
  $('kv-controller').textContent = m.active_controller || '(none)';
  $('kv-tool').textContent = m.tool_synced ? 'yes' : 'no';
  $('kv-hint').textContent = controllersHint(m) || '—';
  $('kv-rsifault').textContent = m.rsi_fault ? 'LATCHED' : 'no';

  const g = buttonGates(m);
  $('btn-start').disabled = !g.startServo;
  $('btn-stop').disabled = !g.stopServo;
  $('btn-zero').disabled = !g.zeroSensor;
  $('btn-reset').disabled = !g.resetFault;
  $('btn-cal-start').disabled = !g.calibrate;
  $('btn-cal-cancel').disabled = !m.calibrating;
}

// --- rosbridge link status ---
ros.onStatus = (up) => {
  setBadge($('ws-status'), up);
  if (!up) cmdResult('rosbridge connection lost — reconnecting');
};
ros.connect();

// --- subscriptions ---
ros.subscribe('/soft_robot/manager_state', 'soft_robot_msgs/ManagerState',
  (m) => { lastMgr = m; render(); });
ros.subscribe('/kuka/eki/state', 'soft_robot_msgs/EkiState', (m) => {
  lastEki = m;
  $('kv-eki-age').textContent =
    m.reconnects + ' / ' + m.state_age.toFixed(2);
  $('kv-tool-frame').textContent =
    [m.tool_x, m.tool_y, m.tool_z, m.tool_a, m.tool_b, m.tool_c]
      .map((v) => v.toFixed(1)).join(', ');
  render();
}, { throttleMs: 200 });
ros.subscribe('/kuka/rsi/state', 'soft_robot_msgs/RsiState', (m) => {
  $('kv-ipoc').textContent = m.ipoc;
  $('kv-timeouts').textContent =
    m.total_timeouts + ' / ' + m.consecutive_timeouts;
  $('kv-badframes').textContent = m.bad_frames + ' / ' + m.ipoc_jumps;
  $('kv-sat').textContent = m.saturation_count;
}, { throttleMs: 200 });
ros.subscribe('/sri_ft/status', 'soft_robot_msgs/SriStatus', (m) => {
  $('kv-sri-samples').textContent = m.samples + ' / ' + m.package_gaps;
  $('kv-sri-age').textContent = m.last_sample_age.toFixed(3);
}, { throttleMs: 200 });
ros.subscribe('/soft_robot/diagnostics',
  'diagnostic_msgs/DiagnosticArray', (m) => {
    const st = m.status && m.status[0];
    $('kv-diag').textContent = st ? st.message : '—';
  });

// --- wrench charts (decision 8: 50 ms throttle, 600-sample window) ---
const forceBuf = new TraceBuffer(3, 600);
const torqueBuf = new TraceBuffer(3, 600);
ros.subscribe('/sri_ft/wrench_raw', 'geometry_msgs/WrenchStamped', (m) => {
  const f = m.wrench.force;
  const t = m.wrench.torque;
  forceBuf.push([f.x, f.y, f.z]);
  torqueBuf.push([t.x, t.y, t.z]);
  $('kv-wrench').textContent =
    'F(' + f.x.toFixed(1) + ', ' + f.y.toFixed(1) + ', ' + f.z.toFixed(1) +
    ') T(' + t.x.toFixed(2) + ', ' + t.y.toFixed(2) + ', ' +
    t.z.toFixed(2) + ')';
}, { throttleMs: 50, queueLength: 1 });

function paint() {
  const fc = $('chart-force');
  const tc = $('chart-torque');
  drawChart(fc.getContext('2d'), fc.width, fc.height, forceBuf,
            ['Fx', 'Fy', 'Fz']);
  drawChart(tc.getContext('2d'), tc.width, tc.height, torqueBuf,
            ['Tx', 'Ty', 'Tz']);
  requestAnimationFrame(paint);
}
requestAnimationFrame(paint);

// --- manager service buttons ---
function call(service, args, label) {
  cmdResult(label + '…');
  ros.callService(service, args).then((res) => {
    cmdResult(label + ': ' + (res.success ? 'ok' : 'REJECTED') +
              (res.message ? ' — ' + res.message : ''));
  }).catch((err) => cmdResult(label + ': ' + err.message));
}
$('btn-start').onclick = () => call('/soft_robot/start_servo', {
  mode: Number($('sel-mode').value),
  profile: Number($('sel-profile').value),
}, 'start_servo');
$('btn-stop').onclick = () => {
  lastStopAtMs = Date.now();   // decision 15 grace window anchor
  call('/soft_robot/stop_servo', {}, 'stop_servo');
};
$('btn-zero').onclick = () => call('/soft_robot/zero_sensor', {},
                                   'zero_sensor');
$('btn-reset').onclick = () => call('/soft_robot/reset_fault', {},
                                    'reset_fault');

// --- calibration wizard ---
const cal = new ActionClient(ros, '/soft_robot/calibrate_payload',
                             'soft_robot_msgs/CalibratePayload');
cal.onFeedback = (fb) => {
  $('cal-phase').textContent =
    calPhaseLabel(fb.phase, fb.pose_index, fb.pose_count);
  $('cal-progress').max = fb.pose_count || 8;
  $('cal-progress').value = fb.pose_index;
};
cal.onResult = (status, res) => {
  const name = status === GOAL_STATUS.SUCCEEDED ? 'SUCCEEDED'
    : status === GOAL_STATUS.PREEMPTED ? 'CANCELLED' : 'ABORTED';
  $('cal-phase').textContent = 'finished: ' + name;
  $('cal-outcome').textContent = name + (res.message
    ? ' — ' + res.message : '');
  $('cal-g').textContent = res.gravity_n.toFixed(2);
  $('cal-com').textContent = [res.com_x, res.com_y, res.com_z]
    .map((v) => v.toFixed(4)).join(', ');
  $('cal-biasf').textContent = [res.bias_fx, res.bias_fy, res.bias_fz]
    .map((v) => v.toFixed(2)).join(', ');
  $('cal-biast').textContent = [res.bias_tx, res.bias_ty, res.bias_tz]
    .map((v) => v.toFixed(3)).join(', ');
  $('cal-r2').textContent =
    res.r2_force.toFixed(4) + ' / ' + res.r2_torque.toFixed(4);
};
$('btn-cal-start').onclick = () => {
  if (cal.sendGoal({})) $('cal-phase').textContent = 'goal sent…';
};
$('btn-cal-cancel').onclick = () => cal.cancel();
