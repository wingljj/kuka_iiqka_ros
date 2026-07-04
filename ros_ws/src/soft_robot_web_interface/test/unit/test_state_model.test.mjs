import { test } from 'node:test';
import assert from 'node:assert/strict';
import {
  STATE, STATE_NAMES, MODE_NAMES, PROFILE_NAMES,
  linkBadges, buttonGates, presentState, controllersHint, calPhaseLabel,
} from '../../www/js/state_model.js';

function mgr(over = {}) {
  return Object.assign({
    system_state: STATE.READY, mode: 0, profile: 0,
    eki_connected: true, eki_program_ready: true,
    rsi_connected: false, rsi_fault: false,
    sri_streaming: true, tool_synced: true,
    calibrating: false, active_controller: '',
  }, over);
}

test('name tables cover the full wire ranges', () => {
  assert.equal(STATE_NAMES.length, 7);
  assert.equal(STATE_NAMES[0], 'OFFLINE');
  assert.equal(STATE_NAMES[6], 'FAULT');
  assert.deepEqual(MODE_NAMES,
    ['IDLE', 'DIRECT_CARTESIAN', 'FORCE_COMPLIANCE', 'CALIBRATION']);
  assert.deepEqual(PROFILE_NAMES, ['DRAG', 'PRECISION']);
});

test('linkBadges reflects the three links', () => {
  const b = linkBadges(mgr(), { connected: true, state_fresh: true,
                               rsi_active: false });
  assert.equal(b.eki.ok, true);
  assert.equal(b.rsi.ok, false);
  assert.equal(b.sri.ok, true);
});

test('I-1 presentation: heartbeat pause during RSI phase is expected', () => {
  // EkiState.state_fresh false + manager rsi_connected true = MOVECORR
  // heartbeat pause (rule R1): informational, never an error badge.
  const b = linkBadges(mgr({ rsi_connected: true }),
                       { connected: true, state_fresh: false,
                         rsi_active: true });
  assert.equal(b.eki.ok, true);
  assert.match(b.eki.note, /heartbeat paused/i);
  // Without RSI alive the stale heartbeat IS a problem.
  const b2 = linkBadges(mgr({ eki_connected: false }),
                        { connected: true, state_fresh: false,
                          rsi_active: false });
  assert.equal(b2.eki.ok, false);
});

test('buttonGates in READY', () => {
  const g = buttonGates(mgr());
  assert.deepEqual(g, { startServo: true, stopServo: false,
                        resetFault: false, zeroSensor: true,
                        calibrate: true });
});

test('buttonGates in SERVOING and DEGRADED', () => {
  const s = buttonGates(mgr({ system_state: STATE.SERVOING }));
  assert.equal(s.startServo, false);
  assert.equal(s.stopServo, true);
  assert.equal(s.zeroSensor, false);   // decision 11 gate mirrored
  assert.equal(s.calibrate, false);
  const d = buttonGates(mgr({ system_state: STATE.DEGRADED }));
  assert.equal(d.stopServo, true);
});

test('buttonGates in FAULT and CALIBRATING', () => {
  const f = buttonGates(mgr({ system_state: STATE.FAULT }));
  assert.deepEqual(f, { startServo: false, stopServo: false,
                        resetFault: true, zeroSensor: false,
                        calibrate: false });
  const c = buttonGates(mgr({ system_state: STATE.CALIBRATING }));
  assert.equal(c.calibrate, false);    // cancel goes through the action
  assert.equal(c.zeroSensor, false);
});

test('zeroSensor also open in CONNECTED (decision 11)', () => {
  const g = buttonGates(mgr({ system_state: STATE.CONNECTED }));
  assert.equal(g.zeroSensor, true);
  assert.equal(g.startServo, false);
});

test('presentState grace window after stop (decision 15)', () => {
  const now = 100000;
  const f = mgr({ system_state: STATE.FAULT, rsi_fault: true });
  const inWindow = presentState(f, now - 5000, now);
  assert.equal(inWindow.level, 'info');
  assert.match(inWindow.text, /after stop/i);
  const outWindow = presentState(f, now - 15000, now);
  assert.equal(outWindow.level, 'error');
  const neverStopped = presentState(f, null, now);
  assert.equal(neverStopped.level, 'error');
});

test('presentState normal levels', () => {
  assert.equal(presentState(mgr(), null, 0).level, 'ok');
  assert.equal(presentState(mgr({ system_state: STATE.DEGRADED }),
                            null, 0).level, 'warn');
  assert.equal(presentState(mgr({ system_state: STATE.SERVOING }),
                            null, 0).level, 'ok');
});

test('controllersHint heuristic (decision 11 / followup Minor 7)', () => {
  // CONNECTED with every visible READY precondition true -> the missing
  // one is controllers_loaded (not a ManagerState field).
  const hinted = controllersHint(mgr({ system_state: STATE.CONNECTED }));
  assert.match(hinted, /controllers/i);
  assert.equal(controllersHint(mgr()), '');           // READY: no hint
  assert.equal(controllersHint(mgr({ system_state: STATE.CONNECTED,
                                     tool_synced: false })), '');
  assert.equal(calPhaseLabel('SAMPLING', 3, 8), 'SAMPLING pose 4/8');
});
