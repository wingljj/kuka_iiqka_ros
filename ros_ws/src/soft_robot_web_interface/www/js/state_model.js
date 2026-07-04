// Pure view-model derivations for the operator UI. No DOM, no network:
// everything here is testable with plain objects (node:test). Wire
// values mirror soft_robot_msgs (test_msgs.cpp pins them on the C++
// side; test_state_model pins the copies here).
export const STATE = Object.freeze({
  OFFLINE: 0, CONNECTED: 1, READY: 2, SERVOING: 3,
  CALIBRATING: 4, DEGRADED: 5, FAULT: 6,
});
export const STATE_NAMES = Object.freeze(['OFFLINE', 'CONNECTED', 'READY',
  'SERVOING', 'CALIBRATING', 'DEGRADED', 'FAULT']);
export const MODE_NAMES = Object.freeze(['IDLE', 'DIRECT_CARTESIAN',
  'FORCE_COMPLIANCE', 'CALIBRATION']);
export const PROFILE_NAMES = Object.freeze(['DRAG', 'PRECISION']);

// Grace window (decision 15): a FAULT appearing this soon after the
// operator's own stop is the expected I-1 stop-chain endgame on cells
// without a Stop-S MOVECORR break, not an alarm.
export const STOP_GRACE_MS = 10000;

// Three-link badges. I-1 presentation rule: a stale EKI heartbeat while
// the RSI stream is alive is the documented MOVECORR pause (rule R1) —
// show the EKI link as OK with an informational note, never as an error.
export function linkBadges(m, eki) {
  const rsiAlive = !!m.rsi_connected;
  const hbStale = !!(eki && eki.connected && !eki.state_fresh);
  const ekiOk = !!m.eki_connected || (hbStale && rsiAlive);
  return {
    eki: {
      ok: ekiOk,
      note: (hbStale && rsiAlive)
        ? 'heartbeat paused (RSI active, expected)' : '',
    },
    rsi: { ok: rsiAlive, note: m.rsi_fault ? 'fault latched' : '' },
    sri: { ok: !!m.sri_streaming, note: '' },
  };
}

// Client-side pre-judgement of the manager gates. The manager remains
// the sole authority (spec 13: dangerous operations go through manager
// services with state checks); these only suppress calls that would
// certainly be rejected.
export function buttonGates(m) {
  const s = m.system_state;
  return {
    startServo: s === STATE.READY,
    stopServo: s === STATE.SERVOING || s === STATE.DEGRADED,
    resetFault: s === STATE.FAULT,
    zeroSensor: s === STATE.CONNECTED || s === STATE.READY, // decision 11
    calibrate: s === STATE.READY,
  };
}

// Header banner: level 'ok' | 'info' | 'warn' | 'error' + text.
export function presentState(m, lastStopAtMs, nowMs) {
  const s = m.system_state;
  if (s === STATE.FAULT) {
    if (lastStopAtMs !== null && lastStopAtMs !== undefined &&
        nowMs - lastStopAtMs <= STOP_GRACE_MS) {
      return { level: 'info',
               text: 'RSI session ended after stop (expected) — ' +
                     'press Reset Fault' };
    }
    return { level: 'error', text: 'FAULT latched: reset required' };
  }
  if (s === STATE.DEGRADED) {
    return { level: 'warn', text: 'DEGRADED: output forced to zero' };
  }
  return { level: 'ok', text: STATE_NAMES[s] ?? ('state ' + s) };
}

// Decision 11 heuristic for the missing controllers_loaded field: stuck
// in CONNECTED with every visible READY precondition true means the
// invisible one (controller preload) is the blocker.
export function controllersHint(m) {
  if (m.system_state !== STATE.CONNECTED) return '';
  if (m.eki_connected && m.eki_program_ready && m.sri_streaming &&
      m.tool_synced) {
    return 'controllers not loaded yet (manager preload pending — is ' +
           'the controller_manager up?)';
  }
  return '';
}

export function calPhaseLabel(phase, poseIndex, poseCount) {
  return phase + ' pose ' + (poseIndex + 1) + '/' + poseCount;
}
