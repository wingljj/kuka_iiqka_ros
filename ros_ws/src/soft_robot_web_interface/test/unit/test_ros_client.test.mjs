// node --test unit tests for the rosbridge protocol client. A FakeSocket
// stands in for WebSocket: tests inspect the frames the client sends and
// inject inbound frames; no network, no timers left running.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { RosClient } from '../../www/js/ros_client.js';

class FakeSocket {
  constructor() {
    this.sent = [];
    this.readyState = 0; // CONNECTING
    this.onopen = null; this.onclose = null;
    this.onerror = null; this.onmessage = null;
  }
  send(text) { this.sent.push(JSON.parse(text)); }
  close() {
    this.readyState = 3;
    if (this.onclose) this.onclose({});
  }
  // test helpers
  open() { this.readyState = 1; if (this.onopen) this.onopen({}); }
  inject(obj) {
    if (this.onmessage) this.onmessage({ data: JSON.stringify(obj) });
  }
}

function makeClient() {
  const sockets = [];
  const timers = [];
  const client = new RosClient('ws://test:9090', () => {
    const s = new FakeSocket();
    sockets.push(s);
    return s;
  }, {
    reconnectMs: 10,
    setTimeoutFn: (fn, ms) => { timers.push({ fn, ms }); return timers.length; },
    clearTimeoutFn: () => {},
  });
  return { client, sockets, timers };
}

test('connect reports status and flushes queued operations', () => {
  const { client, sockets } = makeClient();
  const status = [];
  client.onStatus = (up) => status.push(up);
  client.subscribe('/soft_robot/manager_state',
                   'soft_robot_msgs/ManagerState', () => {});
  client.connect();
  assert.equal(sockets.length, 1);
  assert.equal(sockets[0].sent.length, 0);   // nothing before open
  sockets[0].open();
  assert.deepEqual(status, [true]);
  const sub = sockets[0].sent.find((m) => m.op === 'subscribe');
  assert.ok(sub);
  assert.equal(sub.topic, '/soft_robot/manager_state');
  assert.equal(sub.type, 'soft_robot_msgs/ManagerState');
});

test('subscribe options map to throttle_rate and queue_length', () => {
  const { client, sockets } = makeClient();
  client.connect();
  sockets[0].open();
  client.subscribe('/sri_ft/wrench_raw', 'geometry_msgs/WrenchStamped',
                   () => {}, { throttleMs: 50, queueLength: 1 });
  const sub = sockets[0].sent.find((m) => m.topic === '/sri_ft/wrench_raw');
  assert.equal(sub.throttle_rate, 50);
  assert.equal(sub.queue_length, 1);
});

test('inbound publish frames fan out to topic callbacks', () => {
  const { client, sockets } = makeClient();
  const got = [];
  client.connect();
  sockets[0].open();
  client.subscribe('/kuka/rsi/state', 'soft_robot_msgs/RsiState',
                   (m) => got.push(m));
  sockets[0].inject({ op: 'publish', topic: '/kuka/rsi/state',
                      msg: { connected: true, ipoc: 7 } });
  sockets[0].inject({ op: 'publish', topic: '/other', msg: {} });
  assert.equal(got.length, 1);
  assert.equal(got[0].ipoc, 7);
});

test('callService resolves on matching service_response', async () => {
  const { client, sockets } = makeClient();
  client.connect();
  sockets[0].open();
  const p = client.callService('/soft_robot/stop_servo', {});
  const call = sockets[0].sent.find((m) => m.op === 'call_service');
  assert.ok(call.id);
  sockets[0].inject({ op: 'service_response', id: call.id,
                      result: true, values: { success: true, message: '' } });
  const res = await p;
  assert.equal(res.success, true);
});

test('callService rejects on result=false', async () => {
  const { client, sockets } = makeClient();
  client.connect();
  sockets[0].open();
  const p = client.callService('/soft_robot/start_servo',
                               { mode: 2, profile: 0 });
  const call = sockets[0].sent.find((m) => m.op === 'call_service');
  sockets[0].inject({ op: 'service_response', id: call.id,
                      result: false, values: 'service failed' });
  await assert.rejects(p, /service failed/);
});

test('publish sends advertise once then publish frames', () => {
  const { client, sockets } = makeClient();
  client.connect();
  sockets[0].open();
  client.publish('/x', 'std_msgs/Empty', {});
  client.publish('/x', 'std_msgs/Empty', {});
  const adv = sockets[0].sent.filter((m) => m.op === 'advertise');
  const pub = sockets[0].sent.filter((m) => m.op === 'publish');
  assert.equal(adv.length, 1);
  assert.equal(pub.length, 2);
});

test('socket loss schedules reconnect and re-subscribes', () => {
  const { client, sockets, timers } = makeClient();
  const status = [];
  client.onStatus = (up) => status.push(up);
  client.connect();
  sockets[0].open();
  client.subscribe('/kuka/eki/state', 'soft_robot_msgs/EkiState', () => {});
  sockets[0].close();                       // dropped
  assert.deepEqual(status, [true, false]);
  assert.equal(timers.length >= 1, true);   // reconnect scheduled
  timers[timers.length - 1].fn();           // fire it
  assert.equal(sockets.length, 2);
  sockets[1].open();
  const sub = sockets[1].sent.find((m) => m.op === 'subscribe');
  assert.equal(sub.topic, '/kuka/eki/state');
  assert.deepEqual(status, [true, false, true]);
});

test('close() is final: no reconnect after explicit close', () => {
  const { client, sockets, timers } = makeClient();
  client.connect();
  sockets[0].open();
  const before = timers.length;
  client.close();
  assert.equal(timers.length, before);      // nothing scheduled
  assert.equal(sockets.length, 1);
});
