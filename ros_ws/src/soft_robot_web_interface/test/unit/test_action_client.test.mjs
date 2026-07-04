import { test } from 'node:test';
import assert from 'node:assert/strict';
import { ActionClient, GOAL_STATUS } from '../../www/js/action_client.js';

// Fake RosClient: records publishes, exposes subscribe callbacks.
class FakeRos {
  constructor() { this.pubs = []; this.subs = new Map(); }
  publish(topic, type, msg) { this.pubs.push({ topic, type, msg }); }
  subscribe(topic, type, cb) { this.subs.set(topic, cb); }
  unsubscribe() {}
  inject(topic, msg) { this.subs.get(topic)(msg); }
}

const NS = '/soft_robot/calibrate_payload';
const TYPE = 'soft_robot_msgs/CalibratePayload';

test('sendGoal publishes an ActionGoal with a fresh goal_id', () => {
  const ros = new FakeRos();
  const ac = new ActionClient(ros, NS, TYPE, { nowMs: () => 42 });
  ac.sendGoal({});
  const g = ros.pubs.find((p) => p.topic === NS + '/goal');
  assert.ok(g);
  assert.equal(g.type, TYPE + 'ActionGoal');
  assert.match(g.msg.goal_id.id, /^web_42_/);
  assert.deepEqual(g.msg.goal, {});
});

test('feedback for the active goal fans out, others filtered', () => {
  const ros = new FakeRos();
  const ac = new ActionClient(ros, NS, TYPE, { nowMs: () => 1 });
  const fbs = [];
  ac.onFeedback = (fb) => fbs.push(fb);
  ac.sendGoal({});
  const id = ac.activeGoalId();
  ros.inject(NS + '/feedback', { status: { goal_id: { id } },
                                 feedback: { phase: 'MOVING', pose_index: 0 } });
  ros.inject(NS + '/feedback', { status: { goal_id: { id: 'other' } },
                                 feedback: { phase: 'X' } });
  assert.equal(fbs.length, 1);
  assert.equal(fbs[0].phase, 'MOVING');
});

test('result resolves with terminal status and payload', () => {
  const ros = new FakeRos();
  const ac = new ActionClient(ros, NS, TYPE, { nowMs: () => 1 });
  const results = [];
  ac.onResult = (status, res) => results.push({ status, res });
  ac.sendGoal({});
  const id = ac.activeGoalId();
  ros.inject(NS + '/result', {
    status: { goal_id: { id }, status: GOAL_STATUS.SUCCEEDED },
    result: { success: true, gravity_n: 42.5, r2_force: 1.0 },
  });
  assert.equal(results.length, 1);
  assert.equal(results[0].status, GOAL_STATUS.SUCCEEDED);
  assert.equal(results[0].res.gravity_n, 42.5);
  assert.equal(ac.activeGoalId(), null);   // terminal clears the goal
});

test('cancel publishes GoalID for the active goal', () => {
  const ros = new FakeRos();
  const ac = new ActionClient(ros, NS, TYPE, { nowMs: () => 1 });
  ac.sendGoal({});
  const id = ac.activeGoalId();
  ac.cancel();
  const c = ros.pubs.find((p) => p.topic === NS + '/cancel');
  assert.equal(c.type, 'actionlib_msgs/GoalID');
  assert.equal(c.msg.id, id);
});

test('PREEMPTED terminal status is distinguishable (decision 10)', () => {
  const ros = new FakeRos();
  const ac = new ActionClient(ros, NS, TYPE, { nowMs: () => 1 });
  const results = [];
  ac.onResult = (status) => results.push(status);
  ac.sendGoal({});
  const id = ac.activeGoalId();
  ros.inject(NS + '/result', {
    status: { goal_id: { id }, status: GOAL_STATUS.PREEMPTED },
    result: { success: false },
  });
  assert.deepEqual(results, [GOAL_STATUS.PREEMPTED]);
});

test('second sendGoal while active is refused (single in-flight)', () => {
  const ros = new FakeRos();
  const ac = new ActionClient(ros, NS, TYPE, { nowMs: () => 1 });
  assert.equal(ac.sendGoal({}), true);
  assert.equal(ac.sendGoal({}), false);
  const goals = ros.pubs.filter((p) => p.topic === NS + '/goal');
  assert.equal(goals.length, 1);
});
