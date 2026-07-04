// actionlib client over the rosbridge topic surface (goal/cancel/
// feedback/result). rosbridge 0.11 has no first-class ROS1 action op,
// so this speaks the SimpleActionServer wire protocol directly.
// Single in-flight goal (matches the server's SimpleActionServer
// semantics). Injected clock for tests.
export const GOAL_STATUS = Object.freeze({
  PENDING: 0, ACTIVE: 1, PREEMPTED: 2, SUCCEEDED: 3, ABORTED: 4,
});

export class ActionClient {
  constructor(ros, ns, actionType, opts = {}) {
    this._ros = ros;
    this._ns = ns;
    this._type = actionType;
    this._nowMs = opts.nowMs ?? (() => Date.now());
    this._goalId = null;
    this.onFeedback = null;   // (feedback) => void
    this.onResult = null;     // (status, result) => void
    ros.subscribe(ns + '/feedback', actionType + 'ActionFeedback', (m) => {
      if (this._goalId && m.status.goal_id.id === this._goalId &&
          this.onFeedback) {
        this.onFeedback(m.feedback);
      }
    });
    ros.subscribe(ns + '/result', actionType + 'ActionResult', (m) => {
      if (!this._goalId || m.status.goal_id.id !== this._goalId) return;
      this._goalId = null;
      if (this.onResult) this.onResult(m.status.status, m.result);
    });
  }

  activeGoalId() { return this._goalId; }

  sendGoal(goal) {
    if (this._goalId) return false;   // single in-flight goal
    const id = 'web_' + this._nowMs() + '_' +
               Math.floor(Math.random() * 0xffff).toString(16);
    this._goalId = id;
    this._ros.publish(this._ns + '/goal', this._type + 'ActionGoal', {
      goal_id: { stamp: { secs: 0, nsecs: 0 }, id },
      goal,
    });
    return true;
  }

  cancel() {
    if (!this._goalId) return;
    this._ros.publish(this._ns + '/cancel', 'actionlib_msgs/GoalID',
                      { stamp: { secs: 0, nsecs: 0 }, id: this._goalId });
  }
}
