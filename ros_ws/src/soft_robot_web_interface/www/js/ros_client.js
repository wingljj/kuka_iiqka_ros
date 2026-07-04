// Minimal rosbridge v2.0 protocol client (Plan 6 decision 1: no external
// dependencies; this subset — subscribe/unsubscribe/advertise/publish/
// call_service — is everything the operator UI needs). Pure logic: the
// WebSocket implementation and timer functions are injected so node:test
// drives it with fakes. Reconnects forever with a fixed bounded delay;
// close() is final. Not a general roslib replacement.
export class RosClient {
  constructor(url, wsFactory, opts = {}) {
    this._url = url;
    this._wsFactory = wsFactory;
    this._reconnectMs = opts.reconnectMs ?? 1000;
    this._callTimeoutMs = opts.callTimeoutMs ?? 3000;
    this._setTimeout = opts.setTimeoutFn ?? ((fn, ms) => setTimeout(fn, ms));
    this._clearTimeout = opts.clearTimeoutFn ?? ((h) => clearTimeout(h));
    this._ws = null;
    this._up = false;
    this._closed = false;
    this._nextId = 1;
    this._subs = new Map();       // topic -> {type, opts, cbs:Set}
    this._advertised = new Set(); // topics advertised on current socket
    this._pending = new Map();    // call id -> {resolve, reject, timer}
    this.onStatus = null;         // (connected: boolean) => void
  }

  connect() {
    if (this._closed || this._ws) return;
    const ws = this._wsFactory(this._url);
    this._ws = ws;
    ws.onopen = () => {
      this._up = true;
      this._advertised.clear();
      for (const [topic, s] of this._subs) this._sendSubscribe(topic, s);
      if (this.onStatus) this.onStatus(true);
    };
    ws.onclose = () => this._dropped();
    ws.onerror = () => {};        // onclose always follows
    ws.onmessage = (ev) => this._handle(ev.data);
  }

  close() {
    this._closed = true;
    const ws = this._ws;
    this._ws = null;              // _dropped() sees the explicit close
    if (ws) ws.close();
  }

  _dropped() {
    const wasUp = this._up;
    this._up = false;
    this._ws = null;
    for (const [, p] of this._pending) {
      this._clearTimeout(p.timer);
      p.reject(new Error('rosbridge connection lost'));
    }
    this._pending.clear();
    if (wasUp && this.onStatus) this.onStatus(false);
    if (!this._closed) {
      this._setTimeout(() => this.connect(), this._reconnectMs);
    }
  }

  _send(obj) {
    if (this._up && this._ws) this._ws.send(JSON.stringify(obj));
  }

  _sendSubscribe(topic, s) {
    const frame = { op: 'subscribe', id: 'sub:' + topic, topic, type: s.type };
    if (s.opts.throttleMs !== undefined) frame.throttle_rate = s.opts.throttleMs;
    if (s.opts.queueLength !== undefined) frame.queue_length = s.opts.queueLength;
    this._send(frame);
  }

  subscribe(topic, type, cb, opts = {}) {
    let s = this._subs.get(topic);
    if (!s) {
      s = { type, opts, cbs: new Set() };
      this._subs.set(topic, s);
      this._sendSubscribe(topic, s);
    }
    s.cbs.add(cb);
  }

  unsubscribe(topic, cb) {
    const s = this._subs.get(topic);
    if (!s) return;
    s.cbs.delete(cb);
    if (s.cbs.size === 0) {
      this._subs.delete(topic);
      this._send({ op: 'unsubscribe', id: 'sub:' + topic, topic });
    }
  }

  publish(topic, type, msg) {
    if (!this._advertised.has(topic)) {
      this._advertised.add(topic);
      this._send({ op: 'advertise', id: 'adv:' + topic, topic, type });
    }
    this._send({ op: 'publish', topic, msg });
  }

  callService(service, args = {}) {
    const id = 'call:' + service + ':' + this._nextId++;
    return new Promise((resolve, reject) => {
      const timer = this._setTimeout(() => {
        this._pending.delete(id);
        reject(new Error('service call timed out: ' + service));
      }, this._callTimeoutMs);
      this._pending.set(id, { resolve, reject, timer });
      this._send({ op: 'call_service', id, service, args });
    });
  }

  _handle(text) {
    let m;
    try { m = JSON.parse(text); } catch { return; }
    if (m.op === 'publish') {
      const s = this._subs.get(m.topic);
      if (s) for (const cb of s.cbs) cb(m.msg);
    } else if (m.op === 'service_response') {
      const p = this._pending.get(m.id);
      if (!p) return;
      this._pending.delete(m.id);
      this._clearTimeout(p.timer);
      if (m.result) p.resolve(m.values ?? {});
      else p.reject(new Error(String(m.values ?? 'service call failed')));
    }
  }
}
