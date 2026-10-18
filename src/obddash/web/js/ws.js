/* obddash WebSocket link: reconnecting client for the /ws telemetry stream (docs/architecture.md). */
(function () {
  'use strict';
  class Link {
    constructor(url) {
      this.url = url;
      this.ws = null;
      this.connected = false;
      this.lastFrame = 0;
      this.retry = 0;
      this.hello = null;
      this.onTelemetry = () => {};
      this.onEvent = () => {};
      this.onState = () => {};
      this._timer = null;
      this._stopped = false;
    }
    start() { this._stopped = false; this._connect(); }
    stop() { this._stopped = true; clearTimeout(this._timer); if (this.ws) { try { this.ws.close(); } catch (e) { /* ignore */ } } this.ws = null; this._set(false); }
    _set(c) { if (this.connected !== c) { this.connected = c; this.onState(c); } }
    _connect() {
      if (this._stopped) return;
      let ws;
      try { ws = new WebSocket(this.url); } catch (e) { this._scheduleRetry(); return; }
      this.ws = ws;
      ws.onopen = () => { this.retry = 0; this._set(true); };
      ws.onmessage = (ev) => {
        let msg; try { msg = JSON.parse(ev.data); } catch (e) { return; }
        if (msg.type === 'telemetry') { this.lastFrame = U.now(); this.onTelemetry(msg.data); }
        else if (msg.type === 'event') this.onEvent(msg);
        else if (msg.type === 'hello') this.hello = msg;
      };
      ws.onclose = () => { this._set(false); this.ws = null; this._scheduleRetry(); };
      ws.onerror = () => { try { ws.close(); } catch (e) { /* ignore */ } };
    }
    _scheduleRetry() {
      if (this._stopped) return;
      const delay = Math.min(8000, 500 * Math.pow(2, this.retry++));
      clearTimeout(this._timer);
      this._timer = setTimeout(() => this._connect(), delay);
    }
    send(cmd, args) {
      if (this.ws && this.ws.readyState === 1) { this.ws.send(JSON.stringify({ cmd, args: args || {} })); return true; }
      return false;
    }
    get stale() { return !this.connected || U.now() - this.lastFrame > 2; }
  }
  window.Link = Link;
})();
