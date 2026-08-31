// Validation charts: rolling tip-error timeline (two series) and the
// ensemble-disagreement strip, with a shared crosshair tooltip.
(function (CR) {
  'use strict';

  const INK2 = '#52514e';
  const MUTED = '#898781';
  const GRID = '#e1e0d9';
  const BASELINE = '#c3c2b7';
  const CLASSICAL = '#eb6834';
  const LEARNED = '#2a78d6';
  const WARN_WASH = 'rgba(250,178,25,0.14)';

  const WINDOW_S = 16; // seconds shown
  const PAD = { l: 44, r: 14, t: 10, b: 22 };

  function createCharts(errCanvas, sigmaCanvas, tooltipEl) {
    const samples = []; // {t, errC, errL, sigma, ood}
    const events = [];  // {t, label}
    let hoverX = null;  // css px within err canvas, or null
    let sigmaThresh = 0.5;

    function push(s) {
      samples.push(s);
      const cutoff = s.t - WINDOW_S - 1;
      while (samples.length && samples[0].t < cutoff) samples.shift();
      while (events.length && events[0].t < cutoff) events.shift();
    }

    function addEvent(t, label) { events.push({ t, label }); }
    function reset() { samples.length = 0; events.length = 0; }
    function setSigmaThreshold(v) { sigmaThresh = v; }

    function xScale(W) {
      const tMax = samples.length ? samples[samples.length - 1].t : 0;
      const t0 = Math.max(0, tMax - WINDOW_S);
      return { t0, t1: t0 + WINDOW_S, px: (t) => PAD.l + ((t - t0) / WINDOW_S) * (W - PAD.l - PAD.r) };
    }

    function niceMax(v) {
      const steps = [20, 30, 40, 60, 80, 120, 160, 240, 320];
      for (const s of steps) if (v <= s) return s;
      return Math.ceil(v / 100) * 100;
    }

    function drawLine(ctx, xs, W, H, yPx, key, color) {
      ctx.strokeStyle = color;
      ctx.lineWidth = 2;
      ctx.lineJoin = 'round';
      ctx.beginPath();
      let pen = false;
      for (const s of samples) {
        const x = xs.px(s.t), y = yPx(s[key]);
        if (s[key] == null) { pen = false; continue; }
        if (!pen) { ctx.moveTo(x, y); pen = true; }
        else ctx.lineTo(x, y);
      }
      ctx.stroke();
    }

    function drawErr(ctx, W, H) {
      ctx.clearRect(0, 0, W, H);
      if (!samples.length) {
        ctx.fillStyle = MUTED;
        ctx.font = '12px system-ui, sans-serif';
        ctx.fillText('Waiting for the first target.', PAD.l, H / 2);
        return;
      }
      const xs = xScale(W);
      let maxE = 30;
      for (const s of samples) maxE = Math.max(maxE, s.errC || 0, s.errL || 0);
      const yMax = niceMax(maxE * 1.05);
      const yPx = (v) => PAD.t + (1 - v / yMax) * (H - PAD.t - PAD.b);

      // grid + y labels
      ctx.font = '10.5px ui-monospace, Menlo, Consolas, monospace';
      ctx.textAlign = 'right';
      for (let i = 0; i <= 4; i++) {
        const v = (yMax / 4) * i;
        const y = yPx(v);
        ctx.strokeStyle = i === 0 ? BASELINE : GRID;
        ctx.lineWidth = 1;
        ctx.beginPath(); ctx.moveTo(PAD.l, y); ctx.lineTo(W - PAD.r, y); ctx.stroke();
        ctx.fillStyle = MUTED;
        ctx.fillText(String(Math.round(v)), PAD.l - 6, y + 3.5);
      }
      ctx.textAlign = 'left';
      // x ticks every 4 s
      const tTick = Math.ceil(xs.t0 / 4) * 4;
      ctx.fillStyle = MUTED;
      for (let t = tTick; t <= xs.t1; t += 4) {
        ctx.fillText(t.toFixed(0) + 's', xs.px(t) - 6, H - 6);
      }

      // settle band hairline at 5 mm
      const ySettle = yPx(5);
      ctx.strokeStyle = BASELINE;
      ctx.setLineDash([3, 4]);
      ctx.beginPath(); ctx.moveTo(PAD.l, ySettle); ctx.lineTo(W - PAD.r, ySettle); ctx.stroke();
      ctx.setLineDash([]);

      // condition-change event markers
      ctx.font = '10px ui-monospace, Menlo, Consolas, monospace';
      for (const ev of events) {
        if (ev.t < xs.t0) continue;
        const x = xs.px(ev.t);
        ctx.strokeStyle = GRID;
        ctx.beginPath(); ctx.moveTo(x, PAD.t); ctx.lineTo(x, H - PAD.b); ctx.stroke();
        ctx.fillStyle = MUTED;
        ctx.save();
        ctx.translate(x + 3, PAD.t + 2);
        ctx.rotate(Math.PI / 2);
        ctx.fillText(ev.label, 0, 0);
        ctx.restore();
      }

      drawLine(ctx, xs, W, H, yPx, 'errC', CLASSICAL);
      drawLine(ctx, xs, W, H, yPx, 'errL', LEARNED);

      // direct labels at line ends
      const last = samples[samples.length - 1];
      ctx.font = '600 11px system-ui, sans-serif';
      if (last.errC != null) {
        ctx.fillStyle = CLASSICAL;
        ctx.fillText('classical', Math.min(xs.px(last.t) + 5, W - 60), yPx(last.errC) + 3);
      }
      if (last.errL != null) {
        ctx.fillStyle = LEARNED;
        const yl = yPx(last.errL);
        const yc = last.errC != null ? yPx(last.errC) : -999;
        const gap = yl - yc;
        const y = Math.abs(gap) < 15 ? yc + (gap >= 0 ? 15 : -15) : yl;
        ctx.fillText('learned', Math.min(xs.px(last.t) + 5, W - 60), y + 3);
      }

      // hover crosshair
      if (hoverX != null && hoverX > PAD.l && hoverX < W - PAD.r) {
        ctx.strokeStyle = INK2;
        ctx.lineWidth = 1;
        ctx.setLineDash([2, 3]);
        ctx.beginPath(); ctx.moveTo(hoverX, PAD.t); ctx.lineTo(hoverX, H - PAD.b); ctx.stroke();
        ctx.setLineDash([]);
      }
    }

    function drawSigma(ctx, W, H) {
      ctx.clearRect(0, 0, W, H);
      if (!samples.length) return;
      const xs = xScale(W);
      let maxS = sigmaThresh * 1.6;
      for (const s of samples) maxS = Math.max(maxS, s.sigma || 0);
      const yPx = (v) => 6 + (1 - v / (maxS * 1.05)) * (H - 6 - 16);

      // OOD wash spans
      let spanStart = null;
      for (let i = 0; i < samples.length; i++) {
        const on = !!samples[i].ood;
        if (on && spanStart == null) spanStart = samples[i].t;
        if ((!on || i === samples.length - 1) && spanStart != null) {
          const tEnd = on ? samples[i].t : samples[i - 1].t;
          ctx.fillStyle = WARN_WASH;
          ctx.fillRect(xs.px(spanStart), 6, Math.max(2, xs.px(tEnd) - xs.px(spanStart)), H - 22);
          spanStart = null;
        }
      }

      // baseline + threshold
      ctx.strokeStyle = BASELINE;
      ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(PAD.l, yPx(0)); ctx.lineTo(W - PAD.r, yPx(0)); ctx.stroke();
      ctx.setLineDash([3, 4]);
      ctx.beginPath(); ctx.moveTo(PAD.l, yPx(sigmaThresh)); ctx.lineTo(W - PAD.r, yPx(sigmaThresh)); ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = MUTED;
      ctx.font = '10px ui-monospace, Menlo, Consolas, monospace';
      ctx.fillText('warn', PAD.l - 34, yPx(sigmaThresh) + 3);

      drawLine(ctx, xs, W, H, yPx, 'sigma', LEARNED);

      if (hoverX != null && hoverX > PAD.l && hoverX < W - PAD.r) {
        ctx.strokeStyle = INK2;
        ctx.setLineDash([2, 3]);
        ctx.beginPath(); ctx.moveTo(hoverX, 4); ctx.lineTo(hoverX, H - 14); ctx.stroke();
        ctx.setLineDash([]);
      }
    }

    function draw() {
      for (const [cv, fn] of [[errCanvas, drawErr], [sigmaCanvas, drawSigma]]) {
        const ctx = cv.getContext('2d');
        const dpr = window.devicePixelRatio || 1;
        const W = cv.clientWidth || cv.width;
        // canvases keep their attribute aspect ratio; back the store at dpr
        const bw = Math.round(W * dpr), bh = Math.round((cv.getAttribute('height') / cv.getAttribute('width')) * W * dpr);
        if (cv.width !== bw || cv.height !== bh) { cv.width = bw; cv.height = bh; }
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
        fn(ctx, W, cv.height / dpr);
      }
    }

    // tooltip wiring
    function onMove(ev) {
      const rect = errCanvas.getBoundingClientRect();
      hoverX = ev.clientX - rect.left;
      if (!samples.length) return;
      const W = rect.width;
      const xs = xScale(W);
      const t = xs.t0 + ((hoverX - PAD.l) / (W - PAD.l - PAD.r)) * WINDOW_S;
      let best = null;
      for (const s of samples) if (!best || Math.abs(s.t - t) < Math.abs(best.t - t)) best = s;
      if (best && hoverX > PAD.l) {
        tooltipEl.style.display = 'block';
        tooltipEl.style.left = Math.min(ev.clientX + 14, window.innerWidth - 170) + 'px';
        tooltipEl.style.top = ev.clientY + 14 + 'px';
        tooltipEl.textContent =
          't        ' + best.t.toFixed(1) + ' s\n' +
          'classical ' + (best.errC != null ? best.errC.toFixed(1) + ' mm' : '–') + '\n' +
          'learned   ' + (best.errL != null ? best.errL.toFixed(1) + ' mm' : '–') + '\n' +
          'σ         ' + (best.sigma != null ? best.sigma.toFixed(3) : '–');
      }
    }
    function onLeave() { hoverX = null; tooltipEl.style.display = 'none'; }
    for (const cv of [errCanvas, sigmaCanvas]) {
      cv.addEventListener('mousemove', onMove);
      cv.addEventListener('mouseleave', onLeave);
    }

    return { push, addEvent, reset, draw, setSigmaThreshold };
  }

  CR.chart = { createCharts };
})(typeof globalThis.CR === 'object' ? globalThis.CR : (globalThis.CR = {}));
