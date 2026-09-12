// Small static charts for the acts, drawn on a 944-wide canvas in the style of
// chart.js: hairline grid, mono ticks, direct labels, series colours from the
// page. Each function redraws the whole canvas from the data it is given.
(function (CR) {
  'use strict';
  const MUTED = '#898781', INK2 = '#52514e', GRID = '#e1e0d9', BASE = '#c3c2b7';
  const MONO = '10.5px ui-monospace, Menlo, Consolas, monospace';

  function prep(cv) {
    const ctx = cv.getContext('2d');
    const dpr = window.devicePixelRatio || 1;
    const W = cv.clientWidth || cv.width;
    const H = (cv.getAttribute('height') / cv.getAttribute('width')) * W;
    const bw = Math.round(W * dpr), bh = Math.round(H * dpr);
    if (cv.width !== bw || cv.height !== bh) { cv.width = bw; cv.height = bh; }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, W, H);
    ctx.font = MONO;
    return { ctx, W, H };
  }
  function niceTicks(min, max, n) {
    const span = max - min || 1;
    const raw = span / n;
    const mag = Math.pow(10, Math.floor(Math.log10(raw)));
    const norm = raw / mag;
    const step = (norm < 1.5 ? 1 : norm < 3 ? 2 : norm < 7 ? 5 : 10) * mag;
    const t0 = Math.ceil(min / step) * step;
    const out = [];
    for (let v = t0; v <= max + 1e-9; v += step) out.push(+v.toFixed(10));
    return out;
  }
  function fmt(v) { return Math.abs(v) >= 100 ? v.toFixed(0) : Math.abs(v) >= 10 ? v.toFixed(1) : v.toFixed(2).replace(/\.?0+$/, ''); }

  // Line chart. o = { series:[{name, color, points:[[x,y]], dashed?}], x:{label,min,max}, y:{label,min,max,log?}, marker:{x,label}, hairline:{y,label} }
  function line(cv, o) {
    const { ctx, W, H } = prep(cv);
    ctx.font = '600 11px system-ui, sans-serif';
    const nameW = Math.max(0, ...o.series.map((s) => (s.name ? ctx.measureText(s.name).width : 0)));
    ctx.font = MONO;
    const PAD = { l: 56, r: Math.min(190, Math.max(96, Math.ceil(nameW) + 12)), t: 14, b: 42 };
    const xs = o.x || {}, ys = o.y || {};
    const all = o.series.flatMap((s) => s.points);
    const xmin = xs.min != null ? xs.min : Math.min(...all.map((p) => p[0]));
    const xmax = xs.max != null ? xs.max : Math.max(...all.map((p) => p[0]));
    let ymin = ys.min != null ? ys.min : Math.min(...all.map((p) => p[1]));
    let ymax = ys.max != null ? ys.max : Math.max(...all.map((p) => p[1]));
    if (ys.log) { ymin = Math.max(ymin, ys.floor || 1e-3); ymax = Math.max(ymax, ymin * 1.0001); if (ymax / ymin < 2) { const c = Math.sqrt(ymax * ymin); ymin = c / 1.5; ymax = c * 1.5; } else { ymin /= 1.15; ymax *= 1.15; } }
    else { if (ymin > 0 && ys.min == null) ymin = 0; if (ys.max == null) ymax = ymax + (ymax - ymin) * 0.08; if (ymax === ymin) ymax = ymin + 1; }
    const px = (x) => PAD.l + ((x - xmin) / (xmax - xmin || 1)) * (W - PAD.l - PAD.r);
    const py = (y) => {
      const f = ys.log ? (Math.log10(Math.max(y, ymin)) - Math.log10(ymin)) / (Math.log10(ymax) - Math.log10(ymin)) : (y - ymin) / (ymax - ymin);
      return PAD.t + (1 - f) * (H - PAD.t - PAD.b);
    };
    // grid
    ctx.textAlign = 'right'; ctx.fillStyle = MUTED;
    const yt = ys.log ? (() => {
      const inRange = (v) => v >= ymin * 0.999 && v <= ymax * 1.001;
      for (const m of [[1], [1, 2, 5], [1, 1.5, 2, 3, 5, 7]]) {
        const t = [];
        for (let e = Math.floor(Math.log10(ymin)); e <= Math.ceil(Math.log10(ymax)); e++) for (const k of m) t.push(k * Math.pow(10, e));
        const f = t.filter(inRange);
        if (f.length >= 3) return f;
      }
      return [ymin, Math.sqrt(ymin * ymax), ymax];
    })() : niceTicks(ymin, ymax, 4);
    for (const v of yt) {
      const y = py(v);
      ctx.strokeStyle = v === ymin ? BASE : GRID; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(PAD.l, y); ctx.lineTo(W - PAD.r, y); ctx.stroke();
      ctx.fillText(fmt(v), PAD.l - 6, y + 3.5);
    }
    ctx.textAlign = 'left';
    ctx.textAlign = 'center';
    for (const v of niceTicks(xmin, xmax, 6)) { const x = px(v); if (x < PAD.l - 2 || x > W - PAD.r - 8) continue; ctx.fillText(fmt(v) + (xs.unit || ''), x, H - 26); }
    ctx.textAlign = 'left';
    if (ys.label) { ctx.save(); ctx.translate(12, PAD.t + (H - PAD.t - PAD.b) / 2); ctx.rotate(-Math.PI / 2); ctx.textAlign = 'center'; ctx.fillText(ys.label, 0, 0); ctx.restore(); }
    if (xs.label) { ctx.textAlign = 'center'; ctx.fillText(xs.label, PAD.l + (W - PAD.l - PAD.r) / 2, H - 8); ctx.textAlign = 'left'; }
    if (o.hairline) {
      const y = py(o.hairline.y);
      ctx.strokeStyle = BASE; ctx.setLineDash([3, 4]); ctx.beginPath(); ctx.moveTo(PAD.l, y); ctx.lineTo(W - PAD.r, y); ctx.stroke(); ctx.setLineDash([]);
      if (o.hairline.label) { const tw = ctx.measureText(o.hairline.label).width; ctx.fillStyle = 'rgba(255,255,255,0.85)'; ctx.fillRect(PAD.l + 6, y - 14, tw + 6, 13); ctx.fillStyle = MUTED; ctx.fillText(o.hairline.label, PAD.l + 9, y - 4); }
    }
    if (o.marker && o.marker.x != null) {
      const x = px(o.marker.x);
      ctx.strokeStyle = INK2; ctx.setLineDash([2, 3]); ctx.beginPath(); ctx.moveTo(x, PAD.t); ctx.lineTo(x, H - PAD.b); ctx.stroke(); ctx.setLineDash([]);
      if (o.marker.label) { ctx.fillStyle = INK2; ctx.fillText(o.marker.label, x + 4, PAD.t + 10); }
    }
    if (o.events) {
      ctx.font = '10px ui-monospace, Menlo, Consolas, monospace';
      let k = 0;
      for (const ev of o.events) {
        if (ev.x < xmin || ev.x > xmax) continue;
        const x = px(ev.x);
        ctx.strokeStyle = GRID; ctx.beginPath(); ctx.moveTo(x, PAD.t); ctx.lineTo(x, H - PAD.b); ctx.stroke();
        ctx.fillStyle = MUTED; ctx.textAlign = 'left'; ctx.textBaseline = 'top';
        const tw = ctx.measureText(ev.label).width;
        ctx.fillText(ev.label, Math.min(x + 4, W - PAD.r - tw - 2), PAD.t + 2 + (k++ % 2) * 11);
      }
      ctx.font = MONO;
    }
    const labels = [];
    for (const s of o.series) {
      ctx.strokeStyle = s.color; ctx.lineWidth = s.width || 2; ctx.lineJoin = 'round';
      if (s.dashed) ctx.setLineDash([5, 4]);
      ctx.beginPath();
      let pen = false;
      for (const [x, y] of s.points) { if (y == null || !isFinite(y)) { pen = false; continue; } const X = px(x), Y = py(y); if (!pen) { ctx.moveTo(X, Y); pen = true; } else ctx.lineTo(X, Y); }
      ctx.stroke(); ctx.setLineDash([]);
      const last = s.points.filter((p) => p[1] != null && isFinite(p[1])).pop();
      if (last && s.name) labels.push({ y: py(last[1]), name: s.name, color: s.color });
    }
    // direct labels at the right edge, pushed apart when they would overlap
    labels.sort((a, b) => a.y - b.y);
    for (let i = 1; i < labels.length; i++) if (labels[i].y - labels[i - 1].y < 13) labels[i].y = labels[i - 1].y + 13;
    for (let i = labels.length - 2; i >= 0; i--) if (labels[i + 1].y - labels[i].y < 13) labels[i].y = labels[i + 1].y - 13;
    if (labels.length) { const over = labels[labels.length - 1].y - (H - PAD.b + 2); if (over > 0) for (const l of labels) l.y -= over; const under = PAD.t + 6 - labels[0].y; if (under > 0) for (const l of labels) l.y += under; }
    ctx.font = '600 11px system-ui, sans-serif';
    for (const l of labels) { ctx.fillStyle = l.color; ctx.fillText(l.name, W - PAD.r + 6, l.y + 3); }
    ctx.font = MONO;
  }

  // Grouped bars. o = { groups:[{label, bars:[{label, value, color, light}]}], y:{max,label}, valueLabel:fn }
  function bars(cv, o) {
    const { ctx, W, H } = prep(cv);
    const PAD = { l: 44, r: 14, t: 16, b: 34 };
    const ymax = (o.y && o.y.max) || Math.max(...o.groups.flatMap((g) => g.bars.map((b) => b.value)));
    const py = (v) => PAD.t + (1 - v / ymax) * (H - PAD.t - PAD.b);
    ctx.fillStyle = MUTED; ctx.textAlign = 'right';
    for (const v of niceTicks(0, ymax, 4)) { const y = py(v); ctx.strokeStyle = v === 0 ? BASE : GRID; ctx.beginPath(); ctx.moveTo(PAD.l, y); ctx.lineTo(W - PAD.r, y); ctx.stroke(); ctx.fillText(fmt(v), PAD.l - 6, y + 3.5); }
    ctx.textAlign = 'center';
    const gw = (W - PAD.l - PAD.r) / o.groups.length;
    o.groups.forEach((g, gi) => {
      const n = g.bars.length, bw = Math.min(28, (gw * 0.7) / n);
      const x0 = PAD.l + gi * gw + (gw - bw * n - 4 * (n - 1)) / 2;
      g.bars.forEach((b, bi) => {
        const x = x0 + bi * (bw + 4);
        ctx.fillStyle = b.color; ctx.globalAlpha = b.light ? 0.35 : 1;
        ctx.fillRect(x, py(b.value), bw, py(0) - py(b.value));
        ctx.globalAlpha = 1;
        ctx.fillStyle = INK2; ctx.fillText(o.valueLabel ? o.valueLabel(b.value) : fmt(b.value), x + bw / 2, py(b.value) - 4);
      });
      ctx.fillStyle = MUTED; ctx.fillText(g.label, PAD.l + gi * gw + gw / 2, H - 10);
    });
    ctx.textAlign = 'left';
  }

  // Range bars on a log axis. o = { rows:[{label, min, max, color}], x:{label, floor} }
  function rangeLog(cv, o) {
    const { ctx, W, H } = prep(cv);
    const PAD = { l: 190, r: 90, t: 14, b: 30 };
    const floor = (o.x && o.x.floor) || 1e-4;
    let lo = Infinity, hi = -Infinity;
    for (const r of o.rows) { lo = Math.min(lo, Math.max(floor, r.min)); hi = Math.max(hi, r.max); }
    lo = Math.pow(10, Math.floor(Math.log10(lo))); hi = Math.pow(10, Math.ceil(Math.log10(Math.max(hi, lo * 10))));
    const px = (v) => PAD.l + ((Math.log10(Math.max(floor, v)) - Math.log10(lo)) / (Math.log10(hi) - Math.log10(lo))) * (W - PAD.l - PAD.r);
    ctx.fillStyle = MUTED; ctx.textAlign = 'center';
    for (let e = Math.log10(lo); e <= Math.log10(hi) + 1e-9; e++) { const v = Math.pow(10, e), x = px(v); ctx.strokeStyle = GRID; ctx.beginPath(); ctx.moveTo(x, PAD.t); ctx.lineTo(x, H - PAD.b); ctx.stroke(); ctx.fillText(fmt(v), x, H - 12); }
    if (o.x && o.x.label) ctx.fillText(o.x.label, PAD.l + (W - PAD.l - PAD.r) / 2, H - 1);
    const rh = (H - PAD.t - PAD.b) / o.rows.length;
    o.rows.forEach((r, i) => {
      const y = PAD.t + i * rh + rh / 2;
      ctx.textAlign = 'right'; ctx.fillStyle = INK2; ctx.fillText(r.label, PAD.l - 10, y + 3.5);
      const a = px(r.min), b = Math.max(px(r.max), a + 3);
      ctx.fillStyle = r.color; ctx.fillRect(a, y - 6, b - a, 12);
      ctx.textAlign = 'left'; ctx.fillStyle = MUTED; ctx.fillText(r.text || (fmt(r.min) + ' to ' + fmt(r.max)), b + 6, y + 3.5);
    });
    ctx.textAlign = 'left';
  }

  // Scatter in plane coordinates. o = { points:[{x,y,cls,ring}], classes:{cls:{color,label}}, x:{min,max}, y:{min,max}, outline:[[p,q]] }
  function scatter(cv, o) {
    const { ctx, W, H } = prep(cv);
    const PAD = { l: 44, r: 14, t: 14, b: 26 };
    const xs = o.x, ys = o.y;
    const w = W - PAD.l - PAD.r, h = H - PAD.t - PAD.b;
    const sc = Math.min(w / (xs.max - xs.min), h / (ys.max - ys.min));
    const px = (x) => PAD.l + (w - sc * (xs.max - xs.min)) / 2 + (x - xs.min) * sc;
    const py = (y) => PAD.t + (h - sc * (ys.max - ys.min)) / 2 + (ys.max - y) * sc;
    ctx.strokeStyle = GRID; ctx.strokeRect(px(xs.min), py(ys.max), sc * (xs.max - xs.min), sc * (ys.max - ys.min));
    if (o.outline) { ctx.strokeStyle = BASE; ctx.lineWidth = 1.2; ctx.beginPath(); for (const [p, q] of o.outline) { ctx.moveTo(px(p[0]), py(p[1])); ctx.lineTo(px(q[0]), py(q[1])); } ctx.stroke(); }
    for (const p of o.points) {
      const c = o.classes[p.cls] || { color: MUTED };
      ctx.fillStyle = c.color; ctx.beginPath(); ctx.arc(px(p.x), py(p.y), 3, 0, 2 * Math.PI); ctx.fill();
      if (p.ring) { ctx.strokeStyle = INK2; ctx.lineWidth = 1; ctx.beginPath(); ctx.arc(px(p.x), py(p.y), 5.5, 0, 2 * Math.PI); ctx.stroke(); }
    }
    let lx = PAD.l + 8;
    ctx.font = '600 11px system-ui, sans-serif';
    for (const [k, c] of Object.entries(o.classes)) { ctx.fillStyle = c.color; ctx.fillRect(lx, H - PAD.b + 8, 10, 10); ctx.fillStyle = INK2; ctx.fillText(c.label || k, lx + 14, H - PAD.b + 17); lx += ctx.measureText(c.label || k).width + 34; }
    ctx.font = MONO;
  }

  // Dot strips per group. o = { groups:[{label, values:[], color}], y:{max,label}, hairline:{y,label} }
  function dots(cv, o) {
    const { ctx, W, H } = prep(cv);
    const PAD = { l: 44, r: 14, t: 14, b: 34 };
    const ymax = (o.y && o.y.max) || Math.max(1, ...o.groups.flatMap((g) => g.values));
    const py = (v) => PAD.t + (1 - Math.min(v, ymax) / ymax) * (H - PAD.t - PAD.b);
    ctx.fillStyle = MUTED; ctx.textAlign = 'right';
    for (const v of niceTicks(0, ymax, 4)) { const y = py(v); ctx.strokeStyle = v === 0 ? BASE : GRID; ctx.beginPath(); ctx.moveTo(PAD.l, y); ctx.lineTo(W - PAD.r, y); ctx.stroke(); ctx.fillText(fmt(v), PAD.l - 6, y + 3.5); }
    if (o.hairline) { const y = py(o.hairline.y); ctx.strokeStyle = BASE; ctx.setLineDash([3, 4]); ctx.beginPath(); ctx.moveTo(PAD.l, y); ctx.lineTo(W - PAD.r, y); ctx.stroke(); ctx.setLineDash([]); ctx.textAlign = 'left'; ctx.fillText(o.hairline.label || '', W - PAD.r - 40, y - 4); }
    ctx.textAlign = 'center';
    const gw = (W - PAD.l - PAD.r) / o.groups.length;
    o.groups.forEach((g, gi) => {
      const cx = PAD.l + gi * gw + gw / 2;
      g.values.forEach((v, i) => {
        const jitter = ((i % 7) - 3) * 3;
        ctx.fillStyle = g.color; ctx.globalAlpha = 0.85;
        ctx.beginPath(); ctx.arc(cx + jitter, py(v), 3.5, 0, 2 * Math.PI); ctx.fill();
        if (v > ymax) { ctx.fillText('▲', cx + jitter, PAD.t + 4); }
      });
      ctx.globalAlpha = 1; ctx.fillStyle = MUTED; ctx.fillText(g.label, cx, H - 10);
    });
    ctx.textAlign = 'left';
  }

  CR.charts2 = { line, bars, rangeLog, scatter, dots, prep, niceTicks };
})(typeof globalThis.CR === 'object' ? globalThis.CR : (globalThis.CR = {}));
