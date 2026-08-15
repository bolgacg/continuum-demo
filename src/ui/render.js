// Camera feed rendering: the robot as seen by the fixed pinhole camera, plus
// the vision-HUD overlay (markers, target, uncertainty fan, OOD banner).
(function (CR) {
  'use strict';

  const FEED_BG = '#151614';
  const HUD = '#8a8f88';
  const HUD_DIM = 'rgba(138,143,136,0.28)';
  const TUBE_NEAR = [186, 190, 184]; // rgb of backbone at nearest depth
  const WARNING = '#fab219';

  const R_BASE = 0.048, R_TIP = 0.028; // tube radius, base -> tip

  function drawFeed(ctx, o) {
    // o: { W, H, cam, sim, accent, target, hull, fan, ood, oodGain, label, clickHint, t }
    const { W, H, cam, sim } = o;
    ctx.save();
    ctx.fillStyle = FEED_BG;
    ctx.fillRect(0, 0, W, H);

    // camera reticle: thirds + corner brackets
    ctx.strokeStyle = 'rgba(138,143,136,0.10)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    for (let i = 1; i <= 2; i++) {
      ctx.moveTo((W * i) / 3, 0); ctx.lineTo((W * i) / 3, H);
      ctx.moveTo(0, (H * i) / 3); ctx.lineTo(W, (H * i) / 3);
    }
    ctx.stroke();
    ctx.strokeStyle = HUD_DIM;
    const cb = 14, cm = 8;
    ctx.beginPath();
    for (const [x, y, dx, dy] of [[cm, cm, 1, 1], [W - cm, cm, -1, 1], [cm, H - cm, 1, -1], [W - cm, H - cm, -1, -1]]) {
      ctx.moveTo(x + dx * cb, y); ctx.lineTo(x, y); ctx.lineTo(x, y + dy * cb);
    }
    ctx.stroke();

    // approximate reachable outline (ideal model), dashed
    if (o.hull && o.hull.length > 2) {
      ctx.strokeStyle = 'rgba(138,143,136,0.22)';
      ctx.setLineDash([4, 5]);
      ctx.beginPath();
      ctx.moveTo(o.hull[0][0], o.hull[0][1]);
      for (const p of o.hull) ctx.lineTo(p[0], p[1]);
      ctx.closePath();
      ctx.stroke();
      ctx.setLineDash([]);
    }

    // backbone tube: project samples, draw far-to-near with depth-scaled width
    const pts = sim.backbone(30);
    const proj = pts.map((p) => cam.project(p));
    const segs = [];
    for (let i = 1; i < proj.length; i++) {
      if (!proj[i - 1] || !proj[i]) continue;
      const f = i / (proj.length - 1);
      segs.push({ a: proj[i - 1], b: proj[i], z: (proj[i - 1][2] + proj[i][2]) / 2, f });
    }
    segs.sort((s1, s2) => s2.z - s1.z);
    for (const s of segs) {
      const r = R_BASE + (R_TIP - R_BASE) * s.f;
      const wPx = Math.max(2, (cam.f * 2 * r) / s.z);
      // darken with depth for a cheap shading cue
      const zn = Math.min(1, Math.max(0, (s.z - 1.2) / 2.2));
      const shade = 1 - 0.45 * zn;
      ctx.strokeStyle = `rgb(${TUBE_NEAR.map((c) => Math.round(c * shade)).join(',')})`;
      ctx.lineWidth = wPx;
      ctx.lineCap = 'round';
      ctx.beginPath();
      ctx.moveTo(s.a[0], s.a[1]);
      ctx.lineTo(s.b[0], s.b[1]);
      ctx.stroke();
    }

    // base mount
    const basePx = cam.project([0, 0, 0]);
    if (basePx) {
      ctx.fillStyle = '#3a3d39';
      const bw = (cam.f * 0.16) / basePx[2];
      ctx.fillRect(basePx[0] - bw / 2, basePx[1] - bw / 2, bw, bw * 1.2);
    }

    // markers: rings on the shaft, filled dot at the tip
    const markers = sim.markers3().map((p) => cam.project(p));
    ctx.lineWidth = 2;
    for (let i = 0; i < markers.length; i++) {
      const m = markers[i];
      if (!m) continue;
      if (i < 3) {
        ctx.strokeStyle = o.accent;
        ctx.beginPath();
        ctx.arc(m[0], m[1], 4.5, 0, 2 * Math.PI);
        ctx.stroke();
      } else {
        ctx.fillStyle = o.accent;
        ctx.beginPath();
        ctx.arc(m[0], m[1], 4, 0, 2 * Math.PI);
        ctx.fill();
        ctx.strokeStyle = o.accent;
        ctx.beginPath();
        ctx.arc(m[0], m[1], 8, 0, 2 * Math.PI);
        ctx.stroke();
      }
    }

    // uncertainty fan (learned feed): one thin arrow per ensemble member,
    // then the mean, from the tip
    const tip = markers[3];
    if (o.fan && tip) {
      ctx.lineCap = 'round';
      for (const a of o.fan.members) {
        ctx.strokeStyle = 'rgba(57,135,229,0.45)';
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.moveTo(tip[0], tip[1]);
        ctx.lineTo(tip[0] + a[0], tip[1] + a[1]);
        ctx.stroke();
      }
      const m = o.fan.mean;
      ctx.strokeStyle = '#3987e5';
      ctx.lineWidth = 2.5;
      ctx.beginPath();
      ctx.moveTo(tip[0], tip[1]);
      ctx.lineTo(tip[0] + m[0], tip[1] + m[1]);
      ctx.stroke();
    }

    // target crosshair
    if (o.target) {
      const [u, v] = o.target;
      ctx.strokeStyle = '#e8eae6';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.arc(u, v, 7, 0, 2 * Math.PI);
      ctx.moveTo(u - 12, v); ctx.lineTo(u - 4, v);
      ctx.moveTo(u + 4, v); ctx.lineTo(u + 12, v);
      ctx.moveTo(u, v - 12); ctx.lineTo(u, v - 4);
      ctx.moveTo(u, v + 4); ctx.lineTo(u, v + 12);
      ctx.stroke();
    }

    // HUD text
    ctx.font = '10px ui-monospace, Menlo, Consolas, monospace';
    ctx.fillStyle = HUD;
    ctx.fillText(o.label, 14, H - 14);
    const tstr = 't=' + o.t.toFixed(1).padStart(6) + 's';
    ctx.fillText(tstr, W - 14 - ctx.measureText(tstr).width, H - 14);

    // OOD banner
    if (o.ood) {
      ctx.fillStyle = 'rgba(21,22,20,0.85)';
      ctx.fillRect(0, 0, W, 30);
      ctx.fillStyle = WARNING;
      ctx.font = '600 11px ui-monospace, Menlo, Consolas, monospace';
      const msg = '⚠ OUTSIDE TRAINING ENVELOPE · GAIN ×' + o.oodGain.toFixed(2);
      ctx.fillText(msg, (W - ctx.measureText(msg).width) / 2, 19);
    }

    // idle hint
    if (o.clickHint) {
      ctx.fillStyle = 'rgba(232,234,230,0.75)';
      ctx.font = '12px system-ui, sans-serif';
      const msg = 'click to set a target';
      const pulse = 0.55 + 0.45 * Math.sin(o.t * 2.2);
      ctx.globalAlpha = 0.35 + 0.4 * pulse;
      ctx.fillText(msg, (W - ctx.measureText(msg).width) / 2, H / 2 - 40);
      ctx.globalAlpha = 1;
    }

    ctx.restore();
  }

  CR.render = { drawFeed, FEED_BG };
})(typeof globalThis.CR === 'object' ? globalThis.CR : (globalThis.CR = {}));
