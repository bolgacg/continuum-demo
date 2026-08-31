// Camera feed rendering: the robot as seen by the fixed pinhole camera, plus
// the vision-HUD overlay (markers, target, reference, uncertainty fan, OOD
// banner) and, in version 2, a plan-view inset that is not visible to either
// controller. It exists so a reader can see the fixed geometry that the
// perspective camera foreshortens.
(function (CR) {
  'use strict';
  const { pcc } = CR;

  const FEED_BG = '#151614';
  const HUD = '#8a8f88';
  const HUD_DIM = 'rgba(138,143,136,0.28)';
  const TUBE_NEAR = [186, 190, 184]; // rgb of backbone at nearest depth
  const WARNING = '#fab219';

  // The model has no cross-section at all; the drawn tube has one constant
  // radius so nothing on screen suggests a geometry that is not modelled.
  const R_TUBE = 0.038;

  function drawPolygon(ctx, poly, stroke, dash, width) {
    if (!poly || poly.length < 3) return;
    ctx.strokeStyle = stroke;
    ctx.lineWidth = width || 1;
    ctx.setLineDash(dash || []);
    ctx.beginPath();
    ctx.moveTo(poly[0][0], poly[0][1]);
    for (const p of poly) ctx.lineTo(p[0], p[1]);
    ctx.closePath();
    ctx.stroke();
    ctx.setLineDash([]);
  }

  function drawFeed(ctx, o) {
    // o: { W, H, cam, sim, accent, target, sRef, tracking, plan, outline, hull,
    //      fan, ood, oodGain, label, clickHint, t, planView }
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

    // reachable outline of the ideal model (solid, faint) and the ensemble's
    // training-target envelope (dashed). Two different things, drawn differently.
    drawPolygon(ctx, o.outline, 'rgba(138,143,136,0.30)', [], 1.2);
    drawPolygon(ctx, o.hull, 'rgba(57,135,229,0.30)', [4, 5], 1);

    // backbone tube: project samples, draw far-to-near with depth-scaled width
    const pts = sim.backbone(30);
    const proj = pts.map((p) => cam.project(p));
    const segs = [];
    for (let i = 1; i < proj.length; i++) {
      if (!proj[i - 1] || !proj[i]) continue;
      segs.push({ a: proj[i - 1], b: proj[i], z: (proj[i - 1][2] + proj[i][2]) / 2 });
    }
    segs.sort((s1, s2) => s2.z - s1.z);
    for (const s of segs) {
      const wPx = Math.max(2, (cam.f * 2 * R_TUBE) / s.z);
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

    // moving reference while a plan is being tracked: small diamond
    if (o.tracking && o.sRef) {
      const [u, v] = o.sRef;
      ctx.strokeStyle = o.accent;
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(u, v - 6); ctx.lineTo(u + 6, v); ctx.lineTo(u, v + 6); ctx.lineTo(u - 6, v);
      ctx.closePath();
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

    if (o.planView) drawPlanView(ctx, o);

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

  // Plan view: orthographic, looking down the world y axis (gravity). The
  // backbone runs along +z, drawn to the left to match the camera image,
  // where image-right is roughly -z; +x (toward the camera) is drawn down, so
  // the camera sits below the inset. Shows: backbone and markers at true
  // scale, the viewing ray of the clicked target (a pixel is a ray, not a
  // point), and the tip of the planned goal configuration.
  const PV = { w: 150, h: 104, pad: 6, zMin: -1.15, zMax: 2.05, xMin: -1.5, xMax: 1.5 };
  function drawPlanView(ctx, o) {
    const { W, H, cam, sim } = o;
    const x0 = W - 12 - PV.w, y0 = H - 26 - PV.h;
    const sz = (PV.w - 2 * PV.pad) / (PV.zMax - PV.zMin);
    const sx = (PV.h - 2 * PV.pad) / (PV.xMax - PV.xMin);
    const s = Math.min(sz, sx);
    const cx = x0 + PV.w / 2, cy = y0 + PV.h / 2;
    const zc = (PV.zMax + PV.zMin) / 2, xc = (PV.xMax + PV.xMin) / 2;
    const map = (p) => [cx - (p[2] - zc) * s, cy + (p[0] - xc) * s];

    ctx.save();
    ctx.fillStyle = 'rgba(21,22,20,0.88)';
    ctx.fillRect(x0, y0, PV.w, PV.h);
    ctx.strokeStyle = HUD_DIM;
    ctx.lineWidth = 1;
    ctx.strokeRect(x0 + 0.5, y0 + 0.5, PV.w - 1, PV.h - 1);
    ctx.beginPath();
    ctx.rect(x0 + 1, y0 + 1, PV.w - 2, PV.h - 2);
    ctx.clip();

    // reach circle of the fully straight robot, as a scale cue: radius L1+L2
    const L = pcc.SEG_LEN[0] + pcc.SEG_LEN[1];
    const b = map([0, 0, 0]);
    ctx.strokeStyle = 'rgba(138,143,136,0.14)';
    ctx.setLineDash([2, 4]);
    ctx.beginPath();
    ctx.arc(b[0], b[1], L * s, 0, 2 * Math.PI);
    ctx.stroke();
    ctx.setLineDash([]);

    // viewing ray of the target, projected onto the plan
    if (o.target) {
      const d = cam.rayDir(o.target[0], o.target[1]);
      const a = map(cam.pos);
      const far = map([cam.pos[0] + d[0] * 8, 0, cam.pos[2] + d[2] * 8]);
      ctx.strokeStyle = 'rgba(232,234,230,0.35)';
      ctx.setLineDash([3, 3]);
      ctx.beginPath();
      ctx.moveTo(a[0], a[1]);
      ctx.lineTo(far[0], far[1]);
      ctx.stroke();
      ctx.setLineDash([]);
    }

    // planned goal tip
    if (o.plan && o.plan.qGoal) {
      const g = map(pcc.tip3(o.plan.qGoal));
      ctx.strokeStyle = 'rgba(232,234,230,0.6)';
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.arc(g[0], g[1], 4, 0, 2 * Math.PI);
      ctx.stroke();
    }

    // base
    ctx.fillStyle = '#3a3d39';
    ctx.fillRect(b[0] - 3, b[1] - 4, 6, 8);

    // backbone at true scale, constant width
    const pts = sim.backbone(30).map(map);
    ctx.strokeStyle = 'rgb(170,174,168)';
    ctx.lineWidth = Math.max(2, 2 * R_TUBE * s);
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';
    ctx.beginPath();
    ctx.moveTo(pts[0][0], pts[0][1]);
    for (const p of pts) ctx.lineTo(p[0], p[1]);
    ctx.stroke();

    // markers
    const ms = sim.markers3().map(map);
    ctx.lineWidth = 1.5;
    for (let i = 0; i < ms.length; i++) {
      ctx.strokeStyle = o.accent;
      ctx.fillStyle = o.accent;
      ctx.beginPath();
      ctx.arc(ms[i][0], ms[i][1], i === 3 ? 3 : 2.5, 0, 2 * Math.PI);
      if (i === 3) ctx.fill(); else ctx.stroke();
    }

    // camera icon at the bottom edge, at its true z
    const c = map([PV.xMax, 0, cam.pos[2]]);
    ctx.fillStyle = HUD;
    ctx.beginPath();
    ctx.moveTo(c[0] - 5, c[1] - 1); ctx.lineTo(c[0] + 5, c[1] - 1); ctx.lineTo(c[0], c[1] - 7);
    ctx.closePath();
    ctx.fill();

    ctx.font = '9px ui-monospace, Menlo, Consolas, monospace';
    ctx.fillStyle = HUD;
    ctx.fillText('PLAN VIEW · top-down', x0 + 6, y0 + 11);
    ctx.fillText('not seen by controllers', x0 + 6, y0 + 21);
    ctx.restore();
  }

  CR.render = { drawFeed, FEED_BG };
})(typeof globalThis.CR === 'object' ? globalThis.CR : (globalThis.CR = {}));
