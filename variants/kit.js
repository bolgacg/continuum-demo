// ---- src/ui/scene.js ----
// 3D scene rendering onto a 2D canvas through any camera: both robots, the
// reachable envelope, its cross-section at the target plane, the target, the
// target plane, the sensors, the tracking reference and the ensemble fan.
// Painter's algorithm: every primitive carries its camera depth and the list
// is drawn far to near. The same function draws the inspector (orbit camera)
// and the side sensor feed; the style flag only changes the overlay.
(function (CR) {
  'use strict';
  const { pcc, v3, camera } = CR;

  const FEED_BG = '#151614';
  const HUD = '#8a8f88';
  const HUD_DIM = 'rgba(138,143,136,0.28)';
  const WARNING = '#fab219';
  const R_TUBE = 0.038;   // constant drawn radius; the model has no cross-section
  const PLANE = { x: [-1.7, 1.7], y: [-1.7, 1.7] }; // height plane z = h

  function hexToRgb(hex) {
    return [parseInt(hex.slice(1, 3), 16), parseInt(hex.slice(3, 5), 16), parseInt(hex.slice(5, 7), 16)];
  }

  // Envelope quads split into a far half and a near half around the centre
  // depth, so the robot sits inside the translucent dome correctly enough.
  function domeQuads(cam, mesh, step) {
    const far = [], near = [];
    const proj = mesh.map((row) => row.map((p) => cam.project(p)));
    const rows = mesh.length, cols = mesh[0].length;
    const centre = mesh[Math.floor(rows / 2)].reduce((a, p) => v3.add(a, v3.scale(p, 1 / cols)), [0, 0, 0]);
    const zc = cam.depth(centre);
    for (let i = 0; i < rows - 1; i += step) {
      const i2 = Math.min(rows - 1, i + step);
      for (let j = 0; j < cols; j += step) {
        const j2 = (j + step) % cols;
        const q = [proj[i][j], proj[i][j2], proj[i2][j2], proj[i2][j]];
        if (q.some((p) => !p)) continue;
        const z = (q[0][2] + q[1][2] + q[2][2] + q[3][2]) / 4;
        (z > zc ? far : near).push({ z, q });
      }
    }
    far.sort((a, b) => b.z - a.z);
    near.sort((a, b) => b.z - a.z);
    return { far, near };
  }
  // One union fill and one wireframe stroke per half: two draw calls instead
  // of one per quad, which is what keeps the page at frame rate.
  function fillQuads(ctx, quads, fill, stroke) {
    ctx.beginPath();
    for (const { q } of quads) {
      ctx.moveTo(q[0][0], q[0][1]);
      for (let k = 1; k < 4; k++) ctx.lineTo(q[k][0], q[k][1]);
      ctx.closePath();
    }
    ctx.fillStyle = fill;
    ctx.fill('nonzero');
    ctx.strokeStyle = stroke;
    ctx.lineWidth = 0.6;
    ctx.stroke();
  }

  // Static layers (cages) are expensive at full quality, so: while the camera
  // is moving (orbit drag, demo spin) each frame gets cheap coarse layers,
  // drawn fresh and not cached; the first frame after the camera rests renders
  // the fine version once and caches it.
  const layerCache = new Map();
  const lastSig = new Map();
  function staticLayers(key, cam, W, H, volume, trainVolume) {
    const sig = cam.pos.map((v) => v.toFixed(5)).join(',') + '|' + cam.up.map((v) => v.toFixed(5)).join(',') + '|' + W + 'x' + H + '|' + !!(volume && volume.show) + '|' + !!(trainVolume && trainVolume.mesh);
    let entry = layerCache.get(key);
    if (entry && entry.sig === sig) return entry;
    const moving = lastSig.get(key) !== sig;
    lastSig.set(key, sig);
    const dpr = (typeof window !== 'undefined' && window.devicePixelRatio) || 1;
    const make = () => {
      const c = document.createElement('canvas');
      c.width = Math.round(W * dpr); c.height = Math.round(H * dpr);
      const g = c.getContext('2d');
      g.setTransform(dpr, 0, 0, dpr, 0, 0);
      return { c, g };
    };
    const far = make(), near = make();
    if (trainVolume && trainVolume.mesh) {
      const tq = domeQuads(cam, trainVolume.mesh, moving ? 6 : 4);
      fillQuads(far.g, tq.far, 'rgba(0,0,0,0)', 'rgba(57,135,229,0.16)');
      fillQuads(near.g, tq.near, 'rgba(0,0,0,0)', 'rgba(57,135,229,0.11)');
    }
    if (volume && volume.show && volume.mesh) {
      const coarse = domeQuads(cam, volume.mesh, 3);
      if (moving) {
        // coarse silhouette + wireframe only
        fillQuads(far.g, coarse.far, 'rgba(138,143,136,0.055)', 'rgba(138,143,136,0.10)');
        fillQuads(near.g, coarse.near, 'rgba(138,143,136,0.055)', 'rgba(138,143,136,0.07)');
      } else {
        const fine = domeQuads(cam, volume.mesh, 1);
        fillQuads(far.g, fine.far, 'rgba(138,143,136,0.055)', 'rgba(0,0,0,0)');
        fillQuads(far.g, coarse.far, 'rgba(0,0,0,0)', 'rgba(138,143,136,0.10)');
        fillQuads(near.g, fine.near, 'rgba(138,143,136,0.055)', 'rgba(0,0,0,0)');
        fillQuads(near.g, coarse.near, 'rgba(0,0,0,0)', 'rgba(138,143,136,0.07)');
      }
    }
    entry = { sig, far: far.c, near: near.c };
    if (!moving) layerCache.set(key, entry); // cache only the fine, at-rest render
    return entry;
  }

  function drawPlane(ctx, cam, h, o) {
    const c = [[PLANE.x[0], PLANE.y[0], h], [PLANE.x[0], PLANE.y[1], h], [PLANE.x[1], PLANE.y[1], h], [PLANE.x[1], PLANE.y[0], h]];
    const p = c.map((w) => cam.project(w));
    if (p.some((v) => !v)) return;
    ctx.fillStyle = o.active ? 'rgba(232,234,230,0.10)' : 'rgba(232,234,230,0.06)';
    ctx.strokeStyle = o.active ? 'rgba(232,234,230,0.7)' : 'rgba(232,234,230,0.38)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(p[0][0], p[0][1]);
    for (let k = 1; k < 4; k++) ctx.lineTo(p[k][0], p[k][1]);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
    // grab handle at the near-right corner (toward the side camera, image-right)
    const hpx = cam.project([PLANE.x[1] - 0.2, PLANE.y[1] - 0.15, h]);
    if (hpx) {
      ctx.fillStyle = o.active ? '#e8eae6' : 'rgba(232,234,230,0.7)';
      ctx.fillRect(hpx[0] - 5, hpx[1] - 5, 10, 10);
      ctx.font = '10px ui-monospace, Menlo, Consolas, monospace';
      ctx.fillStyle = HUD;
      const txt = 'target plane  z = ' + (h * 100).toFixed(0) + ' mm';
      const tw = ctx.measureText(txt).width;
      const right = Math.min(hpx[0] - 12, cam.w - 26); // clear of the slider overlay
      ctx.fillText(txt, Math.max(14, right - tw), hpx[1] + 4);
    }
  }

  // Naked view: the three tendons of each segment as the offset curves they
  // are (radius tendonRadius in the segment's local frame at 0, 120, 240
  // degrees), coloured and widened by how hard the simulator says each one is
  // currently pulled (transmitted displacement, so backlash and drift show),
  // plus spacer discs at the base, midpoint and end of each segment.
  const TENDON_N = 14;
  const TENDON_DRAW_SCALE = 2.5; // drawn radius exaggeration, stated in the HUD
  function nakedPrims(ctx, cam, r, rgb, prims) {
    const { truth } = CR;
    const q = r.sim.qEff();
    const rt = truth.PARAMS.tendonRadius * TENDON_DRAW_SCALE;
    for (let i = 0; i < pcc.NSEG; i++) {
      const L = pcc.SEG_LEN[i];
      const pullMax = truth.PARAMS.tendonRadius * L * pcc.KMAX[i];
      for (let j = 0; j < 3; j++) {
        const beta = truth.BETA[j];
        const off = [rt * Math.cos(beta), rt * Math.sin(beta), 0];
        const st = r.sim.tendonState(3 * i + j);
        const pull = Math.max(0, Math.min(1, -st.transmitted / pullMax)); // 1 = fully shortened
        const pts = [];
        let zSum = 0;
        for (let k = 0; k <= TENDON_N; k++) {
          const pose = pcc.poseAt(q, i, (k / TENDON_N) * L);
          const p = cam.project(v3.add(pose.p, CR.m3.mulVec(pose.R, off)));
          if (!p) { pts.length = 0; break; }
          pts.push(p); zSum += p[2];
        }
        if (!pts.length) continue;
        // one polyline per tendon (one stroke), depth = its mean depth
        prims.push({ z: zSum / pts.length, draw() {
          // white = slack, host colour = fully shortened; constant width
          const c = rgb.map((v) => Math.round(236 + (v - 236) * pull));
          ctx.strokeStyle = `rgba(${c[0]},${c[1]},${c[2]},0.9)`;
          ctx.lineWidth = 2.2;
          ctx.lineCap = 'round';
          ctx.lineJoin = 'round';
          ctx.beginPath();
          ctx.moveTo(pts[0][0], pts[0][1]);
          for (let k = 1; k < pts.length; k++) ctx.lineTo(pts[k][0], pts[k][1]);
          ctx.stroke();
        } });
      }
      // spacer discs at base (segment 1 only), midpoint and end
      const stations = i === 0 ? [0, 0.5, 1] : [0.5, 1];
      for (const f of stations) {
        const pose = pcc.poseAt(q, i, f * L);
        const ring = [];
        for (let k = 0; k <= 20; k++) {
          const a = (k / 20) * 2 * Math.PI;
          ring.push(cam.project(v3.add(pose.p, CR.m3.mulVec(pose.R, [rt * Math.cos(a), rt * Math.sin(a), 0]))));
        }
        if (ring.some((p) => !p)) continue;
        const z = cam.depth(pose.p);
        prims.push({ z: z - 1e-4, draw() {
          ctx.strokeStyle = 'rgba(200,203,198,0.32)';
          ctx.lineWidth = 1;
          ctx.beginPath();
          ctx.moveTo(ring[0][0], ring[0][1]);
          for (let k = 1; k < ring.length; k++) ctx.lineTo(ring[k][0], ring[k][1]);
          ctx.stroke();
        } });
      }
    }
  }

  function drawSensor(ctx, cam, sensor, label) {
    // small frustum: apex at the sensor, four corner rays 0.22 long
    const apex = sensor.pos;
    const corners = [[0, 0], [sensor.w, 0], [sensor.w, sensor.h], [0, sensor.h]].map(([u, v]) =>
      v3.add(apex, v3.scale(sensor.rayDir(u, v), 0.26)));
    const a = cam.project(apex);
    const cs = corners.map((p) => cam.project(p));
    if (!a || cs.some((p) => !p)) return;
    ctx.strokeStyle = 'rgba(232,234,230,0.45)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    for (let k = 0; k < 4; k++) {
      ctx.moveTo(a[0], a[1]); ctx.lineTo(cs[k][0], cs[k][1]);
      ctx.moveTo(cs[k][0], cs[k][1]); ctx.lineTo(cs[(k + 1) % 4][0], cs[(k + 1) % 4][1]);
    }
    ctx.stroke();
    ctx.font = '9px ui-monospace, Menlo, Consolas, monospace';
    ctx.fillStyle = HUD;
    ctx.fillText(label, a[0] + 6, a[1] - 6);
  }

  function draw(ctx, o) {
    // o: { W, H, cam, robots:[{sim, accent, fan, sRef, tracking}], target, plane:{y, show, active},
    //      volume:{mesh, show}, sensors:[{cam,label}], style, label, t, ood, oodGain, clickHint, hint }
    const { W, H, cam } = o;
    ctx.save();
    ctx.fillStyle = FEED_BG;
    ctx.fillRect(0, 0, W, H);

    // corner brackets (camera-feed look, both views); no reticle grid: the only
    // grids on screen are the two cages, movement limit and training limit
    ctx.strokeStyle = HUD_DIM;
    const cb = 14, cm = 8;
    ctx.beginPath();
    for (const [x, y, dx, dy] of [[cm, cm, 1, 1], [W - cm, cm, -1, 1], [cm, H - cm, 1, -1], [W - cm, H - cm, -1, -1]]) {
      ctx.moveTo(x + dx * cb, y); ctx.lineTo(x, y); ctx.lineTo(x, y + dy * cb);
    }
    ctx.stroke();

    // static layers (floor grid + envelope halves) are rasterized once per
    // camera pose into offscreen canvases and blitted; re-rendered only when
    // the camera moves (orbit drag) or the view is first drawn
    const layers = staticLayers(o.layerKey || 'default', cam, W, H, o.volume, o.trainVolume);
    ctx.drawImage(layers.far, 0, 0, W, H);

    if (o.plane && o.plane.show) drawPlane(ctx, cam, o.plane.y, o.plane);

    // reachable cross-section at the target plane's height: where a click lands
    // inside reach. Drawn in both views (edge-on in the side sensor view).
    if (o.section && o.section.length) {
      ctx.beginPath();
      for (const [p, q] of o.section) {
        const a = cam.project(p), b = cam.project(q);
        if (!a || !b) continue;
        ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]);
      }
      ctx.strokeStyle = 'rgba(232,234,230,0.5)';
      ctx.lineWidth = 1.2;
      ctx.stroke();
    }

    const tpx = o.target ? cam.project(o.target) : null;

    // depth-sorted primitives: tube segments of both robots, or in the naked
    // view the thin backbone, the three tendons per segment and spacer discs
    const prims = [];
    for (const r of o.robots) {
      const rgb = hexToRgb(r.accent);
      const pts = r.sim.backbone(30);
      const proj = pts.map((p) => cam.project(p));
      for (let i = 1; i < proj.length; i++) {
        if (!proj[i - 1] || !proj[i]) continue;
        const z = (proj[i - 1][2] + proj[i][2]) / 2;
        const a = proj[i - 1], b = proj[i];
        prims.push({ z, draw() {
          const zn = Math.min(1, Math.max(0, (z - 1.6) / 2.6));
          const shade = 1 - 0.4 * zn;
          const col = `rgba(${Math.round((rgb[0] * 0.55 + 120 * 0.45) * shade)},${Math.round((rgb[1] * 0.55 + 122 * 0.45) * shade)},${Math.round((rgb[2] * 0.55 + 118 * 0.45) * shade)},0.85)`;
          ctx.strokeStyle = col;
          ctx.lineWidth = o.naked ? 2 : Math.max(2, (cam.f * 2 * R_TUBE) / z);
          ctx.lineCap = 'round';
          ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke();
        } });
      }
      if (o.naked) nakedPrims(ctx, cam, r, rgb, prims);
    }
    prims.sort((p, q) => q.z - p.z);
    for (const p of prims) p.draw();

    // base mount (shared)
    const basePx = cam.project([0, 0, 0]);
    if (basePx) {
      ctx.fillStyle = '#3a3d39';
      const bw = (cam.f * 0.16) / basePx[2];
      ctx.fillRect(basePx[0] - bw / 2, basePx[1] - bw / 2, bw, bw * 1.2);
    }

    // markers, reference diamonds, fans. Two segments, one midpoint each:
    // segment ends (end of segment 1, tip) are filled discs in a lighter tint
    // of the accent; midpoints are hollow rings in the accent.
    for (const r of o.robots) {
      const markers = r.sim.markers3().map((p) => cam.project(p));
      const rgb = hexToRgb(r.accent);
      const light = `rgb(${rgb.map((c) => Math.round(c + (255 - c) * 0.45)).join(',')})`;
      for (let i = 0; i < markers.length; i++) {
        const m = markers[i];
        if (!m) continue;
        const isEnd = i === 1 || i === 3;
        ctx.beginPath();
        ctx.arc(m[0], m[1], isEnd ? 4.5 : 3.8, 0, 2 * Math.PI);
        if (isEnd) {
          ctx.fillStyle = light;
          ctx.fill();
          ctx.lineWidth = 1.2;
          ctx.strokeStyle = r.accent;
          ctx.stroke();
        } else {
          ctx.lineWidth = 1.6;
          ctx.strokeStyle = r.accent;
          ctx.stroke();
        }
        if (i === 3) {
          ctx.lineWidth = 2;
          ctx.strokeStyle = r.accent;
          ctx.beginPath(); ctx.arc(m[0], m[1], 8.5, 0, 2 * Math.PI); ctx.stroke();
        }
      }
      const tip = markers[3];
      if (r.fan && tip) {
        ctx.lineCap = 'round';
        for (const a of r.fan.members) {
          const p = cam.project(a);
          if (!p) continue;
          ctx.strokeStyle = 'rgba(57,135,229,0.45)';
          ctx.lineWidth = 1.5;
          ctx.beginPath(); ctx.moveTo(tip[0], tip[1]); ctx.lineTo(p[0], p[1]); ctx.stroke();
        }
        const m = cam.project(r.fan.mean);
        if (m) {
          ctx.strokeStyle = '#3987e5';
          ctx.lineWidth = 2.5;
          ctx.beginPath(); ctx.moveTo(tip[0], tip[1]); ctx.lineTo(m[0], m[1]); ctx.stroke();
        }
      }
      if (r.tracking && r.sRef) {
        const s = cam.project(r.sRef);
        if (s) {
          ctx.strokeStyle = r.accent;
          ctx.lineWidth = 1.5;
          ctx.beginPath();
          ctx.moveTo(s[0], s[1] - 6); ctx.lineTo(s[0] + 6, s[1]); ctx.lineTo(s[0], s[1] + 6); ctx.lineTo(s[0] - 6, s[1]);
          ctx.closePath(); ctx.stroke();
        }
      }
    }

    ctx.drawImage(layers.near, 0, 0, W, H);

    // target crosshair on top of everything
    if (tpx) {
      const [u, v] = tpx;
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

    if (o.sensors) for (const s of o.sensors) drawSensor(ctx, cam, s.cam, s.label);

    // HUD
    ctx.font = '10px ui-monospace, Menlo, Consolas, monospace';
    ctx.fillStyle = HUD;
    ctx.fillText(o.label, 14, H - 14);
    const tstr = 't=' + o.t.toFixed(1).padStart(6) + 's';
    ctx.fillText(tstr, W - 14 - ctx.measureText(tstr).width, H - 14);
    let hintY = 22;
    if (o.naked) {
      ctx.fillStyle = 'rgba(232,234,230,0.6)';
      ctx.font = '10px ui-monospace, Menlo, Consolas, monospace';
      ctx.fillText('tendons: white = slack · robot colour = fully shortened', 14, hintY);
      hintY += 14;
    }
    if (o.hint) {
      ctx.fillStyle = 'rgba(232,234,230,0.6)';
      ctx.font = '10.5px ui-monospace, Menlo, Consolas, monospace';
      ctx.fillText(o.hint, 14, hintY);
    }

    if (o.ood) {
      ctx.fillStyle = 'rgba(21,22,20,0.85)';
      ctx.fillRect(0, 0, W, 30);
      ctx.fillStyle = WARNING;
      ctx.font = '600 11px ui-monospace, Menlo, Consolas, monospace';
      const msg = '⚠ LEARNED: OUTSIDE TRAINING ENVELOPE · GAIN ×' + o.oodGain.toFixed(2);
      ctx.fillText(msg, (W - ctx.measureText(msg).width) / 2, 19);
    }

    if (o.clickHint) {
      ctx.fillStyle = 'rgba(232,234,230,0.75)';
      ctx.font = '12px system-ui, sans-serif';
      const msg = o.clickHint;
      const pulse = 0.55 + 0.45 * Math.sin(o.t * 2.2);
      ctx.globalAlpha = 0.35 + 0.4 * pulse;
      ctx.fillText(msg, (W - ctx.measureText(msg).width) / 2, H - 60);
      ctx.globalAlpha = 1;
    }
    ctx.restore();
  }

  // Screen row of the height plane at the axis (for drag hit tests and the
  // 1:1 vertical slider), and the plane height that puts it under a given
  // pixel row: intersect the pixel's ray with the vertical plane x = CENTER.x.
  function planeScreenY(cam, h) {
    const p = cam.project([camera.CENTER[0], camera.CENTER[1], h]);
    return p ? p[1] : null;
  }
  function planeYFromPixel(cam, u, v) {
    const d = cam.rayDir(u, v);
    if (Math.abs(d[0]) < 1e-6) return null;
    const t = (camera.CENTER[0] - cam.pos[0]) / d[0];
    return cam.pos[2] + t * d[2];
  }

  CR.scene = { draw, planeScreenY, planeYFromPixel, invalidateLayers: () => { layerCache.clear(); lastSig.clear(); }, R_TUBE, TENDON_DRAW_SCALE };
})(typeof globalThis.CR === 'object' ? globalThis.CR : (globalThis.CR = {}));

// ---- src/ui/chart.js ----
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

// ---- src/variants/protocol.js ----
// The page's trial protocol as a reusable module, so that numbers computed in
// the browser are the same numbers train/eval.js writes into eval.json.
//
// Protocol (train/eval.js): 6 s window at 60 Hz; settle = tip error under 5 mm
// (0.05 length units) held for 0.8 s, reported as the band-entry time; steady
// state = mean error over the final second; a 2 s settle of the plant before
// the trial; seeds 31000 + 7 n; interior targets from configurations at up to
// 85 percent of the curvature limits, edge targets at 95 to 100 percent.
// Display convention: 1 length unit = 100 mm.
(function (CR) {
  'use strict';
  const { pcc, truth, ibvs, learned, planner, camera, v3, makeRng } = CR;

  const CONST = {
    W: 460, H: 345, DT: 1 / 60, TRIAL_S: 6, SETTLE_U: 0.05, SETTLE_HOLD: 0.8, MM: 100,
    TRIALS: 40, SEED_BASE: 31000, SEED_STEP: 7, PRE_STEPS: 120,
  };
  const CONDITIONS = [
    { name: 'nominal', payload: 0, drift: false },
    { name: 'payload', payload: 1, drift: false },
    { name: 'drift', payload: 0, drift: true },
    { name: 'payload + drift', payload: 1, drift: true },
  ];

  function randomQ(rng, lo, hi) {
    const q = [];
    for (let i = 0; i < 2; i++) {
      const a = rng() * 2 * Math.PI;
      const k = (lo + (hi - lo) * Math.sqrt(rng())) * pcc.KMAX[i];
      q.push(k * Math.cos(a), k * Math.sin(a));
    }
    return q;
  }

  // Everything a trial needs that is expensive to build: cameras, the planner
  // and the training-population test. Build once per page or process.
  function makeEnv(weights, opts) {
    const o = opts || {};
    const W = o.W || CONST.W, H = o.H || CONST.H;
    const trainIK = o.trainIK || planner.createPlanner({ limitScale: 0.9 * pcc.FLEX_RANGE[1], seed: 13 });
    return {
      weights: weights || null,
      camSide: camera.sideCamera(W, H),
      camTop: camera.topCamera(W, H),
      planner: o.planner || planner.createPlanner(),
      trainIK,
      targetTest: (p) => trainIK.solveIK(p, [0, 0, 0, 0]).reachable,
    };
  }

  function makeCtrl(kind, mode, env) {
    const base = kind === 'classical'
      ? ibvs.createClassical()
      : learned.createLearned(env.weights, { targetTest: env.targetTest });
    if (!base) return null;
    return mode === 'planned' ? planner.createTracked(base, env.planner, kind) : planner.createDirect(base, kind);
  }

  // One trial. job = { kind, mode, cond, targetSet, seed }. Returns
  // { settle, ss, oodFrac } or null when a marker left both cameras.
  function runTrial(job, env) {
    const { DT, TRIAL_S, SETTLE_U, SETTLE_HOLD, PRE_STEPS } = CONST;
    const rng = makeRng(job.seed);
    const sim = truth.createTruth(job.seed);
    sim.payload = job.cond.payload;
    sim.driftOn = job.cond.drift;
    const q0 = randomQ(rng, 0, 0.7);
    sim.reset(q0);
    for (let i = 0; i < PRE_STEPS; i++) sim.step(DT);
    const ctrl = makeCtrl(job.kind, job.mode, env);
    if (!ctrl) return null;
    ctrl.reset(q0);
    const target = pcc.tip3(job.targetSet === 'edge' ? randomQ(rng, 0.95, 1.0) : randomQ(rng, 0, 0.85));
    let bandEnter = null, settle = null, oodSteps = 0;
    const tail = [];
    const steps = Math.round(TRIAL_S / DT);
    for (let s = 0; s < steps; s++) {
      const t = s * DT;
      const markers = camera.senseMarkers(sim.markers3(), env.camSide, env.camTop);
      if (!markers) return null;
      const err = v3.norm(v3.sub(markers[3], target));
      if (err < SETTLE_U) {
        if (bandEnter == null) bandEnter = t;
        if (settle == null && t - bandEnter >= SETTLE_HOLD) settle = bandEnter;
      } else bandEnter = null;
      if (t > TRIAL_S - 1) tail.push(err);
      const out = ctrl.step(markers, target, DT);
      sim.setCommand(out.qCmd);
      if (out.ood) oodSteps++;
      sim.step(DT);
    }
    return { settle, ss: tail.reduce((a, b) => a + b, 0) / tail.length, oodFrac: oodSteps / steps, target };
  }

  // The job list for a spec. spec = { trials, targetSets, conditions, controllers, modes }.
  function jobs(spec) {
    const s = spec || {};
    const trials = s.trials || CONST.TRIALS;
    const out = [];
    for (const targetSet of s.targetSets || ['interior', 'edge']) {
      for (const cond of s.conditions || CONDITIONS) {
        for (const kind of s.controllers || ['classical', 'learned']) {
          for (const mode of s.modes || ['direct', 'planned']) {
            for (let n = 0; n < trials; n++) {
              out.push({ kind, mode, cond, targetSet, seed: CONST.SEED_BASE + n * CONST.SEED_STEP, n });
            }
          }
        }
      }
    }
    return out;
  }

  // eval.js's median: the upper middle element.
  function median(a) {
    const s = a.slice().sort((x, y) => x - y);
    return s.length ? s[Math.floor(s.length / 2)] : NaN;
  }

  // Cells exactly as eval.js prints them for one (targetSet, cond, kind):
  // per mode: settled 'x/y', median settle 'n.nn s' or 'n/a', steady 'n.n mm';
  // plus the planned-learned OOD cell.
  function cellsFor(resultsByMode) {
    const cells = [];
    let ood = '–';
    for (const mode of ['direct', 'planned']) {
      const res = (resultsByMode[mode] || []).filter(Boolean);
      const settled = res.filter((r) => r.settle != null);
      cells.push(settled.length + '/' + res.length);
      cells.push(settled.length ? median(settled.map((r) => r.settle)).toFixed(2) + ' s' : 'n/a');
      cells.push(res.length ? (res.reduce((a, r) => a + r.ss, 0) / res.length * CONST.MM).toFixed(1) + ' mm' : '–');
      if (mode === 'planned' && resultsByMode.kind === 'learned') {
        ood = res.length ? (100 * res.reduce((a, r) => a + r.oodFrac, 0) / res.length).toFixed(0) + '% of steps' : '–';
      }
    }
    return { cells, ood };
  }

  // Group finished jobs into eval.json-shaped rows.
  function reduce(jobsDone) {
    const groups = new Map();
    for (const j of jobsDone) {
      const key = j.targetSet + '|' + j.cond.name + '|' + j.kind;
      if (!groups.has(key)) groups.set(key, { targets: j.targetSet, condition: j.cond.name, controller: j.kind, direct: [], planned: [], kind: j.kind });
      groups.get(key)[j.mode].push(j.result);
    }
    const rows = [];
    for (const g of groups.values()) {
      const c = cellsFor(g);
      rows.push({ targets: g.targets, condition: g.condition, controller: g.controller, cells: c.cells, ood: c.ood,
        n: { direct: g.direct.length, planned: g.planned.length } });
    }
    return rows;
  }

  CR.protocol = { CONST, CONDITIONS, randomQ, makeEnv, makeCtrl, runTrial, jobs, cellsFor, reduce, median };
})(typeof globalThis.CR === 'object' ? globalThis.CR : (globalThis.CR = {}));

// ---- src/variants/kit.js ----
// Variants kit: the version 3 scene as a factory. Ports the wiring of
// src/ui/main.js (which stays untouched) so that a page can hold several
// independent scenes, each with its own simulators, views, controls and
// charts, sharing one set of assets (weights, planner, envelope, grid).
//
//   CR.kit.assets()                 memoised shared assets for this page
//   CR.kit.createSim(opts)          DOM-free model: robots, trials, planner, plane
//   CR.kit.createScene(root, opts)  views + controls + charts around a sim
//   CR.kit.createTicker()           one requestAnimationFrame for all scenes
//   CR.kit.runTrials(spec, cb)      the eval protocol, chunked, in the browser
//   CR.kit.tooltipEl()              one shared chart tooltip per document
//   CR.kit.modelCard(meta)          the ensemble's model card as HTML
//   CR.kit.embed()                  child side of the chooser's iframe protocol
(function (CR) {
  'use strict';
  const { pcc, camera, truth, ibvs, learned, scene, chart, workspace, v3, protocol } = CR;
  const plannerMod = CR.planner;

  const CONST = {
    W: 460, H: 345, DT: 1 / 60, SEED: 2026, Q0: [0.5, 0.1, -0.35, 0.3], MM: 100,
    SETTLE_U: 0.05, SETTLE_HOLD: 0.8, TRIAL_S: 6, FAN_HORIZON: 0.35,
    PLANE_MIN: -0.95, PLANE_MAX: 2.1, THUMB: 14,
  };
  // demo targets from main.js: verified to settle under the beat each runs
  const TQ = {
    hook: [2.2 * Math.cos(0.2), 2.2 * Math.sin(0.2), 2.6 * Math.cos(0.2), 2.6 * Math.sin(0.2)],
    sweep: [1.3 * Math.cos(0.2 + Math.PI), 1.3 * Math.sin(0.2 + Math.PI), 0.9 * Math.cos(0.2 + Math.PI), 0.9 * Math.sin(0.2 + Math.PI)],
    lateral: [1.8 * Math.cos(1.77), 1.8 * Math.sin(1.77), 1.4 * Math.cos(1.77), 1.4 * Math.sin(1.77)],
    top: [0.5 * Math.cos(2.5), 0.5 * Math.sin(2.5), 0.5 * Math.cos(2.5 - Math.PI), 0.5 * Math.sin(2.5 - Math.PI)],
  };
  const BEYOND = [0.1, 0.0, 2.3];
  const T3 = (q) => pcc.tip3(q);
  const ACCENT = { classical: '#d95926', learned: '#3987e5' };

  // ---------------- shared assets ----------------
  let _assets = null;
  function assets() {
    if (_assets) return _assets;
    const weights = typeof CR_WEIGHTS !== 'undefined' ? CR_WEIGHTS : null;
    const ws = (typeof CR_WORKSPACE !== 'undefined' && CR_WORKSPACE && CR_WORKSPACE.reach) ? CR_WORKSPACE : null;
    const planner = plannerMod.createPlanner();
    const trainIK = plannerMod.createPlanner({ limitScale: 0.9 * pcc.FLEX_RANGE[1], seed: 13 });
    const targetTest = (p) => trainIK.solveIK(p, [0, 0, 0, 0]).reachable;
    const trainMesh = ws && ws.train ? workspace.envelopeMesh(ws.train) : null;
    const geoCache = new Map();
    // reach geometry at a flexibility: embedded at x1.0, computed otherwise
    function geometry(f) {
      const key = f.toFixed(2);
      if (geoCache.has(key)) return geoCache.get(key);
      let volume, grid;
      if (Math.abs(f - 1) < 1e-9 && ws) {
        volume = ws.reach; grid = workspace.gridFromJSON(ws.grid);
      } else {
        volume = workspace.reachableVolume({ samples: 100000 });
        grid = workspace.reachableGrid({ samples: 150000 });
      }
      const g = { volume, mesh: workspace.envelopeMesh(volume), grid };
      geoCache.set(key, g);
      return g;
    }
    _assets = { weights, ws, planner, trainIK, targetTest, trainMesh, geometry };
    return _assets;
  }

  function makeEmitter() {
    const map = new Map();
    return {
      on(ev, fn) { if (!map.has(ev)) map.set(ev, new Set()); map.get(ev).add(fn); return () => map.get(ev).delete(fn); },
      off(ev, fn) { if (map.has(ev)) map.get(ev).delete(fn); },
      emit(ev, arg) { if (map.has(ev)) for (const fn of map.get(ev)) fn(arg); },
    };
  }

  // ---------------- the DOM-free simulation model ----------------
  function createSim(opts) {
    const o = opts || {};
    const A = assets();
    const { W, H, DT, MM, SETTLE_U, SETTLE_HOLD, TRIAL_S, PLANE_MIN, PLANE_MAX } = CONST;
    const seed = o.seed != null ? o.seed : CONST.SEED;
    const q0 = (o.q0 || CONST.Q0).slice();
    const kinds = o.robots || ['classical', 'learned'];
    const camSide = camera.sideCamera(W, H), camTop = camera.topCamera(W, H);
    const em = makeEmitter();

    const robots = {};
    function buildRobots() {
      for (const k of Object.keys(robots)) delete robots[k];
      if (kinds.includes('classical')) {
        const inner = ibvs.createClassical();
        robots.classical = { key: 'classical', sim: truth.createTruth(seed), inner,
          tracked: plannerMod.createTracked(inner, A.planner, 'classical'),
          direct: plannerMod.createDirect(inner, 'classical'), accent: ACCENT.classical, lastOut: null };
      }
      if (kinds.includes('learned') && A.weights) {
        const inner = learned.createLearned(A.weights, { targetTest: A.targetTest });
        if (inner) robots.learned = { key: 'learned', sim: truth.createTruth(seed), inner,
          tracked: plannerMod.createTracked(inner, A.planner, 'learned'),
          direct: plannerMod.createDirect(inner, 'learned'), accent: ACCENT.learned, lastOut: null };
      }
    }
    buildRobots();
    const list = () => Object.values(robots);

    const st = {
      t: 0, target: null, lastRay: null, rayNote: '', payloadTarget: 0, driftOn: false, usePlan: true,
      trial: null, trialCount: 0, planeY: o.planeY != null ? o.planeY : 1.2, lastSample: null,
    };
    let geo = A.geometry(pcc.flex());
    let section = workspace.gridSectionSegments(geo.grid, st.planeY);

    const ctrlOf = (r) => (st.usePlan ? r.tracked : r.direct);

    function condString(reachable) {
      const c = [];
      if (st.payloadTarget > 0) c.push('payload');
      if (st.driftOn) c.push('drift');
      if (!st.usePlan) c.push('no plan');
      if (!reachable) c.push('beyond reach');
      return c.length ? c.join(' + ') : 'nominal';
    }

    function startTrial(p3) {
      st.target = p3.slice();
      for (const r of list()) ctrlOf(r).newTarget(st.target);
      const ref = robots.classical ? robots.classical.inner.qBelief() : (robots.learned ? robots.learned.inner.qCmd() : [0, 0, 0, 0]);
      const ik = A.planner.solveIK(st.target, ref);
      st.trial = { id: ++st.trialCount, t0: st.t, cond: condString(ik.reachable), reachable: ik.reachable,
        per: { classical: { bandEnter: null, settle: null, tail: [] }, learned: { bandEnter: null, settle: null, tail: [] } } };
      em.emit('target', { target: st.target.slice(), reachable: ik.reachable, trial: st.trial });
      for (const r of list()) em.emit('state', { key: r.key, state: 'servoing' });
      return st.trial;
    }
    function abandonTrial() { st.trial = null; }
    function finishTrial() {
      const tr = st.trial;
      const row = { id: tr.id, cond: tr.cond, reachable: tr.reachable };
      for (const key of ['classical', 'learned']) {
        if (!robots[key]) { row[key] = null; continue; }
        const p = tr.per[key];
        row[key] = { settle: p.settle, steady: p.tail.length ? p.tail.reduce((a, b) => a + b, 0) / p.tail.length * MM : null };
        em.emit('state', { key, state: p.settle != null ? 'settled' : 'did not settle' });
      }
      st.trial = null;
      em.emit('trial', row);
    }

    function step() {
      st.t += DT;
      const sample = { t: st.t, errC: null, errL: null, sigma: null, ood: false };
      for (const r of list()) {
        const pt = st.payloadTarget;
        if (r.sim.payload < pt) r.sim.payload = Math.min(pt, r.sim.payload + 1.2 * DT);
        if (r.sim.payload > pt) r.sim.payload = Math.max(pt, r.sim.payload - 1.2 * DT);
        r.sim.driftOn = st.driftOn;
        const markers = camera.senseMarkers(r.sim.markers3(), camSide, camTop);
        let err = null;
        if (st.target && markers) {
          err = v3.norm(v3.sub(markers[3], st.target));
          const out = ctrlOf(r).step(markers, st.target, DT);
          r.sim.setCommand(out.qCmd);
          r.lastOut = out;
          if (r.key === 'learned') { sample.sigma = out.sigma; sample.ood = out.ood; }
        }
        r.sim.step(DT);
        if (err != null) sample[r.key === 'classical' ? 'errC' : 'errL'] = err * MM;
        if (st.trial && err != null) {
          const p = st.trial.per[r.key];
          const tt = st.t - st.trial.t0;
          if (err < SETTLE_U) {
            if (p.bandEnter == null) p.bandEnter = tt;
            if (p.settle == null && tt - p.bandEnter >= SETTLE_HOLD) p.settle = p.bandEnter;
          } else p.bandEnter = null;
          if (tt > TRIAL_S - 1) p.tail.push(err);
        }
      }
      if (st.trial && st.t - st.trial.t0 >= TRIAL_S) finishTrial();
      if (st.target) { st.lastSample = sample; em.emit('sample', sample); }
      return sample;
    }

    function setPlaneY(z, live) {
      st.planeY = Math.max(PLANE_MIN, Math.min(PLANE_MAX, z));
      section = workspace.gridSectionSegments(geo.grid, st.planeY);
      if (live && st.target) {
        st.target = [st.target[0], st.target[1], st.planeY];
        if (st.trial) abandonTrial();
      }
      em.emit('plane', st.planeY);
    }
    function targetFromRay(origin, dir) {
      const hit = camera.rayPlaneZ(origin, dir, st.planeY);
      st.lastRay = { origin, dir };
      if (hit) { st.rayNote = ''; return hit; }
      const w = v3.sub(camera.CENTER, origin);
      const tt = Math.max(0.1, v3.dot(w, dir));
      st.rayNote = 'ray nearly parallel to the plane: target placed at the nearest point to the workspace centre';
      return v3.add(origin, v3.scale(dir, tt));
    }
    function event(label) { em.emit('event', { t: st.t, label }); }
    function setPlan(on) {
      if (on === st.usePlan) return;
      st.usePlan = on; event(on ? 'plan on' : 'plan off');
      if (st.target) for (const r of list()) ctrlOf(r).newTarget(st.target);
      if (st.trial && st.t - st.trial.t0 < TRIAL_S) abandonTrial();
    }
    function setPayload(v) {
      const p = v ? 1 : 0;
      if (p === st.payloadTarget) return;
      st.payloadTarget = p; event(p ? 'payload on' : 'payload off');
      if (st.trial && st.t - st.trial.t0 < TRIAL_S) abandonTrial();
    }
    function setDrift(on) {
      if (on === st.driftOn) return;
      st.driftOn = on; event(on ? 'drift on' : 'drift off');
      if (st.trial && st.t - st.trial.t0 < TRIAL_S) abandonTrial();
    }
    function refreshGeometry() {
      geo = A.geometry(pcc.flex());
      section = workspace.gridSectionSegments(geo.grid, st.planeY);
      if (st.trial) abandonTrial();
      if (st.target) for (const r of list()) ctrlOf(r).newTarget(st.target);
    }
    function reset() {
      buildRobots();
      for (const r of list()) { r.sim.reset(q0); r.tracked.reset(q0); r.direct.reset(q0); r.lastOut = null; }
      st.target = null; st.lastRay = null; st.rayNote = ''; st.trial = null; st.lastSample = null;
      em.emit('reset');
    }
    reset();

    return {
      robots, list, ctrlOf, camSide, camTop, st,
      step, startTrial, abandonTrial, setPlaneY, targetFromRay, setPlan, setPayload, setDrift,
      refreshGeometry, reset, event,
      geometry: () => geo, section: () => section,
      state: () => ({ t: st.t, target: st.target ? st.target.slice() : null, planeY: st.planeY, usePlan: st.usePlan,
        payload: st.payloadTarget, drift: st.driftOn, trial: st.trial, lastSample: st.lastSample, rayNote: st.rayNote }),
      on: em.on, off: em.off,
    };
  }

  // ---------------- ticker ----------------
  function createTicker() {
    const scenes = new Set();
    let prev = null, frames = 0, fpsT = 0, fps = 0, running = true, raf = null;
    function loop(now) {
      if (prev == null) prev = now;
      let dt = (now - prev) / 1000;
      prev = now;
      if (dt > 0.1) dt = 0.1;
      if (running) for (const s of scenes) if (s.visible && !s.paused) s.advance(dt);
      frames++;
      if (now - fpsT > 1000) { fps = frames; frames = 0; fpsT = now; }
      raf = requestAnimationFrame(loop);
    }
    raf = requestAnimationFrame(loop);
    return {
      add(s) { scenes.add(s); }, remove(s) { scenes.delete(s); },
      pause() { running = false; }, resume() { running = true; prev = null; },
      get running() { return running; }, fps: () => fps, scenes,
    };
  }
  let _ticker = null;
  const ticker = () => (_ticker || (_ticker = createTicker()));

  // ---------------- shared DOM bits ----------------
  function tooltipEl() {
    let el = document.getElementById('tooltip');
    if (!el) { el = document.createElement('div'); el.id = 'tooltip'; document.body.appendChild(el); }
    return el;
  }

  // Markup for a scene block. Every element is found by data-cr, never id.
  function markup(o) {
    const views = o.views || ['inspector', 'side'];
    const parts = [];
    if (views.length) {
      parts.push('<div class="feeds' + (views.length === 1 ? ' one' : '') + '">');
      if (views.includes('inspector')) parts.push(
        '<div class="feed-card"><div class="feed-head"><span class="feed-title">Inspector</span><span class="feed-sub">orbit view</span></div>' +
        '<div class="feed-frame"><canvas data-cr="inspector" width="460" height="345"></canvas>' +
        (o.presets === false ? '' : '<div class="hud-chips"><button class="chip" data-cr-preset="iso">Iso</button><button class="chip" data-cr-preset="side">Side</button><button class="chip" data-cr-preset="top">Top</button></div>') +
        '</div><div class="feed-cap"><span class="grow" data-cr="cap-inspector">drag to orbit · click inside the outline to aim</span></div></div>');
      if (views.includes('side')) parts.push(
        '<div class="feed-card"><div class="feed-head"><span class="feed-title">CAM 01 · side sensor</span><span class="feed-sub">what the controllers see</span></div>' +
        '<div class="feed-frame"><canvas data-cr="side" width="460" height="345"></canvas>' +
        (o.planeSlider === false ? '' : '<input type="range" data-cr="plane" class="vslider" min="0" max="345" step="0.5" value="172" aria-label="target plane height">') +
        '</div><div class="feed-cap"><span class="grow" data-cr="cap-side">drag the plane or the slider</span><span class="cr-plane-val" data-cr="plane-val">target plane z = 120 mm</span></div></div>');
      parts.push('</div>');
    }
    if (o.readouts !== false) {
      const robots = o.robots || ['classical', 'learned'];
      parts.push('<div class="readouts stats">');
      if (robots.includes('classical')) parts.push(
        '<div class="grp stat" data-cr-robot="classical" title="click to hide or show this robot"><div class="toprow"><span class="dot" style="background:var(--classical)"></span><span class="name">Classical</span><span class="state" data-cr="ro-c-state">idle</span></div>' +
        '<div class="metrics"><span><span class="lbl">error</span> <span class="val" data-cr="ro-c-err">–</span></span><span><span class="lbl">settle</span> <span class="val" data-cr="ro-c-settle">–</span></span></div></div>');
      if (robots.includes('learned')) parts.push(
        '<div class="grp stat" data-cr-robot="learned" title="click to hide or show this robot"><div class="toprow"><span class="dot" style="background:var(--learned)"></span><span class="name">Learned ensemble</span><span class="state" data-cr="ro-l-state">idle</span></div>' +
        '<div class="metrics"><span><span class="lbl">error</span> <span class="val" data-cr="ro-l-err">–</span></span><span><span class="lbl">settle</span> <span class="val" data-cr="ro-l-settle">–</span></span><span><span class="lbl">σ</span> <span class="val" data-cr="ro-l-sigma">–</span></span></div></div>');
      parts.push('</div>');
    }
    const tb = o.toolbar === false ? [] : (o.toolbar || ['flex', 'tendons', 'plan', 'payload', 'drift', 'reset']);
    if (tb.length) {
      parts.push('<div class="toolbar">');
      if (tb.includes('flex')) parts.push('<span class="tgroup"><label>Flexibility</label><input type="range" data-cr="flex" min="0.7" max="1.8" step="0.1" value="1"><span class="tval" data-cr="flex-val">×1.0</span></span>');
      const sw = [];
      if (tb.includes('tendons')) sw.push('<label class="switch"><input type="checkbox" data-cr="tgl-tendons"><span class="knob"></span>Tendons</label>');
      if (tb.includes('plan')) sw.push('<label class="switch" title="On: each new target is first solved by inverse kinematics on the ideal model and the controllers track that path. Off: the feedback laws chase the target directly."><input type="checkbox" data-cr="tgl-plan" checked><span class="knob"></span>Planner</label>');
      if (tb.includes('payload')) sw.push('<label class="switch"><input type="checkbox" data-cr="tgl-payload"><span class="knob"></span>Payload</label>');
      if (tb.includes('drift')) sw.push('<label class="switch"><input type="checkbox" data-cr="tgl-drift"><span class="knob"></span>Tendon drift</label>');
      if (sw.length) parts.push('<span class="tgroup">' + sw.join('') + '</span>');
      if (tb.includes('reset')) parts.push('<span class="tgroup last"><button data-cr="btn-reset">Reset</button></span>');
      parts.push('</div>');
    }
    parts.push('<div class="demo-note" data-cr="note"></div>');
    if (o.charts) {
      parts.push('<div class="panel"><div class="panel-title">Tip error over time</div><div class="panel-sub">Distance from the triangulated tip to the target. Hairline: 5 mm settle band. Lengths are shown as if the robot were 180 mm long.</div>' +
        '<div class="legend"><span><i style="background:var(--classical)"></i>classical</span><span><i style="background:var(--learned)"></i>learned</span></div><canvas data-cr="chart-err" width="944" height="200"></canvas></div>');
      parts.push('<div class="panel' + (o.charts === 'err' ? ' cr-hidden' : '') + '"><div class="panel-title">Ensemble disagreement</div><div class="panel-sub">Spread of the five networks\' commands. Shaded spans: outside the training envelope, gain reduced.</div><canvas data-cr="chart-sigma" width="944" height="80"></canvas></div>');
    }
    if (o.table) {
      parts.push('<div class="panel"><div class="panel-title">Trials</div><div class="panel-sub">One row per target, 6 s window. Settle: first time the error stays under 5 mm for 0.8 s. Steady state: mean error over the final second.</div>' +
        '<div class="table-scroll"><table><thead><tr><th>#</th><th>Condition</th><th class="col-c">classical settle</th><th class="col-c">steady state</th><th class="col-l">learned settle</th><th class="col-l">steady state</th></tr></thead>' +
        '<tbody data-cr="rows"><tr data-cr="empty"><td colspan="6">No trials yet. Click in the inspector to place a target.</td></tr></tbody></table></div></div>');
    }
    return parts.join('\n');
  }

  // ---------------- the scene ----------------
  const usedIds = new Set();
  function createScene(root, opts) {
    const o = opts || {};
    if (!o.id) throw new Error('createScene needs a unique id');
    if (usedIds.has(o.id)) throw new Error('scene id already used: ' + o.id);
    usedIds.add(o.id);
    const { W, H, DT, MM, FAN_HORIZON, THUMB, TRIAL_S } = CONST;
    const A = assets();
    if (!root.querySelector('[data-cr]')) root.innerHTML = markup(o);
    const $ = (k) => root.querySelector('[data-cr="' + k + '"]');
    const sim = o.sim || createSim(o);
    const views = {};
    for (const key of ['inspector', 'side']) {
      const cv = $(key);
      if (!cv) continue;
      const dpr = window.devicePixelRatio || 1;
      cv.width = W * dpr; cv.height = H * dpr;
      const ctx = cv.getContext('2d');
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      views[key] = { canvas: cv, ctx };
    }
    const orbit = Object.assign({}, camera.PRESETS[o.preset || 'iso']);
    let camOverride = null;
    const orbitCam = () => camOverride || camera.orbitCamera(orbit.az, orbit.el, W, H, o.dist);
    const visible = { classical: true, learned: true };
    let showTendons = !!o.tendons, showCages = o.cages !== false, showPlane = o.plane !== false;
    let hadClick = false, planeDrag = false, spinning = false, play = null;
    let charts = null;
    if (o.charts && $('chart-err') && $('chart-sigma')) {
      charts = chart.createCharts($('chart-err'), $('chart-sigma'), tooltipEl());
      if (sim.robots.learned && sim.robots.learned.inner.sigmaWarn) charts.setSigmaThreshold(sim.robots.learned.inner.sigmaWarn);
    }
    const em = makeEmitter();

    function presetName() {
      const near = (p) => Math.abs(orbit.az - p.az) < 1e-6 && Math.abs(orbit.el - p.el) < 1e-6;
      if (near(camera.PRESETS.side)) return 'CAM 01 SIDE';
      if (near(camera.PRESETS.top)) return 'CAM 02 TOP';
      if (camOverride) return camOverride.name || 'FIXED CAMERA';
      if (near(camera.PRESETS.iso)) return 'ISO';
      return 'az ' + ((orbit.az * 180) / Math.PI).toFixed(0) + '° el ' + ((orbit.el * 180) / Math.PI).toFixed(0) + '°';
    }
    function fanFor(r) {
      const st = sim.st;
      if (r.key !== 'learned' || !r.lastOut || !st.target || !r.lastOut.membersV) return null;
      const J = ibvs.idealJacobian3(sim.ctrlOf(r).qCmd());
      const tip = r.sim.markers3()[3];
      const toPoint = (vel) => {
        const d = [0, 1, 2].map((k) => (J[k][0] * vel[0] + J[k][1] * vel[1] + J[k][2] * vel[2] + J[k][3] * vel[3]) * FAN_HORIZON);
        const n = v3.norm(d);
        return v3.add(tip, n > 0.4 ? v3.scale(d, 0.4 / n) : d);
      };
      return { members: r.lastOut.membersV.map(toPoint), mean: toPoint(r.lastOut.meanV) };
    }
    function drawAll() {
      const st = sim.st;
      const geo = sim.geometry();
      const learnedOut = sim.robots.learned && sim.robots.learned.lastOut;
      const first = sim.list()[0];
      const live = st.target && first && first.lastOut;
      const phase = live ? (first.lastOut.tracking ? ' · PLAN' : ' · LOOP') : '';
      const robotsDraw = sim.list().filter((r) => visible[r.key]).map((r) => ({
        sim: r.sim, accent: r.accent, fan: fanFor(r),
        sRef: r.lastOut ? r.lastOut.sRef : null, tracking: r.lastOut ? r.lastOut.tracking : false,
      }));
      const common = {
        W, H, robots: robotsDraw, target: st.target, naked: showTendons, section: showPlane ? sim.section() : null,
        volume: { mesh: geo.mesh, show: showCages }, trainVolume: { mesh: showCages ? A.trainMesh : null },
        ood: !!(st.target && learnedOut && learnedOut.ood), oodGain: learned.OOD_GAIN, t: st.t,
      };
      const tendonTag = showTendons ? ' · TENDONS ×' + scene.TENDON_DRAW_SCALE : '';
      if (views.inspector) scene.draw(views.inspector.ctx, Object.assign({}, common, {
        layerKey: o.id + '/inspector', cam: orbitCam(), plane: { y: st.planeY, show: false },
        sensors: o.sensors ? [{ cam: sim.camSide, label: 'CAM 01' }, { cam: sim.camTop, label: 'CAM 02' }] : null,
        label: 'INSPECTOR · ' + presetName() + phase + tendonTag,
        clickHint: !hadClick && !play && o.clickToPlace !== false ? (o.clickHint || 'drag to orbit · click inside the outline to place a target') : null,
        hint: st.rayNote,
      }));
      if (views.side) scene.draw(views.side.ctx, Object.assign({}, common, {
        layerKey: o.id + '/side', cam: sim.camSide, plane: { y: st.planeY, show: showPlane, active: planeDrag }, sensors: null,
        label: 'CAM 01 SIDE SENSOR' + phase + tendonTag,
        clickHint: !hadClick && !play && showPlane && o.planeSlider !== false ? 'drag the plane to set the target height' : null, hint: '',
      }));
      if (charts) charts.draw();
      const last = st.target ? st.lastSample : null;
      const set = (k, v) => { const el = $(k); if (el) el.textContent = v; };
      if (last) {
        set('ro-c-err', last.errC != null ? last.errC.toFixed(1) + ' mm' : '–');
        set('ro-l-err', last.errL != null ? last.errL.toFixed(1) + ' mm' : '–');
        set('ro-l-sigma', last.sigma != null ? last.sigma.toFixed(3) : '–');
      }
      set('plane-val', 'target plane z = ' + (st.planeY * MM).toFixed(0) + ' mm');
      for (const b of root.querySelectorAll('.chip[data-cr-preset]')) {
        const p = camera.PRESETS[b.dataset.crPreset];
        b.classList.toggle('active', Math.abs(orbit.az - p.az) < 1e-6 && Math.abs(orbit.el - p.el) < 1e-6);
      }
      em.emit('draw');
    }

    // sim events into DOM
    sim.on('sample', (s) => { if (charts) charts.push(s); });
    sim.on('event', (e) => { if (charts) charts.addEvent(e.t, e.label); });
    sim.on('state', ({ key, state }) => { const el = $(key === 'classical' ? 'ro-c-state' : 'ro-l-state'); if (el) el.textContent = state; });
    sim.on('trial', (row) => {
      const fmt = (p) => p ? [p.settle != null ? p.settle.toFixed(2) + ' s' : 'dns', p.steady != null ? p.steady.toFixed(1) + ' mm' : '–'] : ['–', '–'];
      const [cs, css] = fmt(row.classical), [ls, lss] = fmt(row.learned);
      const body = $('rows');
      if (body) {
        const empty = $('empty'); if (empty) empty.remove();
        const tr = document.createElement('tr');
        tr.innerHTML = '<td>' + row.id + '</td><td class="cond">' + row.cond + '</td><td>' + cs + '</td><td>' + css + '</td><td>' + ls + '</td><td>' + lss + '</td>';
        body.insertBefore(tr, body.firstChild);
        const max = typeof o.table === 'number' ? o.table : 10;
        while (body.children.length > max) body.removeChild(body.lastChild);
      }
      const set = (k, v) => { const el = $(k); if (el) el.textContent = v; };
      if (row.classical) set('ro-c-settle', row.classical.settle != null ? row.classical.settle.toFixed(2) + ' s' : 'dns');
      if (row.learned) set('ro-l-settle', row.learned.settle != null ? row.learned.settle.toFixed(2) + ' s' : 'dns');
      em.emit('trial', row);
    });
    sim.on('reset', () => {
      if (charts) charts.reset();
      for (const k of ['ro-c-err', 'ro-l-err', 'ro-c-settle', 'ro-l-settle', 'ro-l-sigma']) { const el = $(k); if (el) el.textContent = '–'; }
      for (const k of ['ro-c-state', 'ro-l-state']) { const el = $(k); if (el) el.textContent = 'idle'; }
    });

    // ---- plane slider (1:1 with the plane line in the side view) ----
    function sliderHcss() { const r = views.side ? views.side.canvas.getBoundingClientRect() : null; return (r && r.height) || H; }
    function syncPlaneSlider() {
      const sl = $('plane'); if (!sl || !views.side) return;
      const row = scene.planeScreenY(sim.camSide, sim.st.planeY);
      if (row == null) return;
      const Hc = sliderHcss();
      const v = ((row * Hc / H) - THUMB / 2) / (Hc - THUMB) * H;
      sl.value = Math.max(0, Math.min(H, v)).toFixed(1);
    }
    function setPlaneY(z, live) { sim.setPlaneY(z, live); syncPlaneSlider(); }
    sim.on('plane', () => syncPlaneSlider());
    function canvasPx(ev, cv) { const rect = cv.getBoundingClientRect(); return [(ev.clientX - rect.left) * (W / rect.width), (ev.clientY - rect.top) * (H / rect.height)]; }

    function startTrial(p3) { hadClick = true; return sim.startTrial(p3); }
    function demoTarget(p3) { setPlaneY(p3[2], false); return startTrial(p3); }

    if (views.inspector) {
      const cv = views.inspector.canvas;
      let down = null;
      cv.addEventListener('pointerdown', (ev) => { if (o.orbitDrag === false && o.clickToPlace === false) return; down = { x: ev.clientX, y: ev.clientY, az: orbit.az, el: orbit.el, moved: false }; cv.setPointerCapture(ev.pointerId); });
      cv.addEventListener('pointermove', (ev) => {
        if (!down || o.orbitDrag === false) return;
        const dx = ev.clientX - down.x, dy = ev.clientY - down.y;
        if (!down.moved && Math.hypot(dx, dy) < 4) return;
        down.moved = true;
        orbit.az = down.az - dx * 0.008;
        orbit.el = Math.max(-1.55, Math.min(1.55, down.el + dy * 0.008));
        em.emit('orbit', orbit);
      });
      const up = (ev) => {
        if (!down) return;
        const wasClick = !down.moved; down = null;
        if (!wasClick || o.clickToPlace === false) return;
        if (play) stop('demo stopped, you have the controls');
        const [u, v] = canvasPx(ev, cv);
        const cam = orbitCam();
        startTrial(sim.targetFromRay(cam.pos, cam.rayDir(u, v)));
      };
      cv.addEventListener('pointerup', up);
      cv.addEventListener('pointercancel', () => { down = null; });
    }
    if (views.side && o.planeSlider !== false) {
      const cv = views.side.canvas;
      let moved = false;
      cv.addEventListener('pointerdown', (ev) => {
        const [, v] = canvasPx(ev, cv);
        const py = scene.planeScreenY(sim.camSide, sim.st.planeY);
        if (py != null && Math.abs(v - py) < 16) { planeDrag = true; moved = false; cv.setPointerCapture(ev.pointerId); hadClick = true; if (play) stop('demo stopped, you have the controls'); }
      });
      cv.addEventListener('pointermove', (ev) => {
        if (!planeDrag) return;
        const [u, v] = canvasPx(ev, cv);
        const y = scene.planeYFromPixel(sim.camSide, u, v);
        if (y != null) { setPlaneY(y, true); moved = true; }
      });
      const up = () => { if (!planeDrag) return; planeDrag = false; if (moved && sim.st.target) startTrial(sim.st.target); };
      cv.addEventListener('pointerup', up);
      cv.addEventListener('pointercancel', up);
      const sl = $('plane');
      if (sl) {
        sl.min = 0; sl.max = H; sl.step = 0.5;
        sl.addEventListener('input', (ev) => {
          const Hc = sliderHcss();
          const rowCss = THUMB / 2 + (parseFloat(ev.target.value) / H) * (Hc - THUMB);
          const z = scene.planeYFromPixel(sim.camSide, W / 2, rowCss * H / Hc);
          if (z != null) setPlaneY(z, true);
        });
        sl.addEventListener('change', () => { if (sim.st.target) startTrial(sim.st.target); });
      }
    }
    for (const b of root.querySelectorAll('.chip[data-cr-preset]')) b.addEventListener('click', () => setPreset(b.dataset.crPreset));
    for (const g of root.querySelectorAll('[data-cr-robot]')) g.addEventListener('click', () => { const k = g.dataset.crRobot; visible[k] = !visible[k]; g.classList.toggle('off', !visible[k]); });
    const tglT = $('tgl-tendons'); if (tglT) tglT.addEventListener('change', (ev) => { showTendons = ev.target.checked; });
    const tglP = $('tgl-plan'); if (tglP) tglP.addEventListener('change', (ev) => sim.setPlan(ev.target.checked));
    const tglL = $('tgl-payload'); if (tglL) tglL.addEventListener('change', (ev) => sim.setPayload(ev.target.checked));
    const tglD = $('tgl-drift'); if (tglD) tglD.addEventListener('change', (ev) => sim.setDrift(ev.target.checked));
    const flexEl = $('flex');
    const flexLabel = (f) => '×' + f.toFixed(1) + ' · ' + Math.round((pcc.KMAX_BASE[0] * f * pcc.SEG_LEN[0] * 180) / Math.PI) + '°/' + Math.round((pcc.KMAX_BASE[1] * f * pcc.SEG_LEN[1] * 180) / Math.PI) + '°';
    if (flexEl) {
      flexEl.addEventListener('input', (ev) => { const el = $('flex-val'); if (el) el.textContent = flexLabel(parseFloat(ev.target.value)); });
      flexEl.addEventListener('change', (ev) => setFlex(parseFloat(ev.target.value)));
      const el = $('flex-val'); if (el) el.textContent = flexLabel(pcc.flex());
    }
    const btnReset = $('btn-reset'); if (btnReset) btnReset.addEventListener('click', () => { if (play) stop(''); reset(); });

    function setPreset(name) { camOverride = null; const p = camera.PRESETS[name]; if (p) { orbit.az = p.az; orbit.el = p.el; em.emit('orbit', orbit); } }
    function setInspectorCamera(cam) { camOverride = cam || null; }
    function setOrbit(az, el) { orbit.az = az; orbit.el = el; }
    function note(text) { const el = $('note'); if (el) el.textContent = text || ''; }
    function setToggle(k, on) { const el = $(k); if (el) el.checked = !!on; }
    function setPlan(on) { setToggle('tgl-plan', on); sim.setPlan(!!on); }
    function setPayload(on) { setToggle('tgl-payload', on); sim.setPayload(on ? 1 : 0); }
    function setDrift(on) { setToggle('tgl-drift', on); sim.setDrift(!!on); }
    function setTendons(on) { setToggle('tgl-tendons', on); showTendons = !!on; }
    function setVisible(k, on) { visible[k] = !!on; const g = root.querySelector('[data-cr-robot="' + k + '"]'); if (g) g.classList.toggle('off', !on); }
    function reset() {
      setToggle('tgl-payload', false); setToggle('tgl-drift', false); setToggle('tgl-plan', true);
      sim.st.payloadTarget = 0; sim.st.driftOn = false; sim.st.usePlan = true;
      sim.reset(); note('');
    }

    // page-wide flexibility: every scene in this context follows
    function setFlex(f) {
      note('recomputing reach for flexibility ' + flexLabel(f) + ' ...');
      return new Promise((resolve) => setTimeout(() => {
        pcc.setFlex(f);
        A.planner.rebuild();
        for (const s of ticker().scenes) {
          s.sim.refreshGeometry();
          s.syncFlexUI(f);
          if (s.charts) s.charts.addEvent(s.sim.st.t, 'flex ×' + f.toFixed(1));
        }
        scene.invalidateLayers();
        note('');
        resolve(f);
      }, 30));
    }
    function syncFlexUI(f) { if (flexEl) flexEl.value = String(f); const el = $('flex-val'); if (el) el.textContent = flexLabel(f); }

    // ---- scripted acts: [{at, fn, note}] ----
    function playSteps(steps) { play = { steps, idx: 0, tStart: sim.st.t }; hadClick = true; em.emit('play', true); }
    function runPlay() {
      if (!play) return;
      if (spinning) orbit.az -= 0.035 * DT;
      while (play && play.idx < play.steps.length && sim.st.t - play.tStart >= play.steps[play.idx].at) {
        const st = play.steps[play.idx++];
        st.fn();
        if (st.note) note('▸ ' + st.note);
      }
    }
    function stop(msg) { play = null; spinning = false; if (typeof msg === 'string') note(msg); em.emit('play', false); }

    // ---- stepping and visibility ----
    let acc = 0;
    const self = {
      id: o.id, root, sim, charts, views, orbit, visible: true, paused: false,
      robots: sim.robots,
      advance(dtReal) {
        acc += dtReal;
        let n = 0;
        while (acc >= DT && n < 4) { sim.step(); runPlay(); acc -= DT; n++; }
        drawAll();
      },
      step: () => { sim.step(); runPlay(); }, draw: drawAll,
      startTrial, demoTarget, setPlaneY, setPlan, setPayload, setDrift, setTendons, setVisible, setPreset, setOrbit,
      setCages(on) { showCages = !!on; }, setPlaneShown(on) { showPlane = !!on; }, setInspectorCamera, orbitCam,
      spin(on) { spinning = !!on; }, setFlex, syncFlexUI, reset, play: playSteps, stop, note, isPlaying: () => !!play,
      pause() { self.paused = true; }, resume() { self.paused = false; },
      on: em.on, off: em.off, $,
      destroy() { ticker().remove(self); usedIds.delete(o.id); },
    };
    if (o.autoVisibility !== false && typeof IntersectionObserver === 'function') {
      const io = new IntersectionObserver((entries) => { self.visible = entries.some((e) => e.isIntersecting); }, { rootMargin: '120px' });
      io.observe(root);
    }
    syncPlaneSlider();
    drawAll();
    ticker().add(self);
    return self;
  }

  // ---------------- in-browser evaluation ----------------
  function runTrials(spec, onProgress) {
    const A = assets();
    const env = protocol.makeEnv(A.weights, { planner: A.planner, trainIK: A.trainIK });
    const js = protocol.jobs(spec);
    let i = 0, cancelled = false;
    const budget = (spec && spec.budgetMs) || 30;
    const promise = new Promise((resolve) => {
      function chunk() {
        if (cancelled) { resolve(null); return; }
        const t0 = performance.now();
        while (i < js.length && performance.now() - t0 < budget) { js[i].result = protocol.runTrial(js[i], env); i++; }
        if (onProgress) onProgress({ done: i, total: js.length, rows: protocol.reduce(js.slice(0, i)), jobs: js });
        if (i < js.length) setTimeout(chunk, 0); else resolve({ rows: protocol.reduce(js), jobs: js });
      }
      setTimeout(chunk, 0);
    });
    return { promise, cancel: () => { cancelled = true; } };
  }

  // ---------------- the setup, drawn once with labels ----------------
  function schematic(box) {
    if (!box) return;
    const cv = box.querySelector('canvas'), svg = box.querySelector('svg');
    const W = 920, H = 690, dpr = window.devicePixelRatio || 1;
    cv.width = W * dpr; cv.height = H * dpr;
    const ctx = cv.getContext('2d'); ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    const cam = camera.orbitCamera(0.75, 0.42, W, H);
    const sim = truth.createTruth(1); sim.reset([0.9, 0.25, 0.6, -0.4]);
    const target = [0.55, 0.65, 1.1];
    const g = assets().geometry(pcc.flex());
    scene.draw(ctx, { W, H, cam, robots: [{ sim, accent: ACCENT.classical }], target, plane: { y: 1.1, show: true },
      section: workspace.gridSectionSegments(g.grid, 1.1), volume: { mesh: g.mesh, show: true }, trainVolume: { mesh: null },
      sensors: [{ cam: camera.sideCamera(460, 345), label: '' }, { cam: camera.topCamera(460, 345), label: '' }],
      label: 'THE SETUP', t: 0, layerKey: 'kit/schematic' });
    const m = sim.markers3();
    const NS = 'http://www.w3.org/2000/svg';
    const lab = (p, text, dx, dy) => { const s = cam.project(p); if (!s) return; const el = document.createElementNS(NS, 'text'); el.setAttribute('x', s[0] + (dx || 8)); el.setAttribute('y', s[1] + (dy || -8)); el.textContent = text; svg.appendChild(el); };
    lab([0, 0, 0], 'base', 10, 18);
    lab(pcc.poseAt(sim.qEff(), 0, 0.3).p, 'segment 1, length 1.0', -150, 0);
    lab(pcc.poseAt(sim.qEff(), 1, 0.3).p, 'segment 2, length 0.8', 12, 0);
    ['m1 (mid)', 'm2 (end)', 'm3 (mid)', 'm4 (tip)'].forEach((s, i) => lab(m[i], s, 10, -8));
    lab(camera.sideCamera(460, 345).pos, 'CAM 01, side', -40, -12);
    lab(camera.topCamera(460, 345).pos, 'CAM 02, top', 10, -8);
    lab(target, 'target on the plane', 12, 20);
    lab([1.5, -1.5, 1.1], 'target plane, z = 110 mm', -40, -8);
    const dir = v3.normalize(v3.sub(target, cam.pos));
    const a = cam.project(v3.sub(target, v3.scale(dir, 1.4))), b = cam.project(target);
    if (a && b) { const l = document.createElementNS(NS, 'line'); l.setAttribute('x1', a[0]); l.setAttribute('y1', a[1]); l.setAttribute('x2', b[0]); l.setAttribute('y2', b[1]); l.setAttribute('stroke-dasharray', '4 4'); svg.appendChild(l); lab(v3.sub(target, v3.scale(dir, 0.9)), 'click ray', 10, -6); }
  }

  // ---------------- model card ----------------
  function modelCard(meta) {
    const m = meta || (typeof CR_WEIGHTS !== 'undefined' && CR_WEIGHTS && CR_WEIGHTS.meta) || {};
    const arch = (m.arch || []).join(', ');
    return '<dl class="cr-card">' +
      '<dt>What it is</dt><dd>' + (m.members || 5) + ' multilayer perceptrons, layer sizes ' + arch + ', an ensemble.</dd>' +
      '<dt>What it predicts</dt><dd>A 4 by 3 gain matrix G from the triangulated markers and the tip error; the command is v = G e, so it is zero at zero error.</dd>' +
      '<dt>Fitted on</dt><dd>' + (m.samples || 0).toLocaleString('en-US') + ' samples from ' + (m.episodes || 0) + ' simulated episodes across flexibility ×' + (m.flexRange ? m.flexRange[0] : '') + ' to ×' + (m.flexRange ? m.flexRange[1] : '') + ', labels from a training-time expert that reads the simulator\'s true Jacobian; ' + (m.epochs || 0) + ' epochs.</dd>' +
      '<dt>Held out</dt><dd>10 percent of the samples, holdout MSE ' + (m.holdoutMse != null ? m.holdoutMse.toFixed(3) : '–') + ' in normalised units; the two extrapolation thresholds sit at the held-out 99.5th percentile.</dd>' +
      '</dl>';
  }

  // ---------------- iframe protocol (child side) ----------------
  function embed(opts) {
    if (typeof window === 'undefined' || window === window.top) return null;
    const id = (opts && opts.id) || document.body.dataset.variant || '';
    const post = (msg) => window.parent.postMessage(Object.assign({ cr: 1, id }, msg), '*');
    const height = () => post({ type: 'height', h: document.documentElement.scrollHeight });
    window.addEventListener('message', (ev) => {
      const d = ev.data;
      if (!d || d.cr !== 1) return;
      if (d.type === 'pause') ticker().pause();
      if (d.type === 'resume') ticker().resume();
      if (d.type === 'tour' && window.CR_TOUR) { if (d.action === 'open') window.CR_TOUR.open(0); else window.CR_TOUR.close(); }
    });
    if (typeof ResizeObserver === 'function') new ResizeObserver(height).observe(document.body);
    window.addEventListener('load', height);
    setInterval(() => post({ type: 'fps', fps: ticker().fps() }), 1000);
    post({ type: 'ready' });
    height();
    return { post, scrollTo: (y) => post({ type: 'scrollTo', y }) };
  }

  CR.kit = { CONST, TQ, BEYOND, T3, ACCENT, assets, createSim, createScene, createTicker, ticker, markup, tooltipEl, runTrials, modelCard, schematic, embed };
})(typeof globalThis.CR === 'object' ? globalThis.CR : (globalThis.CR = {}));

// ---- src/variants/spotlight.js ----
// Spotlight walkthrough (DEMO-STANDARDS section 6): dims the page, cuts a
// window around one element, and puts a card beside it with Back, Next, Close
// and progress dots. The dim layer takes no pointer events, so every control
// keeps working while the tour is open. Placement is computed from document
// coordinates, then the page scrolls; nothing waits on a timer.
// Ported from grid-health-demo/app.js with the embedded (iframe) case added.
(function (CR) {
  'use strict';

  function createTour(steps, opts) {
    const o = opts || {};
    const doc = document;
    const root = doc.createElement('div');
    root.className = 'cr-tour';
    root.innerHTML = '<div class="cr-tour-hl"></div><div class="cr-tour-card" role="dialog" aria-live="polite"></div>';
    doc.body.appendChild(root);
    const hl = root.firstChild, card = root.lastChild;
    let idx = 0, open = false;
    const reduce = () => matchMedia('(prefers-reduced-motion: reduce)').matches;
    const embedded = () => window !== window.top;

    function scrollTo(y) {
      if (embedded() && window.CR_EMBED) { window.CR_EMBED.scrollTo(y); return; }
      // long jumps go instantly: a smooth scroll over thousands of pixels reads as a lost cursor
      const far = Math.abs(window.scrollY - y) > 2500;
      window.scrollTo({ top: y, behavior: reduce() || far ? 'auto' : 'smooth' });
    }
    function place() {
      const st = steps[idx];
      const el = st && doc.querySelector(st.target);
      if (!el) { if (idx < steps.length - 1) { idx++; place(); } else close(); return; }
      if (st.onEnter) { try { st.onEnter(); } catch (e) { /* the tour must survive an act's error */ } }
      const r = el.getBoundingClientRect(), sx = window.scrollX, sy = window.scrollY;
      const top = r.top + sy, left = r.left + sx;
      root.style.height = doc.documentElement.scrollHeight + 'px';
      hl.style.left = (left - 8) + 'px'; hl.style.top = (top - 8) + 'px';
      hl.style.width = (r.width + 16) + 'px'; hl.style.height = (r.height + 16) + 'px';
      const dots = steps.map((_, i) => '<i class="' + (i === idx ? 'on' : '') + '"></i>').join('');
      const body = typeof st.body === 'function' ? st.body() : st.body;
      card.innerHTML = '<div class="tk">' + (st.title || '') + ' · ' + (idx + 1) + ' of ' + steps.length + '</div><p>' + body + '</p>' +
        '<div class="cr-tour-nav"><div class="dots">' + dots + '</div>' +
        (idx > 0 ? '<button class="cr-tour-btn" data-t="back">Back</button>' : '') +
        '<button class="cr-tour-btn" data-t="close">Close</button>' +
        '<button class="cr-tour-btn primary" data-t="next">' + (idx < steps.length - 1 ? 'Next' : 'Done') + '</button></div>';
      const cw = Math.min(400, window.innerWidth - 32);
      const narrow = window.innerWidth < 700;
      let cx = left + r.width + 18, cy = top;
      if (narrow || st.place === 'below' || cx + cw > sx + window.innerWidth - 16) { cx = Math.max(16 + sx, Math.min(left, sx + window.innerWidth - cw - 16)); cy = top + r.height + 14; }
      card.style.left = cx + 'px'; card.style.top = cy + 'px';
      card.querySelector('[data-t="next"]').onclick = next;
      card.querySelector('[data-t="close"]').onclick = close;
      const b = card.querySelector('[data-t="back"]'); if (b) b.onclick = back;
      // scroll so the highlighted element sits above its card and inside the viewport
      const want = top - Math.max(24, (window.innerHeight - r.height - 260) / 2);
      scrollTo(Math.max(0, want));
    }
    function next() { if (idx >= steps.length - 1) { close(); return; } idx++; place(); }
    function back() { idx = Math.max(0, idx - 1); place(); }
    function goto(i) { idx = Math.max(0, Math.min(steps.length - 1, i)); if (!open) openAt(idx); else place(); }
    function openAt(i) { idx = i || 0; open = true; root.classList.add('on'); place(); }
    function close() { open = false; root.classList.remove('on'); try { if (o.storageKey) localStorage.setItem(o.storageKey, 'done'); } catch (e) { /* storage may be unavailable */ } }
    function onKey(ev) {
      if (!open) return;
      if (ev.key === 'Escape') close();
      else if (ev.key === 'ArrowRight') next();
      else if (ev.key === 'ArrowLeft') back();
    }
    doc.addEventListener('keydown', onKey);
    window.addEventListener('resize', () => { if (open) place(); });
    if (o.button) { const b = typeof o.button === 'string' ? doc.querySelector(o.button) : o.button; if (b) b.addEventListener('click', () => openAt(0)); }
    let seen = null;
    try { seen = o.storageKey ? localStorage.getItem(o.storageKey) : null; } catch (e) { seen = null; }
    if (o.autoOpenOnce && !seen && !embedded()) setTimeout(() => openAt(0), 700);

    const api = { open: openAt, close, next, back, goto, isOpen: () => open, place, steps,
      destroy() { doc.removeEventListener('keydown', onKey); root.remove(); } };
    return api;
  }

  CR.spotlight = { createTour };
})(typeof globalThis.CR === 'object' ? globalThis.CR : (globalThis.CR = {}));

// ---- src/variants/charts2.js ----
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
