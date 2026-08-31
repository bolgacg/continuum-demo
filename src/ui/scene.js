// 3D scene rendering onto a 2D canvas through any camera: both robots, the
// reachable envelope, the floor grid, the target with its drop line, the
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
  const FLOOR_Y = -1.75;  // ground reference plane, below the workspace
  const PLANE = { x: [-1.7, 1.7], z: [-1.3, 2.1] };

  function hexToRgb(hex) {
    return [parseInt(hex.slice(1, 3), 16), parseInt(hex.slice(3, 5), 16), parseInt(hex.slice(5, 7), 16)];
  }

  function drawFloor(ctx, cam) {
    ctx.strokeStyle = 'rgba(138,143,136,0.10)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    for (let x = -1.6; x <= 1.6001; x += 0.4) {
      const a = cam.project([x, FLOOR_Y, -1.2]), b = cam.project([x, FLOOR_Y, 2.0]);
      if (a && b) { ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); }
    }
    for (let z = -1.2; z <= 2.0001; z += 0.4) {
      const a = cam.project([-1.6, FLOOR_Y, z]), b = cam.project([1.6, FLOOR_Y, z]);
      if (a && b) { ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); }
    }
    ctx.stroke();
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

  const layerCache = new Map();
  function staticLayers(key, cam, W, H, volume) {
    const sig = cam.pos.map((v) => v.toFixed(5)).join(',') + '|' + cam.up.map((v) => v.toFixed(5)).join(',') + '|' + W + 'x' + H + '|' + !!(volume && volume.show);
    let entry = layerCache.get(key);
    if (entry && entry.sig === sig) return entry;
    const dpr = (typeof window !== 'undefined' && window.devicePixelRatio) || 1;
    const make = () => {
      const c = document.createElement('canvas');
      c.width = Math.round(W * dpr); c.height = Math.round(H * dpr);
      const g = c.getContext('2d');
      g.setTransform(dpr, 0, 0, dpr, 0, 0);
      return { c, g };
    };
    const far = make(), near = make();
    drawFloor(far.g, cam);
    if (volume && volume.show && volume.mesh) {
      // smooth silhouette from the full-resolution mesh, wireframe from a
      // coarser one; both rasterized once per camera pose
      const fine = domeQuads(cam, volume.mesh, 1);
      const coarse = domeQuads(cam, volume.mesh, 3);
      fillQuads(far.g, fine.far, 'rgba(138,143,136,0.06)', 'rgba(0,0,0,0)');
      fillQuads(far.g, coarse.far, 'rgba(0,0,0,0)', 'rgba(138,143,136,0.10)');
      fillQuads(near.g, fine.near, 'rgba(138,143,136,0.05)', 'rgba(0,0,0,0)');
      fillQuads(near.g, coarse.near, 'rgba(0,0,0,0)', 'rgba(138,143,136,0.07)');
    }
    entry = { sig, far: far.c, near: near.c };
    layerCache.set(key, entry);
    return entry;
  }

  function drawPlane(ctx, cam, y, o) {
    const c = [[PLANE.x[0], y, PLANE.z[0]], [PLANE.x[0], y, PLANE.z[1]], [PLANE.x[1], y, PLANE.z[1]], [PLANE.x[1], y, PLANE.z[0]]];
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
    // grab handle at the near-right edge (base side, toward the camera)
    const hpx = cam.project([PLANE.x[1], y, PLANE.z[0] + 0.15]);
    if (hpx) {
      ctx.fillStyle = o.active ? '#e8eae6' : 'rgba(232,234,230,0.7)';
      ctx.fillRect(hpx[0] - 5, hpx[1] - 5, 10, 10);
      ctx.font = '10px ui-monospace, Menlo, Consolas, monospace';
      ctx.fillStyle = HUD;
      ctx.fillText('target plane  y = ' + (y * 100).toFixed(0) + ' mm', hpx[0] - 150, hpx[1] - 9);
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

    // reticle + corner brackets (camera-feed look, both views)
    ctx.strokeStyle = 'rgba(138,143,136,0.08)';
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

    // static layers (floor grid + envelope halves) are rasterized once per
    // camera pose into offscreen canvases and blitted; re-rendered only when
    // the camera moves (orbit drag) or the view is first drawn
    const layers = staticLayers(o.layerKey || 'default', cam, W, H, o.volume);
    ctx.drawImage(layers.far, 0, 0, W, H);

    if (o.plane && o.plane.show) drawPlane(ctx, cam, o.plane.y, o.plane);

    // target: drop line to the floor, then the marker itself
    let tpx = null;
    if (o.target) {
      tpx = cam.project(o.target);
      const foot = cam.project([o.target[0], FLOOR_Y, o.target[2]]);
      if (tpx && foot) {
        ctx.strokeStyle = 'rgba(232,234,230,0.25)';
        ctx.setLineDash([2, 4]);
        ctx.beginPath(); ctx.moveTo(tpx[0], tpx[1]); ctx.lineTo(foot[0], foot[1]); ctx.stroke();
        ctx.setLineDash([]);
        ctx.strokeStyle = 'rgba(232,234,230,0.35)';
        ctx.beginPath(); ctx.arc(foot[0], foot[1], 3, 0, 2 * Math.PI); ctx.stroke();
      }
    }

    // depth-sorted primitives: tube segments of both robots
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
          const wPx = Math.max(2, (cam.f * 2 * R_TUBE) / z);
          const zn = Math.min(1, Math.max(0, (z - 1.6) / 2.6));
          const shade = 1 - 0.4 * zn;
          ctx.strokeStyle = `rgba(${Math.round((rgb[0] * 0.55 + 120 * 0.45) * shade)},${Math.round((rgb[1] * 0.55 + 122 * 0.45) * shade)},${Math.round((rgb[2] * 0.55 + 118 * 0.45) * shade)},0.85)`;
          ctx.lineWidth = wPx;
          ctx.lineCap = 'round';
          ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke();
        } });
      }
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
    if (o.hint) {
      ctx.fillStyle = 'rgba(232,234,230,0.6)';
      ctx.font = '10.5px ui-monospace, Menlo, Consolas, monospace';
      ctx.fillText(o.hint, 14, 22);
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
      ctx.fillText(msg, (W - ctx.measureText(msg).width) / 2, H / 2 - 40);
      ctx.globalAlpha = 1;
    }
    ctx.restore();
  }

  // Screen v of the plane's centre line in a camera (for drag hit tests), and
  // the plane height that puts it under a given pixel row: intersect the
  // pixel's ray with the vertical plane x = CENTER.x.
  function planeScreenY(cam, y) {
    const p = cam.project([camera.CENTER[0], y, camera.CENTER[2]]);
    return p ? p[1] : null;
  }
  function planeYFromPixel(cam, u, v) {
    const d = cam.rayDir(u, v);
    if (Math.abs(d[0]) < 1e-6) return null;
    const t = (camera.CENTER[0] - cam.pos[0]) / d[0];
    return cam.pos[1] + t * d[1];
  }

  CR.scene = { draw, planeScreenY, planeYFromPixel, FLOOR_Y, R_TUBE };
})(typeof globalThis.CR === 'object' ? globalThis.CR : (globalThis.CR = {}));
