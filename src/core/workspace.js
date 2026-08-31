// Reachable workspace of the ideal model, as the camera sees it.
//
// Version 1 drew the convex hull of the training targets and called it
// "close to the reachable set". Version 2 draws the reachable set itself:
// sample the configuration space densely up to the curvature limits, project
// every tip through the camera, rasterize the hits, and trace the boundary of
// the occupied region. It is a silhouette of a 3D workspace under a
// perspective camera, so it is irregular by nature; nothing about it is
// hand-drawn. The truth simulator clamps its effective curvature to the same
// limits, so the real tip never leaves this outline (checked in test/sanity.js).
(function (CR) {
  'use strict';
  const { pcc } = CR;

  function reachableOutline(cam, opts) {
    const o = Object.assign({ samples: 200000, cell: 3, pad: 80, seed: 3, tol: 1.5 }, opts || {});
    const rng = CR.makeRng(o.seed);
    const x0 = -o.pad, y0 = -o.pad;
    const nx = Math.ceil((cam.w + 2 * o.pad) / o.cell), ny = Math.ceil((cam.h + 2 * o.pad) / o.cell);
    const grid = new Uint8Array(nx * ny);
    for (let n = 0; n < o.samples; n++) {
      const q = [];
      for (let i = 0; i < pcc.NSEG; i++) {
        const a = rng() * 2 * Math.PI, k = Math.sqrt(rng()) * pcc.KMAX[i];
        q.push(k * Math.cos(a), k * Math.sin(a));
      }
      const p = cam.project(pcc.tip3(q));
      if (!p) continue;
      const ix = Math.floor((p[0] - x0) / o.cell), iy = Math.floor((p[1] - y0) / o.cell);
      if (ix >= 0 && ix < nx && iy >= 0 && iy < ny) grid[iy * nx + ix] = 1;
    }
    // morphological closing (one cell) removes sampling holes near the edge
    const closed = close(grid, nx, ny);
    const contour = traceOuter(closed, nx, ny);
    const px = contour.map(([ix, iy]) => [x0 + (ix + 0.5) * o.cell, y0 + (iy + 0.5) * o.cell]);
    return simplify(px, o.tol);
  }

  function close(g, nx, ny) {
    const d = new Uint8Array(nx * ny), e = new Uint8Array(nx * ny);
    const at = (a, x, y) => (x < 0 || y < 0 || x >= nx || y >= ny ? 0 : a[y * nx + x]);
    for (let y = 0; y < ny; y++) for (let x = 0; x < nx; x++) {
      let v = 0;
      for (let dy = -1; dy <= 1 && !v; dy++) for (let dx = -1; dx <= 1; dx++) if (at(g, x + dx, y + dy)) { v = 1; break; }
      d[y * nx + x] = v;
    }
    for (let y = 0; y < ny; y++) for (let x = 0; x < nx; x++) {
      let v = 1;
      for (let dy = -1; dy <= 1 && v; dy++) for (let dx = -1; dx <= 1; dx++) if (!at(d, x + dx, y + dy)) { v = 0; break; }
      e[y * nx + x] = v;
    }
    return e;
  }

  // Moore-neighbour boundary trace of the component containing the topmost,
  // leftmost filled cell (the outer contour of the largest region in practice).
  function traceOuter(g, nx, ny) {
    const at = (x, y) => (x < 0 || y < 0 || x >= nx || y >= ny ? 0 : g[y * nx + x]);
    let sx = -1, sy = -1;
    for (let y = 0; y < ny && sx < 0; y++) for (let x = 0; x < nx; x++) if (g[y * nx + x]) { sx = x; sy = y; break; }
    if (sx < 0) return [];
    // 8 neighbours, clockwise starting from west
    const N = [[-1, 0], [-1, -1], [0, -1], [1, -1], [1, 0], [1, 1], [0, 1], [-1, 1]];
    const out = [[sx, sy]];
    let cx = sx, cy = sy, back = 0; // index of the neighbour we came from (west)
    for (let guard = 0; guard < nx * ny * 4; guard++) {
      let found = -1;
      for (let k = 0; k < 8; k++) {
        const idx = (back + k) % 8;
        if (at(cx + N[idx][0], cy + N[idx][1])) { found = idx; break; }
      }
      if (found < 0) break; // isolated cell
      cx += N[found][0]; cy += N[found][1];
      back = (found + 5) % 8; // start next search from the cell behind us
      if (cx === sx && cy === sy) break;
      out.push([cx, cy]);
    }
    return out;
  }

  // Douglas-Peucker on a closed polygon (split at the farthest point from
  // the first vertex so both halves are open chains).
  function simplify(poly, tol) {
    if (poly.length < 8) return poly;
    let far = 0, fd = -1;
    for (let i = 1; i < poly.length; i++) {
      const d = Math.hypot(poly[i][0] - poly[0][0], poly[i][1] - poly[0][1]);
      if (d > fd) { fd = d; far = i; }
    }
    const a = dp(poly.slice(0, far + 1), tol);
    const b = dp(poly.slice(far).concat([poly[0]]), tol);
    return a.slice(0, -1).concat(b.slice(0, -1));
  }
  function dp(pts, tol) {
    if (pts.length < 3) return pts;
    const [ax, ay] = pts[0], [bx, by] = pts[pts.length - 1];
    const L = Math.hypot(bx - ax, by - ay) || 1e-9;
    let idx = 0, dmax = 0;
    for (let i = 1; i < pts.length - 1; i++) {
      const d = Math.abs((bx - ax) * (ay - pts[i][1]) - (ax - pts[i][0]) * (by - ay)) / L;
      if (d > dmax) { dmax = d; idx = i; }
    }
    if (dmax <= tol) return [pts[0], pts[pts.length - 1]];
    return dp(pts.slice(0, idx + 1), tol).slice(0, -1).concat(dp(pts.slice(idx), tol));
  }

  function pointInPolygon(p, poly) {
    let inside = false;
    for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
      const a = poly[i], b = poly[j];
      if ((a[1] > p[1]) !== (b[1] > p[1]) &&
          p[0] < ((b[0] - a[0]) * (p[1] - a[1])) / (b[1] - a[1]) + a[0]) inside = !inside;
    }
    return inside;
  }

  CR.workspace = { reachableOutline, pointInPolygon };
})(typeof globalThis.CR === 'object' ? globalThis.CR : (globalThis.CR = {}));
