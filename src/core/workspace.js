// Reachable workspace of the ideal model, in 3D.
//
// Sample the configuration space densely up to the curvature limits, keep the
// tip positions, and describe their outer boundary as a radial envelope
// around the cloud's centroid: for each direction (theta, phi) the largest
// tip radius seen in that angular bin, hole-filled, smoothed, and then scaled
// up by the smallest factor that keeps 99.95% of the samples inside, plus
// half a percent. The
// result is a closed, smooth surface, the "dome" drawn in the inspector. It
// is an outer envelope: the reachable set is a solid inside it, and where the
// true set is concave the envelope is generous. Reachability of a specific
// target is decided by the planner's inverse kinematics, not by this surface.
(function (CR) {
  'use strict';
  const { pcc, v3 } = CR;

  function reachableVolume(opts) {
    const o = Object.assign({ samples: 200000, nt: 36, np: 72, seed: 3, keep: 0.9995, margin: 1.005, smooth: 2, scale: 1.0 }, opts || {});
    const rng = CR.makeRng(o.seed);
    const pts = new Float64Array(o.samples * 3);
    const c = [0, 0, 0];
    for (let n = 0; n < o.samples; n++) {
      const q = [];
      for (let i = 0; i < pcc.NSEG; i++) {
        const a = rng() * 2 * Math.PI, k = Math.sqrt(rng()) * o.scale * pcc.KMAX[i];
        q.push(k * Math.cos(a), k * Math.sin(a));
      }
      const p = pcc.tip3(q);
      pts[3 * n] = p[0]; pts[3 * n + 1] = p[1]; pts[3 * n + 2] = p[2];
      c[0] += p[0] / o.samples; c[1] += p[1] / o.samples; c[2] += p[2] / o.samples;
    }
    const { nt, np } = o;
    const rmax = new Float64Array(nt * np);
    const binOf = (p) => {
      const d = v3.sub(p, c), r = v3.norm(d);
      const th = Math.acos(Math.max(-1, Math.min(1, d[2] / (r || 1e-9)))); // polar axis = robot axis (+z)
      const ph = Math.atan2(d[1], d[0]);
      const i = Math.min(nt - 1, Math.floor((th / Math.PI) * nt));
      const j = ((Math.floor(((ph + Math.PI) / (2 * Math.PI)) * np) % np) + np) % np;
      return { i, j, r };
    };
    for (let n = 0; n < o.samples; n++) {
      const b = binOf([pts[3 * n], pts[3 * n + 1], pts[3 * n + 2]]);
      if (b.r > rmax[b.i * np + b.j]) rmax[b.i * np + b.j] = b.r;
    }
    // hole filling: empty bins take the mean of their filled neighbours
    let grid = Float64Array.from(rmax);
    for (let pass = 0; pass < 8; pass++) {
      const next = Float64Array.from(grid);
      let holes = 0;
      for (let i = 0; i < nt; i++) for (let j = 0; j < np; j++) {
        if (grid[i * np + j] > 0) continue;
        let s = 0, k = 0;
        for (let di = -1; di <= 1; di++) for (let dj = -1; dj <= 1; dj++) {
          const ii = i + di, jj = ((j + dj) % np + np) % np;
          if (ii < 0 || ii >= nt) continue;
          const v = grid[ii * np + jj];
          if (v > 0) { s += v; k++; }
        }
        if (k) next[i * np + j] = s / k; else holes++;
      }
      grid = next;
      if (!holes) break;
    }
    // smoothing: box filter, wrap in phi, clamp in theta
    for (let pass = 0; pass < o.smooth; pass++) {
      const next = new Float64Array(nt * np);
      for (let i = 0; i < nt; i++) for (let j = 0; j < np; j++) {
        let s = 0, k = 0;
        for (let di = -1; di <= 1; di++) for (let dj = -1; dj <= 1; dj++) {
          const ii = i + di, jj = ((j + dj) % np + np) % np;
          if (ii < 0 || ii >= nt) continue;
          s += grid[ii * np + jj]; k++;
        }
        next[i * np + j] = s / k;
      }
      grid = next;
    }
    // containment margin: scale so that `keep` of the samples are inside
    const ratios = new Float64Array(o.samples);
    for (let n = 0; n < o.samples; n++) {
      const b = binOf([pts[3 * n], pts[3 * n + 1], pts[3 * n + 2]]);
      ratios[n] = b.r / grid[b.i * np + b.j];
    }
    ratios.sort();
    const scale = Math.max(1, ratios[Math.floor(o.keep * (o.samples - 1))]) * o.margin;
    for (let k = 0; k < grid.length; k++) grid[k] *= scale;
    let inside = 0;
    for (let n = 0; n < o.samples; n++) if (ratios[n] <= scale) inside++;
    return {
      center: c.map((v) => Math.round(v * 1e4) / 1e4),
      nt, np,
      r: Array.from(grid, (v) => Math.round(v * 1e4) / 1e4),
      meta: { samples: o.samples, curvatureFraction: o.scale, marginScale: Math.round(scale * 1e4) / 1e4, insideFrac: inside / o.samples },
    };
  }

  // Direction for bin centres (theta from +z, phi about +z measured from +x
  // toward +y, matching binOf above).
  function dir(theta, phi) {
    const st = Math.sin(theta);
    return [st * Math.cos(phi), st * Math.sin(phi), Math.cos(theta)];
  }

  // Vertex grid of the envelope surface: rows theta (with pole rows added),
  // columns phi; each entry a world point.
  function envelopeMesh(vol) {
    const { nt, np, r, center } = vol;
    const rows = [];
    const poleR = (i) => { let s = 0; for (let j = 0; j < np; j++) s += r[i * np + j]; return s / np; };
    rows.push(new Array(np).fill(0).map(() => v3.add(center, v3.scale([0, 0, 1], poleR(0)))));
    for (let i = 0; i < nt; i++) {
      const th = ((i + 0.5) / nt) * Math.PI;
      const row = [];
      for (let j = 0; j < np; j++) {
        const ph = ((j + 0.5) / np) * 2 * Math.PI - Math.PI;
        row.push(v3.add(center, v3.scale(dir(th, ph), r[i * np + j])));
      }
      rows.push(row);
    }
    rows.push(new Array(np).fill(0).map(() => v3.add(center, v3.scale([0, 0, -1], poleR(nt - 1)))));
    return rows;
  }

  // Occupancy grid of the reachable tip set (the envelope is an outer surface
  // and hides the unreachable interior near the base; this does not). Cells of
  // side `res` over a fixed box, filled from sampled tips, then a 3D closing
  // (dilate, erode, 6-neighbour) to remove sampling holes. Packed bits.
  const GRID_BOX = { min: [-1.7, -1.7, -1.2], max: [1.7, 1.7, 2.2] };
  function reachableGrid(opts) {
    const o = Object.assign({ samples: 300000, res: 0.06, seed: 5, scale: 1.0 }, opts || {});
    const n = [0, 1, 2].map((k) => Math.ceil((GRID_BOX.max[k] - GRID_BOX.min[k]) / o.res));
    const total = n[0] * n[1] * n[2];
    let g = new Uint8Array(total);
    const idx = (i, j, k) => (i * n[1] + j) * n[2] + k;
    const rng = CR.makeRng(o.seed);
    for (let s = 0; s < o.samples; s++) {
      const q = [];
      for (let m = 0; m < pcc.NSEG; m++) {
        const a = rng() * 2 * Math.PI, kk = Math.sqrt(rng()) * o.scale * pcc.KMAX[m];
        q.push(kk * Math.cos(a), kk * Math.sin(a));
      }
      const p = pcc.tip3(q);
      const i = Math.floor((p[0] - GRID_BOX.min[0]) / o.res), j = Math.floor((p[1] - GRID_BOX.min[1]) / o.res), k = Math.floor((p[2] - GRID_BOX.min[2]) / o.res);
      if (i >= 0 && j >= 0 && k >= 0 && i < n[0] && j < n[1] && k < n[2]) g[idx(i, j, k)] = 1;
    }
    const morph = (src, fill) => {
      const out = new Uint8Array(total);
      for (let i = 0; i < n[0]; i++) for (let j = 0; j < n[1]; j++) for (let k = 0; k < n[2]; k++) {
        const c = src[idx(i, j, k)];
        const nb = [
          i > 0 ? src[idx(i - 1, j, k)] : 0, i < n[0] - 1 ? src[idx(i + 1, j, k)] : 0,
          j > 0 ? src[idx(i, j - 1, k)] : 0, j < n[1] - 1 ? src[idx(i, j + 1, k)] : 0,
          k > 0 ? src[idx(i, j, k - 1)] : 0, k < n[2] - 1 ? src[idx(i, j, k + 1)] : 0,
        ];
        out[idx(i, j, k)] = fill ? (c || nb.some((v) => v) ? 1 : 0) : (c && nb.every((v) => v) ? 1 : 0);
      }
      return out;
    };
    g = morph(morph(g, true), false);
    return { min: GRID_BOX.min.slice(), res: o.res, n, bits: packBits(g) };
  }
  function packBits(u8) {
    const out = new Uint8Array(Math.ceil(u8.length / 8));
    for (let i = 0; i < u8.length; i++) if (u8[i]) out[i >> 3] |= 1 << (i & 7);
    return out;
  }
  function gridGet(grid, i, j, k) {
    const { n } = grid;
    if (i < 0 || j < 0 || k < 0 || i >= n[0] || j >= n[1] || k >= n[2]) return 0;
    const b = (i * n[1] + j) * n[2] + k;
    return (grid.bits[b >> 3] >> (b & 7)) & 1;
  }
  function gridContains(grid, p) {
    const i = Math.floor((p[0] - grid.min[0]) / grid.res), j = Math.floor((p[1] - grid.min[1]) / grid.res), k = Math.floor((p[2] - grid.min[2]) / grid.res);
    return !!gridGet(grid, i, j, k);
  }
  // base64 round trip for JSON embedding (Node and browser)
  function gridToJSON(grid) {
    const bytes = grid.bits;
    let bin = '';
    for (let i = 0; i < bytes.length; i++) bin += String.fromCharCode(bytes[i]);
    const b64 = typeof btoa === 'function' ? btoa(bin) : Buffer.from(bin, 'binary').toString('base64');
    return { min: grid.min, res: grid.res, n: grid.n, bits64: b64 };
  }
  function gridFromJSON(j) {
    const bin = typeof atob === 'function' ? atob(j.bits64) : Buffer.from(j.bits64, 'base64').toString('binary');
    const bits = new Uint8Array(bin.length);
    for (let i = 0; i < bin.length; i++) bits[i] = bin.charCodeAt(i);
    return { min: j.min, res: j.res, n: j.n, bits };
  }

  // Horizontal slice of the grid at height h (z), traced with marching squares:
  // returns line segments (pairs of world points) covering every contour,
  // holes included. Corner values are cell occupancies; segments pass through
  // edge midpoints, so the outline sits half a cell outside occupied centres.
  function gridSectionSegments(grid, h) {
    const { n, res, min } = grid;
    const k = Math.floor((h - min[2]) / res);
    if (k < 0 || k >= n[2]) return [];
    const cx = (i) => min[0] + (i + 0.5) * res, cy = (j) => min[1] + (j + 0.5) * res;
    const segs = [];
    // corners of the marching cell (i,j): a=(i,j) b=(i+1,j) c=(i+1,j+1) d=(i,j+1)
    for (let i = -1; i < n[0]; i++) for (let j = -1; j < n[1]; j++) {
      const a = gridGet(grid, i, j, k), b = gridGet(grid, i + 1, j, k), c = gridGet(grid, i + 1, j + 1, k), d = gridGet(grid, i, j + 1, k);
      const code = (a << 3) | (b << 2) | (c << 1) | d;
      if (code === 0 || code === 15) continue;
      const top = [cx(i) + res / 2, cy(j), h], right = [cx(i + 1), cy(j) + res / 2, h];
      const bottom = [cx(i) + res / 2, cy(j + 1), h], left = [cx(i), cy(j) + res / 2, h];
      const add = (p, q) => segs.push([p, q]);
      switch (code) {
        case 1: case 14: add(left, bottom); break;
        case 2: case 13: add(bottom, right); break;
        case 3: case 12: add(left, right); break;
        case 4: case 11: add(top, right); break;
        case 5: add(top, left); add(bottom, right); break;
        case 6: case 9: add(top, bottom); break;
        case 7: case 8: add(top, left); break;
        case 10: add(top, right); add(left, bottom); break;
        default: break;
      }
    }
    return segs;
  }
  // occupied cell centres on the slice (for tests and hints)
  function gridSliceCells(grid, h) {
    const { n, res, min } = grid;
    const k = Math.floor((h - min[2]) / res);
    const out = [];
    if (k < 0 || k >= n[2]) return out;
    for (let i = 0; i < n[0]; i++) for (let j = 0; j < n[1]; j++) {
      if (gridGet(grid, i, j, k)) out.push([min[0] + (i + 0.5) * res, min[1] + (j + 0.5) * res, h]);
    }
    return out;
  }

  // Is a world point inside the envelope (nearest-bin radial test)?
  function insideEnvelope(vol, p) {
    const { nt, np, r, center } = vol;
    const d = v3.sub(p, center), rr = v3.norm(d);
    if (rr < 1e-9) return true;
    const th = Math.acos(Math.max(-1, Math.min(1, d[2] / rr)));
    const ph = Math.atan2(d[1], d[0]);
    const i = Math.min(nt - 1, Math.floor((th / Math.PI) * nt));
    const j = ((Math.floor(((ph + Math.PI) / (2 * Math.PI)) * np) % np) + np) % np;
    return rr <= r[i * np + j];
  }

  CR.workspace = { reachableVolume, envelopeMesh, insideEnvelope,
    reachableGrid, gridContains, gridToJSON, gridFromJSON, gridSectionSegments, gridSliceCells };
})(typeof globalThis.CR === 'object' ? globalThis.CR : (globalThis.CR = {}));
