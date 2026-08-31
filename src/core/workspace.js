// Reachable workspace of the ideal model, in 3D.
//
// Sample the configuration space densely up to the curvature limits, keep the
// tip positions, and describe their outer boundary as a radial envelope
// around the cloud's centroid: for each direction (theta, phi) the largest
// tip radius seen in that angular bin, hole-filled, smoothed, and then scaled
// up by the smallest factor that keeps 99.9% of the samples inside. The
// result is a closed, smooth surface, the "dome" drawn in the inspector. It
// is an outer envelope: the reachable set is a solid inside it, and where the
// true set is concave the envelope is generous. Reachability of a specific
// target is decided by the planner's inverse kinematics, not by this surface.
(function (CR) {
  'use strict';
  const { pcc, v3 } = CR;

  function reachableVolume(opts) {
    const o = Object.assign({ samples: 200000, nt: 36, np: 72, seed: 3, keep: 0.999, smooth: 2, scale: 1.0 }, opts || {});
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
      const th = Math.acos(Math.max(-1, Math.min(1, d[1] / (r || 1e-9))));
      const ph = Math.atan2(d[2], d[0]);
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
    const scale = Math.max(1, ratios[Math.floor(o.keep * (o.samples - 1))]);
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

  // Direction for bin centres (theta from +y, phi about +y measured from +x
  // toward +z, matching binOf above).
  function dir(theta, phi) {
    const st = Math.sin(theta);
    return [st * Math.cos(phi), Math.cos(theta), st * Math.sin(phi)];
  }

  // Vertex grid of the envelope surface: rows theta (with pole rows added),
  // columns phi; each entry a world point.
  function envelopeMesh(vol) {
    const { nt, np, r, center } = vol;
    const rows = [];
    const poleR = (i) => { let s = 0; for (let j = 0; j < np; j++) s += r[i * np + j]; return s / np; };
    rows.push(new Array(np).fill(0).map(() => v3.add(center, v3.scale([0, 1, 0], poleR(0)))));
    for (let i = 0; i < nt; i++) {
      const th = ((i + 0.5) / nt) * Math.PI;
      const row = [];
      for (let j = 0; j < np; j++) {
        const ph = ((j + 0.5) / np) * 2 * Math.PI - Math.PI;
        row.push(v3.add(center, v3.scale(dir(th, ph), r[i * np + j])));
      }
      rows.push(row);
    }
    rows.push(new Array(np).fill(0).map(() => v3.add(center, v3.scale([0, -1, 0], poleR(nt - 1)))));
    return rows;
  }

  // Is a world point inside the envelope (nearest-bin radial test)?
  function insideEnvelope(vol, p) {
    const { nt, np, r, center } = vol;
    const d = v3.sub(p, center), rr = v3.norm(d);
    if (rr < 1e-9) return true;
    const th = Math.acos(Math.max(-1, Math.min(1, d[1] / rr)));
    const ph = Math.atan2(d[2], d[0]);
    const i = Math.min(nt - 1, Math.floor((th / Math.PI) * nt));
    const j = ((Math.floor(((ph + Math.PI) / (2 * Math.PI)) * np) % np) + np) % np;
    return rr <= r[i * np + j];
  }

  CR.workspace = { reachableVolume, envelopeMesh, insideEnvelope };
})(typeof globalThis.CR === 'object' ? globalThis.CR : (globalThis.CR = {}));
