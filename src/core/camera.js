// Pinhole cameras: fixed poses, perspective projection world -> pixels, the
// inverse ray for a pixel, and two-view triangulation.
//
// Version 3 sensing: two fixed, calibrated cameras (side and top) observe the
// four markers; each marker is triangulated to a 3D point and that estimate
// is what the controllers get. A third, orbiting camera is the human's
// inspector view; it is not a sensor.
(function (CR) {
  'use strict';
  const { v3, m3 } = CR;

  // Workspace center every camera looks at (on the robot's axis), orbit radius.
  // World frame: the robot stands on the origin with its axis along +z (up).
  const CENTER = [0, 0, 0.85];
  const DIST = 3.2;
  const EL_SIDE = Math.asin(0.2 / DIST); // side sensor sits 0.2 above the center

  function lookAtRotation(pos, target, up) {
    // Camera frame: +z forward (into the scene), +x right, +y down (image v).
    const fwd = v3.normalize(v3.sub(target, pos));
    const right = v3.normalize(v3.cross(fwd, up));
    const down = v3.cross(fwd, right);
    // Rows of R map world vectors into camera coords.
    return [
      right[0], right[1], right[2],
      down[0], down[1], down[2],
      fwd[0], fwd[1], fwd[2],
    ];
  }

  function makeCamera(opts) {
    const { pos, target, up, f, w, h } = opts;
    const R = lookAtRotation(pos, target, up);
    return {
      pos, target, up, f, w, h, R,
      // Returns [u, v, zCam] or null if the point is behind the camera.
      project(p) {
        const d = v3.sub(p, pos);
        const c = m3.mulVec(R, d);
        if (c[2] < 1e-4) return null;
        return [w / 2 + (f * c[0]) / c[2], h / 2 + (f * c[1]) / c[2], c[2]];
      },
      // Camera-space depth of a world point (for painter's sorting).
      depth(p) {
        const d = v3.sub(p, pos);
        return R[6] * d[0] + R[7] * d[1] + R[8] * d[2];
      },
      // Unit world direction of the viewing ray through pixel (u, v). A pixel
      // is this ray, not a point: one camera does not observe depth.
      rayDir(u, v) {
        const c = [(u - w / 2) / f, (v - h / 2) / f, 1];
        return v3.normalize([
          R[0] * c[0] + R[3] * c[1] + R[6] * c[2],
          R[1] * c[0] + R[4] * c[1] + R[7] * c[2],
          R[2] * c[0] + R[5] * c[1] + R[8] * c[2],
        ]);
      },
    };
  }

  // Orbit camera on a sphere around CENTER: azimuth about the axis (+z),
  // elevation from the horizontal plane. The up vector is the sphere's tangent
  // toward higher elevation, so it stays perpendicular to the view direction
  // everywhere, including straight down (no gimbal degeneracy).
  function orbitCamera(az, el, w, h, dist) {
    const d = dist || DIST;
    const ce = Math.cos(el), se = Math.sin(el);
    const dir = [ce * Math.cos(az), ce * Math.sin(az), se];
    const up = [-se * Math.cos(az), -se * Math.sin(az), ce];
    return makeCamera({ pos: v3.add(CENTER, v3.scale(dir, d)), target: CENTER, up, f: 0.62 * w, w, h });
  }

  // The two sensors. Side: from +x, nearly level with the workspace centre
  // (image-up is the axis, image-right is +y). Top: straight down the axis
  // from above (image-right +y, image-down +x, the side sensor's side).
  const PRESETS = {
    side: { az: 0, el: EL_SIDE },
    top: { az: 0, el: Math.PI / 2 - 1e-3 },
    iso: { az: 0.6, el: 0.5 },
  };
  function sideCamera(w, h) { return orbitCamera(PRESETS.side.az, PRESETS.side.el, w, h); }
  function topCamera(w, h) { return orbitCamera(PRESETS.top.az, PRESETS.top.el, w, h); }

  // Two-view triangulation: midpoint of the closest points of the two viewing
  // rays. Exact when the pixels are exact, which in this simulation they are;
  // the cameras add no noise, and the page says so.
  function triangulate(camA, pxA, camB, pxB) {
    const a = camA.pos, b = camB.pos;
    const dA = camA.rayDir(pxA[0], pxA[1]), dB = camB.rayDir(pxB[0], pxB[1]);
    const w0 = v3.sub(a, b);
    const B = v3.dot(dA, dB), D = v3.dot(dA, w0), E = v3.dot(dB, w0);
    const den = 1 - B * B;
    if (den < 1e-9) return null; // parallel rays
    const s = (B * E - D) / den;
    const t = (E - B * D) / den;
    const pA = v3.add(a, v3.scale(dA, s));
    const pB = v3.add(b, v3.scale(dB, t));
    return v3.scale(v3.add(pA, pB), 0.5);
  }

  // Sensing layer: project the markers into both sensors and triangulate.
  // Returns null if any marker is behind a camera.
  function senseMarkers(markers3, camSide, camTop) {
    const out = [];
    for (const m of markers3) {
      const pa = camSide.project(m), pb = camTop.project(m);
      if (!pa || !pb) return null;
      const p = triangulate(camSide, pa, camTop, pb);
      if (!p) return null;
      out.push(p);
    }
    return out;
  }

  // Ray-plane intersection with the height plane z = h (horizontal, since the
  // axis is up). Returns the point, or null when the ray is within ~3 degrees
  // of parallel to the plane.
  function rayPlaneZ(origin, dir, h) {
    if (Math.abs(dir[2]) < 0.05) return null;
    const t = (h - origin[2]) / dir[2];
    if (t <= 0) return null;
    return v3.add(origin, v3.scale(dir, t));
  }

  CR.camera = { CENTER, DIST, PRESETS, makeCamera, orbitCamera, sideCamera, topCamera,
    triangulate, senseMarkers, rayPlaneZ };
})(typeof globalThis.CR === 'object' ? globalThis.CR : (globalThis.CR = {}));
