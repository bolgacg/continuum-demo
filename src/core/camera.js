// Pinhole camera: fixed pose, perspective projection world -> pixels.
(function (CR) {
  'use strict';
  const { v3, m3 } = CR;

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
      pos, f, w, h,
      // Returns [u, v, zCam] or null if the point is behind the camera.
      project(p) {
        const d = v3.sub(p, pos);
        const c = m3.mulVec(R, d);
        if (c[2] < 1e-4) return null;
        return [w / 2 + (f * c[0]) / c[2], h / 2 + (f * c[1]) / c[2], c[2]];
      },
    };
  }

  // The demo's fixed camera: off to the side of the horizontal robot,
  // slightly above, looking at the middle of the workspace.
  function defaultCamera(w, h) {
    return makeCamera({
      pos: [2.15, 0.85, 0.35],
      target: [0.0, -0.1, 1.0],
      up: [0, 1, 0],
      f: 0.95 * w,
      w, h,
    });
  }

  CR.camera = { makeCamera, defaultCamera };
})(typeof globalThis.CR === 'object' ? globalThis.CR : (globalThis.CR = {}));
