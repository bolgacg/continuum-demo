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
      // Unit world direction of the viewing ray through pixel (u, v). A pixel
      // target is this ray, not a point: depth is not observed.
      rayDir(u, v) {
        const c = [(u - w / 2) / f, (v - h / 2) / f, 1];
        // R^T c (rows of R are the camera axes in world coordinates)
        const d = [
          R[0] * c[0] + R[3] * c[1] + R[6] * c[2],
          R[1] * c[0] + R[4] * c[1] + R[7] * c[2],
          R[2] * c[0] + R[5] * c[1] + R[8] * c[2],
        ];
        return v3.normalize(d);
      },
    };
  }

  // The demo's fixed camera: off to the side of the horizontal robot,
  // slightly above, looking at the middle of the workspace.
  function defaultCamera(w, h) {
    return makeCamera({
      pos: [2.9, 1.1, 0.5],
      target: [0.1, -0.05, 0.9],
      up: [0, 1, 0],
      f: 0.78 * w,
      w, h,
    });
  }

  CR.camera = { makeCamera, defaultCamera };
})(typeof globalThis.CR === 'object' ? globalThis.CR : (globalThis.CR = {}));
