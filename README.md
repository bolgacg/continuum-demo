# Continuum robot visual servoing, two ways

A single-file browser demo: a simulated two-segment tendon-driven continuum robot
chases 3D targets using only four markers seen by two fixed cameras. A classical
resolved-rate controller (Jacobian from the textbook constant curvature model)
runs against a small learned ensemble that was trained on the simulator's less
polite physics and reports its own uncertainty. Both sit under the same
plan-then-track layer, which can be switched off.

I built the first version over a weekend as a study of the problems in the SDU
Robotics job ad on sensing and AI-based control of continuum surgical robots. It
is a toy, and the gap between the idealized model and the simulated truth is the
point.

## Quick start

Live at <https://bolgacg.github.io/continuum-demo/>, nothing to install. Or open
`index.html` in a browser: no install, no network, no libraries.

Drag in the inspector to orbit, click to place a target on the height plane, move
the plane in the side sensor view (or with the slider), flip Payload / Tendon
drift / Plan first, or press "Run scripted demo" (`index.html#demo` starts it
automatically). Version 1, as published on 17 August, is kept byte for byte as
`v1.html` and embedded at the bottom of the page.

## Version 3, and why

Version 1 simulated the robot in 3D and showed it through one perspective camera,
and its controllers chased a clicked pixel. Three questions from a robotics
researcher who looked at it pointed at what that produced:

1. **Artifacts of the view.** Markers seemed to slide apart and together and the
   tube changed thickness. The geometry never changed: two segments of constant
   arc length (1.0 and 0.8 normalized units), one constant-curvature arc per
   segment, four markers at fixed arc positions, no joints, no cross-section. The
   3D chord between neighbouring markers varies under 4%; the oblique camera
   foreshortened it from about 1 px to about 60 px, and a decorative taper on the
   drawn tube made it worse. Version 3 has an orbiting inspector with side, top
   and isometric presets, a constant drawn radius, and the side sensor's own view
   next to it.
2. **A point target was a line target.** With one camera an image point fixes two
   of the tip's three coordinates; depth along the viewing ray was never observed
   and never asked for, so the loop accepted any point on the ray. That is the
   known limitation of a single point feature, and for a surgical robot it is the
   wrong task. It also produced the path dependence a reader could provoke (a
   pixel is reachable from several bending planes, and a local law picks one
   greedily). Version 3 senses with two fixed calibrated cameras (side and top),
   triangulates the four markers, and gives both controllers the same 3D estimate
   and a 3D target. A click in the inspector gives a ray; the height plane in the
   side view turns the ray back into a point. The cameras add no noise, so the
   triangulation is exact (a test asserts it below 1e-9).
3. **The boundary.** Version 1 drew the convex hull of the training targets in the
   image and called it close to the reachable set; it was not (about 35 px short
   where the robot curls back). Version 3 draws the reachable set in 3D: 200,000
   sampled configurations at the curvature limits, tips binned by direction
   around the cloud's centroid, largest radius per bin, holes filled, smoothed,
   scaled so 99.9% of samples are inside (`train/workspace.js`). It is an outer
   envelope; reachability of a given target is decided by the planner's inverse
   kinematics, and the trial table says "beyond reach" when it fails.
4. **Inverse kinematics.** Version 1 solved none; both controllers were local
   laws. Version 3 keeps local feedback laws on the 3D error (classical: damped
   pseudo-inverse of the ideal model's 3x4 Jacobian; learned: a regressed 4x3
   gain matrix from triangulated markers and the 3D error) and adds a planning
   layer above both (`src/core/planner.js`): numerical IK on the ideal model
   (coarse search over 12,000 sampled configurations, damped Gauss-Newton from
   the candidate nearest the current configuration, limits respected, closest
   reachable point if the target is out of reach), a configuration path at 60% of
   the rate limit, and each controller tracking that reference with feed-forward
   plus its own feedback. Whether the plan is needed with the 3D task is measured
   below, not assumed; the page has a "Plan first" switch.
5. **Bug.** The last step of version 1's scripted demo threw an exception that
   stopped the animation loop. Fixed; `v1.html` keeps the bug because it keeps
   everything.

## How it works

**Simulator.** The idealized model is piecewise constant curvature (PCC), two
bending segments, each actuated by a triplet of tendons. The "truth" simulator is
PCC plus the effects the idealized model ignores: actuator lag and rate limits,
tendon backlash, gravity droop that grows with payload, inter-segment coupling,
and optional slow tendon drift. It is phenomenological on purpose; it is not rod
mechanics, and its curvature is biased by load rather than redistributed along
the arc, so the model mismatch is milder than a real robot's.

**Sensing.** Two fixed pinhole cameras (CAM 01 side, CAM 02 top) see four markers
(three shaft markers and the tip). Each marker is triangulated from the two views;
that 3D estimate is all either controller gets. Neither reads simulator state. The
two robots on screen are identical simulator instances with the same random seed,
so the comparison is fair.

**Classical controller.** Integrate commands into a believed configuration,
differentiate the ideal PCC model there for the 3x4 Jacobian of tip position with
respect to curvature, and do damped least squares descent on the 3D tip error.
Honestly implemented and honestly degraded: under payload its Jacobian points the
wrong way during transients, and backlash plus drift leave it hunting.

**Learned controller.** Five small MLPs (15 inputs, two hidden layers, 12 outputs)
each regress a 4x3 feedback gain matrix G from the triangulated markers and the
error; the command is v = G e. Structuring the output as a gain matrix means
commands vanish exactly at zero error. Training labels come from a privileged
expert, damped least squares on the truth simulator's own Jacobian, which exists
only at training time. Data: 300 episodes with randomized payload, drift, poses
and 3D targets, exploration noise on the applied actions, 46,800 samples. Training
runs offline in Node (`train/train.js`, plain JavaScript, no frameworks) and the
weights ship embedded in the page.

**Uncertainty.** Three signals. The target is tested against the population the
training targets were drawn from (tips at up to 90% of the curvature limits) by
running the planner's inverse kinematics with the limits scaled to 90%: if no
such configuration reaches it, the ensemble was never shown it. The feature
vector is checked by nearest-neighbour distance against a stored training
subsample, threshold at the holdout 99.5th percentile. And the five networks'
command spread is monitored (same percentile rule). Any of the three flags the
state, the page says so, and the networks' authority drops to 0.3x; the plan's
feed-forward is not scaled, because it does not come from the networks. The
explicit target test exists because the feature-space distance cannot separate
absolute target position sharply; version 1 had the same test as a convex hull
in the image, and an intermediate build that dropped it failed to flag a target
well outside the workspace.

**Planner.** See point 4 above. The plan is only as good as the ideal model; under
payload the reference path is not where the real tip goes and the feedback term
carries the difference.

## The numbers

Headless closed-loop evaluation, 40 trials per cell, random poses and targets,
6 s per trial. Settle: tip error under 5 mm held for 0.8 s. Steady state: mean
error over the final second. Display convention: the model is dimensionless;
lengths are shown as if the robot were 180 mm long (1 unit = 100 mm). Edge
targets are tips at 95 to 100% of the curvature limit; interior targets up to
85%. `node train/eval.js` reproduces both tables.

### Interior targets

| condition | controller | direct settled | direct median settle | direct steady-state | planned settled | planned median settle | planned steady-state |
|---|---|---|---|---|---|---|---|
| nominal | classical | 36/40 | 1.48 s | 2.9 mm | 39/40 | 1.85 s | 0.5 mm |
| nominal | learned | 40/40 | 2.00 s | 0.9 mm | 40/40 | 1.83 s | 0.7 mm |
| payload | classical | 35/40 | 1.97 s | 5.3 mm | 38/40 | 2.15 s | 1.9 mm |
| payload | learned | 36/40 | 1.88 s | 4.6 mm | 39/40 | 1.80 s | 1.5 mm |
| drift | classical | 34/40 | 2.02 s | 5.2 mm | 39/40 | 2.10 s | 3.3 mm |
| drift | learned | 36/40 | 2.43 s | 3.3 mm | 39/40 | 2.13 s | 3.1 mm |
| payload + drift | classical | 31/40 | 2.43 s | 6.5 mm | 38/40 | 2.77 s | 3.5 mm |
| payload + drift | learned | 34/40 | 2.47 s | 4.0 mm | 38/40 | 2.12 s | 3.1 mm |

The ensemble flagged nothing on interior targets.

### Edge targets

| condition | controller | direct settled | direct median settle | direct steady-state | planned settled | planned median settle | planned steady-state |
|---|---|---|---|---|---|---|---|
| nominal | classical | 23/40 | 1.50 s | 24.8 mm | 40/40 | 2.00 s | 0.4 mm |
| nominal | learned | 22/40 | 1.73 s | 9.3 mm | 35/40 | 1.88 s | 1.6 mm |
| payload | classical | 22/40 | 2.20 s | 28.2 mm | 35/40 | 2.02 s | 2.2 mm |
| payload | learned | 16/40 | 2.10 s | 15.6 mm | 30/40 | 1.97 s | 3.0 mm |
| drift | classical | 18/40 | 1.58 s | 27.6 mm | 37/40 | 2.10 s | 3.2 mm |
| drift | learned | 21/40 | 2.33 s | 14.0 mm | 36/40 | 2.28 s | 3.3 mm |
| payload + drift | classical | 21/40 | 2.47 s | 28.8 mm | 35/40 | 2.12 s | 3.7 mm |
| payload + drift | learned | 16/40 | 2.37 s | 16.0 mm | 32/40 | 2.43 s | 4.1 mm |

The ensemble flagged about 30% of edge-target steps (the targets outside its
90%-of-limits training population) and ran at 0.3x authority there.

What the tables say. The direct classical law loses about half of the edge
targets and stalls tens of millimetres short (the wrong bending plane, at the
curvature limit); the plan takes it to 40 of 40 in the nominal case and lifts it
under every disturbance. The learned law flags the edge targets outside its
training population and slows down by design, which costs it settle rate there
(22 of 40 direct, 35 of 40 planned); when it does settle its steady state is
small. Interior targets are the everyday case: nothing flagged, the plan costs
the classical controller about 0.4 s, the learned controller is marginally
faster with it. Under payload and drift the plan is the ideal model's plan and
the feedback term carries the difference; the gains shrink but do not vanish. A
null-space term (curvature minimization, or a pull toward a good configuration)
was tried before the planner and moved the numbers only slightly, because the
failure is a basin, not a redundancy resolution. Earlier in this build, before
the explicit target test was restored, the learned law reached 34 of 40 edge
targets directly at full authority; the warning is what costs it, and it is kept
because a controller that does not know it is extrapolating is the worse
failure.

## What this does not claim

- It is a simulation. No hardware, no clinical anything. The job ad is about
  surgical robots; this page makes no claim beyond that sentence.
- The truth model is invented, not identified from a physical robot, and it stays
  in the constant-curvature shape family.
- The cameras are perfect: no noise, no calibration error, exact triangulation.
- The learned controller is behavior cloning of a simulator-privileged expert,
  the simplest thing that could work. No RL, no online adaptation.
- The uncertainty display flags extrapolation; it does not certify anything.
- The planner is model-based and inherits the ideal model's errors.
- Millimetres are a display convention, not a claim about any device.

## Repository layout

```
index.html / demo.html   the deliverable (version 3); everything inlined, open from disk
v1.html                  version 1, frozen byte for byte, embedded at the bottom of the page
template.html            page shell that build.js fills in
build.js                 node build.js, writes demo.html + index.html
src/core/                simulator, cameras + triangulation, controllers, planner, envelope
src/ui/                  3D scene rendering, charts, app wiring
train/train.js           data generation + ensemble training (writes weights.json)
train/workspace.js       reachable envelope of the ideal model (writes workspace.json)
train/eval.js            closed-loop evaluation tables (writes eval.json, rendered into the page)
test/sanity.js           kinematics, cameras, truth-sim, control, planner and envelope checks
```

To rebuild from scratch: `node train/train.js` (about 5 minutes on a laptop),
`node train/workspace.js`, `node train/eval.js` (about 5 minutes), then
`node build.js`. Everything is deterministic via seeded RNGs.

Bolgaç Gülen, August 2026
