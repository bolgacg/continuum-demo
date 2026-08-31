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

The simulation is 3D. Version 1 showed it through one 2D camera and gave the
controllers a pixel to chase, and everything a robotics researcher questioned
about it followed from that:

1. **The point was a ray.** With one camera an image point fixes two of the tip's
   three coordinates; depth along the viewing ray was never observed and never
   asked for, so the loop accepted any point on that line. The known limitation of
   a single point feature, and the wrong task for a robot that has to reach a
   point.
2. **The markers seemed to move along the robot.** Perspective: a segment bending
   toward the camera projected long, one bending away projected short, and the
   tube was drawn wider when nearer. In 3D the chord between neighbouring markers
   varies under 4%; in that image it ran from about 1 px to about 60 px. A
   decorative taper on the drawn tube made it worse. The geometry never changed:
   two segments of constant arc length (1.0 and 0.8), one constant-curvature arc
   per segment, four markers at fixed arc positions, no joints, no cross-section.
3. **Reaching looked more inconsistent than it was.** A pixel is reachable from
   several bending planes and a local law picks one greedily; from some poses it
   committed to the wrong plane, hit the curvature limit and stalled. In 2D that
   reads as random; in 3D it is a hidden degree of freedom.

Version 3 stands the robot upright on its base, axis up and gravity along
the axis (a straight upright robot does not sag; a leaning one sags in
proportion to its lean and the payload), senses with two fixed calibrated
cameras (side and top), triangulates the four markers, and gives both
controllers the same 3D estimate and a 3D target. A click in the orbiting inspector gives a ray; the height plane in the
side sensor view turns the ray back into a point, and the outline drawn at the
plane's height is the reachable set's cross-section there (from a sampled
occupancy grid, holes included: near the base the reachable set is a ring, not a
disc). The inspector has side, top and isometric presets, the tube a constant
radius, the markers two classes (filled at segment ends, hollow at midpoints),
a Tendons switch that strips the tube and draws the three tendons per segment
(radius drawn at 2.5x for legibility, stated on screen; colour from white when
slack to the robot's colour when fully shortened), a Flexibility slider that
scales the mechanism's curvature limits from x0.7 to x1.8 with the planner,
envelope and outline following (the crescent-shaped reach is the mechanism, not
the controller; the ensemble is trained across the whole range), a click on
either robot's name to hide or show it, and a vertical slider beside the side
view that sits 1:1 with the height plane. Two further changes and a fix:

4. **The boundary.** Version 1 drew the convex hull of the training targets in the
   image and called it close to the reachable set; it was not (about 35 px short
   where the robot curls back). Version 3 draws the reachable set in 3D: 200,000
   sampled configurations at the curvature limits, tips binned by direction
   around the cloud's centroid, largest radius per bin, holes filled, smoothed,
   scaled so 99.9% of samples are inside (`train/workspace.js`). It is an outer
   envelope and hides the unreachable interior near the base, so the cross-section
   shown on the target plane comes from a 0.06-unit occupancy grid of 300,000
   sampled tips instead (it agrees with the planner's reachability on 98.7% of
   random points). Reachability of a given target is decided by the planner's
   inverse kinematics, and the trial table says "beyond reach" when it fails.
5. **Inverse kinematics.** Version 1 solved none; both controllers were local
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
6. **Bug.** The last step of version 1's scripted demo threw an exception that
   stopped the animation loop. Fixed; `v1.html` keeps the bug because it keeps
   everything.

## How it works

**Simulator.** The idealized model is piecewise constant curvature (PCC), two
bending segments, each actuated by a triplet of tendons, standing upright with
gravity along the axis. The "truth" simulator is PCC plus the effects the
idealized model ignores: actuator lag and rate limits, tendon backlash, gravity
droop proportional to each segment's lean and to the payload, inter-segment
coupling, and optional slow tendon drift. It is phenomenological on purpose; it is not rod
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

**Learned controller.** Five small MLPs (15 inputs, hidden layers of 64 and 48, 12 outputs)
each regress a 4x3 feedback gain matrix G from the triangulated markers and the
error; the command is v = G e. Structuring the output as a gain matrix means
commands vanish exactly at zero error. Training labels come from a privileged
expert, damped least squares on the truth simulator's own Jacobian, which exists
only at training time. Data: episodes with randomized flexibility (x0.7 to
x1.8), payload, drift, poses and 3D targets, exploration noise on the applied
actions; 600 episodes, 93,600 samples. Training runs offline in Node
(`train/train.js`, plain JavaScript, no frameworks) and the weights ship embedded
in the page.

**Uncertainty.** Three signals. The target is tested against the population the
training targets were drawn from (tips at up to 90% of the curvature limits at
the highest flexibility trained, x1.8) by running the planner's inverse
kinematics with those limits: if no such configuration reaches it, the ensemble
was never shown it. That population is the faint blue cage on the page; the grey
cage is the current mechanism's reach. The feature
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

<!--EVAL_TABLES-->
### Interior targets

| condition | controller | direct settled | direct median settle | direct steady-state | planned settled | planned median settle | planned steady-state |
|---|---|---|---|---|---|---|---|
| nominal | classical | 36/40 | 1.48 s | 2.7 mm | 40/40 | 1.83 s | 0.5 mm |
| nominal | learned | 36/40 | 2.02 s | 1.7 mm | 40/40 | 2.00 s | 0.7 mm |
| payload | classical | 34/40 | 1.43 s | 2.9 mm | 39/40 | 1.73 s | 0.9 mm |
| payload | learned | 36/40 | 2.03 s | 1.7 mm | 38/40 | 1.78 s | 1.1 mm |
| drift | classical | 34/40 | 2.00 s | 5.1 mm | 39/40 | 2.13 s | 3.4 mm |
| drift | learned | 30/40 | 2.38 s | 4.0 mm | 37/40 | 2.25 s | 3.4 mm |
| payload + drift | classical | 28/40 | 2.78 s | 5.5 mm | 34/40 | 2.87 s | 4.1 mm |
| payload + drift | learned | 28/40 | 2.58 s | 4.2 mm | 33/40 | 2.92 s | 4.0 mm |

### Edge targets

| condition | controller | direct settled | direct median settle | direct steady-state | planned settled | planned median settle | planned steady-state |
|---|---|---|---|---|---|---|---|
| nominal | classical | 22/40 | 1.47 s | 24.5 mm | 40/40 | 2.02 s | 0.4 mm |
| nominal | learned | 35/40 | 1.87 s | 3.9 mm | 40/40 | 2.00 s | 0.5 mm |
| payload | classical | 24/40 | 1.62 s | 23.9 mm | 40/40 | 2.18 s | 1.0 mm |
| payload | learned | 34/40 | 1.83 s | 2.1 mm | 40/40 | 1.97 s | 0.6 mm |
| drift | classical | 20/40 | 1.93 s | 26.4 mm | 37/40 | 2.10 s | 3.0 mm |
| drift | learned | 29/40 | 2.12 s | 6.3 mm | 39/40 | 2.13 s | 3.0 mm |
| payload + drift | classical | 20/40 | 2.43 s | 25.4 mm | 38/40 | 2.85 s | 2.8 mm |
| payload + drift | learned | 33/40 | 2.02 s | 4.0 mm | 38/40 | 2.37 s | 2.7 mm |
<!--/EVAL_TABLES-->

The ensemble flagged nothing in these conditions: every target lies inside the
population it was trained on (tips at up to 90% of the limits, flexibility up
to x1.8).

What the tables say. Over the full reach, the direct classical law loses 18 of
40 edge targets and stalls about 25 mm short, committed to a bending plane from
which the target is not reachable within the limits, most often on targets
curled back below the base; the direct learned law loses 5. The plan takes both
to 40 of 40 and keeps that advantage under payload and drift, where it lifts the
classical law from 20 of 40 to 37 or 38, at a cost of about 0.3 s on interior
targets. One ensemble covers the whole flexibility range: a first attempt at
the original size fell behind the classical law under payload; doubling the data
and enlarging the network (93,600 samples, hidden layers of 64 and 48) brought
it back, ahead of the classical law under payload and on edge targets, level
under payload with drift, slightly behind under drift alone. A null-space term
(curvature minimization, or a pull toward a good configuration) was tried before
the planner and moved the numbers only slightly, because the failure is a basin,
not a redundancy resolution.

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
train/workspace.js       reachable envelope + occupancy grid of the ideal model (writes workspace.json)
train/eval.js            closed-loop evaluation tables (writes eval.json, rendered into the page)
train/readme-tables.js   rewrites the README tables from eval.json
test/sanity.js           kinematics, cameras, truth-sim, control, planner and envelope checks
```

To rebuild from scratch: `node train/train.js` (about 45 minutes on a laptop),
`node train/workspace.js`, `node train/eval.js` (about 6 minutes),
`node train/readme-tables.js`, then `node build.js`. Everything is deterministic via seeded RNGs.

Bolgaç Gülen, August 2026
