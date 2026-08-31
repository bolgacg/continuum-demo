# Continuum robot visual servoing, two ways

A single-file browser demo: a simulated two-segment tendon-driven continuum robot
chases clicked targets using only marker pixels from one fixed camera. A classical
image-based visual servoing controller (Jacobian from the textbook constant
curvature model) runs side by side with a small learned ensemble that was trained
on the simulator's less polite physics and reports its own uncertainty.

I built this over a weekend as a study of the problems in the SDU Robotics job ad
on sensing and AI-based control of continuum surgical robots. It is a toy, and the
gap between the idealized model and the simulated truth is the point.

**Version 2 (31 August)** answers three questions a robotics researcher asked about
version 1: where the boundary came from, whether the geometry varies, and how the
inverse kinematics was solved. The answers, what changed, why version 1 did it the
other way, and what each change cost are in the "What changed in version 2" section
of the page. Version 1 is kept byte for byte as `v1.html` and embedded at the bottom
of the page. Summary:

1. **Boundary.** Version 1 drew the convex hull of the training targets and called
   it close to the reachable set. Version 2 draws the reachable set of the ideal
   model (200,000 sampled configurations, projected, boundary traced;
   `train/workspace.js`) as a solid outline and keeps the training envelope as a
   dashed one. They differ by about 35 px where the robot curls back on itself.
2. **Geometry.** Fixed: two segments of constant arc length, constant curvature per
   segment, four markers at fixed arc positions, no joints, no cross-section. The
   marker spacing on screen changes because one oblique camera foreshortens a body
   bending in 3D (3D chord varies under 4%, pixel spacing from about 1 px to about
   60 px). The decorative taper on the drawn tube is gone; a plan-view inset shows
   the robot from above at true scale, with the viewing ray of the clicked target.
3. **Inverse kinematics.** Version 1 solved none; both controllers were local
   image-space laws, and from some poses they commit to the wrong bending plane and
   stall at the curvature limit short of an edge target. Version 2 plans, then
   tracks: numerical IK on the ideal model (coarse global search over 8,000 sampled
   configurations, damped Gauss-Newton refinement, minimum change from the current
   configuration, limits respected), a configuration path at 60% of the rate limit,
   and each controller tracking the projected reference with feed-forward plus its
   own feedback law (`src/core/planner.js`). Same planner above both controllers.
4. **Bug.** The last step of version 1's scripted demo threw an exception that
   stopped the animation loop; the page went dead after the demo. Fixed in
   version 2; `v1.html` keeps the bug because it keeps everything.

## Quick start

Live at <https://bolgacg.github.io/continuum-demo/>, nothing to install.

Or open `demo.html` in a browser. That is all: no install, no network, no libraries.
Click either camera view to set a target, flip the payload and drift switches, or
press "Run scripted demo" for a guided 60 second tour (`demo.html#demo` starts it
automatically).

## How it works

**Simulator.** The idealized model is piecewise constant curvature (PCC), two
bending segments, each actuated by a triplet of tendons. The "truth" simulator is
PCC plus the effects the idealized model ignores: actuator lag and rate limits,
tendon backlash, gravity droop that grows with payload, inter-segment coupling,
and optional slow tendon drift. It is phenomenological on purpose; it is not rod
mechanics, it just needs to be wrong relative to the ideal model in realistic
directions.

**Sensing.** Both controllers see exactly four marker pixels (three shaft markers
and the tip) from one fixed pinhole camera, plus the clicked target pixel. Neither
reads simulator state. The two robots on screen are identical simulator instances
with the same random seed, so the comparison is fair.

**Classical controller.** Standard IBVS: integrate commands into a believed
configuration, differentiate the ideal PCC model there for a pixel Jacobian, and
do damped least squares descent on the image error. It is honestly implemented
and honestly degraded: under payload its Jacobian points the wrong way during
transients, and backlash plus drift leave it hunting.

**Learned controller.** Five small MLPs (10 inputs, two hidden layers, 8 outputs)
each regress a feedback gain matrix G from the image features; the command is
v = G e, where e is the pixel error. Structuring the output as a gain matrix
means commands vanish exactly at zero error, so network noise does not dither
the tip at the target. Training labels come from a privileged expert, damped
least squares on the truth simulator's own Jacobian, which exists only at
training time. Data: 300 episodes with randomized payload, drift, poses and
targets, exploration noise on the applied actions, about 47k samples. Training
runs offline in Node (`train/train.js`, plain JavaScript, no frameworks) and the
weights ship embedded in the page.

**Uncertainty.** Three signals, all set from data, none hand-tuned on the demo:
the clicked target is checked against the convex hull of training targets; the
feature vector is checked by nearest-neighbor distance against a stored training
subsample (threshold at the holdout 99.5th percentile); and the five networks'
command spread is monitored (same percentile rule). Any of the three flags the
state, the page says so, and the controller drops its authority to 0.3x instead
of extrapolating at full gain. The fan of arrows at the tip is the per-member
predicted motion, so disagreement is visible directly.

## The numbers

Headless closed-loop evaluation, 40 trials per condition, random poses and
targets, 6 s per trial. Settle: error under 6 px held for 0.8 s. Steady state:
mean error over the final second. `node train/eval.js` reproduces both tables
below (version 1 and version 2, interior and edge targets).

### Version 1 (interior targets, as published 17 August)

| condition | controller | settled | median settle | mean steady-state |
|---|---|---|---|---|
| nominal | classical | 40/40 | 1.20 s | 0.2 px |
| nominal | learned | 38/40 | 1.45 s | 1.2 px |
| payload | classical | 38/40 | 1.82 s | 1.6 px |
| payload | learned | 39/40 | 1.47 s | 0.6 px |
| drift | classical | 38/40 | 1.38 s | 3.2 px |
| drift | learned | 37/40 | 1.82 s | 3.0 px |
| payload + drift | classical | 37/40 | 1.93 s | 3.5 px |
| payload + drift | learned | 39/40 | 1.57 s | 2.8 px |

The pattern is the story. With the model right (nominal), classical wins, as it
should. With unmodeled load, the learned controller keeps its transient speed and
precision while classical slows down. Drift alone is roughly a wash: it is a
moving disturbance both are chasing through the same backlash. The learned
controller never flags OOD inside these conditions, and always flags targets
outside its training envelope.

One result I did not expect: the learned controller holds up at twice the
maximum payload it was trained on, because the gain it predicts keys on the
observed marker geometry rather than on the load itself.

### Version 2 against version 1

Edge targets are tips at 95 to 100% of the curvature limit, inside the frame;
interior targets are the version 1 protocol. Both controllers, both versions,
same seeds.

| targets | condition | controller | v1 settled | v1 median settle | v1 steady-state | v2 settled | v2 median settle | v2 steady-state |
|---|---|---|---|---|---|---|---|---|
| edge | nominal | classical | 33/40 | 1.33 s | 5.8 px | 40/40 | 1.83 s | 0.2 px |
| edge | nominal | learned | 31/40 | 1.50 s | 10.6 px | 39/40 | 1.58 s | 2.4 px |
| edge | payload | classical | 29/40 | 1.80 s | 7.9 px | 35/40 | 1.93 s | 3.3 px |
| edge | payload | learned | 30/40 | 1.47 s | 9.9 px | 34/40 | 1.67 s | 8.9 px |
| edge | drift | classical | 30/40 | 1.37 s | 9.6 px | 33/40 | 2.08 s | 3.9 px |
| edge | drift | learned | 29/40 | 1.63 s | 13.9 px | 31/40 | 1.83 s | 6.6 px |
| edge | payload + drift | classical | 30/40 | 2.00 s | 9.9 px | 31/40 | 1.98 s | 6.3 px |
| edge | payload + drift | learned | 29/40 | 1.45 s | 11.1 px | 31/40 | 1.98 s | 10.5 px |
| interior | nominal | classical | 40/40 | 1.20 s | 0.2 px | 40/40 | 1.62 s | 0.3 px |
| interior | nominal | learned | 38/40 | 1.42 s | 1.0 px | 40/40 | 1.65 s | 0.4 px |
| interior | payload | classical | 39/40 | 1.80 s | 1.4 px | 40/40 | 1.87 s | 1.2 px |
| interior | payload | learned | 40/40 | 1.47 s | 0.3 px | 40/40 | 1.67 s | 0.4 px |
| interior | drift | classical | 38/40 | 1.37 s | 3.2 px | 38/40 | 1.83 s | 3.4 px |
| interior | drift | learned | 37/40 | 1.72 s | 2.9 px | 38/40 | 1.72 s | 4.1 px |
| interior | payload + drift | classical | 37/40 | 1.87 s | 3.5 px | 37/40 | 2.47 s | 3.5 px |
| interior | payload + drift | learned | 39/40 | 1.52 s | 2.7 px | 37/40 | 1.98 s | 3.1 px |

The plan fixes the basin problem on edge targets and costs time on interior
ones, because it paces the motion at 60% of the rate limit so the feedback term
keeps authority. Under payload the plan is wrong (it is the ideal model's) and
the feedback term carries the difference; that shows as the smaller gains in
the payload rows. A null-space term instead of a plan (curvature minimization,
or a pull toward a good configuration) was tried first and moved the edge
numbers only slightly, because the failure is a basin, not a redundancy
resolution.

The version 1 table above differs slightly from the v1 rows here (38/40 vs
39/40 payload classical, for instance) because the target sampler was changed
to reject targets within 8 px of the frame edge for both versions.

## What this does not claim

- It is a simulation. No hardware, no clinical anything. The job ad is about
  surgical robots; this page makes no claim beyond that sentence.
- The truth model is invented, not identified from a physical robot. The claim is
  not "this is how continuum robots behave"; it is "when the plant deviates from
  the model in these plausible ways, this is what happens to each controller".
- The learned controller is behavior cloning of a simulator-privileged expert,
  which is the simplest thing that could work. No RL, no online adaptation.
- The uncertainty display is honest but simple: an envelope test and an ensemble
  spread. It flags extrapolation; it does not certify anything.
- The truth simulator shares the constant-curvature shape family with the ideal
  model: its curvature is biased by load and coupling, not redistributed along
  the arc. A rod model would be the next step, and the model mismatch here is
  milder than a real tendon-driven robot's.
- The planner is model-based and inherits the ideal model's errors. It solves a
  pixel target, which with one camera is a ray, by choosing the reachable
  configuration nearest the current one; depth is not observed, only assumed.

## Repository layout

```
demo.html / index.html   the deliverable (version 2); everything inlined, open from disk
v1.html                  version 1, frozen byte for byte, embedded at the bottom of the page
template.html            page shell that build.js fills in
build.js                 node build.js, writes demo.html + index.html
src/core/                simulator, controllers, planner, workspace outline (browser + Node)
src/ui/                  rendering (feeds, plan-view inset), charts, app wiring
train/train.js           data generation + ensemble training (writes weights.json)
train/workspace.js       reachable outline of the ideal model (writes workspace.json)
train/eval.js            closed-loop evaluation tables, v1 and v2
test/sanity.js           kinematics, truth-sim, planner and edge-target checks
```

To retrain from scratch: `node train/train.js` (about 4 minutes on a laptop),
`node train/workspace.js`, then `node build.js`. Everything is deterministic via
seeded RNGs.

Bolgaç Gülen, August 2026
