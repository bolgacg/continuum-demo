# Continuum robot visual servoing, two ways

A single-file browser demo: a simulated two-segment tendon-driven continuum robot
chases clicked targets using only marker pixels from one fixed camera. A classical
image-based visual servoing controller (Jacobian from the textbook constant
curvature model) runs side by side with a small learned ensemble that was trained
on the simulator's less polite physics and reports its own uncertainty.

I built this over a weekend as a study of the problems in the SDU Robotics job ad
on sensing and AI-based control of continuum surgical robots. It is a toy, and the
gap between the idealized model and the simulated truth is the point.

## Quick start

Live at <https://bolgacg.github.io/continuum-demo/> — nothing to install.

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
mean error over the final second. `node train/eval.js` reproduces the table.

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

## Repository layout

```
demo.html          the deliverable; everything inlined, open from disk
template.html      page shell that build.js fills in
build.js           node build.js -> writes demo.html
src/core/          simulator + controllers, shared by browser and Node
src/ui/            rendering, charts, app wiring
train/train.js     data generation + ensemble training (writes weights.json)
train/eval.js      closed-loop evaluation table
test/sanity.js     kinematics and closed-loop sanity checks
```

To retrain from scratch: `node train/train.js` (about 4 minutes on a laptop),
then `node build.js`. Everything is deterministic via seeded RNGs.

Bolgaç Gülen, August 2026
