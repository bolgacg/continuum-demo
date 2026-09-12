# Variants: five readings of "three acts", the design record (10 Sep 2026)

Bo's order: the demo must be restructured into three acts based on the three answers given to
the researcher who questioned version 1 (Q1 the irregular boundary, Q2 whether the distance
between joints varies, Q3 how the inverse kinematics was solved). Five separate readings of
that order are built as five full pages under `variants/`, all on one chooser page, so he can
pick. Nothing here touches `index.html`, `template.html`, `build.js` or `src/ui/main.js`.

Approved plan and architecture: `~/.claude/plans/come-up-with-5-elegant-volcano.md`.
Content design below is the reference for `src/variants/v1.js` .. `v5.js` and their templates.

## Two facts that change the copy everywhere

1. The chord between neighbouring markers is NOT "under 4%": at flexibility x1.0 over random
   poses the first pair spans 0.4752 to 0.5000 units (4.96 percent); test/sanity.js asserts
   under 5 percent; at x1.8 it is about 16 percent (a chord shortens as its arc bends). Every
   variant computes the spread live per pair. The main page and README carry the wrong figure
   today; correcting them is Bo's call.
2. Any single camera collapses marker spacing: in v3's own CAM 01 the on-screen distance
   between the first two markers runs 0.1 to 57 px over random poses. The principled claim is
   "one view cannot show fixed geometry; two views and the 3D estimate can". Version 1's
   specific figures appear only where a variant replays v1 inside the v3 stack and recomputes.

Numbers a page may show, with sources: eval.json cells (40 trials); 200,000 envelope samples,
99.97 percent inside, margin 1.023, training population at x1.62 (workspace.json meta);
300,000 grid samples at 0.06 (workspace.js); grid-vs-IK agreement (sanity.js 98.7 percent on
300; recompute live); 12,000-entry IK table, 30 Gauss-Newton steps, 2 mm tolerance, path at 60
percent of the rate limit (planner.js); 93,600 samples, 600 episodes, five members 15-64-48-12,
holdout MSE 0.323, thresholds 3.696 and 4.858 (weights.json meta, train.js); arc lengths 1.0
and 0.8, limits 2.2 and 2.6 (pcc.js); truth coefficients incl. play half-width 0.035
(truth.js); v1 camera pose and taper (v1.html lines 266 and 471).

## Shared skeleton

Header: eyebrow; title as a question; one-job paragraph; three headline lines (one question,
one number, one comparison each; filled at build or on load): does a feedback law alone reach
the edge (22 of 40 direct vs 40 of 40 planned, eval.json); does anything move along the robot
(chord spread under 5 percent in 3D vs a factor above 100 on one screen, live); is the outline
the reachable set (agreement P percent on N points, live). Byline with a Restart-walkthrough
button. Primer of five statements: (1) a continuum robot bends along its length instead of
turning at joints; two segments of fixed length, like two circle arcs joined end to end; (2)
four markers seen by two fixed cameras, each triangulated to a 3D point, and that is all either
controller gets; (3) a controller turns the gap between tip and target into a change of bend;
orange uses the textbook model, blue learned from a simulator with lag, backlash, sag and
drift; (4) the reachable set is every point the tip can occupy within the bending limits; (5)
lengths are a display convention, the robot drawn as if 180 mm long. The domain drawn once
(scene.draw at a fixed pose + SVG labels: base, segments 1.0 and 0.8, markers m1..m4 filled
ends and hollow midpoints, CAM 01, CAM 02, plane, click ray). Glossary open under the primer:
constant curvature; marker; triangulation; visual servoing; Jacobian; inverse kinematics;
reachable set and cross-section; planner (plan then track); ensemble; extrapolation flag;
settle and steady state; play operator; edge and interior targets. "Three questions a
robotics researcher asked about version 1 (29 August)" quoted verbatim, then the 6 September
follow-up (geometric model or some other type), then Webster and Jones 2010 (the constant-
curvature arc), Chaumette and Hutchinson 2006 (the feedback law), Kuhnen 2003 (the play
operator). Attribution by name is Bo's decision (default: unattributed).

Each act: lede (why it matters), ONE control, one chart with its three lines (what you see,
how to read it, what to take), a verdict box with a plain opener and evidence, a handover
sentence. After act 3, the model card (five MLPs 15-64-48-12; predicts a 4 by 3 gain matrix,
v = G e; features the four triangulated markers and the tip error; fitted on 93,600 samples
from 600 episodes across x0.7 to x1.8 with payload in 60 percent and drift in half, labels
from a training-time expert reading the truth Jacobian; 10 percent kept aside to set the two
thresholds at the 99.5th percentile; scored on the 40-trial tables; edge direct 3.9 mm vs 24.5
mm steady, drift interior direct 30 vs 34 settled; verdict: ahead on edge targets and under
payload, level under payload with drift, slightly behind under drift alone; the flag fired on
0 percent of steps in every cell, so only the beyond-reach target tests it). Failure pane
before the coda (learned direct still loses 5 of 40 edge targets; payload+drift interior 34
and 33 of 40 near 4 mm; invented truth coefficients typed by the author; perfect cameras and
exact triangulation; the plan costs about 0.3 s median on interior targets (1.48 vs 1.83 s);
the outer cage over-states the set near the base, the outline on the plane does not; the old
"under 4%" corrected; v1 figures are probes unless recomputed; millimetres are a convention).
Coda: how built (train.js about 45 min, workspace.js, eval.js about 6 min, seeded), limits,
sources, terms, the frozen v1 frame. Footer: every number generated by the build or computed
in the browser, none typed. Spotlight walkthrough per DEMO-STANDARDS 6, restartable.

## V1. Chapters: one act per question, one scene per act
Title: How far can a two-segment continuum robot reach, and how does it get there?
One job: three questions, one chapter each, each answer computed in front of you.
Act 1 The reachable set and its cross-section. Scene: inspector iso + side feed, grey cage,
plane with outline, planner on, robots idle until a click. Control: plane height slider.
Chart: reachable area against height (grid slice area solid; cage slice grey; marker at the
current plane). Three lines: the area of the reachable cross-section at each height, from the
300,000-sample occupancy grid and from the outer cage's slice; where grey sits above solid the
cage claims space the tip cannot occupy, and near the base the solid line drops because the
set is a ring; the outline on the plane is the reachable set itself. Live: gridSliceCells
area x res^2; insideEnvelope on the same lattice; N random points on the plane classified by
grid and by solveIK. Verdict: "The outline is the reachable set's cross-section at this
height, and inverse kinematics decides each target." + agreement P percent on N points; the
trial table says beyond reach when the solve fails. Handover: the set was computed from a
model whose geometry never changes; the next chapter shows what does and does not change.
Act 2 Fixed arc lengths under a moving viewpoint. Scene: inspector only, enlarged, iso with
Side/Top chips, Tendons pinned on, one robot (classical hidden) on a slow planned sweep. Control:
Flexibility slider. Chart: against flexibility x0.7..x1.8, arc length (flat at 1.8), 3D chord
spread per pair over 500 random poses (few percent, rising), first pair's CAM 01 pixel spread
(hundreds, log axis). Live: pcc.setFlex sweep at load (worker or chunked), current value marked.
Verdict: "Arc length is constant to four decimals at every flexibility; the chord shortens by
under 5 percent at x1.0 because an arc's chord shortens as it bends." Handover: a fixed
geometry leaves four numbers to choose; the next chapter asks how, and what a feedback law
does alone.
Act 3 Inverse kinematics above the feedback law. Scene: both views, both robots, cages, plane,
Payload and Drift hidden, hook target from rest. Control: Planner switch (each flip re-runs the
same target). Chart: the tip-error chart with the switch as an event. Three lines: distance
from the triangulated tip to the target for both controllers with the 5 mm hairline; a line
that flattens above the band is a stall; direct, the classical law stalls short on this target
(live), planned both reach it. Verdict: "A feedback law alone loses edge targets; a plan on the
ideal model reaches them." + 22 of 40 vs 40 of 40, learned 35 vs 40; null-space term tried,
basin not redundancy. Handover to the model card and the free-play scene.
Coda adds a free-play scene with the full toolbar. Tour: 12 steps (title; schematic; their
questions; act 1 slider below the base; act 1 chart gap; act 2 slider to x1.8; act 2 Side/Top;
act 3 switch off; act 3 chart and table; model card; failure pane; free play).

## V2. The three consequences of one camera, replayed then corrected
Title: What does one camera hide about a continuum robot?
One job: version 1 showed a 3D robot through one 2D camera; three questions each landed on a
consequence; each act replays the consequence inside the current simulator, then the
correction under the same physics, so both can be measured.
Single control in every act: a two-position switch "as in version 1 | as now"; charts record
both states.
Act 1 A point target seen by one camera is a line. Scene: inspector iso with CAM 01's ray
through the target drawn, side feed, classical robot only (v1's ensemble has no honest v3
equivalent; said on the panel), cages off, plane on. v1 = a 2x4 pixel law on CAM 01 (central
differences of cam.project(pcc.tip3(q)); ibvs.dampedPinv handles m=2); now = the same law on
the triangulated 3D error. Chart: distance to the point (solid) and to CAM 01's ray (dashed),
mm. Verdict live: "With one camera the controller settled on the ray, D mm from the point;
with two it settled E mm from it." Handover: the same camera drew the boundary and the robot.
Act 2 Perspective, and a taper that was never modelled. One feed; the robot runs the same slow
planned sweep in both states. v1 = v1's oblique camera rebuilt with camera.makeCamera (pose
and focal from v1.html 471) plus a taper flag (R_BASE to R_TIP) in the variants' copy of
scene.js; now = CAM 01 with the constant-radius tube and the Top chip. Chart: first pair's
on-screen spacing (accent) vs 3D chord in mm (grey), switch as event. Verdict live: "The 3D
chord varied by X percent during this sweep; its on-screen spacing varied by a factor of Y
under the old camera and Z under the new one" (Z larger than Y is the point). Handover: four
numbers remain to be chosen.
Act 3 The bending plane a local law picks. Both views, both robots, cages, hook target. v1 =
planner off; now = planner on. Chart: angle between each controller's commanded segment-1
bending plane and the IK solution's plane, degrees over time. Verdict live: "Direct, the
classical law parks G degrees from the solution plane and stops H mm short; planned, both
reach the target" + eval.json. Failure pane adds: the replays are reconstructions on the v3
stack; v1 itself runs unchanged in the frame. Tour: 12 steps.

## V3. Sensing, model, control: the three layers in the reader's vocabulary
Title: Geometric model or learned model: what is this controller built on?
One job: the controller is built on a geometric model; the page is the three layers of that
build (what the cameras sense, what the model assumes, what the control law does), each
answering one question and stating what it is allowed to know.
Each act opens with a one-line "what this layer knows" box (sensing: pixels only; model:
configuration only; control: the point and the model).
Act 1 Sensing: two cameras, four markers, one point. Scene: inspector iso with both sensor
frustums, side feed, plane with outline, grey cage, robots idle until a click. Control: plane
height. Chart: reachable stretches along the click ray (gridContains sampled along the ray at
grid resolution; plane marker). Verdict: "The target is a point because two constraints fix it,
the click ray and the plane; the outline on the plane is the reachable set's slice there." +
agreement P percent on N points. Handover: sensing gives the point; whether the tip can reach
it is the model's business.
Act 2 Model: constant curvature, fixed lengths, four invented effects. Scene: Side preset,
Tendons pinned, both robots at a leaning pose, no cages, plane hidden. Control: Payload toggle
(ramps over about a second). Chart: sag against lean from truth.applyStatic at payload 0 and
1, current pose marked. Verdict: "It is a geometric model: two constant-curvature arcs of fixed
length, four numbers of configuration; the effects are additions with invented coefficients."
+ arc length 1.8000, chord spread live, the coefficient list from truth.js with a "typed by the
author" pill (the 8 Sep email's bullets live here). Handover: the control layer sees none of
the truth model.
Act 3 Control: inverse kinematics on the model, tracked by two laws. Both views, cages, hook
target. Control: Planner switch. Chart: edge targets settled out of 40, direct (light) vs
planned (solid), paired bars per condition and controller from eval.json at build. Verdict:
"Above both laws, the plan reaches every nominal edge target; alone, neither law does." Tour:
12 steps.

## V4. Three experiments the reader can rerun
Title: Do the three answers hold when you rerun them?
One job: each answer is a claim; each act is its test, run in the browser on the code that
runs the robots; press Run and the result is recomputed.
Each act: hypothesis box (prediction, baseline, caveat), Run button (the one control), result
panel, one chart, computed verdict.
Act 1 Audit of the boundary. Scene: inspector at Top preset over the plane and outline, side
feed, robots idle. Prediction: outline and IK agree on at least 95 percent of random points on
this plane. Baseline: the cage's slice. Caveat: 2 mm solver tolerance, 12,000-entry table,
6 mm cells. Run: 400 random points on the plane classified by gridContains, insideEnvelope,
solveIK. Chart: top-view scatter, agree/disagree colours, solver-reachable ringed. Verdict
live: outline agrees on P percent, cage on Q percent.
Act 2 Audit of the geometry. Scene: inspector iso, Tendons on, one robot cycling the sampled
poses as the audit runs. Prediction: arc length 1.8; each pair's 3D chord within 5 percent at
x1.0; CAM 01 spacing by more than a factor of ten. Run: 500 random configurations. Chart: range
chart on a log axis, one bar per quantity. Verdict live, two numbers per sentence.
Act 3 Audit of the planner. Scene: both views, both robots running the audit trials visibly
while CR.kit.runTrials runs the rest. Run: ten edge targets, first ten seeds of the protocol,
direct vs planned, both laws. Chart: per-trial final-second error dots with the 5 mm band.
Verdict live: a, b of 10 direct; c, d of 10 planned; the 40-trial numbers beside them.
Failure pane adds: ten trials check the protocol, they do not replace 40. Tour: 12 steps.

## V5. One scene, a three-stop spotlight walkthrough
Title: What decides whether a continuum robot's tip reaches a target?
One job: one scene, three stops; each stop pins one control and answers one question.
Layout: scene (inspector, side feed, readouts) sticky left on desktop, stops right; scene on
top on a phone. Entering a stop sets the scene state and greys the other controls ("pinned for
this stop; unpinned in free play"). The tour replaces the scripted demo.
Stop 1 Where the tip can go: cages, plane, planner on; pinned control plane height; verdict per
click ("your last target at z = ... mm was reachable / beyond reach; the outline agreed").
Stop 2 Nothing moves along the robot: Tendons on, cages off; pinned control the orbit; two live
readouts per pair (3D chord, spacing in the current view); verdict "since you started orbiting
the first pair's chord moved X percent and its on-screen spacing by a factor of Y".
Stop 3 Why a feedback law stalls: cages on, hook target; pinned control the Planner switch;
verdict live stall distance + eval.json. Departure from the standard: one shared chart across
stops; the repair, if wanted, is a small strip chart per stop. Tour: 14 steps.

## Distinctness and ranking
V1 answers with the current model only; V2 runs a reconstructed v1 condition in every act
behind one before/after switch; V3 is organised by system boundary with "what this layer
knows" and its act 2 is the truth model; V4 is run, not read; V5 is one sticky scene with
pinned states. Effort lowest first: V5, V1, V3, V2, V4. Risk of failing the standard lowest
first: V1, V3, V4, V2, V5. A later hysteresis chapter fits V3 (fourth layer item) and V4
(fourth audit, reusing the parked `hysteresis-act` branch) without restructuring.
