# Recording script (about 75 seconds)

Open `index.html#demo` and hit record; the scripted demo runs about 75 seconds on
its own. Roughly one line per beat:

1. 0:00 "Two identical simulated continuum robots in one scene, same physics, same seed. Both controllers see four markers triangulated from two fixed cameras; the inspector is for us."
2. 0:01 "Planner off first. A target at the edge of the workspace, from the rest pose. The classical law commits to the wrong bending plane and stalls."
3. 0:08 "Planner on, same target, same pose. Inverse kinematics on the model first, then the feedback loop tracks the plan."
4. 0:15 "Interior targets. The plan costs a little time here; the table keeps score."
5. 0:28 "Payload on, seen from the side sensor. The ideal model knows nothing about sag; watch the orange error wander while the blue one stays tighter."
6. 0:43 "Tendon drift on. A slow bias neither controller was told about. Both get worse; nobody is pretending otherwise."
7. 0:58 "A target outside the reachable envelope. The plan stops at the closest reachable point; the ensemble flags what it was never shown."
8. 1:08 "Everything here is one HTML file: simulator, cameras, planner, both controllers, weights. Training and evaluation scripts are in the repo."
9. Close on the trials table for a beat so the numbers are readable.
