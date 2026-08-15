# Recording script (60 to 90 seconds)

Open `demo.html#demo` and hit record; the scripted demo runs about 63 seconds on
its own. What to say, roughly one line per beat:

1. 0:00 "Two identical simulated continuum robots, same physics, same seed. Both controllers only see four marker pixels from this camera."
2. 0:03 "Left: classical visual servoing with the textbook constant-curvature Jacobian. Right: five small networks trained on the simulator's uglier physics."
3. 0:08 "Nominal conditions first. Both converge; the classical one is actually a touch faster here, and that is fine, its model is correct right now."
4. 0:21 "Payload on. The ideal model knows nothing about sag. Watch the orange error curve wander on each new target while the learned one stays tight."
5. 0:35 "Steady state tells the same story, the trial table below is keeping score."
6. 0:41 "Tendon drift on. A slow bias neither controller was told about. Both get worse; nobody is pretending otherwise."
7. 0:55 "Now a target outside the training envelope. The learned controller flags it, shows the ensemble disagreeing, and cuts its own gain instead of bluffing. The classical one just runs."
8. 1:03 "Everything here is one HTML file: simulator, both controllers, weights. Training and evaluation scripts are in the repo."
9. Close on the trials table for a beat so the numbers are readable.
10. Do not narrate over the OOD banner moment; let it sit on screen for two seconds, it is the point of the demo.
