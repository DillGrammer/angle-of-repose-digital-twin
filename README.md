# Angle of Repose Digital Twin

An interactive 3D digital twin of an angle-of-repose laboratory experiment. It simulates six quartz-sand and glass-bead conditions, runs three trials per condition, measures each pile, saves reproducible data, and compares the results with a UCF experiment and independently published experimental benchmarks.

![The 3D simulator running an automatic quartz-sand trial](assets/simulator-running.png)

## View the completed research

- Open [`index.html`](index.html) or [`01_MAIN_RESEARCH_REPORT.html`](01_MAIN_RESEARCH_REPORT.html) for the final graphs and comparison tables.
- The report clearly identifies the simulation, UCF experiment, and published benchmark series.
- All 18 completed trials and their browser replays are included.

## Start the experiment

1. Double-click `RUN_ANGLE_OF_REPOSE.command`.
2. On the first launch, wait for the simulator verification to finish.
3. Click **RUN CURRENT TRIAL**. The simulator performs the trial, calculates the angle, records the result, and creates its replay automatically.
4. Review the result or use the **REPLAYS** player.
5. Click **NEXT TRIAL** to prepare the next condition, then click **RUN CURRENT TRIAL** again.

The experiment contains three trials for each of six conditions:

1. Unsieved quartz sand
2. Quartz, 125–250 μm
3. Quartz, 250–500 μm
4. Quartz, greater than 500 μm
5. Glass beads, one cup
6. Glass beads, two cups

## Controls

- **Replays:** previous trial, play, pause, restart, next trial, timeline, and close.
- **Universal Speed:** controls both live trials and replay playback. Speed changes waiting time, not gravity, material settings, the physics timestep, settling decisions, or angle calculation.
- Trial results remain visible inside the simulator.

The simulator does not silently retry an invalid measurement. It stops and reports the problem so repeated attempts cannot introduce selection bias.

## Scientific safeguards

- UCF comparison measurements are not used to set or tune trial physics.
- Every one-cup condition uses the same normalized material volume; two cups uses twice that volume.
- Quartz conditions share one contact model, and both bead conditions share one contact model.
- Particles are released by gravity without artificial horizontal launch velocity.
- The measurement rejects isolated runout beads and requires agreement between perpendicular views.
- Every completed trial records its seed, settings, timing, result, and replay data.

## Included research files

- `00_SIMULATOR_DIAGNOSTIC_REPORT.html`
- `01_MAIN_RESEARCH_REPORT.html`
- `02_COMPLETE_TRIAL_DATA_AND_SETTINGS.csv`
- `03_VISUAL_REPLAYS_OPEN_THESE/`
- `04_RAW_REPLAY_DATA_FOR_REPRODUCIBILITY/`
- `05_SIMULATION_VALIDATION_AND_ACCURACY.html`

## Technology

- Python 3.13
- PyBullet for true 3D rigid-body physics
- VPython for the interactive laboratory and controls
- Static HTML/Canvas reports and replays

## Scientific scope

This is a research simulation, not a replacement for physical measurement. Angle of repose varies with particle shape, roughness, moisture, preparation, and measurement method. The report therefore treats published values as declared comparison benchmarks rather than universal ground truth.

## AI disclosure

Codex was used as a development assistant for implementation, debugging, interface refinement, validation logic, and report generation. The project owner directed the experimental design, reviewed the simulated trials, and made the final research and product decisions.
