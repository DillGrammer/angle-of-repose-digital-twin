import os

HEADLESS_MODE = os.environ.get("AOR_HEADLESS_PRECHECK", "0") == "1"

if not HEADLESS_MODE:
    from vpython import *
else:
    class _HeadlessVector:
        def __init__(self, x=0.0, y=0.0, z=0.0):
            self.x, self.y, self.z = x, y, z

    class _HeadlessObject:
        def __init__(self, **kwargs):
            self.visible = True
            self.pos = kwargs.get("pos", _HeadlessVector())
            self.axis = kwargs.get("axis", _HeadlessVector())
            self.color = kwargs.get("color", _HeadlessVector())
            self.opacity = kwargs.get("opacity", 1.0)
            self.text = kwargs.get("text", "")
            self.value = kwargs.get("value", 0)
            self.min = kwargs.get("min", 0)
            self.max = kwargs.get("max", 1)
            self.choices = kwargs.get("choices", [])
            self.selected = kwargs.get("selected", "")

        def append_to_caption(self, _text):
            return None

    class _HeadlessColor:
        cyan = _HeadlessVector(0, 1, 1)
        white = _HeadlessVector(1, 1, 1)
        red = _HeadlessVector(1, 0, 0)
        green = _HeadlessVector(0, 1, 0)
        yellow = _HeadlessVector(1, 1, 0)

    def vector(x=0.0, y=0.0, z=0.0):
        return _HeadlessVector(x, y, z)

    def _headless_shape(**kwargs):
        return _HeadlessObject(**kwargs)

    box = cylinder = cone = sphere = label = curve = button = slider = menu = wtext = _headless_shape
    color = _HeadlessColor()
    scene = _HeadlessObject()

    def rate(_fps):
        return None

try:
    import pybullet as p
except ImportError as exc:
    raise SystemExit(
        "PyBullet is not installed in this virtual environment. "
        "On Apple Silicon/Python 3.13 use: pip install pybullet-arm64"
    ) from exc

import atexit
import csv
import hashlib
import json
import math
import random
import statistics
import time
from datetime import datetime

# ==========================================================
# ANGLE OF REPOSE RESEARCH SIMULATOR — 3D PYBULLET VERSION
#
# Built from the most recent 2D pre-validation simulator.
#
# Major architecture change:
#   - PyBullet performs the particle physics in true 3D.
#   - VPython remains the visual laboratory / interface.
#   - The existing research workflow, reports, replay concept,
#     robust angle measurement, calibration/held-out split,
#     and automatic pre-validation are retained.
#
# IMPORTANT SCIENTIFIC RULE:
# Physical/UCF values are never used to directly set a trial's
# simulated angle. A designated calibration subset may influence
# bounded shared material parameters during pre-validation.
# Held-out conditions are not allowed to change the frozen model.
# ==========================================================

MODEL_VERSION = "AOR_3D_SCIENCE_GATED_DIGITAL_TWIN_2026_09_02_REVIEW_CONTROLS"
PHYSICS_ENGINE_NAME = "PyBullet 3D rigid-body physics"

# ==========================================================
# FILES
# ==========================================================

PREVALIDATION_FILE = "00_SIMULATOR_DIAGNOSTIC_REPORT.html"
# This small internal file makes repeat launches faster. It is hidden in
# Finder because it is not a research result and users do not need to manage it.
PREVALIDATION_CACHE_FILE = ".aor_diagnostic_state.json"
REPORT_FILE = "01_MAIN_RESEARCH_REPORT.html"
METADATA_FILE = "02_COMPLETE_TRIAL_DATA_AND_SETTINGS.csv"
VISUAL_REPLAY_FOLDER = "03_VISUAL_REPLAYS_OPEN_THESE"
RAW_REPLAY_FOLDER = "04_RAW_REPLAY_DATA_FOR_REPRODUCIBILITY"
VALIDATION_FILE = "05_SIMULATION_VALIDATION_AND_ACCURACY.html"

os.makedirs(VISUAL_REPLAY_FOLDER, exist_ok=True)
os.makedirs(RAW_REPLAY_FOLDER, exist_ok=True)

# ==========================================================
# SCENE / VPYTHON VISUAL LAB
# ==========================================================

scene.width = 1100
scene.height = 700
scene.background = vector(0.10, 0.10, 0.12)
scene.center = vector(0, 4, 0)
scene.range = 7
scene.userspin = True
scene.userzoom = True
scene.title = """
<b>ANGLE OF REPOSE RESEARCH SIMULATOR — 3D PHYSICS</b><br>
"""

metal = vector(0.65, 0.65, 0.65)
dark = vector(0.12, 0.12, 0.12)
clear_color = vector(0.75, 0.85, 0.90)
sand_color = vector(0.84, 0.68, 0.36)
glass_color = vector(0.65, 0.78, 0.95)

# Apparatus visuals. PyBullet uses its own invisible collision geometry.
box(pos=vector(0, 0, 0), size=vector(7, 0.35, 5), color=metal)

for post_pos in [
    vector(-3, 4, -2),
    vector(3, 4, -2),
    vector(-3, 4, 2),
    vector(3, 4, 2),
]:
    box(pos=post_pos, size=vector(0.35, 8, 0.35), color=metal)

box(pos=vector(0, 7.7, 0), size=vector(6.5, 0.35, 4.5), color=dark)
box(
    pos=vector(0, 5.25, 0),
    size=vector(5, 0.25, 4),
    color=vector(0.25, 0.25, 0.25),
)

upper_tube = cylinder(
    pos=vector(0, 5.28, 0),
    axis=vector(0, 2.25, 0),
    radius=1.02,
    color=clear_color,
    opacity=0.18,
)

material_column = cylinder(
    pos=vector(0, 5.33, 0),
    axis=vector(0, 1.85, 0),
    radius=0.88,
    color=sand_color,
    opacity=0.50,
)

release_plate = box(
    pos=vector(0, 5.10, 0),
    size=vector(3.2, 0.16, 3.2),
    color=vector(0.70, 0.70, 0.75),
)

cone(
    pos=vector(0, 4.93, 0),
    axis=vector(0, -0.75, 0),
    radius=0.95,
    color=vector(0.90, 0.90, 0.90),
    opacity=0.30,
)

cylinder(
    pos=vector(0, 4.18, 0),
    axis=vector(0, -0.35, 0),
    radius=0.22,
    color=vector(0.85, 0.85, 0.85),
    opacity=0.40,
)

cylinder(
    pos=vector(0, 0.55, 0),
    axis=vector(0, 3.05, 0),
    radius=1.72,
    color=clear_color,
    opacity=0.10,
)

cylinder(
    pos=vector(0, 0.62, 0),
    axis=vector(0, 0.18, 0),
    radius=1.62,
    color=vector(0.52, 0.52, 0.52),
)

next_step_label = label(
    pos=vector(0, 9.35, 0),
    text="RUNNING SIMULATOR DIAGNOSTICS",
    height=22,
    box=True,
    opacity=0.70,
    color=color.cyan,
)

trial_info_label = label(
    pos=vector(0, 8.75, 0),
    text="",
    height=14,
    box=False,
    color=color.white,
)

result_info_label = label(
    pos=vector(0, 8.28, 0),
    text="",
    height=13,
    box=False,
    color=color.white,
)

# ==========================================================
# 3D PHYSICS CONSTANTS
# PyBullet convention: X/Y horizontal, Z vertical.
# VPython mapping:      X -> x, Z -> y, Y -> z.
# ==========================================================

GRAVITY = 9.81
PHYSICS_DT = 1.0 / 240.0
VISUAL_FPS = 30
PHYSICS_STEPS_PER_VISUAL_FRAME = 12
SIMULATION_SPEEDS = {
    "Normal": 1,
    "Fast": 2,
    "Maximum": 4,
}

FLOOR_Z = 0.80
COLLECTOR_RADIUS = 1.55
COLLECTOR_WALL_HEIGHT = 3.10
COLLECTOR_WALL_SEGMENTS = 48
COLLECTOR_WALL_THICKNESS = 0.075

OUTLET_Z = 4.0
OUTLET_RADIUS = 0.20
RELEASE_INTERVAL = 0.013

# ----------------------------------------------------------
# CONTROLLED GRAVITY FEED
#
# Previous diagnostic runs showed particles spraying sideways from
# the outlet instead of simply dropping. The cause was a combination
# of:
#   1) artificial horizontal launch velocity, and
#   2) new bodies being inserted before the previous bodies had cleared
#      the outlet, creating overlapping rigid bodies and explosive
#      collision impulses.
#
# This build fixes the RELEASE MECHANISM rather than tuning the final
# angle. Particles begin with ZERO horizontal velocity and are inserted
# only when a non-overlapping outlet position is available.
# ----------------------------------------------------------

CONTROLLED_GRAVITY_FEED = True
SPAWN_CLEARANCE_FACTOR = 1.04
SPAWN_LOOKAHEAD = 36
SPAWN_Z_OFFSET_FACTOR = 1.20
OUTLET_EXIT_SPEED = 0.0
MAX_SAFE_SPAWNS_PER_PHYSICS_STEP = 2

# ----------------------------------------------------------
# 3D COARSE-GRAIN RESOLUTION
#
# V1/V2 reused the old ~180-particle 2D count in a true 3D volume.
# That can produce an under-resolved, very shallow bulk pile.
#
# V3 doubles the number of physics particles while shrinking them
# so total solid volume is unchanged:
#
#     N_new * r_new^3 = N_reference * r_reference^3
#
# This is a numerical-resolution refinement, NOT extra material.
# ----------------------------------------------------------

# ----------------------------------------------------------
# TRUE 3D COARSE-GRAIN BULK NORMALIZATION
#
# IMPORTANT V4 CHANGE:
#
# The old 2D prototype's "180 particles" was a SIDE CROSS-SECTION.
# V1-V3 incorrectly treated that same count/volume as if it represented
# an entire 3D cup of material. That made the 3D heap far too sparse.
#
# V4 stops preserving that invalid 2D->3D volume mapping.
# Instead, the DEM uses a declared NORMALIZED bulk amount:
#
#   1 cup  = 0.25 simulation solid-volume units
#   2 cups = 0.50 simulation solid-volume units
#
# This is coarse-graining, not a claim that these units are literal mL.
# It is independent of the known UCF repose angles.
# Every one-cup condition has the SAME total solid volume.
# ----------------------------------------------------------

QUARTZ_PARTICLES_PER_CUP = 600
GLASS_PARTICLES_PER_CUP = 500
NORMALIZED_SOLID_VOLUME_PER_CUP = 0.25
MIN_PILE_LAYERS_FOR_VALID_MEASUREMENT = 3.0

# Recorded replay is intentionally lower frequency than display.
REPLAY_FPS = 10
REPLAY_RECORD_EVERY_VISUAL_FRAME = 3
DISPLAY_UPDATE_EVERY_VISUAL_FRAME = 3

# Settling checks are based on PyBullet body velocities rather than
# the old custom "sleeping" rules.
SETTLE_MIN_TIME = 1.20
SETTLE_MAX_TIME_QUARTZ = 6.0
SETTLE_MAX_TIME_GLASS = 8.0
SETTLE_LINEAR_SPEED = 0.040
SETTLE_ANGULAR_SPEED = 0.80
SETTLE_REQUIRED_STABLE_TIME = 0.65

# 3D angle measurement uses two orthogonal full-depth side silhouettes.
# This is more appropriate for a 3D conical pile than sampling only a
# thin center slab, which can look artificially flat when particles
# happen to sit outside the chosen slice.
MEASUREMENT_PROJECTION_AXES = ("x", "y")

# ==========================================================
# PRE-VALIDATION DESIGN
# ==========================================================

CALIBRATION_QUARTZ_CONDITIONS = [1, 2]
CALIBRATION_GLASS_CONDITIONS = [4]
HELD_OUT_VALIDATION_CONDITIONS = [0, 3, 5]

CALIBRATION_SEED = 104729
CONFIRMATION_SEED = 155921
HELD_OUT_SEEDS = [196613, 221713]

# Deterministic bounded search. No random parameter hunting.
QUARTZ_SCALE_CANDIDATES = [0.80, 1.00, 1.25]
GLASS_SCALE_CANDIDATES = [0.80, 1.00, 1.25]
EXTREME_LOW_SCALE = 0.65
EXTREME_HIGH_SCALE = 1.80

CALIBRATION_ERROR_LIMIT_PERCENT = 15.0
HELD_OUT_CONDITION_ERROR_LIMIT_PERCENT = 20.0
HELD_OUT_MEAN_ERROR_LIMIT_PERCENT = 15.0
MAX_PILOT_SD_DEG = 3.5
MAX_PILOT_RANGE_DEG = 6.0
MAX_ORTHOGONAL_PROFILE_DIFFERENCE_DEG = 10.0

# Scientific-integrity pre-check. These gates test mechanics and geometry,
# not agreement with the UCF answers. Reference results remain blinded until
# the completed research trials are reported.
PRECHECK_SEED = 7331
PRECHECK_QUARTZ_ANGLE_RANGE_DEG = (25.0, 40.0)
PRECHECK_GLASS_ANGLE_RANGE_DEG = (18.0, 30.0)
PRECHECK_MAX_PROFILE_DIFFERENCE_DEG = 16.0
PRECHECK_MAX_QUARTZ_SPREAD_DEG = 8.0
PRECHECK_MAX_GLASS_AMOUNT_EFFECT_DEG = 6.0

quartz_friction_scale = 1.25
glass_friction_scale = 1.0
model_prevalidation_passed = False
model_frozen = False
DIAGNOSTIC_MODE = True

# ==========================================================
# EXPERIMENT CONDITIONS
#
# Shared quartz properties are deliberate: particle size / size
# distribution changes by condition, but the model is not given an
# independently tuned answer-specific material model. Quartz angularity
# is represented structurally by angular coarse-grain collision shapes.
# Glass 1-cup and 2-cup use exactly the same material properties.
# The 2-cup condition has twice the 3D material count.
# ==========================================================

QUARTZ_SHARED = {
    # ------------------------------------------------------
    # FINE-GRAIN DRY QUARTZ DEM PROXY
    #
    # Dry quartz sand is treated as NON-COHESIVE.
    #
    # The old coarse model looked like bouncing / locked rocks because each
    # representative body was very large and rolling resistance was too high.
    #
    # V7 uses many smaller representative grains and a moderate contact model:
    #   - sliding resistance does most of the pile support,
    #   - modest rolling resistance represents irregular grain shape,
    #   - almost no restitution prevents rock-like bouncing,
    #   - no cohesion or per-grain friction anchoring is used.
    #
    # Values are shared by every quartz condition.
    # ------------------------------------------------------
    "lateral_friction": 0.72,
    "floor_friction": 0.76,
    "rolling_friction": 0.018,
    "spinning_friction": 0.005,
    "restitution": 0.015,
    "linear_damping": 0.030,
    "angular_damping": 0.040,
}

GLASS_SHARED = {
    # Manufactured beads remain spheres. These are PER-BODY Bullet
    # friction values; Bullet forms the contact value from both bodies.
    "lateral_friction": 0.70,
    "floor_friction": 0.74,
    "rolling_friction": 0.0100,
    "spinning_friction": 0.0015,
    "restitution": 0.010,
    "linear_damping": 0.040,
    "angular_damping": 0.030,
}

CONDITIONS = [
    {
        "name": "Unsieved Quartz Sand",
        "material_class": "quartz",
        "base_radius": 0.055,
        "radius_variation": 0.18,
        "size_distribution_model": "broad_log_volume_normalized",
        "size_distribution_min_multiplier": 0.65,
        "size_distribution_max_multiplier": 1.35,
        "size_distribution_assumption": (
            "Generic broad unsieved distribution because measured sieve proportions "
            "were not supplied. Representative coarse-grain spread is limited "
            "to avoid artificial rock-sized outliers; total 3D solid volume is normalized."
        ),
        "particle_count": 180,
        "amount_cups": 1,
        "physical_trials": [27.60, 32.27, 29.38],
    },
    {
        "name": "Quartz 125-250 um",
        "material_class": "quartz",
        "base_radius": 0.050,
        "radius_variation": 0.07,
        "size_distribution_model": "narrow_uniform",
        "size_distribution_assumption": "Sieved size class represented as a narrow 3D distribution",
        "particle_count": 180,
        "amount_cups": 1,
        "physical_trials": [29.09, 30.21, 34.76],
    },
    {
        "name": "Quartz 250-500 um",
        "material_class": "quartz",
        "base_radius": 0.057,
        "radius_variation": 0.08,
        "size_distribution_model": "narrow_uniform",
        "size_distribution_assumption": "Sieved size class represented as a narrow 3D distribution",
        "particle_count": 180,
        "amount_cups": 1,
        "physical_trials": [29.31, 33.98, 25.74],
    },
    {
        "name": "Quartz >500 um",
        "material_class": "quartz",
        "base_radius": 0.064,
        "radius_variation": 0.10,
        "size_distribution_model": "narrow_uniform",
        "size_distribution_assumption": "Sieved size class represented as a narrow 3D distribution",
        "particle_count": 180,
        "amount_cups": 1,
        "physical_trials": [33.07, 30.60, 28.83],
    },
    {
        "name": "Glass Beads - 1 Cup",
        "material_class": "glass",
        "base_radius": 0.058,
        "radius_variation": 0.025,
        "size_distribution_model": "narrow_uniform",
        "size_distribution_assumption": "Manufactured spherical beads represented as a narrow 3D distribution",
        "particle_count": 180,
        "amount_cups": 1,
        "physical_trials": [22.07, 18.79, 16.42],
    },
    {
        "name": "Glass Beads - 2 Cups",
        "material_class": "glass",
        "base_radius": 0.058,
        "radius_variation": 0.025,
        "size_distribution_model": "narrow_uniform",
        "size_distribution_assumption": (
            "Same bead material as 1 cup; the second cup is represented by twice the "
            "3D particle volume/count rather than forcing extra particles into a 2D plane"
        ),
        "particle_count": 360,
        "amount_cups": 2,
        "physical_trials": [18.76, 21.06, 18.82],
    },
]

TRIALS_PER_CONDITION = 3
TOTAL_EXPERIMENT_TRIALS = len(CONDITIONS) * TRIALS_PER_CONDITION

# ==========================================================
# STATES / SESSION
# ==========================================================

STATE_PREVALIDATING = "PREVALIDATING"
STATE_PREVALIDATION_FAILED = "PREVALIDATION_FAILED"
STATE_READY = "READY"
STATE_RUNNING = "RUNNING"
STATE_SETTLING = "SETTLING"
STATE_READY_TO_MEASURE = "READY_TO_MEASURE"
STATE_SAVED = "SAVED"
STATE_REPLAYING = "REPLAYING"
STATE_COMPLETE = "COMPLETE"

state = STATE_READY
condition_index = 0
trial_number = 1
trial_seed = None

particles = []
measurement_objects = []
replay_objects = []
replay_frames = []
session_results = []
current_trial_settled = False
current_trial_simulated_seconds = 0.0
current_trial_wall_seconds = 0.0
abort_current_run = False
full_experiment_active = False
simulation_speed_name = "Normal"

# ==========================================================
# PYBULLET CONNECTION
# ==========================================================

physics_client = p.connect(p.DIRECT)
if physics_client < 0:
    raise RuntimeError("PyBullet DIRECT physics client could not be created.")


def _disconnect_pybullet():
    try:
        if p.isConnected(physics_client):
            p.disconnect(physics_client)
    except Exception:
        pass


atexit.register(_disconnect_pybullet)

# ==========================================================
# SMALL HELPERS
# ==========================================================


def mean(values):
    return sum(values) / len(values) if values else None


def current_condition():
    return CONDITIONS[condition_index]


def physical_average(condition):
    return mean(condition["physical_trials"])


def current_overall_trial_number():
    return condition_index * TRIALS_PER_CONDITION + trial_number


def is_final_trial():
    return (
        condition_index == len(CONDITIONS) - 1
        and trial_number == TRIALS_PER_CONDITION
    )


def safe_float(value):
    try:
        return float(value)
    except Exception:
        return None


def material_color(condition):
    return glass_color if condition["material_class"] == "glass" else sand_color


def current_material_scale():
    return (
        glass_friction_scale
        if current_condition()["material_class"] == "glass"
        else quartz_friction_scale
    )


def pb_to_vpython(position):
    x, depth_y, vertical_z = position
    return vector(x, vertical_z, depth_y)


def _fraction(value):
    return value - math.floor(value)


def target_solid_volume(condition):
    return (
        NORMALIZED_SOLID_VOLUME_PER_CUP
        *
        condition.get("amount_cups", 1)
    )


def particles_per_cup_for_condition(condition):
    if condition["material_class"] == "quartz":
        return QUARTZ_PARTICLES_PER_CUP

    return GLASS_PARTICLES_PER_CUP


def effective_particle_count(condition):
    return (
        particles_per_cup_for_condition(condition)
        *
        condition.get("amount_cups", 1)
    )


def nominal_coarse_radius(condition):
    count = effective_particle_count(condition)

    return (
        target_solid_volume(condition)
        /
        (
            count
            *
            (4.0 / 3.0)
            *
            math.pi
        )
    ) ** (1.0 / 3.0)


def resolution_radius_scale(condition):
    # Compatibility helper for code that still references the original
    # condition radius metadata.
    return (
        nominal_coarse_radius(condition)
        /
        condition["base_radius"]
    )


def effective_release_interval(condition):
    # Keep the bulk pour duration roughly comparable even when quartz uses
    # more, smaller representative particles.
    original_particles_per_cup = 180

    return (
        RELEASE_INTERVAL
        *
        original_particles_per_cup
        /
        particles_per_cup_for_condition(condition)
    )


def update_material_preview():
    if current_condition()["material_class"] == "glass":
        material_column.color = glass_color
        material_column.opacity = 0.42
    else:
        material_column.color = sand_color
        material_column.opacity = 0.50


def update_saved_result_label(result=None):
    if result is None:
        result_info_label.text = ""
        return

    result_info_label.text = (
        f"RESULT: {result['simulated_average']:.2f}°"
        f"   |   Front view {result['profile_x_angle']:.2f}°"
        f"   |   Side view {result['profile_y_angle']:.2f}°"
        f"   |   Physical avg {result['physical_average']:.2f}°"
        f"   |   Difference {result['difference']:+.2f}°"
    )


def update_instruction_labels():
    next_step_label.color = color.white

    if state == STATE_PREVALIDATING:
        next_step_label.text = "RUNNING SIMULATOR DIAGNOSTICS — PLEASE WAIT"
        next_step_label.color = color.cyan
        return

    if state == STATE_PREVALIDATION_FAILED:
        next_step_label.text = "SIMULATOR DIAGNOSTICS FOUND A PROBLEM — EXPERIMENT PAUSED"
        next_step_label.color = color.red
        trial_info_label.text = "Open 00_SIMULATOR_DIAGNOSTIC_REPORT.html to see what needs attention"
        return

    if state == STATE_COMPLETE:
        next_step_label.text = "EXPERIMENT COMPLETE — ALL 18 TRIALS FINISHED"
        next_step_label.color = color.green
        trial_info_label.text = "DONE: 18 OF 18   |   NO MORE SIMULATION TRIALS ARE REQUIRED"
        result_info_label.text = (
            "FINAL FILES: OPEN 01_MAIN_RESEARCH_REPORT.html AND "
            "05_SIMULATION_VALIDATION_AND_ACCURACY.html"
        )
        return

    name = current_condition()["name"]
    trial_info_label.text = (
        f"CURRENT: {name}   |   Trial {trial_number} of 3"
        f"   |   Overall {current_overall_trial_number()} of {TOTAL_EXPERIMENT_TRIALS}"
        f"   |   Speed: {simulation_speed_name}"
    )

    if state == STATE_READY:
        next_step_label.text = "READY — CLICK START EXPERIMENT"
        next_step_label.color = color.yellow
    elif state == STATE_RUNNING:
        next_step_label.text = "SIMULATION RUNNING IN 3D — NO ACTION NEEDED"
    elif state == STATE_SETTLING:
        next_step_label.text = "WAIT — 3D PILE IS SETTLING"
    elif state == STATE_READY_TO_MEASURE:
        next_step_label.text = "CALCULATING THE TRIAL RESULT"
        next_step_label.color = color.yellow
    elif state == STATE_SAVED:
        next_step_label.text = "TRIAL COMPLETE — REVIEW IT OR CLICK NEXT TRIAL"
        next_step_label.color = color.green
    elif state == STATE_REPLAYING:
        next_step_label.text = "REPLAYING THE SELECTED TRIAL"


def show_diagnostics_still_running():
    """Explain a temporary startup wait without suggesting a permanent lock."""
    next_step_label.text = "STILL VERIFYING THE SIMULATOR — PLEASE WAIT"
    next_step_label.color = color.cyan
    trial_info_label.text = "The experiment will become available automatically when verification finishes."
    result_info_label.text = "Nothing is wrong and no action is needed."

# ==========================================================
# MATERIAL / ENGINE PARAMETERS
# ==========================================================


def material_parameters(condition, quartz_scale=None, glass_scale=None):
    if quartz_scale is None:
        quartz_scale = quartz_friction_scale
    if glass_scale is None:
        glass_scale = glass_friction_scale

    if condition["material_class"] == "quartz":
        base = QUARTZ_SHARED
        scale = quartz_scale

        # Shared dry-sand resistance scale. Sliding changes moderately;
        # rolling/spinning resistance changes directly.
        sliding_scale = math.sqrt(max(0.20, scale))
        rolling_scale = scale

    else:
        base = GLASS_SHARED
        scale = glass_scale

        # Glass remains spherical, so sliding resistance is the dominant
        # adjustable material-class term. Rolling resistance stays very low
        # but is allowed to scale within a narrow bounded family.
        sliding_scale = scale
        rolling_scale = math.sqrt(max(0.05, scale))

    return {
        "lateral_friction": max(
            0.02,
            min(1.50, base["lateral_friction"] * sliding_scale)
        ),
        "floor_friction": max(
            0.02,
            min(1.50, base["floor_friction"] * sliding_scale)
        ),
        "rolling_friction": max(
            0.00005,
            min(0.25, base["rolling_friction"] * rolling_scale)
        ),
        "spinning_friction": max(
            0.00002,
            min(0.15, base["spinning_friction"] * rolling_scale)
        ),
        "restitution": base["restitution"],
        "linear_damping": base["linear_damping"],
        "angular_damping": base["angular_damping"],
        "scale": scale,
    }

# ==========================================================
# PARTICLE PLAN — TRUE 3D
# ==========================================================


def build_spawn_plan(condition, seed):
    rng = random.Random(seed)
    count = effective_particle_count(condition)
    base_radius = nominal_coarse_radius(condition)
    variation = condition["radius_variation"]

    # Low-discrepancy sequences spread positions/radii/velocities
    # through their allowed ranges more evenly than raw random draws.
    phi = (math.sqrt(5) - 1) / 2
    alpha = math.sqrt(2) - 1
    beta = math.sqrt(3) - 1
    gamma = math.sqrt(5) - 2

    quantiles = []
    for i in range(count):
        q_radius = _fraction((i + 1) * alpha + rng.uniform(-0.012, 0.012))
        q_r = _fraction((i + 1) * phi + rng.uniform(-0.012, 0.012))
        q_theta = _fraction((i + 1) * beta + rng.uniform(-0.012, 0.012))
        q_vel = _fraction((i + 1) * gamma + rng.uniform(-0.012, 0.012))
        q_vel2 = _fraction((i + 1) * 0.754877666 + rng.uniform(-0.012, 0.012))
        quantiles.append((q_radius, q_r, q_theta, q_vel, q_vel2))

    model = condition.get("size_distribution_model", "narrow_uniform")

    if model == "broad_log_volume_normalized":
        minimum = condition.get("size_distribution_min_multiplier", 0.55)
        maximum = condition.get("size_distribution_max_multiplier", 1.55)
        ratio = maximum / minimum

        raw_multipliers = [
            minimum
            *
            (ratio ** q[0])
            for q in quantiles
        ]

    else:
        raw_multipliers = [
            (1 - variation)
            +
            2
            *
            variation
            *
            q[0]
            for q in quantiles
        ]

    # EXACT volume normalization for EVERY condition.
    mean_cube = (
        sum(
            multiplier ** 3
            for multiplier
            in raw_multipliers
        )
        /
        len(raw_multipliers)
    )

    volume_normalization = (
        1.0
        /
        (
            mean_cube
            **
            (1.0 / 3.0)
        )
    )

    radii = [
        base_radius
        *
        multiplier
        *
        volume_normalization
        for multiplier
        in raw_multipliers
    ]

    plan = []

    for i, q in enumerate(quantiles):
        _, q_r, q_theta, q_vel, q_vel2 = q

        # Keep the particle CENTER inside the physical outlet.
        # Larger grains therefore get a slightly smaller allowed radius.
        safe_center_radius = max(
            0.0,
            OUTLET_RADIUS
            -
            radii[i] * 1.08
        )

        radial = (
            safe_center_radius
            *
            math.sqrt(q_r)
        )

        theta = (
            2
            *
            math.pi
            *
            q_theta
        )

        x = (
            radial
            *
            math.cos(theta)
        )

        y = (
            radial
            *
            math.sin(theta)
        )

        # CRITICAL FIX:
        # The physical material is released by gravity. It is NOT fired
        # sideways out of the nozzle. Any lateral motion must emerge from
        # real collisions after release.
        vx = 0.0
        vy = 0.0

        plan.append({
            "x": x,
            "y": y,
            "z": (
                OUTLET_Z
                +
                radii[i]
                *
                SPAWN_Z_OFFSET_FACTOR
            ),
            "radius": radii[i],
            "vx": vx,
            "vy": vy,
            "vz": -OUTLET_EXIT_SPEED,

            "orientation_euler": [
                rng.uniform(-math.pi, math.pi),
                rng.uniform(-math.pi, math.pi),
                rng.uniform(-math.pi, math.pi),
            ],
        })

    return plan

# ==========================================================
# PYBULLET WORLD / PARTICLES
# ==========================================================


def clear_measurements():
    for obj in measurement_objects:
        try:
            obj.visible = False
        except Exception:
            pass
    measurement_objects.clear()


def clear_visual_particles():
    global particles
    for item in particles:
        obj = item.get("visual")
        if obj is not None:
            try:
                obj.visible = False
            except Exception:
                pass
    particles = []


def hide_replay_objects():
    for obj in replay_objects:
        try:
            obj.visible = False
        except Exception:
            pass
    replay_objects.clear()


def configure_pybullet_world(condition, quartz_scale=None, glass_scale=None):
    p.resetSimulation(physicsClientId=physics_client)
    p.setGravity(0, 0, -GRAVITY, physicsClientId=physics_client)
    p.setTimeStep(PHYSICS_DT, physicsClientId=physics_client)

    try:
        p.setPhysicsEngineParameter(
            numSolverIterations=36,
            deterministicOverlappingPairs=1,
            physicsClientId=physics_client,
        )
    except TypeError:
        p.setPhysicsEngineParameter(
            numSolverIterations=36,
            physicsClientId=physics_client,
        )

    params = material_parameters(condition, quartz_scale, glass_scale)

    # Static circular floor disk.
    floor_shape = p.createCollisionShape(
        p.GEOM_CYLINDER,
        radius=COLLECTOR_RADIUS + 0.08,
        height=0.14,
        physicsClientId=physics_client,
    )
    floor_body = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=floor_shape,
        basePosition=[0, 0, FLOOR_Z - 0.07],
        physicsClientId=physics_client,
    )
    try:
        p.changeDynamics(
            floor_body,
            -1,
            lateralFriction=params["floor_friction"],
            restitution=params["restitution"],
            frictionAnchor=1,
            physicsClientId=physics_client,
        )
    except TypeError:
        p.changeDynamics(
            floor_body,
            -1,
            lateralFriction=params["floor_friction"],
            restitution=params["restitution"],
            physicsClientId=physics_client,
        )

    # Hollow cylindrical collector approximated by overlapping static
    # tangent wall segments. This gives real X/Y depth while keeping the
    # same visible cylindrical apparatus.
    circumference = 2 * math.pi * COLLECTOR_RADIUS
    segment_length = circumference / COLLECTOR_WALL_SEGMENTS * 1.22
    half_extents = [
        segment_length / 2,
        COLLECTOR_WALL_THICKNESS / 2,
        COLLECTOR_WALL_HEIGHT / 2,
    ]
    wall_shape = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=half_extents,
        physicsClientId=physics_client,
    )

    wall_center_radius = COLLECTOR_RADIUS + COLLECTOR_WALL_THICKNESS / 2
    for i in range(COLLECTOR_WALL_SEGMENTS):
        theta = 2 * math.pi * i / COLLECTOR_WALL_SEGMENTS
        x = wall_center_radius * math.cos(theta)
        y = wall_center_radius * math.sin(theta)
        yaw = theta + math.pi / 2
        orientation = p.getQuaternionFromEuler([0, 0, yaw])
        body = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=wall_shape,
            basePosition=[x, y, FLOOR_Z + COLLECTOR_WALL_HEIGHT / 2],
            baseOrientation=orientation,
            physicsClientId=physics_client,
        )
        try:
            p.changeDynamics(
                body,
                -1,
                lateralFriction=params["floor_friction"],
                restitution=params["restitution"],
                frictionAnchor=1,
                physicsClientId=physics_client,
            )
        except TypeError:
            p.changeDynamics(
                body,
                -1,
                lateralFriction=params["floor_friction"],
                restitution=params["restitution"],
                physicsClientId=physics_client,
            )

    return params


def create_particle_body(condition, params, entry, visible, shape_cache):
    radius = entry["radius"]

    if condition["material_class"] == "quartz":

        # ------------------------------------------------------
        # ROUGH-SPHERE DEM PROXY FOR SAND
        #
        # The DISPLAY already looked spherical, but previous PyBullet
        # collisions used cuboids. Those cuboids behaved like little rocks:
        # corners kicked grains sideways and retained too much rotation.
        #
        # A common coarse DEM approach is to use spheres for stable contact
        # detection, then represent irregular-grain resistance with high
        # sliding / rolling / spinning friction and near-zero restitution.
        # ------------------------------------------------------

        shape_key = (
            "quartz_rough_sphere",
            round(radius, 4),
        )

        if shape_key not in shape_cache:
            shape_cache[shape_key] = p.createCollisionShape(
                p.GEOM_SPHERE,
                radius=radius,
                physicsClientId=physics_client,
            )

        orientation = [0, 0, 0, 1]

    else:

        # Manufactured glass beads remain smooth spheres with much lower
        # rolling resistance than quartz sand.
        shape_key = (
            "glass_sphere",
            round(radius, 4),
        )

        if shape_key not in shape_cache:
            shape_cache[shape_key] = p.createCollisionShape(
                p.GEOM_SPHERE,
                radius=radius,
                physicsClientId=physics_client,
            )

        orientation = [0, 0, 0, 1]

    # Relative mass follows the nominal equivalent-sphere volume.
    reference_radius = nominal_coarse_radius(condition)
    mass = 0.012 * (radius / reference_radius) ** 3

    body = p.createMultiBody(
        baseMass=mass,
        baseCollisionShapeIndex=shape_cache[shape_key],
        basePosition=[entry["x"], entry["y"], entry["z"]],
        baseOrientation=orientation,
        physicsClientId=physics_client,
    )

    p.resetBaseVelocity(
        body,
        linearVelocity=[entry["vx"], entry["vy"], entry.get("vz", 0.0)],
        angularVelocity=[0, 0, 0],
        physicsClientId=physics_client,
    )

    # Moving sand grains are allowed to slide/roll/avalanche naturally.
    # Do NOT use frictionAnchor on every mobile particle: that can make a
    # coarse pile behave artificially "glued" once contacts form.
    p.changeDynamics(
        body,
        -1,
        lateralFriction=params["lateral_friction"],
        rollingFriction=params["rolling_friction"],
        spinningFriction=params["spinning_friction"],
        restitution=params["restitution"],
        linearDamping=params["linear_damping"],
        angularDamping=params["angular_damping"],
        physicsClientId=physics_client,
    )


    visual = None

    if visible:
        # The display remains intentionally lightweight. Collision
        # geometry is authoritative; VPython is only the visual lab.
        visual = sphere(
            pos=pb_to_vpython([entry["x"], entry["y"], entry["z"]]),
            radius=radius,
            color=material_color(condition),
        )

    return {
        "body": body,
        "radius": radius,
        "visual": visual,
        "shape_model": (
            "rough_sphere_relaxing_dry_sand"
            if condition["material_class"] == "quartz"
            else "sphere"
        ),
    }


def spawn_location_clear(entry):
    """
    Prevent a new PyBullet rigid body from being created on top of an
    existing one. Overlapping rigid bodies can generate very large solver
    impulses that look like the material is being shot from the outlet.
    """

    target_x = entry["x"]
    target_y = entry["y"]
    target_z = entry["z"]
    target_radius = entry["radius"]

    # Only recently spawned / nearby bodies can block the outlet.
    # Looking at the latest bodies keeps this inexpensive.
    for item in particles[-55:]:
        try:
            position, _ = p.getBasePositionAndOrientation(
                item["body"],
                physicsClientId=physics_client,
            )
        except Exception:
            continue

        # Bodies already well below the outlet cannot overlap a new body.
        if position[2] < target_z - 0.40:
            continue

        minimum_distance = (
            target_radius
            +
            item["radius"]
        ) * SPAWN_CLEARANCE_FACTOR

        dx = position[0] - target_x
        dy = position[1] - target_y
        dz = position[2] - target_z

        if (
            dx * dx
            +
            dy * dy
            +
            dz * dz
            <
            minimum_distance
            *
            minimum_distance
        ):
            return False

    return True


def find_clear_spawn_index(plan, next_index):
    """
    Look a short distance ahead in the deterministic feed plan for a free
    outlet lane. This keeps the stream moving without ever spawning several
    overlapping bodies at once.
    """

    end = min(
        len(plan),
        next_index
        +
        SPAWN_LOOKAHEAD
    )

    for candidate_index in range(
        next_index,
        end
    ):
        if spawn_location_clear(
            plan[candidate_index]
        ):
            return candidate_index

    return None


def sync_visual_particles():
    for item in particles:
        if item["visual"] is None:
            continue
        pos, _ = p.getBasePositionAndOrientation(
            item["body"], physicsClientId=physics_client
        )
        item["visual"].pos = pb_to_vpython(pos)


def particle_snapshot():
    snapshot = []
    for item in particles:
        pos, orn = p.getBasePositionAndOrientation(
            item["body"], physicsClientId=physics_client
        )
        snapshot.append({
            "x": round(pos[0], 6),
            "y": round(pos[1], 6),
            "z": round(pos[2], 6),
            "r": round(item["radius"], 6),
            "q": [round(v, 6) for v in orn],
        })
    return snapshot


def current_settling_metrics():
    if not particles:
        return float("inf"), float("inf")

    linear_speeds = []
    angular_speeds = []
    for item in particles:
        linear, angular = p.getBaseVelocity(
            item["body"], physicsClientId=physics_client
        )
        linear_speeds.append(math.sqrt(sum(v * v for v in linear)))
        angular_speeds.append(math.sqrt(sum(v * v for v in angular)))

    # Settling is a bulk-pile property. A single detached or wall-running
    # grain must not hold the whole experiment open indefinitely.
    linear_speeds.sort()
    angular_speeds.sort()
    q_index = int(0.90 * (len(linear_speeds) - 1))
    return linear_speeds[q_index], angular_speeds[q_index]

# ==========================================================
# ROBUST ANGLE MEASUREMENT
# ==========================================================


def best_fit(points):
    if len(points) < 3:
        return None

    n = len(points)
    sx = sum(x for x, _ in points)
    sy = sum(y for _, y in points)
    sxy = sum(x * y for x, y in points)
    sx2 = sum(x * x for x, _ in points)
    denominator = n * sx2 - sx * sx
    if abs(denominator) < 1e-9:
        return None

    slope = (n * sxy - sx * sy) / denominator
    intercept = (sy - slope * sx) / n
    return slope, intercept


def robust_fit(points):
    if len(points) < 4:
        return None

    working = [[p[0], p[1]] for p in points]

    for _ in range(4):
        result = best_fit(working)
        if result is None:
            return None
        slope, intercept = result
        residuals = [abs(y - (slope * x + intercept)) for x, y in working]
        med = statistics.median(residuals)
        mad = statistics.median(abs(r - med) for r in residuals)
        threshold = max(0.025, med + 2.5 * max(mad, 0.005))
        filtered = [pnt for pnt, residual in zip(working, residuals) if residual <= threshold]
        if len(filtered) < 4 or len(filtered) == len(working):
            break
        working = filtered

    final = best_fit(working)
    if final is None:
        return None
    return final[0], final[1], working


def _particle_top_points():
    points = []

    for item in particles:
        pos, _ = p.getBasePositionAndOrientation(
            item["body"],
            physicsClientId=physics_client,
        )

        points.append(
            (
                pos[0],
                pos[1],
                pos[2] + item["radius"],
                item["radius"],
            )
        )

    return points


def _estimate_pile_center(points):
    if not points:
        return 0.0, 0.0

    heights = sorted(
        point[2]
        for point in points
    )

    threshold_index = max(
        0,
        int(
            0.82
            *
            (len(heights) - 1)
        )
    )

    height_threshold = heights[
        threshold_index
    ]

    apex_points = [
        point
        for point in points
        if point[2] >= height_threshold
    ]

    if not apex_points:
        apex_points = points

    return (
        statistics.median(
            point[0]
            for point in apex_points
        ),
        statistics.median(
            point[1]
            for point in apex_points
        ),
    )


def _fit_radial_surface_from_points(points, base_z):
    """
    Measure the true 3D bulk free surface as height versus radial distance.

    For an approximately conical pile:
        z = apex_height - tan(theta) * r

    IMPORTANT V3.1 FIX:
    The first V3 implementation selected fit points using a vertical
    height window derived partly from the maximum measured height.
    A single high outlier could therefore distort the allowed window and
    make a mathematically valid cone fail its own self-test.

    V3.1 instead selects the middle RADIAL portion of the free surface.
    This directly matches the geometry of an angle-of-repose pile:
      - ignore the noisy apex,
      - ignore the rounded/collector-influenced foot,
      - fit the stable middle slope.
    """

    if len(points) < 40:
        return None

    center_x, center_y = _estimate_pile_center(
        points
    )

    radial_points = []

    for x, y, top_z, particle_radius in points:
        radial_points.append(
            (
                math.hypot(
                    x - center_x,
                    y - center_y,
                ),
                top_z,
            )
        )

    distances = sorted(
        r
        for r, _
        in radial_points
    )

    # 95th percentile keeps isolated runaway grains from defining the pile.
    footprint_index = min(
        len(distances) - 1,
        max(
            0,
            int(
                0.95
                *
                (len(distances) - 1)
            )
        )
    )

    footprint_radius = distances[
        footprint_index
    ]

    if footprint_radius <= 0:
        return None

    bins = 36
    bin_width = footprint_radius / bins
    surface = []

    for i in range(bins):
        r_left = i * bin_width
        r_right = r_left + bin_width

        heights = [
            top_z
            for radius_value, top_z
            in radial_points
            if r_left <= radius_value < r_right
        ]

        if not heights:
            continue

        heights.sort(
            reverse=True
        )

        # Robust upper envelope: median of highest few grains.
        top_count = min(
            4,
            len(heights)
        )

        surface.append(
            [
                r_left + bin_width / 2,
                statistics.median(
                    heights[:top_count]
                ),
            ]
        )

    if len(surface) < 10:
        return None

    # Mild smoothing.
    smooth = []

    for i in range(len(surface)):
        nearby = [
            surface[j][1]
            for j in range(
                max(0, i - 1),
                min(len(surface), i + 2),
            )
        ]

        smooth.append(
            [
                surface[i][0],
                statistics.median(
                    nearby
                ),
            ]
        )

    # Fit the middle radial face rather than using a max-height-derived
    # vertical window. This is robust to a high apex outlier.
    inner_radius = 0.12 * footprint_radius
    outer_radius = 0.82 * footprint_radius

    fit_points = [
        [radius_value, z]
        for radius_value, z
        in smooth
        if (
            inner_radius
            <=
            radius_value
            <=
            outer_radius
        )
    ]

    if len(fit_points) < 6:
        return None

    fit = robust_fit(
        fit_points
    )

    if fit is None:
        return None

    slope, intercept, used = fit

    # Robust pile height for diagnostics only.
    surface_heights = sorted(
        z
        for _, z
        in smooth
    )

    apex_index = min(
        len(surface_heights) - 1,
        max(
            0,
            int(
                0.95
                *
                (len(surface_heights) - 1)
            )
        )
    )

    robust_apex_height = surface_heights[
        apex_index
    ]

    pile_height = max(
        0.0,
        robust_apex_height
        -
        base_z
    )

    return {
        "angle": math.degrees(
            math.atan(
                abs(slope)
            )
        ),
        "slope": slope,
        "intercept": intercept,
        "used": used,
        "center_x": center_x,
        "center_y": center_y,
        "pile_height": pile_height,
        "footprint_radius": footprint_radius,
    }


def _projected_particles(axis):
    """
    Return projected grain circles for one side view.

    axis="x": front view (Bullet X horizontal, Bullet Z vertical)
    axis="y": perpendicular side view (Bullet Y horizontal, Bullet Z vertical)
    """

    projected = []

    for item in particles:
        pos, _ = p.getBasePositionAndOrientation(
            item["body"],
            physicsClientId=physics_client,
        )

        horizontal = (
            pos[0]
            if axis == "x"
            else pos[1]
        )

        projected.append(
            (
                horizontal,
                pos[2],
                item["radius"],
            )
        )

    return projected


def _trace_outer_edges_by_height(axis):
    """
    Trace the ACTUAL left/right projected pile boundary at many HEIGHTS.

    For each horizontal height z, every projected grain is a circle.
    If that height cuts the circle, the visible interval is:

        x = center_x ± sqrt(r^2 - (z-center_z)^2)

    The pile's left edge is the minimum of all such x values.
    The pile's right edge is the maximum.

    This cannot place the measurement through the pile interior: by
    construction it follows the outside boundary.
    """

    projected = _projected_particles(
        axis
    )

    if len(projected) < 20:
        return None

    # Robust pile top: ignore a tiny number of isolated high grains.
    top_candidates = sorted(
        z + radius
        for _, z, radius
        in projected
    )

    bottom_candidates = sorted(
        z - radius
        for _, z, radius
        in projected
    )

    top_index = min(
        len(top_candidates) - 1,
        max(
            0,
            int(
                0.985
                *
                (len(top_candidates) - 1)
            )
        )
    )

    robust_top = top_candidates[
        top_index
    ]

    # Physical floor is the correct base reference.
    base_z = FLOOR_Z

    pile_height = (
        robust_top
        -
        base_z
    )

    if pile_height <= 0.03:
        return None

    # Fit the main face only, excluding rounded apex and runout toe.
    # Physical-ruler band:
    # start close to the pile foot, end below the rounded apex.
    lower_z = (
        base_z
        +
        0.10
        *
        pile_height
    )

    upper_z = (
        base_z
        +
        0.72
        *
        pile_height
    )

    levels = 56
    left_edge_points = []
    right_edge_points = []

    for i in range(levels):

        z_level = (
            lower_z
            +
            (
                upper_z
                -
                lower_z
            )
            *
            i
            /
            (levels - 1)
        )

        leftmost = None
        rightmost = None

        for center_x, center_z, radius in projected:

            dz = (
                z_level
                -
                center_z
            )

            if abs(dz) > radius:
                continue

            half_width = math.sqrt(
                max(
                    0.0,
                    radius * radius
                    -
                    dz * dz
                )
            )

            left_x = (
                center_x
                -
                half_width
            )

            right_x = (
                center_x
                +
                half_width
            )

            if (
                leftmost is None
                or
                left_x < leftmost
            ):
                leftmost = left_x

            if (
                rightmost is None
                or
                right_x > rightmost
            ):
                rightmost = right_x

        if (
            leftmost is not None
            and
            rightmost is not None
        ):

            left_edge_points.append(
                [
                    leftmost,
                    z_level,
                ]
            )

            right_edge_points.append(
                [
                    rightmost,
                    z_level,
                ]
            )

    if (
        len(left_edge_points) < 10
        or
        len(right_edge_points) < 10
    ):
        return None

    return {
        "axis": axis,
        "left_points": left_edge_points,
        "right_points": right_edge_points,
        "base_z": base_z,
        "lower_z": lower_z,
        "upper_z": upper_z,
        "pile_height": pile_height,
    }


def _fit_physical_edge(points):
    """
    Fit a straight ruler line to traced OUTER edge points.

    Since the points are already guaranteed to be on the outside boundary,
    robust fitting is only used to ignore a few local bumps.
    """

    if len(points) < 6:
        return None

    fit = robust_fit(
        points
    )

    if fit is None:
        return None

    slope, intercept, used = fit

    return (
        slope,
        intercept,
        used,
    )


def _fit_projected_profile(axis, condition):
    """Measure one readable side-view ruler: supported foot -> apex -> foot.

    A single detached runout bead must not become a ruler endpoint. The pile
    feet are therefore selected from the supported bulk of the low-particle
    distribution (5th/95th percentiles), while the apex is a robust average
    of several high central grains. This preserves the physical experiment's
    ruler geometry without cherry-picking a steeper middle-face fit.
    """
    points = []
    for item in particles:
        pos, _ = p.getBasePositionAndOrientation(item["body"], physicsClientId=physics_client)
        horizontal = pos[0] if axis == "x" else pos[1]
        perpendicular = pos[1] if axis == "x" else pos[0]
        radius = item["radius"]
        radial = math.hypot(pos[0], pos[1])
        # The collector wall/rim is apparatus, not pile geometry.
        if radial <= COLLECTOR_RADIUS - 2.2 * radius:
            points.append((horizontal, perpendicular, pos[2], radius, radial))

    if len(points) < 40:
        return None

    mean_diameter = mean([2 * q[3] for q in points])
    base_z = FLOOR_Z
    central = [q for q in points if q[4] <= COLLECTOR_RADIUS * 0.38]
    if len(central) < 12:
        return None

    # Use a robust high central percentile, never a lone high grain.
    central_tops = sorted(q[2] + q[3] for q in central)
    apex_z = central_tops[int(0.90 * (len(central_tops) - 1))]
    apex_candidates = [q for q in central if q[2] + q[3] >= apex_z - mean_diameter]
    if len(apex_candidates) < 4:
        return None
    apex_x = mean([q[0] for q in apex_candidates])
    apex_z = mean([q[2] + q[3] for q in apex_candidates])
    pile_height = apex_z - base_z

    foot_band = [q for q in points if q[2] - q[3] <= base_z + 1.25 * mean_diameter]
    left = sorted(
        [q for q in foot_band if q[0] < apex_x - 0.5 * mean_diameter],
        key=lambda q: q[0],
    )
    right = sorted(
        [q for q in foot_band if q[0] > apex_x + 0.5 * mean_diameter],
        key=lambda q: q[0],
    )
    if len(left) < 10 or len(right) < 10 or pile_height < 2.5 * mean_diameter:
        return None

    # Supported endpoints: ignore only the outermost one percent on each side.
    # With hundreds of grains this removes a few isolated runout beads while
    # preserving essentially the entire physical toe of the pile.
    left_foot = left[int(0.01 * (len(left) - 1))]
    right_foot = right[int(0.99 * (len(right) - 1))]
    left_run = apex_x - left_foot[0]
    right_run = right_foot[0] - apex_x
    min_run = 4.0 * mean_diameter
    if left_run < min_run or right_run < min_run:
        return None

    left_angle = math.degrees(math.atan2(pile_height, left_run))
    right_angle = math.degrees(math.atan2(pile_height, right_run))
    if max(left_angle, right_angle) > 55 or abs(left_angle - right_angle) > 16:
        return None

    return {
        "axis": axis,
        "angle": (left_angle + right_angle) / 2.0,
        "negative_angle": left_angle,
        "positive_angle": right_angle,
        "apex": (apex_x, apex_z),
        "left_foot": (left_foot[0], base_z),
        "right_foot": (right_foot[0], base_z),
        "pile_height": pile_height,
    }


def _draw_projected_profile(profile):
    """
    Draw the two ruler lines directly on the FRONT visible outer edges.

    Only the X/front view is drawn. The Y-side view is still calculated
    internally for consistency and the final average.
    """

    if (
        profile is None
        or
        profile["axis"] != "x"
    ):
        return

    FRONT_DISPLAY_DEPTH = -0.24
    apex = profile["apex"]
    for foot in [profile["left_foot"], profile["right_foot"]]:
        measurement_objects.append(
            curve(
                pos=[
                    vector(foot[0], foot[1], FRONT_DISPLAY_DEPTH),
                    vector(apex[0], apex[1], FRONT_DISPLAY_DEPTH),
                ],
                radius=0.024,
                color=color.red,
            )
        )



def pile_resolution_metrics(condition):
    points = _particle_top_points()

    if not points:
        return {
            "pile_height": None,
            "footprint_radius": None,
            "particle_layers": None,
        }

    refined_radius = (
        condition["base_radius"]
        *
        resolution_radius_scale(condition)
    )

    base_z = (
        FLOOR_Z
        +
        refined_radius
    )

    radial_fit = _fit_radial_surface_from_points(
        points,
        base_z,
    )

    if radial_fit is None:
        return {
            "pile_height": None,
            "footprint_radius": None,
            "particle_layers": None,
        }

    mean_diameter = mean(
        [
            2.0
            *
            point[3]
            for point in points
        ]
    )

    layers = (
        radial_fit["pile_height"]
        /
        mean_diameter
        if mean_diameter > 0
        else None
    )

    return {
        "pile_height": radial_fit["pile_height"],
        "footprint_radius": radial_fit["footprint_radius"],
        "particle_layers": layers,
    }


def _draw_radial_fit(radial_fit):
    if radial_fit is None:
        return

    used = radial_fit["used"]

    if not used:
        return

    r1 = min(
        point[0]
        for point in used
    )

    r2 = max(
        point[0]
        for point in used
    )

    z1 = (
        radial_fit["slope"]
        *
        r1
        +
        radial_fit["intercept"]
    )

    z2 = (
        radial_fit["slope"]
        *
        r2
        +
        radial_fit["intercept"]
    )

    measurement_objects.append(
        curve(
            pos=[
                vector(
                    radial_fit["center_x"] + r1,
                    z1,
                    radial_fit["center_y"],
                ),
                vector(
                    radial_fit["center_x"] + r2,
                    z2,
                    radial_fit["center_y"],
                ),
            ],
            radius=0.018,
            color=color.red,
        )
    )


def calculate_angle(draw=True, condition=None):
    if condition is None:
        condition = current_condition()

    # PRIMARY: side-view measurement, matching the physical experiment.
    profile_x = _fit_projected_profile(
        "x",
        condition,
    )

    profile_y = _fit_projected_profile(
        "y",
        condition,
    )

    if (
        profile_x is None
        or
        profile_y is None
    ):
        return None

    x_angle = profile_x["angle"]
    y_angle = profile_y["angle"]

    simulated_average = (
        x_angle
        +
        y_angle
    ) / 2.0

    # Keep the 3D radial fit only as a diagnostic. It is NOT the reported
    # research angle because the physical reference was measured from a
    # side profile.
    points = _particle_top_points()

    base_z = (
        FLOOR_Z
        +
        nominal_coarse_radius(
            condition
        )
    )

    radial_fit = _fit_radial_surface_from_points(
        points,
        base_z,
    )

    radial_angle = (
        None
        if radial_fit is None
        else radial_fit["angle"]
    )

    if draw:
        # Display the physical-style ruler lines only on the front side view.
        # The Y view is still calculated for the final 3D average.
        _draw_projected_profile(
            profile_x
        )

    return (
        x_angle,
        y_angle,
        simulated_average,
        radial_angle,
    )

# ==========================================================
# MEASUREMENT MATH SELF-TEST
# ==========================================================


def run_measurement_math_self_check():
    known_angles = [15, 20, 25, 30, 35, 40]
    checks = []
    rng = random.Random(8675309)

    for expected in known_angles:
        slope_value = math.tan(
            math.radians(expected)
        )

        # Legacy 2D robust-fit test.
        left_points = []
        right_points = []

        for i in range(18):
            x = 0.20 + i * 0.065

            left_points.append([
                -x,
                2.0 - slope_value * x + rng.uniform(-0.006, 0.006),
            ])

            right_points.append([
                x,
                2.0 - slope_value * x + rng.uniform(-0.006, 0.006),
            ])

        left_points.append([-0.80, 2.65])
        right_points.append([0.72, 1.20])

        left_result = robust_fit(left_points)
        right_result = robust_fit(right_points)

        if left_result is None or right_result is None:
            checks.append({
                "expected": expected,
                "measured": None,
                "radial_measured": None,
                "error": None,
                "radial_error": None,
                "passed": False,
            })
            continue

        left_angle = math.degrees(
            math.atan(
                abs(
                    left_result[0]
                )
            )
        )

        right_angle = math.degrees(
            math.atan(
                abs(
                    right_result[0]
                )
            )
        )

        measured_2d = (
            left_angle
            +
            right_angle
        ) / 2.0

        # NEW: test the actual 3D radial-cone algorithm.
        synthetic_points = []

        for ring in range(1, 31):
            radius_value = 0.03 + ring * 0.035

            for azimuth_index in range(16):
                theta = (
                    2
                    *
                    math.pi
                    *
                    azimuth_index
                    /
                    16
                )

                synthetic_points.append(
                    (
                        radius_value * math.cos(theta),
                        radius_value * math.sin(theta),
                        2.2
                        -
                        slope_value
                        *
                        radius_value
                        +
                        rng.uniform(
                            -0.004,
                            0.004
                        ),
                        0.01,
                    )
                )

        synthetic_points.extend([
            (0.72, 0.10, 2.55, 0.01),
            (-0.65, -0.25, 1.15, 0.01),
            (0.20, -0.80, 2.45, 0.01),
        ])

        radial_fit = _fit_radial_surface_from_points(
            synthetic_points,
            base_z=0.5,
        )

        radial_measured = (
            None
            if radial_fit is None
            else radial_fit["angle"]
        )

        error_2d = abs(
            measured_2d
            -
            expected
        )

        radial_error = (
            None
            if radial_measured is None
            else abs(
                radial_measured
                -
                expected
            )
        )

        checks.append({
            "expected": expected,
            "measured": measured_2d,
            "radial_measured": radial_measured,
            "error": error_2d,
            "radial_error": radial_error,
            "passed": (
                error_2d <= 1.0
                and
                radial_error is not None
                and
                radial_error <= 1.0
            ),
        })

    return checks

# ==========================================================
# GENERIC HEADLESS 3D SIMULATION
# ==========================================================


def run_hidden_pilot(target_condition_index, seed, quartz_scale, glass_scale):
    global particles

    condition = CONDITIONS[target_condition_index]
    clear_visual_particles()
    params = configure_pybullet_world(condition, quartz_scale, glass_scale)
    plan = build_spawn_plan(condition, seed)
    shape_cache = {}
    particles = []

    release_clock = 0.0
    release_interval = effective_release_interval(condition)
    next_index = 0
    simulation_time = 0.0
    all_released_time = None
    stable_time = 0.0

    settle_max = (
        SETTLE_MAX_TIME_GLASS
        if condition["material_class"] == "glass"
        else SETTLE_MAX_TIME_QUARTZ
    )

    release_interval = effective_release_interval(condition)
    # Outlet-clearance protection can intentionally delay the feed. Give it
    # a bounded allowance based on particle count, then reserve the complete
    # settling window after the final grain is released.
    protected_feed_allowance = max(
        len(plan) * release_interval * 2.5,
        len(plan) * PHYSICS_DT * 6.0,
    )
    hard_stop = protected_feed_allowance + settle_max + 5.0

    step_count = 0

    while simulation_time < hard_stop:
        step_count += 1
        release_clock += PHYSICS_DT
        spawned_this_step = 0
        while (
            release_clock >= release_interval
            and next_index < len(plan)
            and spawned_this_step < MAX_SAFE_SPAWNS_PER_PHYSICS_STEP
        ):
            clear_index = find_clear_spawn_index(plan, next_index)
            if clear_index is None:
                release_clock = min(release_clock, release_interval)
                break
            if clear_index != next_index:
                plan[next_index], plan[clear_index] = plan[clear_index], plan[next_index]
            particles.append(
                create_particle_body(
                    condition,
                    params,
                    plan[next_index],
                    visible=False,
                    shape_cache=shape_cache,
                )
            )
            next_index += 1
            spawned_this_step += 1
            release_clock = max(0.0, release_clock - release_interval)

        p.stepSimulation(physicsClientId=physics_client)
        simulation_time += PHYSICS_DT

        if next_index == len(plan):
            if all_released_time is None:
                all_released_time = simulation_time

            elapsed_settling = simulation_time - all_released_time

            if step_count % 8 == 0:
                max_linear, max_angular = current_settling_metrics()
                if (
                    max_linear <= SETTLE_LINEAR_SPEED
                    and max_angular <= SETTLE_ANGULAR_SPEED
                ):
                    stable_time += PHYSICS_DT * 8
                else:
                    stable_time = 0.0

            if (
                elapsed_settling >= SETTLE_MIN_TIME
                and stable_time >= SETTLE_REQUIRED_STABLE_TIME
            ):
                break

            if elapsed_settling >= settle_max:
                break

    settled = stable_time >= SETTLE_REQUIRED_STABLE_TIME
    measured = calculate_angle(draw=False, condition=condition)
    resolution_metrics = pile_resolution_metrics(condition)

    if measured is None:
        angle = None
        profile_x = None
        profile_y = None
        extent = None
    else:
        profile_x, profile_y, angle, extent = measured

    particle_layers = resolution_metrics["particle_layers"]

    return {
        "condition_index": target_condition_index,
        "condition": condition["name"],
        "seed": seed,
        "angle": angle,
        "left": profile_x,
        "right": profile_y,
        "left_right_difference": (
            abs(profile_x - profile_y)
            if profile_x is not None and profile_y is not None
            else None
        ),
        "radial_angle_diagnostic": extent,
        "settled": settled,
        "simulated_seconds": simulation_time,
        "scale": params["scale"],
        "particle_layers": particle_layers,
        "pile_height": resolution_metrics["pile_height"],
        "footprint_radius": resolution_metrics["footprint_radius"],
        "resolution_adequate": (
            particle_layers is not None
            and
            particle_layers >= MIN_PILE_LAYERS_FOR_VALID_MEASUREMENT
        ),
        "effective_particle_count": effective_particle_count(condition),
        "released_particle_count": len(particles),
        "all_released": next_index == len(plan),
    }


def summarize_pilot_records(records, condition_indices):
    summaries = []
    for idx in condition_indices:
        items = [r for r in records if r["condition_index"] == idx]
        angles = [r["angle"] for r in items if r["angle"] is not None]
        physical = physical_average(CONDITIONS[idx])

        if not angles:
            summaries.append({
                "condition_index": idx,
                "condition": CONDITIONS[idx]["name"],
                "mean": None,
                "sd": None,
                "range": None,
                "error_percent": None,
                "physical_average": physical,
                "all_settled": False,
                "max_lr_difference": None,
                "min_particle_layers": None,
                "all_resolution_adequate": False,
            })
            continue

        avg = mean(angles)
        sd = statistics.stdev(angles) if len(angles) >= 2 else 0.0
        spread = max(angles) - min(angles)
        error = abs(avg - physical) / physical * 100
        lr_values = [
            r["left_right_difference"]
            for r in items
            if r["left_right_difference"] is not None
        ]

        layer_values = [
            r.get("particle_layers")
            for r in items
            if r.get("particle_layers") is not None
        ]

        summaries.append({
            "condition_index": idx,
            "condition": CONDITIONS[idx]["name"],
            "mean": avg,
            "sd": sd,
            "range": spread,
            "error_percent": error,
            "physical_average": physical,
            "all_settled": all(r["settled"] for r in items),
            "max_lr_difference": max(lr_values) if lr_values else None,
            "min_particle_layers": min(layer_values) if layer_values else None,
            "all_resolution_adequate": all(
                r.get("resolution_adequate", False)
                for r in items
            ),
        })

    return summaries


def summaries_stable(summaries, require_repeatability):
    for item in summaries:
        if item["mean"] is None or not item["all_settled"]:
            return False

        if not item.get("all_resolution_adequate", False):
            return False
        if item["max_lr_difference"] is not None and item["max_lr_difference"] > MAX_ORTHOGONAL_PROFILE_DIFFERENCE_DEG:
            return False
        if require_repeatability:
            if item["sd"] > MAX_PILOT_SD_DEG:
                return False
            if item["range"] > MAX_PILOT_RANGE_DEG:
                return False
    return True


def average_summary_error(summaries):
    values = [s["error_percent"] for s in summaries if s["error_percent"] is not None]
    return mean(values) if values else None


def run_group_once(indices, seed, quartz_scale, glass_scale, progress_prefix):
    records = []
    total = len(indices)
    for n, idx in enumerate(indices, start=1):
        set_prevalidation_status(
            "PRE-VALIDATING 3D MODEL",
            f"{progress_prefix} — hidden 3D pilot {n} of {total}: {CONDITIONS[idx]['name']}",
        )
        records.append(
            run_hidden_pilot(idx, seed + idx * 1009, quartz_scale, glass_scale)
        )
    return records


def deterministic_calibration(material_name, indices, candidate_scales, current_quartz, current_glass):
    # Stage A is adaptive for speed:
    #   1) test the neutral scale 1.00 first;
    #   2) only run low/high sensitivity candidates if neutral does not
    #      already satisfy the calibration gate.
    # This keeps the first successful pre-validation substantially shorter.
    history = []
    best = None

    ordered_candidates = list(candidate_scales)
    if 1.0 in ordered_candidates:
        ordered_candidates.remove(1.0)
        ordered_candidates.insert(0, 1.0)

    def evaluate(scale, stage):
        q_scale = scale if material_name == "quartz" else current_quartz
        g_scale = scale if material_name == "glass" else current_glass
        records = run_group_once(
            indices,
            CALIBRATION_SEED,
            q_scale,
            g_scale,
            f"{material_name.title()} {stage} — scale {scale:.2f}",
        )
        summaries = summarize_pilot_records(records, indices)
        error = average_summary_error(summaries)
        stable_geometry = summaries_stable(summaries, require_repeatability=False)
        entry = {
            "scale": scale,
            "summaries": summaries,
            "records": records,
            "mean_error_percent": error,
            "stable_geometry": stable_geometry,
            "stage": stage,
        }
        history.append(entry)
        return entry

    neutral = evaluate(ordered_candidates[0], "neutral baseline")
    if neutral["stable_geometry"] and neutral["mean_error_percent"] is not None:
        best = neutral

    # Only spend time on sensitivity candidates when baseline is not
    # already good enough.
    if (
        best is None
        or best["mean_error_percent"] > CALIBRATION_ERROR_LIMIT_PERCENT
    ):
        for scale in ordered_candidates[1:]:
            entry = evaluate(scale, "sensitivity")
            if entry["stable_geometry"] and entry["mean_error_percent"] is not None:
                if best is None or entry["mean_error_percent"] < best["mean_error_percent"]:
                    best = entry

    if best is None:
        return False, None, history

    # One bounded extension only if the best tested value is an edge and
    # calibration is still outside the accuracy gate.
    if best["mean_error_percent"] > CALIBRATION_ERROR_LIMIT_PERCENT:
        scales_tested = sorted({item["scale"] for item in history})
        extra_scale = None
        if best["scale"] == min(scales_tested):
            extra_scale = EXTREME_LOW_SCALE
        elif best["scale"] == max(scales_tested):
            extra_scale = EXTREME_HIGH_SCALE

        if extra_scale is not None and extra_scale not in scales_tested:
            entry = evaluate(extra_scale, "bounded extension")
            if entry["stable_geometry"] and entry["mean_error_percent"] is not None:
                if entry["mean_error_percent"] < best["mean_error_percent"]:
                    best = entry

    chosen_scale = best["scale"]

    # Reuse the already-computed chosen seed instead of running it again.
    first_records = best["records"]

    q_scale = chosen_scale if material_name == "quartz" else current_quartz
    g_scale = chosen_scale if material_name == "glass" else current_glass
    second_records = run_group_once(
        indices,
        CONFIRMATION_SEED,
        q_scale,
        g_scale,
        f"{material_name.title()} chosen model — independent confirmation",
    )

    combined = first_records + second_records
    confirmation_summaries = summarize_pilot_records(combined, indices)
    confirmation_error = average_summary_error(confirmation_summaries)
    confirmation_stable = summaries_stable(
        confirmation_summaries, require_repeatability=True
    )

    history.append({
        "scale": chosen_scale,
        "summaries": confirmation_summaries,
        "mean_error_percent": confirmation_error,
        "stable_geometry": confirmation_stable,
        "stage": "repeatability confirmation",
    })

    passed = (
        confirmation_stable
        and confirmation_error is not None
        and confirmation_error <= CALIBRATION_ERROR_LIMIT_PERCENT
    )

    # Strip raw records before caching/reporting to keep the cache concise.
    for item in history:
        item.pop("records", None)

    return passed, chosen_scale, history


def run_held_out_validation(q_scale, g_scale):
    # First held-out pass can fail fast on geometry/accuracy before paying
    # for the second repeatability seed.
    first_records = run_group_once(
        HELD_OUT_VALIDATION_CONDITIONS,
        HELD_OUT_SEEDS[0],
        q_scale,
        g_scale,
        "Held-out validation — first independent set",
    )
    first_summaries = summarize_pilot_records(
        first_records, HELD_OUT_VALIDATION_CONDITIONS
    )

    if not summaries_stable(first_summaries, require_repeatability=False):
        return False, first_summaries

    first_errors = [
        s["error_percent"] for s in first_summaries
        if s["error_percent"] is not None
    ]
    if len(first_errors) != len(HELD_OUT_VALIDATION_CONDITIONS):
        return False, first_summaries
    if max(first_errors) > HELD_OUT_CONDITION_ERROR_LIMIT_PERCENT:
        return False, first_summaries
    if mean(first_errors) > HELD_OUT_MEAN_ERROR_LIMIT_PERCENT:
        return False, first_summaries

    # Only a promising frozen model earns the second seed used to verify
    # held-out repeatability.
    second_records = run_group_once(
        HELD_OUT_VALIDATION_CONDITIONS,
        HELD_OUT_SEEDS[1],
        q_scale,
        g_scale,
        "Held-out validation — repeatability confirmation",
    )

    summaries = summarize_pilot_records(
        first_records + second_records,
        HELD_OUT_VALIDATION_CONDITIONS,
    )
    if not summaries_stable(summaries, require_repeatability=True):
        return False, summaries

    errors = [s["error_percent"] for s in summaries if s["error_percent"] is not None]
    if max(errors) > HELD_OUT_CONDITION_ERROR_LIMIT_PERCENT:
        return False, summaries
    if mean(errors) > HELD_OUT_MEAN_ERROR_LIMIT_PERCENT:
        return False, summaries

    return True, summaries


# ==========================================================
# PRE-VALIDATION CACHE / SIGNATURE
# ==========================================================


def model_signature():
    try:
        with open(__file__, "rb") as source_file:
            source_sha256 = hashlib.sha256(source_file.read()).hexdigest()
    except Exception:
        source_sha256 = MODEL_VERSION

    payload = {
        "version": MODEL_VERSION,
        "source_sha256": source_sha256,
        "pybullet_api_version": p.getAPIVersion(),
        "physics_dt": PHYSICS_DT,
        "collector_radius": COLLECTOR_RADIUS,
        "wall_segments": COLLECTOR_WALL_SEGMENTS,
        "outlet_radius": OUTLET_RADIUS,
        "release_interval": RELEASE_INTERVAL,
        "quartz_particles_per_cup": QUARTZ_PARTICLES_PER_CUP,
        "glass_particles_per_cup": GLASS_PARTICLES_PER_CUP,
        "normalized_solid_volume_per_cup": NORMALIZED_SOLID_VOLUME_PER_CUP,
        "quartz_shared": QUARTZ_SHARED,
        "glass_shared": GLASS_SHARED,
        # Physical trials are included in the CACHE signature because a
        # changed validation dataset must trigger re-validation. They are
        # still not fed into individual research-trial physics.
        "conditions": CONDITIONS,
        "calibration_indices": CALIBRATION_QUARTZ_CONDITIONS + CALIBRATION_GLASS_CONDITIONS,
        "heldout_indices": HELD_OUT_VALIDATION_CONDITIONS,
    }
    raw = json.dumps(payload, sort_keys=True).encode("utf-8")
    return hashlib.sha256(raw).hexdigest()


def load_valid_prevalidation_cache():
    if not os.path.exists(PREVALIDATION_CACHE_FILE):
        return None
    try:
        with open(PREVALIDATION_CACHE_FILE, "r") as f:
            data = json.load(f)
    except Exception:
        return None

    if not data.get("passed"):
        return None
    if data.get("model_signature") != model_signature():
        return None
    return data


def save_prevalidation_cache(data):
    with open(PREVALIDATION_CACHE_FILE, "w") as f:
        json.dump(data, f, indent=2)

# ==========================================================
# PRE-VALIDATION UI / REPORT
# ==========================================================


def set_prevalidation_status(title, detail=""):
    if HEADLESS_MODE:
        print(f"{title}: {detail}", flush=True)
    next_step_label.text = title
    next_step_label.color = color.cyan
    trial_info_label.text = detail
    result_info_label.text = "The experiment will unlock when the simulator checks finish."
    rate(30)


def _geometry_gate_reasons(summary):
    reasons = []

    if summary["mean"] is None:
        reasons.append("no measurable angle")

    if not summary.get("all_settled", False):
        reasons.append("not fully settled")

    layers = summary.get("min_particle_layers")

    if layers is None:
        reasons.append("pile-layer count unavailable")
    elif layers < MIN_PILE_LAYERS_FOR_VALID_MEASUREMENT:
        reasons.append(
            f"under-resolved ({layers:.1f} layers)"
        )

    profile_difference = summary.get("max_lr_difference")

    if (
        profile_difference is not None
        and
        profile_difference
        >
        MAX_ORTHOGONAL_PROFILE_DIFFERENCE_DEG
    ):
        reasons.append(
            f"X/Y profile disagreement {profile_difference:.1f}°"
        )

    if not reasons:
        return "geometry gate passed"

    return "; ".join(reasons)


def _summary_text(summary):
    if summary["mean"] is None:
        return (
            "no measurable result; "
            +
            _geometry_gate_reasons(summary)
        )

    layers = summary.get("min_particle_layers")
    layers_text = (
        "—"
        if layers is None
        else f"{layers:.1f}"
    )

    profile_difference = summary.get("max_lr_difference")
    profile_text = (
        "—"
        if profile_difference is None
        else f"{profile_difference:.1f}°"
    )

    return (
        f"{summary['mean']:.2f}° mean, "
        f"{summary['sd']:.2f}° SD, "
        f"{summary['range']:.2f}° range, "
        f"{summary['error_percent']:.1f}% error, "
        f"{layers_text} layers, "
        f"X/Y diff {profile_text}, "
        f"{_geometry_gate_reasons(summary)}"
    )


def build_prevalidation_report(
    measurement_checks,
    quartz_history,
    glass_history,
    heldout_summaries,
    passed,
    reason,
    elapsed_seconds,
    cache_used=False,
):
    measurement_rows = "".join(
        "<tr>"
        f"<td>{item['expected']:.1f}°</td>"
        f"<td>{'—' if item['measured'] is None else format(item['measured'], '.2f') + '°'}</td>"
        f"<td>{'—' if item.get('radial_measured') is None else format(item['radial_measured'], '.2f') + '°'}</td>"
        f"<td>{'—' if item.get('radial_error') is None else format(item['radial_error'], '.2f') + '°'}</td>"
        f"<td>{'PASS' if item['passed'] else 'FAIL'}</td>"
        "</tr>"
        for item in measurement_checks
    )

    calibration_rows = []
    for material_name, history in [("Quartz", quartz_history), ("Glass beads", glass_history)]:
        for item in history:
            pilots = "<br>".join(
                f"{s['condition']}: {_summary_text(s)}"
                for s in item.get("summaries", [])
            )
            error = item.get("mean_error_percent")
            calibration_rows.append(
                "<tr>"
                f"<td>{material_name}</td>"
                f"<td>{item.get('stage','')}</td>"
                f"<td>{item.get('scale', 0):.3f}</td>"
                f"<td>{pilots}</td>"
                f"<td>{'—' if error is None else f'{error:.1f}%'} </td>"
                f"<td>{'YES' if item.get('stable_geometry') else 'NO'}</td>"
                "</tr>"
            )

    heldout_rows = "".join(
        "<tr>"
        f"<td>{s['condition']}</td>"
        f"<td>{'—' if s['mean'] is None else format(s['mean'], '.2f') + '°'}</td>"
        f"<td>{s['physical_average']:.2f}°</td>"
        f"<td>{'—' if s['error_percent'] is None else format(s['error_percent'], '.1f') + '%'}</td>"
        f"<td>{'—' if s['sd'] is None else format(s['sd'], '.2f') + '°'}</td>"
        f"<td>{'YES' if s['all_settled'] else 'NO'}</td>"
        "</tr>"
        for s in heldout_summaries
    )

    status_class = "pass" if passed else "fail"
    status_text = "PASSED — 3D MODEL FROZEN" if passed else "FAILED — TRIALS LOCKED"
    diagnostic_run_text = "Prior verified result reused" if cache_used else "Full checks executed"

    html = f'''<!doctype html>
<html><head><meta charset="utf-8"><title>3D Pre-validation and Calibration</title>
<style>
body{{margin:0;background:#101216;color:#eef1f5;font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Arial,sans-serif}}
main{{max-width:1150px;margin:0 auto;padding:34px 24px 60px}}.card{{background:#181b21;border:1px solid #2b3039;border-radius:12px;padding:20px;margin-top:20px}}
table{{width:100%;border-collapse:collapse}}th,td{{text-align:left;padding:10px;border-bottom:1px solid #2b3039;vertical-align:top;font-size:13px}}.pass{{color:#9cf0ad}}.fail{{color:#ff9da5}}.note{{color:#aeb6c1;line-height:1.6}}code{{background:#0f1318;padding:2px 5px;border-radius:4px}}
</style></head><body><main>
<h1>Automatic 3D Pre-validation and Calibration</h1>
<div class="card"><h2 class="{status_class}">{status_text}</h2><p class="note">{reason}</p>
<p class="note"><b>Physics engine:</b> {PHYSICS_ENGINE_NAME}<br><b>PyBullet API version:</b> {p.getAPIVersion()}<br><b>Model version:</b> <code>{MODEL_VERSION}</code><br><b>Diagnostic time:</b> {elapsed_seconds:.1f} s<br><b>Diagnostic run:</b> {diagnostic_run_text}<br><b>Frozen quartz friction scale:</b> {quartz_friction_scale:.4f}<br><b>Frozen glass friction scale:</b> {glass_friction_scale:.4f}</p></div>
<div class="card"><h2>1. Measurement-math self-test</h2><p class="note">Synthetic known slopes test both the legacy 2D fit and the actual 3D radial-cone measurement without using any UCF result.</p><table><thead><tr><th>Known</th><th>2D fit</th><th>3D radial fit</th><th>3D error</th><th>Status</th></tr></thead><tbody>{measurement_rows}</tbody></table></div>
<div class="card"><h2>2. What V4 fixes</h2><p class="note"><b>The key V4 correction is the 2D-to-3D amount mapping.</b> The old 180 particles represented a side cross-section, not an entire 3D cup. V1-V3 preserved that invalid 3D material volume and therefore produced extremely shallow heaps. V4 uses an explicit normalized 3D bulk amount: 0.25 solid-volume units per cup, represented by 450 coarse-grained rigid bodies per cup. Every 1-cup condition has the same total solid volume; the 2-cup condition has exactly twice that amount. This normalization is independent of the known UCF angles.<br><br><b>PyBullet contact-friction correction:</b> the program also treats lateral-friction settings as per-body Bullet properties. Bullet combines the default friction values of two touching bodies, so the earlier low per-body values created much lower contact resistance than intended.<br><br> V4 performs a volume-preserving resolution refinement (more, smaller particles) and uses robust full-depth side-profile slopes as the primary angle. The radial fitter now selects the middle radial face directly, so a single high particle cannot distort the fit window. A minimum particle-layer gate blocks under-resolved piles before calibration can call them valid.<br><br>Particle motion and collisions occur in true X/Y/Z space inside PyBullet. <b>Quartz is represented by volume-matched angular rigid grains rather than perfect spheres</b>, while manufactured glass beads remain spherical. The two-cup bead condition contains twice the 3D material. Unsieved quartz uses a broad <b>volume-normalized</b> size distribution. Angle measurement now uses two orthogonal full-depth side silhouettes instead of a thin center slice.</p></div>
<div class="card"><h2>3. Deterministic bounded calibration</h2><p class="note">The program tests a small, declared set of shared friction scales on calibration conditions. It does not randomly search until it happens to match an answer. After a scale is chosen, an independent seed checks repeatability.</p><table><thead><tr><th>Class</th><th>Stage</th><th>Scale</th><th>Pilot results</th><th>Mean error</th><th>Stable?</th></tr></thead><tbody>{''.join(calibration_rows)}</tbody></table></div>
<div class="card"><h2>4. Held-out validation</h2><p class="note">Unsieved quartz, quartz &gt;500 μm, and glass beads — 2 cups are not permitted to change the frozen model. Their known physical means are used only after calibration is frozen.</p><table><thead><tr><th>Condition</th><th>Pilot mean</th><th>Physical mean</th><th>Error</th><th>SD</th><th>Settled?</th></tr></thead><tbody>{heldout_rows}</tbody></table></div>
<div class="card"><h2>Interpretation</h2><p class="note">Passing means this particular frozen 3D model met the project's repeatability and held-out agreement criteria for this six-condition test set. It is evidence, not proof, of general granular-physics accuracy. Failure locks research trials instead of manufacturing a pass.</p></div>
</main></body></html>'''

    with open(PREVALIDATION_FILE, "w") as f:
        f.write(html)

# ==========================================================
# AUTOMATIC PRE-VALIDATION
# ==========================================================


def run_automatic_prevalidation():
    global quartz_friction_scale
    global glass_friction_scale
    global model_prevalidation_passed
    global model_frozen
    global state

    started = time.perf_counter()

    set_prevalidation_status(
        "RUNNING SIMULATOR DIAGNOSTICS",
        "Step 1 — checking the measurement system",
    )
    measurement_checks = run_measurement_math_self_check()
    measurement_ok = all(item["passed"] for item in measurement_checks)

    if not measurement_ok:
        reason = ("The angle-measurement mathematics failed a known-geometry self-test. "
          "Research physics calibration did not run; this is a measurement-code failure, "
          "not evidence that the material model failed.")
        build_prevalidation_report(
            measurement_checks, [], [], [], False, reason,
            time.perf_counter() - started,
        )
        state = STATE_PREVALIDATION_FAILED
        update_instruction_labels()
        return False

    cache = load_valid_prevalidation_cache()
    if cache is not None and cache.get("precheck_kind") != "calibration_and_heldout":
        cache = None
    if cache is not None:
        quartz_friction_scale = float(cache["quartz_friction_scale"])
        glass_friction_scale = float(cache["glass_friction_scale"])
        model_prevalidation_passed = True
        model_frozen = True
        elapsed = time.perf_counter() - started
        reason = (
            "A previously passed validation record matched the exact model signature, "
            "so expensive 3D pilot calibration was not repeated. The measurement self-test "
            "was still run on this launch."
        )
        build_prevalidation_report(
            measurement_checks,
            cache.get("quartz_history", []),
            cache.get("glass_history", []),
            cache.get("heldout_summaries", []),
            True,
            reason,
            elapsed,
            cache_used=True,
        )
        state = STATE_READY
        prepare_fresh_trial()
        next_step_label.text = "SIMULATOR CHECK PASSED — READY TO RUN"
        next_step_label.color = color.green
        result_info_label.text = "This simulator version already passed its full checks."
        return True

    set_prevalidation_status(
        "RUNNING SIMULATOR DIAGNOSTICS",
        "Step 2 — bounded quartz sensitivity calibration",
    )
    quartz_ok, chosen_quartz, quartz_history = deterministic_calibration(
        "quartz",
        CALIBRATION_QUARTZ_CONDITIONS,
        QUARTZ_SCALE_CANDIDATES,
        1.0,
        1.0,
    )

    if not quartz_ok:
        if chosen_quartz is not None:
            quartz_friction_scale = chosen_quartz
        reason = (
            "The 3D quartz model could not meet both calibration accuracy and "
            "repeatability limits using the declared bounded shared-friction search."
        )
        build_prevalidation_report(
            measurement_checks, quartz_history, [], [], False, reason,
            time.perf_counter() - started,
        )
        state = STATE_PREVALIDATION_FAILED
        update_instruction_labels()
        return False

    quartz_friction_scale = chosen_quartz

    set_prevalidation_status(
        "RUNNING SIMULATOR DIAGNOSTICS",
        "Step 3 — bounded glass-bead sensitivity calibration",
    )
    glass_ok, chosen_glass, glass_history = deterministic_calibration(
        "glass",
        CALIBRATION_GLASS_CONDITIONS,
        GLASS_SCALE_CANDIDATES,
        quartz_friction_scale,
        1.0,
    )

    if not glass_ok:
        if chosen_glass is not None:
            glass_friction_scale = chosen_glass
        reason = (
            "The 3D glass-bead model could not meet both calibration accuracy and "
            "repeatability limits using the declared bounded shared-friction search."
        )
        build_prevalidation_report(
            measurement_checks, quartz_history, glass_history, [], False, reason,
            time.perf_counter() - started,
        )
        state = STATE_PREVALIDATION_FAILED
        update_instruction_labels()
        return False

    glass_friction_scale = chosen_glass
    model_frozen = True

    set_prevalidation_status(
        "RUNNING SIMULATOR DIAGNOSTICS",
        "Step 4 — testing the frozen model on held-out conditions",
    )
    heldout_ok, heldout_summaries = run_held_out_validation(
        quartz_friction_scale,
        glass_friction_scale,
    )

    if not heldout_ok:
        reason = (
            "The frozen 3D model failed held-out agreement, repeatability, settling, "
            "or orthogonal-profile consistency. Research trials were locked and held-out "
            "answers were not used to retune the model."
        )
        build_prevalidation_report(
            measurement_checks,
            quartz_history,
            glass_history,
            heldout_summaries,
            False,
            reason,
            time.perf_counter() - started,
        )
        state = STATE_PREVALIDATION_FAILED
        update_instruction_labels()
        return False

    model_prevalidation_passed = True
    elapsed = time.perf_counter() - started
    reason = (
        "Measurement self-tests passed, the bounded calibration conditions met the "
        "project criteria, and the frozen 3D model passed the held-out validation set."
    )

    cache_payload = {
        "passed": True,
        "precheck_kind": "calibration_and_heldout",
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "model_version": MODEL_VERSION,
        "model_signature": model_signature(),
        "quartz_friction_scale": quartz_friction_scale,
        "glass_friction_scale": glass_friction_scale,
        "quartz_history": quartz_history,
        "glass_history": glass_history,
        "heldout_summaries": heldout_summaries,
    }
    save_prevalidation_cache(cache_payload)

    build_prevalidation_report(
        measurement_checks,
        quartz_history,
        glass_history,
        heldout_summaries,
        True,
        reason,
        elapsed,
    )

    state = STATE_READY
    prepare_fresh_trial()
    next_step_label.text = "SIMULATOR CHECK PASSED — READY TO RUN"
    next_step_label.color = color.green
    result_info_label.text = (
        f"Simulator diagnostics completed in {elapsed:.1f} s."
    )
    return True


def build_integrity_precheck_report(measurement_checks, invariant_checks, pilots, passed, elapsed):
    """Write an auditable pre-check report without exposing UCF outcomes."""
    measurement_rows = "".join(
        "<tr>"
        f"<td>{item['expected']:.1f}°</td>"
        f"<td>{'—' if item['measured'] is None else format(item['measured'], '.2f') + '°'}</td>"
        f"<td>{'PASS' if item['passed'] else 'FAIL'}</td>"
        "</tr>"
        for item in measurement_checks
    )
    invariant_rows = "".join(
        "<tr>"
        f"<td>{item['name']}</td><td>{item['detail']}</td>"
        f"<td>{'PASS' if item['passed'] else 'FAIL'}</td>"
        "</tr>"
        for item in invariant_checks
    )
    pilot_rows = "".join(
        "<tr>"
        f"<td>{item['condition']}</td>"
        f"<td>{'—' if item['angle'] is None else format(item['angle'], '.2f') + '°'}</td>"
        f"<td>{'YES' if item['settled'] else 'NO'}</td>"
        f"<td>{'—' if item['particle_layers'] is None else format(item['particle_layers'], '.1f')}</td>"
        f"<td>{'—' if item['left_right_difference'] is None else format(item['left_right_difference'], '.2f') + '°'}</td>"
        f"<td>{'PASS' if item['precheck_passed'] else 'FAIL'}</td>"
        "</tr>"
        for item in pilots
    )
    status = "PASSED — FULL EXPERIMENT UNLOCKED" if passed else "FAILED — EXPERIMENT LOCKED"
    cls = "pass" if passed else "fail"
    html = f'''<!doctype html><html><head><meta charset="utf-8"><title>Simulator Diagnostic Report</title>
<style>body{{margin:0;background:#101216;color:#eef1f5;font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Arial,sans-serif}}main{{max-width:1120px;margin:auto;padding:36px 24px 70px}}.card{{background:#181b21;border:1px solid #2b3039;border-radius:12px;padding:20px;margin-top:22px}}table{{width:100%;border-collapse:collapse}}th,td{{text-align:left;padding:10px;border-bottom:1px solid #2b3039}}.note{{color:#aeb6c1;line-height:1.6}}.pass{{color:#9cf0ad}}.fail{{color:#ff9da5}}</style></head><body><main>
<h1>Angle of Repose Digital Twin — Simulator Diagnostic Report</h1>
<div class="card"><h2 class="{cls}">{status}</h2><p class="note">Completed in {elapsed:.1f} seconds. These diagnostics check measurement mathematics, amount normalization, shared-material rules, gravity-only release, settling, resolution, and valid pile geometry.</p></div>
<div class="card"><h2>Scientific basis</h2><p class="note"><b>The UCF angle measurements are not used by these diagnostics and cannot tune the physics.</b> They are revealed only in the final comparison report after research trials are saved. The acceptance bands come from published dry-granular behavior: glass-bead repose is commonly reported in the low-to-mid 20s and largely independent of bead size, while dry quartz sand varies from the high 20s into the 30s with particle shape, packing, moisture, and measurement method. Passing means the simulator is internally coherent and scientifically plausible; it does not guarantee agreement with the UCF study.</p><p class="note">References: <a href="https://agupubs.onlinelibrary.wiley.com/doi/10.1029/2008JB005999">Higashi &amp; Sumita (2009)</a>; <a href="https://academic.oup.com/mnras/article/498/1/1062/5893804">Sunday et al. (2020)</a>.</p></div>
<div class="card"><h2>Measurement mathematics</h2><table><thead><tr><th>Known synthetic angle</th><th>Measured</th><th>Status</th></tr></thead><tbody>{measurement_rows}</tbody></table></div>
<div class="card"><h2>Model invariants</h2><table><thead><tr><th>Check</th><th>Evidence</th><th>Status</th></tr></thead><tbody>{invariant_rows}</tbody></table></div>
<div class="card"><h2>Headless physics pilots</h2><table><thead><tr><th>Condition</th><th>Angle</th><th>Settled</th><th>Particle layers</th><th>X/Y difference</th><th>Status</th></tr></thead><tbody>{pilot_rows}</tbody></table></div>
</main></body></html>'''
    with open(PREVALIDATION_FILE, "w") as f:
        f.write(html)


def run_scientific_integrity_precheck():
    """Run a blinded, deterministic gate before any research trial."""
    global quartz_friction_scale
    global glass_friction_scale
    global model_prevalidation_passed
    global model_frozen
    global state

    started = time.perf_counter()
    state = STATE_PREVALIDATING
    quartz_friction_scale = 1.25
    glass_friction_scale = 1.0
    set_prevalidation_status(
        "RUNNING SIMULATOR DIAGNOSTICS",
        "Checking the measurement system",
    )
    measurement_checks = run_measurement_math_self_check()

    cache = load_valid_prevalidation_cache()
    if cache is not None and cache.get("precheck_kind") == "blinded_scientific_integrity":
        quartz_friction_scale = float(cache["quartz_friction_scale"])
        glass_friction_scale = float(cache["glass_friction_scale"])
        model_prevalidation_passed = True
        model_frozen = True
        prepare_fresh_trial()
        next_step_label.text = "SIMULATOR CHECK PASSED — READY TO RUN"
        next_step_label.color = color.green
        result_info_label.text = "The simulator is ready. Research comparison values were not used to set the physics."
        return True

    one_cup_volumes = [
        target_solid_volume(c) for c in CONDITIONS if c.get("amount_cups", 1) == 1
    ]
    spawn_plans = [build_spawn_plan(c, PRECHECK_SEED + i) for i, c in enumerate(CONDITIONS)]
    quartz_params = [material_parameters(c) for c in CONDITIONS if c["material_class"] == "quartz"]
    glass_params = [material_parameters(c) for c in CONDITIONS if c["material_class"] == "glass"]
    invariant_checks = [
        {
            "name": "Equal one-cup solid volume",
            "detail": f"{one_cup_volumes[0]:.6f} normalized units in every one-cup condition",
            "passed": max(one_cup_volumes) - min(one_cup_volumes) < 1e-12,
        },
        {
            "name": "Two cups means exactly twice the amount",
            "detail": f"{effective_particle_count(CONDITIONS[5])} particles and {target_solid_volume(CONDITIONS[5]):.6f} volume units",
            "passed": (
                effective_particle_count(CONDITIONS[5]) == 2 * effective_particle_count(CONDITIONS[4])
                and abs(target_solid_volume(CONDITIONS[5]) - 2 * target_solid_volume(CONDITIONS[4])) < 1e-12
            ),
        },
        {
            "name": "One shared quartz contact model",
            "detail": "All four quartz conditions use identical contact properties",
            "passed": all(q == quartz_params[0] for q in quartz_params[1:]),
        },
        {
            "name": "One shared glass contact model",
            "detail": "One-cup and two-cup beads use identical contact properties",
            "passed": all(q == glass_params[0] for q in glass_params[1:]),
        },
        {
            "name": "Gravity-only release",
            "detail": "Every planned particle begins with zero horizontal launch velocity",
            "passed": all(
                abs(entry["vx"]) < 1e-12 and abs(entry["vy"]) < 1e-12
                for plan in spawn_plans for entry in plan
            ),
        },
    ]

    pilots = []
    for idx, condition in enumerate(CONDITIONS):
        set_prevalidation_status(
            "RUNNING SIMULATOR DIAGNOSTICS",
            f"Testing condition {idx + 1} of {len(CONDITIONS)} — {condition['name']}",
        )
        angle_range = (
            PRECHECK_QUARTZ_ANGLE_RANGE_DEG
            if condition["material_class"] == "quartz"
            else PRECHECK_GLASS_ANGLE_RANGE_DEG
        )
        pilot = run_hidden_pilot(
            idx,
            PRECHECK_SEED + idx * 1009,
            quartz_friction_scale,
            glass_friction_scale,
        )
        pilot["precheck_passed"] = (
            pilot["angle"] is not None
            and angle_range[0] <= pilot["angle"] <= angle_range[1]
            and pilot["settled"]
            and pilot["resolution_adequate"]
            and pilot["left_right_difference"] is not None
            and pilot["left_right_difference"] <= PRECHECK_MAX_PROFILE_DIFFERENCE_DEG
        )
        pilots.append(pilot)

    quartz_angles = [p["angle"] for p in pilots[:4] if p["angle"] is not None]
    glass_angles = [p["angle"] for p in pilots[4:] if p["angle"] is not None]
    invariant_checks.extend([
        {
            "name": "Quartz size-class consistency",
            "detail": "Shared quartz model should vary moderately, not become four different materials",
            "passed": (
                len(quartz_angles) == 4
                and max(quartz_angles) - min(quartz_angles) <= PRECHECK_MAX_QUARTZ_SPREAD_DEG
            ),
        },
        {
            "name": "Glass amount independence",
            "detail": "Doubling material amount must not redefine the bead material angle",
            "passed": (
                len(glass_angles) == 2
                and abs(glass_angles[1] - glass_angles[0]) <= PRECHECK_MAX_GLASS_AMOUNT_EFFECT_DEG
            ),
        },
    ])

    passed = (
        all(item["passed"] for item in measurement_checks)
        and all(item["passed"] for item in invariant_checks)
        and all(item["precheck_passed"] for item in pilots)
    )
    elapsed = time.perf_counter() - started
    build_integrity_precheck_report(
        measurement_checks, invariant_checks, pilots, passed, elapsed
    )

    if not passed:
        model_prevalidation_passed = False
        model_frozen = False
        state = STATE_PREVALIDATION_FAILED
        update_instruction_labels()
        return False

    model_prevalidation_passed = True
    model_frozen = True
    save_prevalidation_cache({
        "passed": True,
        "precheck_kind": "blinded_scientific_integrity",
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "model_version": MODEL_VERSION,
        "model_signature": model_signature(),
        "quartz_friction_scale": quartz_friction_scale,
        "glass_friction_scale": 1.0,
    })
    prepare_fresh_trial()
    next_step_label.text = "SIMULATOR CHECK PASSED — READY TO RUN"
    next_step_label.color = color.green
    result_info_label.text = "The simulator is ready. Research comparison values were not used to set the physics."
    return True

# ==========================================================
# RESULTS / VALIDATION DATA
# ==========================================================

CSV_HEADER = [
    "timestamp",
    "condition",
    "simulation_trial",
    "seed",
    "physics_engine",
    "model_version",
    "simulation_dimensions",
    "profile_x_angle_deg",
    "profile_y_angle_deg",
    "simulated_average_deg",
    "radial_angle_diagnostic_deg",
    "physical_trial_1_deg",
    "physical_trial_2_deg",
    "physical_trial_3_deg",
    "physical_average_deg",
    "difference_sim_minus_physical_deg",
    "absolute_difference_deg",
    "percent_error",
    "prevalidation_passed",
    "frozen_material_friction_scale",
    "amount_cups",
    "reference_particle_count_3d",
    "effective_particle_count_3d",
    "normalized_solid_volume",
    "coarse_grain_particles_per_cup",
    "nominal_coarse_radius",
    "particle_size_distribution_model",
    "particle_size_distribution_assumption",
    "particle_collision_shape_model",
    "measurement_method",
    "orthogonal_profile_difference_deg",
    "lateral_friction",
    "floor_friction",
    "rolling_friction",
    "spinning_friction",
    "restitution",
    "base_radius",
    "radius_variation",
    "gravity",
    "physics_timestep",
    "settled_before_measurement",
    "simulated_seconds",
    "wall_clock_seconds",
    "average_physics_steps_per_second",
    "replay_frame_count",
    "raw_replay_file",
    "visual_replay_page",
]


def initialize_results_file():
    if os.path.exists(METADATA_FILE):
        try:
            with open(METADATA_FILE, "r", newline="") as f:
                reader = csv.reader(f)
                existing = next(reader, [])
        except Exception:
            existing = []

        if existing and existing != CSV_HEADER:
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            backup = f"02_OLD_DATA_BACKUP_{stamp}.csv"
            os.replace(METADATA_FILE, backup)

    if not os.path.exists(METADATA_FILE):
        with open(METADATA_FILE, "w", newline="") as f:
            csv.writer(f).writerow(CSV_HEADER)

    build_research_report()
    build_validation_report()


def read_metadata_rows():
    if not os.path.exists(METADATA_FILE):
        return []
    with open(METADATA_FILE, "r", newline="") as f:
        return list(csv.DictReader(f))


def calculate_validation_summary():
    rows = read_metadata_rows()
    grouped = {}
    lr_values = []

    for row in rows:
        angle = safe_float(row.get("simulated_average_deg"))
        if angle is not None:
            grouped.setdefault(row.get("condition", ""), []).append(angle)
        lr = safe_float(row.get("orthogonal_profile_difference_deg"))
        if lr is not None:
            lr_values.append(lr)

    condition_checks = []
    completed_errors = []
    completed_sds = []

    for condition in CONDITIONS:
        values = grouped.get(condition["name"], [])
        phys = physical_average(condition)
        sim_mean = mean(values) if values else None
        error = abs(sim_mean - phys) / phys * 100 if sim_mean is not None else None
        sd = statistics.stdev(values) if len(values) >= 3 else None

        if len(values) >= 3:
            completed_errors.append(error)
            completed_sds.append(sd)

        condition_checks.append({
            "name": condition["name"],
            "trial_count": len(values),
            "physical_average": phys,
            "simulation_average": sim_mean,
            "percent_error": error,
            "sd": sd,
        })

    complete = len(rows) >= TOTAL_EXPERIMENT_TRIALS
    mape = mean(completed_errors) if completed_errors else None
    worst = max(completed_errors) if completed_errors else None
    median_error = statistics.median(completed_errors) if completed_errors else None
    mean_lr = mean(lr_values) if lr_values else None

    if not complete:
        overall = "INCOMPLETE"
        cls = "neutral"
        msg = "This report summarizes the saved 3D simulation results and comparison against the physical experiment."
    else:
        repeatability_good = completed_sds and max(completed_sds) <= 4.0
        external_good = mape is not None and mape <= 10.0 and worst <= 20.0
        lr_good = mean_lr is not None and mean_lr <= 7.0
        if repeatability_good and external_good and lr_good:
            overall = "VALIDATED FOR THIS TEST SET"
            cls = "good"
            msg = "The frozen 3D model showed good internal repeatability and agreement with this physical test set."
        elif (mape is not None and mape > 20.0) or (completed_sds and max(completed_sds) > 6.0):
            overall = "LOW CONFIDENCE"
            cls = "poor"
            msg = "A major external-accuracy or repeatability problem was detected."
        else:
            overall = "USE WITH CAUTION"
            cls = "warning"
            msg = "One or more checks are not strong enough to call the model fully validated."

    return {
        "rows": rows,
        "condition_checks": condition_checks,
        "complete": complete,
        "saved_trial_count": len(rows),
        "completed_conditions": sum(1 for c in condition_checks if c["trial_count"] >= 3),
        "mape": mape,
        "median_error": median_error,
        "worst_error": worst,
        "mean_lr": mean_lr,
        "overall_status": overall,
        "overall_class": cls,
        "overall_message": msg,
    }


def build_validation_report():
    summary = calculate_validation_summary()
    condition_rows = []
    for c in summary["condition_checks"]:
        condition_rows.append(
            "<tr>"
            f"<td>{c['name']}</td>"
            f"<td>{c['trial_count']} / 3</td>"
            f"<td>{c['physical_average']:.2f}°</td>"
            f"<td>{'—' if c['simulation_average'] is None else format(c['simulation_average'], '.2f') + '°'}</td>"
            f"<td>{'—' if c['percent_error'] is None else format(c['percent_error'], '.2f') + '%'}</td>"
            f"<td>{'—' if c['sd'] is None else format(c['sd'], '.2f') + '°'}</td>"
            "</tr>"
        )

    mape = "—" if summary["mape"] is None else f"{summary['mape']:.2f}%"
    median = "—" if summary["median_error"] is None else f"{summary['median_error']:.2f}%"
    worst = "—" if summary["worst_error"] is None else f"{summary['worst_error']:.2f}%"
    mean_lr = "—" if summary["mean_lr"] is None else f"{summary['mean_lr']:.2f}°"

    html = f'''<!doctype html><html><head><meta charset="utf-8"><title>3D Simulation Validation and Accuracy</title>
<style>body{{margin:0;background:#101216;color:#eef1f5;font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Arial,sans-serif}}main{{max-width:1120px;margin:auto;padding:36px 24px 70px}}.card{{background:#181b21;border:1px solid #2b3039;border-radius:12px;padding:20px;margin-top:22px}}table{{width:100%;border-collapse:collapse}}th,td{{text-align:left;padding:10px;border-bottom:1px solid #2b3039}}.note{{color:#aeb6c1;line-height:1.6}}.good{{color:#9cf0ad}}.warning{{color:#f6e8a9}}.poor{{color:#ff9da5}}.neutral{{color:#d9dee6}}.grid{{display:grid;grid-template-columns:repeat(auto-fit,minmax(180px,1fr));gap:12px}}.metric{{background:#11151b;border:1px solid #29313c;border-radius:9px;padding:14px}}</style></head><body><main>
<h1>Simulation Validation and Accuracy — 3D PyBullet</h1>
<div class="card"><h2 class="{summary['overall_class']}">{summary['overall_status']}</h2><p class="note">{summary['overall_message']}</p><p class="note">{summary['saved_trial_count']} of {TOTAL_EXPERIMENT_TRIALS} trials saved.</p></div>
<div class="card"><h2>Key metrics</h2><div class="grid"><div class="metric"><b>Condition MAPE</b><br>{mape}</div><div class="metric"><b>Median condition error</b><br>{median}</div><div class="metric"><b>Worst condition error</b><br>{worst}</div><div class="metric"><b>Mean X/Y profile disagreement</b><br>{mean_lr}</div></div></div>
<div class="card"><h2>Condition-by-condition</h2><table><thead><tr><th>Condition</th><th>Trials</th><th>Physical mean</th><th>Simulation mean</th><th>Error</th><th>Simulation SD</th></tr></thead><tbody>{''.join(condition_rows)}</tbody></table></div>
<div class="card"><h2>Interpretation</h2><p class="note"><b>Internal quality</b> includes repeatability, settling, and orthogonal-profile consistency. <b>External validation</b> compares completed simulation means with the known physical experiment. The physical values do not directly control individual research trial results.</p></div>
<div class="card"><h2>Model limitation</h2><p class="note">This version uses true 3D rigid-body dynamics. Glass beads are smooth spherical rigid bodies. Quartz sand uses a rough-sphere DEM proxy with high sliding, rolling, and spinning resistance plus near-zero restitution so coarse particles dissipate impact energy like a granular bed rather than behaving like bouncing rocks. Passing this report validates the frozen model only for this specific test set, not for every granular material or apparatus.</p></div>
</main></body></html>'''
    with open(VALIDATION_FILE, "w") as f:
        f.write(html)


def _graph_data_from_rows(rows):
    grouped = {}
    for row in rows:
        value = safe_float(row.get("simulated_average_deg"))
        if value is not None:
            grouped.setdefault(row.get("condition", ""), []).append(value)

    data = []
    for condition in CONDITIONS:
        values = grouped.get(condition["name"], [])
        phys = physical_average(condition)
        sim = mean(values) if values else None
        error = abs(sim - phys) / phys * 100 if sim is not None else None
        data.append({
            "label": condition["name"],
            "physical": round(phys, 4),
            "simulation": None if sim is None else round(sim, 4),
            "percent_error": None if error is None else round(error, 4),
        })
    return data


def build_research_report():
    rows = read_metadata_rows()
    summary = calculate_validation_summary()
    graph_json = json.dumps(_graph_data_from_rows(rows))

    trial_rows = []
    for row in rows:
        replay = row.get("visual_replay_page", "")
        replay_cell = f'<a href="{replay.replace(os.sep, "/")}">Open replay</a>' if replay else "—"
        trial_rows.append(
            "<tr>"
            f"<td>{row.get('condition','')}</td>"
            f"<td>{row.get('simulation_trial','')}</td>"
            f"<td>{row.get('simulated_average_deg','')}°</td>"
            f"<td>{row.get('physical_average_deg','')}°</td>"
            f"<td>{row.get('percent_error','')}%</td>"
            f"<td>{replay_cell}</td>"
            "</tr>"
        )

    condition_rows = []
    for c in summary["condition_checks"]:
        condition_rows.append(
            "<tr>"
            f"<td>{c['name']}</td><td>{c['trial_count']} / 3</td>"
            f"<td>{c['physical_average']:.2f}°</td>"
            f"<td>{'—' if c['simulation_average'] is None else format(c['simulation_average'], '.2f') + '°'}</td>"
            f"<td>{'—' if c['percent_error'] is None else format(c['percent_error'], '.2f') + '%'}</td>"
            "</tr>"
        )

    progress = (
        "COMPLETE — all 18 required research trials are saved."
        if summary["complete"]
        else f"{summary['saved_trial_count']} of {TOTAL_EXPERIMENT_TRIALS} required research trials are saved."
    )

    html = '''<!doctype html><html><head><meta charset="utf-8"><title>3D Angle of Repose Research Report</title>
<style>body{margin:0;background:#101216;color:#eef1f5;font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Arial,sans-serif}main{max-width:1150px;margin:auto;padding:36px 24px 70px}.card{background:#181b21;border:1px solid #2b3039;border-radius:12px;padding:20px;margin-top:22px}table{width:100%;border-collapse:collapse}th,td{text-align:left;padding:10px;border-bottom:1px solid #2b3039}.note{color:#aeb6c1;line-height:1.6}canvas{display:block;width:100%;max-width:1040px;background:#11151b;border:1px solid #2f3540;border-radius:10px}a{color:#8ec5ff}</style></head><body><main>
<h1>Angle of Repose Research Report — 3D Physics</h1>
<p class="note">Physics is calculated in true 3D by PyBullet. VPython is the visual laboratory. Physical reference values are shown for comparison and validation.</p>
<div class="card"><h2>Progress</h2><p>__PROGRESS__</p><p><b>Reliability:</b> __STATUS__</p><p class="note">__MESSAGE__</p></div>
<div class="card"><h2>Graph 1 — Physical vs Simulation</h2><canvas id="comparison" width="1040" height="420"></canvas></div>
<div class="card"><h2>Graph 2 — Percent Error</h2><canvas id="error" width="1040" height="420"></canvas></div>
<div class="card"><h2>Condition summary</h2><table><thead><tr><th>Condition</th><th>Trials</th><th>Physical mean</th><th>Simulation mean</th><th>Error</th></tr></thead><tbody>__CONDITION_ROWS__</tbody></table></div>
<div class="card"><h2>Individual saved trials</h2><table><thead><tr><th>Condition</th><th>Trial</th><th>Simulation</th><th>Physical mean</th><th>Error</th><th>Replay</th></tr></thead><tbody>__TRIAL_ROWS__</tbody></table></div>
<div class="card"><h2>3D model notes</h2><p class="note"><b>Two cups:</b> twice the 3D material count is used, so extra material can spread in depth instead of being forced upward in a 2D plane.<br><br><b>Unsieved quartz:</b> broad particle sizes are normalized by total sphere volume, not 2D area.<br><br><b>Quartz vs glass:</b> both use stable spherical contact geometry. Quartz uses more, smaller representative grains with moderate sliding/rolling resistance and low restitution to represent dry non-cohesive sand; glass beads remain smoother and more mobile. Each class has one shared contact model. The simulator diagnostics use published behavior bands and do not read or tune against the UCF measurements.<br><br><b>Replay:</b> raw replay data stores X/Y/Z particle coordinates. Browser replay pages show a side projection, while the in-app replay reconstructs the saved 3D positions.</p></div>
<script>
const data=__GRAPH_DATA__;
function draw(id,series,startZero){const c=document.getElementById(id),ctx=c.getContext('2d'),W=c.width,H=c.height,L=70,R=W-25,T=25,B=H-85;let vals=[];series.forEach(s=>s.values.forEach(v=>{if(v!==null)vals.push(v)}));ctx.fillStyle='#11151b';ctx.fillRect(0,0,W,H);if(!vals.length){ctx.fillStyle='#eee';ctx.fillText('No data yet',30,40);return;}let mn=Math.min(...vals),mx=Math.max(...vals);if(startZero)mn=0;let pad=Math.max(1,(mx-mn)*.1);mn=Math.max(0,mn-pad);mx+=pad;if(mx<=mn)mx=mn+1;const xp=i=>L+(R-L)*(i/(data.length-1));const yp=v=>B-(v-mn)/(mx-mn)*(B-T);ctx.strokeStyle='#29313c';ctx.fillStyle='#c9d1d9';ctx.font='12px Arial';for(let t=0;t<=5;t++){let v=mn+(mx-mn)*t/5,y=yp(v);ctx.beginPath();ctx.moveTo(L,y);ctx.lineTo(R,y);ctx.stroke();ctx.fillText(v.toFixed(1),15,y+4)}series.forEach(s=>{ctx.strokeStyle=s.color;ctx.fillStyle=s.color;ctx.lineWidth=3;ctx.beginPath();let started=false;s.values.forEach((v,i)=>{if(v===null){started=false;return;}let x=xp(i),y=yp(v);if(!started){ctx.moveTo(x,y);started=true}else ctx.lineTo(x,y)});ctx.stroke();s.values.forEach((v,i)=>{if(v===null)return;let x=xp(i),y=yp(v);ctx.beginPath();ctx.arc(x,y,5,0,Math.PI*2);ctx.fill()})});ctx.fillStyle='#d9dfe6';ctx.font='11px Arial';data.forEach((d,i)=>{let words=d.label.split(' '),lines=[],cur='';words.forEach(w=>{let cand=cur?cur+' '+w:w;if(cand.length<=14)cur=cand;else{if(cur)lines.push(cur);cur=w}});if(cur)lines.push(cur);lines.forEach((line,j)=>{let w=ctx.measureText(line).width;ctx.fillText(line,xp(i)-w/2,B+18+j*13)})})}
draw('comparison',[{color:'#f0b04e',values:data.map(d=>d.physical)},{color:'#6fb6ff',values:data.map(d=>d.simulation)}],false);draw('error',[{color:'#8ce99a',values:data.map(d=>d.percent_error)}],true);
</script></main></body></html>'''

    html = (
        html.replace("__PROGRESS__", progress)
        .replace("__STATUS__", summary["overall_status"])
        .replace("__MESSAGE__", summary["overall_message"])
        .replace("__CONDITION_ROWS__", "".join(condition_rows))
        .replace("__TRIAL_ROWS__", "".join(trial_rows))
        .replace("__GRAPH_DATA__", graph_json)
    )

    with open(REPORT_FILE, "w") as f:
        f.write(html)

# ==========================================================
# REPLAY FILES
# ==========================================================


def record_replay_frame(gate_x=None, material_height=None):
    if gate_x is None:
        gate_x = release_plate.pos.x
    if material_height is None:
        material_height = material_column.axis.y

    replay_frames.append({
        "gate_x": round(gate_x, 6),
        "material_height": round(material_height, 6),
        "particles": particle_snapshot(),
    })


def _safe_condition_name(name):
    return (
        name.replace(" ", "_")
        .replace(">", "GT")
        .replace("-", "_")
        .replace("μ", "u")
        .lower()
    )


def save_replay_file(simulated_angle):
    safe_name = _safe_condition_name(current_condition()["name"])
    filename = (
        f"RAW_EXACT_3D_REPLAY__{safe_name}__trial_{trial_number}__seed_{trial_seed}.json"
    )
    path = os.path.join(RAW_REPLAY_FOLDER, filename)
    payload = {
        "physics_engine": PHYSICS_ENGINE_NAME,
        "model_version": MODEL_VERSION,
        "coordinate_system": "PyBullet x/y horizontal, z vertical",
        "condition": current_condition()["name"],
        "trial": trial_number,
        "seed": trial_seed,
        "simulated_angle": simulated_angle,
        "replay_fps": REPLAY_FPS,
        "frames": replay_frames,
    }
    with open(path, "w") as f:
        json.dump(payload, f)
    return path


def create_visual_replay(raw_replay_path):
    with open(raw_replay_path, "r") as f:
        data = json.load(f)

    safe_name = _safe_condition_name(data["condition"])
    visual_filename = f"WATCH_3D_REPLAY__{safe_name}__trial_{data['trial']}.html"
    visual_path = os.path.join(VISUAL_REPLAY_FOLDER, visual_filename)

    frames_json = json.dumps(data["frames"])
    condition_json = json.dumps(data["condition"])
    angle_json = json.dumps(round(data["simulated_angle"], 2))

    html = r'''<!doctype html><html><head><meta charset="utf-8"><title>Saved 3D Simulation Replay</title>
<style>body{margin:0;background:#0f1115;color:#eef1f5;font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Arial,sans-serif}main{max-width:980px;margin:auto;padding:28px 20px 50px}canvas{display:block;width:100%;max-width:900px;aspect-ratio:3/2;background:#15181e;border:1px solid #343a45;border-radius:12px}.controls{margin-top:16px;display:flex;gap:10px;flex-wrap:wrap}button{background:#242a33;color:white;border:1px solid #3b4350;border-radius:8px;padding:10px 16px}.note,.status{color:#aeb6c1;line-height:1.5}</style></head><body><main><h1>Saved 3D Simulation Replay</h1><p id="subtitle" class="note"></p><canvas id="canvas" width="900" height="600"></canvas><div class="controls"><button onclick="play()">Play</button><button onclick="pause()">Pause</button><button onclick="restart()">Restart</button><button onclick="exportVideo()">Export WebM Video</button></div><p id="status" class="status"></p><p class="note">The raw replay stores exact X/Y/Z particle positions. This browser page shows the same saved trial as a side projection so it can be viewed without a 3D engine.</p></main><script>
const frames=__FRAMES__,condition=__CONDITION__,resultAngle=__ANGLE__,fps=30;const canvas=document.getElementById('canvas'),ctx=canvas.getContext('2d');document.getElementById('subtitle').textContent=condition+' | Saved angle: '+resultAngle.toFixed(2)+'°';let i=0,timer=null;function sx(x){return (x+3.6)/7.2*canvas.width}function sy(z){return canvas.height-(z/8.4)*canvas.height}function drawFrame(n){if(!frames.length)return;const f=frames[Math.max(0,Math.min(n,frames.length-1))];ctx.fillStyle='#15181e';ctx.fillRect(0,0,canvas.width,canvas.height);ctx.strokeStyle='#83a7b8';ctx.lineWidth=3;ctx.strokeRect(sx(-1.72),sy(3.60),sx(1.72)-sx(-1.72),sy(.55)-sy(3.60));ctx.strokeStyle='#888d95';ctx.lineWidth=6;ctx.beginPath();ctx.moveTo(sx(-1.62),sy(.80));ctx.lineTo(sx(1.62),sy(.80));ctx.stroke();const sorted=[...f.particles].sort((a,b)=>a.y-b.y);for(const q of sorted){let depth=Math.max(-1,Math.min(1,q.y/1.55));let alpha=.48+.40*(depth+1)/2;ctx.beginPath();ctx.arc(sx(q.x),sy(q.z),Math.max(2,q.r/7.2*canvas.width),0,Math.PI*2);ctx.fillStyle=condition.includes('Glass')?`rgba(168,205,234,${alpha})`:`rgba(216,173,93,${alpha})`;ctx.fill()}document.getElementById('status').textContent='Frame '+(n+1)+' of '+frames.length}function play(){if(timer)return;timer=setInterval(()=>{if(i>=frames.length){pause();return}drawFrame(i++)},1000/fps)}function pause(){if(timer){clearInterval(timer);timer=null}}function restart(){pause();i=0;drawFrame(0)}async function exportVideo(){pause();if(!canvas.captureStream||!window.MediaRecorder){alert('Video export is not supported by this browser.');return}i=0;const stream=canvas.captureStream(fps);let rec;try{rec=new MediaRecorder(stream,{mimeType:'video/webm'})}catch(e){rec=new MediaRecorder(stream)}let chunks=[];rec.ondataavailable=e=>{if(e.data.size)chunks.push(e.data)};rec.onstop=()=>{let blob=new Blob(chunks,{type:'video/webm'}),url=URL.createObjectURL(blob),a=document.createElement('a');a.href=url;a.download=condition.replace(/[^a-z0-9]+/gi,'_')+'_3D_replay.webm';a.click();URL.revokeObjectURL(url)};rec.start();let n=0;let ex=setInterval(()=>{if(n>=frames.length){clearInterval(ex);rec.stop();return}drawFrame(n++)},1000/fps)}drawFrame(0);
</script></body></html>'''

    html = (
        html.replace("__FRAMES__", frames_json)
        .replace("__CONDITION__", condition_json)
        .replace("__ANGLE__", angle_json)
    )
    with open(visual_path, "w") as f:
        f.write(html)
    return visual_path

# ==========================================================
# FRESH TRIAL / VISIBLE 3D SIMULATION
# ==========================================================


def prepare_fresh_trial():
    global state
    global trial_seed
    global replay_frames
    global current_trial_settled
    global current_trial_simulated_seconds
    global current_trial_wall_seconds
    global abort_current_run

    clear_visual_particles()
    clear_measurements()
    hide_replay_objects()
    try:
        p.resetSimulation(physicsClientId=physics_client)
    except Exception:
        pass

    trial_seed = None
    replay_frames = []
    current_trial_settled = False
    current_trial_simulated_seconds = 0.0
    current_trial_wall_seconds = 0.0
    abort_current_run = False
    if "replay_player" in globals():
        replay_player["playing"] = False

    release_plate.pos = vector(0, 5.10, 0)
    material_column.axis = vector(0, 1.85, 0)
    update_material_preview()
    update_saved_result_label()
    state = STATE_READY
    update_instruction_labels()


def start_trial(evt=None):
    global state
    global trial_seed
    global particles
    global current_trial_settled
    global current_trial_simulated_seconds
    global abort_current_run
    global current_trial_wall_seconds

    if state == STATE_PREVALIDATING:
        show_diagnostics_still_running()
        return

    if state != STATE_READY:
        return

    abort_current_run = False
    replay_player["playing"] = False
    clear_measurements()
    clear_visual_particles()
    hide_replay_objects()
    replay_frames.clear()

    condition = current_condition()
    params = configure_pybullet_world(condition)
    trial_seed = random.SystemRandom().randint(1, 2_000_000_000)
    plan = build_spawn_plan(condition, trial_seed)
    shape_cache = {}
    particles = []

    release_clock = 0.0
    release_interval = effective_release_interval(condition)
    next_index = 0
    simulation_time = 0.0
    all_released_time = None
    stable_time = 0.0
    visual_frame = 0
    wall_start = time.perf_counter()

    settle_max = (
        SETTLE_MAX_TIME_GLASS
        if condition["material_class"] == "glass"
        else SETTLE_MAX_TIME_QUARTZ
    )

    state = STATE_RUNNING
    update_instruction_labels()
    record_replay_frame()

    while True:
        speed_multiplier = SIMULATION_SPEEDS[simulation_speed_name]
        rate(VISUAL_FPS * speed_multiplier)
        visual_frame += 1

        if abort_current_run:
            prepare_fresh_trial()
            return

        for _ in range(PHYSICS_STEPS_PER_VISUAL_FRAME):
            release_clock += PHYSICS_DT

            # CONTROLLED GRAVITY FEED:
            # Fast enough to look like a normal pour, but still protected
            # against overlapping spawn bodies. Horizontal launch is zero.
            spawned_this_step = 0

            while (
                release_clock >= release_interval
                and next_index < len(plan)
                and spawned_this_step < MAX_SAFE_SPAWNS_PER_PHYSICS_STEP
            ):

                clear_index = find_clear_spawn_index(
                    plan,
                    next_index
                )

                if clear_index is None:
                    # Never build up a huge release backlog.
                    release_clock = min(
                        release_clock,
                        release_interval
                    )
                    break

                if clear_index != next_index:
                    plan[next_index], plan[clear_index] = (
                        plan[clear_index],
                        plan[next_index],
                    )

                particles.append(
                    create_particle_body(
                        condition,
                        params,
                        plan[next_index],
                        visible=True,
                        shape_cache=shape_cache,
                    )
                )

                next_index += 1
                spawned_this_step += 1
                release_clock = max(
                    0.0,
                    release_clock - release_interval
                )

            p.stepSimulation(physicsClientId=physics_client)
            simulation_time += PHYSICS_DT

            if next_index == len(plan) and all_released_time is None:
                all_released_time = simulation_time
                state = STATE_SETTLING
                update_instruction_labels()

        if all_released_time is not None:
            max_linear, max_angular = current_settling_metrics()
            if (
                max_linear <= SETTLE_LINEAR_SPEED
                and max_angular <= SETTLE_ANGULAR_SPEED
            ):
                stable_time += PHYSICS_DT * PHYSICS_STEPS_PER_VISUAL_FRAME
            else:
                stable_time = 0.0

        # Visual gate and material preview.
        release_plate.pos.x = min(4.0, release_plate.pos.x + 0.09)
        remaining = (len(plan) - next_index) / max(1, len(plan))
        material_column.axis = vector(0, max(0.01, 1.85 * remaining), 0)

        # VPython object updates are much more expensive than the PyBullet
        # math. Updating the browser at ~15 fps keeps the interface responsive.
        if (
            visual_frame
            % (DISPLAY_UPDATE_EVERY_VISUAL_FRAME * speed_multiplier)
            == 0
        ):
            sync_visual_particles()

        if visual_frame % REPLAY_RECORD_EVERY_VISUAL_FRAME == 0:
            record_replay_frame()

        if all_released_time is not None:
            elapsed_settling = simulation_time - all_released_time
            if (
                elapsed_settling >= SETTLE_MIN_TIME
                and stable_time >= SETTLE_REQUIRED_STABLE_TIME
            ):
                current_trial_settled = True
                break
            if elapsed_settling >= settle_max:
                current_trial_settled = False
                break

    current_trial_simulated_seconds = simulation_time
    current_trial_wall_seconds = time.perf_counter() - wall_start
    sync_visual_particles()
    record_replay_frame()

    # No automatic pre-validation gate in this development build.
    # Once the run has finished its settling window, return to the original
    # simple workflow and allow measurement.
    state = STATE_READY_TO_MEASURE
    next_step_label.text = "STEP 2: CLICK MEASURE + SAVE RESULT"
    next_step_label.color = color.yellow

    if not current_trial_settled:
        trial_info_label.text = (
            f"CURRENT: {current_condition()['name']}   |   "
            f"Trial {trial_number} of 3   |   Settling window complete"
        )


def reset_unsaved_trial(evt=None):
    global abort_current_run
    global full_experiment_active

    if state in [STATE_SAVED, STATE_COMPLETE, STATE_PREVALIDATING, STATE_PREVALIDATION_FAILED]:
        return

    if state in [STATE_RUNNING, STATE_SETTLING]:
        full_experiment_active = False
        abort_current_run = True
        next_step_label.text = "PREPARING THIS TRIAL AGAIN..."
        return

    full_experiment_active = False
    prepare_fresh_trial()

# ==========================================================
# SAVE RESULT
# ==========================================================


def save_result(profile_x_angle, profile_y_angle, simulated_average, radial_diagnostic_angle):
    condition = current_condition()
    phys_trials = condition["physical_trials"]
    phys_avg = physical_average(condition)
    difference = simulated_average - phys_avg
    abs_difference = abs(difference)
    percent_error = abs_difference / phys_avg * 100

    raw_replay_file = save_replay_file(simulated_average)
    visual_replay_file = create_visual_replay(raw_replay_file)
    params = material_parameters(condition)

    result = {
        "condition": condition["name"],
        "trial": trial_number,
        "seed": trial_seed,
        "profile_x_angle": profile_x_angle,
        "profile_y_angle": profile_y_angle,
        "simulated_average": simulated_average,
        "radial_diagnostic_angle": radial_diagnostic_angle,
        "physical_average": phys_avg,
        "difference": difference,
        "absolute_difference": abs_difference,
        "percent_error": percent_error,
        "replay_file": raw_replay_file,
        "visual_replay_file": visual_replay_file,
        "wall_seconds": current_trial_wall_seconds,
        "particle_count": len(particles),
        "replay_frames": len(replay_frames),
    }
    session_results.append(result)

    row = [
        datetime.now().isoformat(timespec="seconds"),
        condition["name"],
        trial_number,
        trial_seed,
        PHYSICS_ENGINE_NAME,
        MODEL_VERSION,
        3,
        round(profile_x_angle, 6),
        round(profile_y_angle, 6),
        round(simulated_average, 6),
        round(radial_diagnostic_angle, 6) if radial_diagnostic_angle is not None else "",
        phys_trials[0],
        phys_trials[1],
        phys_trials[2],
        round(phys_avg, 6),
        round(difference, 6),
        round(abs_difference, 6),
        round(percent_error, 6),
        model_prevalidation_passed,
        round(current_material_scale(), 6),
        condition.get("amount_cups", 1),
        condition["particle_count"],
        effective_particle_count(condition),
        round(target_solid_volume(condition), 8),
        particles_per_cup_for_condition(condition),
        round(nominal_coarse_radius(condition), 8),
        condition.get("size_distribution_model", "narrow_uniform"),
        condition.get("size_distribution_assumption", ""),
        (
            "rough_sphere_relaxing_dry_sand"
            if condition["material_class"] == "quartz"
            else "sphere"
        ),
        "Primary: supported-foot to robust-apex ruler geometry in X/Y side projections; isolated outer runout grains are excluded; radial 3D fit retained as diagnostic",
        round(abs(profile_x_angle - profile_y_angle), 6),
        params["lateral_friction"],
        params["floor_friction"],
        params["rolling_friction"],
        params["spinning_friction"],
        params["restitution"],
        condition["base_radius"],
        condition["radius_variation"],
        GRAVITY,
        PHYSICS_DT,
        current_trial_settled,
        round(current_trial_simulated_seconds, 6),
        round(current_trial_wall_seconds, 6),
        round(current_trial_simulated_seconds / current_trial_wall_seconds, 3) if current_trial_wall_seconds else "",
        len(replay_frames),
        raw_replay_file,
        visual_replay_file,
    ]

    with open(METADATA_FILE, "a", newline="") as f:
        csv.writer(f).writerow(row)

    build_research_report()
    build_validation_report()
    return result


def measure_and_save(evt=None):
    global state

    if state != STATE_READY_TO_MEASURE:
        return

    # Diagnostic mode intentionally permits measurement even if the
    # strict settling gate failed. The CSV records that status.
    clear_measurements()
    measured = calculate_angle(draw=True)
    if measured is None:
        next_step_label.text = "TRIAL COULD NOT BE MEASURED — EXPERIMENT STOPPED TO AVOID BIAS"
        next_step_label.color = color.red
        trial_info_label.text = (
            "No central pile with a defensible stable-face measurement was found. "
            "No result was recorded and the program will not retry automatically."
        )
        return

    profile_x_angle, profile_y_angle, simulated_average, radial_diagnostic = measured
    saved = save_result(
        profile_x_angle,
        profile_y_angle,
        simulated_average,
        radial_diagnostic,
    )
    update_saved_result_label(saved)

    if is_final_trial():
        state = STATE_COMPLETE
    else:
        state = STATE_SAVED
    update_instruction_labels()
    library = build_replay_library()
    if library:
        replay_player["library_index"] = len(library) - 1
        replay_player["data"] = None
        replay_status_text.text = (
            f"New replay ready · {saved['condition']} · Trial {saved['trial']} · "
            f"{saved['simulated_average']:.2f}° · Press ▶ PLAY"
        )

# ==========================================================
# NEXT TRIAL
# ==========================================================


def next_trial(evt=None):
    global condition_index
    global trial_number
    global state
    global full_experiment_active

    if state != STATE_SAVED:
        return

    # Clicking the manual Next button exits the guided full-experiment mode.
    # The guided Continue button calls this function without an event.
    if evt is not None:
        full_experiment_active = False

    if trial_number < TRIALS_PER_CONDITION:
        trial_number += 1
    else:
        trial_number = 1
        condition_index += 1

    if condition_index >= len(CONDITIONS):
        state = STATE_COMPLETE
        clear_visual_particles()
        clear_measurements()
        hide_replay_objects()
        update_instruction_labels()
        return

    prepare_fresh_trial()

# ==========================================================
# IN-APP EXACT 3D REPLAY
# ==========================================================


replay_player = {
    "library": [],
    "library_index": None,
    "data": None,
    "frame_index": 0,
    "playing": False,
    "speed": 1.0,
    "last_tick": None,
    "accumulator": 0.0,
    "updating_slider": False,
}


def build_replay_library():
    """Load every valid saved replay listed in the research data file."""
    entries = []
    seen = set()

    if os.path.exists(METADATA_FILE):
        try:
            with open(METADATA_FILE, "r", newline="") as f:
                for row in csv.DictReader(f):
                    path = row.get("raw_replay_file", "")
                    if not path or not os.path.exists(path):
                        continue
                    canonical = os.path.abspath(path)
                    if canonical in seen:
                        continue
                    seen.add(canonical)
                    entries.append({
                        "path": path,
                        "condition": row.get("condition", "Saved trial"),
                        "trial": int(float(row.get("simulation_trial", 0) or 0)),
                        "angle": safe_float(row.get("simulated_average_deg")),
                        "result": None,
                    })
        except Exception:
            pass

    # Session results are included even if a user moved the CSV while the app
    # was open. Deduplication keeps guided and manual saves in one timeline.
    for result in session_results:
        path = result.get("replay_file", "")
        if not path or not os.path.exists(path):
            continue
        canonical = os.path.abspath(path)
        if canonical in seen:
            for entry in entries:
                if os.path.abspath(entry["path"]) == canonical:
                    entry["result"] = result
                    break
            continue
        seen.add(canonical)
        entries.append({
            "path": path,
            "condition": result["condition"],
            "trial": result["trial"],
            "angle": result["simulated_average"],
            "result": result,
        })

    replay_player["library"] = entries
    return entries


def replay_is_safe_to_open():
    if state == STATE_PREVALIDATING:
        show_diagnostics_still_running()
        return False
    if state not in [STATE_READY, STATE_SAVED, STATE_COMPLETE]:
        next_step_label.text = "WAIT FOR THE CURRENT TRIAL TO FINISH BEFORE OPENING REPLAYS"
        next_step_label.color = color.yellow
        return False
    return True


def update_replay_status(prefix=None):
    library = replay_player["library"]
    index = replay_player["library_index"]
    data = replay_player["data"]
    if not library or index is None or data is None:
        replay_status_text.text = "No replays yet. Complete the first trial to create one."
        return

    entry = library[index]
    frames = data.get("frames", [])
    frame_number = min(replay_player["frame_index"] + 1, len(frames)) if frames else 0
    state_text = prefix or ("Playing" if replay_player["playing"] else "Paused")
    angle_text = "" if entry["angle"] is None else f" — {entry['angle']:.2f}°"
    replay_status_text.text = (
        f"{state_text} · Replay {index + 1} of {len(library)} · "
        f"{entry['condition']} · Trial {entry['trial']}{angle_text} · "
        f"Frame {frame_number} of {len(frames)}"
    )


def render_replay_frame(frame_index):
    data = replay_player["data"]
    if data is None:
        return
    frames = data.get("frames", [])
    if not frames:
        update_replay_status("Replay contains no frames")
        return

    frame_index = max(0, min(int(frame_index), len(frames) - 1))
    replay_player["frame_index"] = frame_index
    frame = frames[frame_index]
    release_plate.pos.x = frame.get("gate_x", 0)
    material_column.axis = vector(0, frame.get("material_height", 0.01), 0)
    plist = frame.get("particles", [])
    condition = next(c for c in CONDITIONS if c["name"] == data["condition"])

    while len(replay_objects) < len(plist):
        replay_objects.append(
            sphere(
                pos=vector(0, 0, 0),
                radius=0.05,
                color=material_color(condition),
            )
        )

    for i, q in enumerate(plist):
        obj = replay_objects[i]
        obj.visible = True
        obj.radius = q["r"]
        obj.pos = vector(q["x"], q["z"], q["y"])
    for i in range(len(plist), len(replay_objects)):
        replay_objects[i].visible = False

    replay_player["updating_slider"] = True
    replay_timeline.value = (
        0
        if len(frames) <= 1
        else 1000 * frame_index / (len(frames) - 1)
    )
    replay_player["updating_slider"] = False
    update_replay_status()


def select_saved_replay(index, autoplay=False):
    if not replay_is_safe_to_open():
        return False
    library = build_replay_library()
    if not library:
        update_replay_status()
        next_step_label.text = "NO REPLAYS YET — COMPLETE THE FIRST TRIAL"
        next_step_label.color = color.yellow
        return False

    index = max(0, min(int(index), len(library) - 1))
    entry = library[index]
    try:
        with open(entry["path"], "r") as f:
            data = json.load(f)
    except Exception:
        next_step_label.text = "THIS REPLAY COULD NOT BE OPENED"
        next_step_label.color = color.red
        return False

    replay_player.update({
        "library_index": index,
        "data": data,
        "frame_index": 0,
        "playing": bool(autoplay),
        "last_tick": time.perf_counter(),
        "accumulator": 0.0,
    })
    clear_visual_particles()
    clear_measurements()
    hide_replay_objects()
    if entry.get("result") is not None:
        update_saved_result_label(entry["result"])
    else:
        result_info_label.text = (
            f"RESULT: {entry['condition']} — Trial {entry['trial']}"
            + ("" if entry["angle"] is None else f" — {entry['angle']:.2f}°")
        )
    render_replay_frame(0)
    next_step_label.text = "REPLAY PLAYER READY — USE THE CONTROLS BELOW"
    next_step_label.color = color.cyan
    return True


def replay_saved_trial(evt=None):
    """Play the selected replay, or the newest saved trial if none is selected."""
    library = build_replay_library()
    if not library:
        update_replay_status()
        next_step_label.text = "NO REPLAYS YET — COMPLETE THE FIRST TRIAL"
        next_step_label.color = color.yellow
        return
    index = replay_player["library_index"]
    if index is None or replay_player["data"] is None:
        select_saved_replay(len(library) - 1, autoplay=True)
        return
    frames = replay_player["data"].get("frames", [])
    if frames and replay_player["frame_index"] >= len(frames) - 1:
        replay_player["frame_index"] = 0
        render_replay_frame(0)
    replay_player["playing"] = True
    replay_player["last_tick"] = time.perf_counter()
    replay_player["accumulator"] = 0.0
    update_replay_status("Playing")


def pause_replay(evt=None):
    replay_player["playing"] = False
    update_replay_status("Paused")


def restart_replay(evt=None):
    library = build_replay_library()
    if not library:
        update_replay_status()
        return
    index = replay_player["library_index"]
    if index is None or replay_player["data"] is None:
        select_saved_replay(len(library) - 1, autoplay=True)
        return
    replay_player["playing"] = True
    replay_player["last_tick"] = time.perf_counter()
    replay_player["accumulator"] = 0.0
    render_replay_frame(0)
    update_replay_status("Replaying")


def previous_saved_replay(evt=None):
    library = build_replay_library()
    if not library:
        update_replay_status()
        return
    index = replay_player["library_index"]
    select_saved_replay((len(library) - 1) if index is None else index - 1)


def next_saved_replay(evt=None):
    library = build_replay_library()
    if not library:
        update_replay_status()
        return
    index = replay_player["library_index"]
    select_saved_replay((len(library) - 1) if index is None else index + 1)


def scrub_replay(evt=None):
    if replay_player["updating_slider"] or replay_player["data"] is None:
        return
    value = getattr(evt, "value", replay_timeline.value)
    frames = replay_player["data"].get("frames", [])
    if not frames:
        return
    frame_index = int(round((float(value) / 1000.0) * (len(frames) - 1)))
    render_replay_frame(frame_index)


def change_universal_speed(evt=None):
    """Set one speed for both live trials and replay playback."""
    global simulation_speed_name

    selected = getattr(evt, "selected", universal_speed_menu.selected)
    simulation_speed_name, replay_player["speed"] = {
        "1× Normal": ("Normal", 1.0),
        "2× Fast": ("Fast", 2.0),
        "4× Maximum": ("Maximum", 4.0),
    }.get(selected, ("Normal", 1.0))
    update_instruction_labels()
    update_replay_status()


def close_replay_player(evt=None):
    replay_player["playing"] = False
    hide_replay_objects()
    release_plate.pos = vector(0, 5.10, 0)
    material_column.axis = vector(0, 1.85, 0)
    update_material_preview()
    update_instruction_labels()
    update_replay_status("Closed")


def replay_tick():
    """Advance the non-blocking video-style player from the main UI loop."""
    if not replay_player["playing"] or replay_player["data"] is None:
        return
    frames = replay_player["data"].get("frames", [])
    if not frames:
        replay_player["playing"] = False
        update_replay_status("Replay contains no frames")
        return

    now = time.perf_counter()
    last = replay_player["last_tick"] or now
    replay_player["last_tick"] = now
    replay_player["accumulator"] += (now - last) * replay_player["speed"]
    frame_interval = 1.0 / max(1, replay_player["data"].get("replay_fps", REPLAY_FPS))

    while replay_player["accumulator"] >= frame_interval:
        replay_player["accumulator"] -= frame_interval
        next_index = replay_player["frame_index"] + 1
        if next_index >= len(frames):
            replay_player["playing"] = False
            render_replay_frame(len(frames) - 1)
            update_replay_status("Finished")
            return
        render_replay_frame(next_index)


def stop_replay(evt=None):
    close_replay_player(evt)


def set_simulation_speed(name):
    """Change the target wall-clock rate without changing physics steps."""
    global simulation_speed_name

    if name not in SIMULATION_SPEEDS:
        return
    simulation_speed_name = name
    update_instruction_labels()


def set_speed_normal(evt=None):
    set_simulation_speed("Normal")


def set_speed_fast(evt=None):
    set_simulation_speed("Fast")


def set_speed_maximum(evt=None):
    set_simulation_speed("Maximum")


def _run_and_save_current_full_trial():
    """Run, measure, and record one complete trial automatically."""
    global full_experiment_active

    if state != STATE_READY:
        return

    start_trial()

    if state == STATE_READY_TO_MEASURE:
        measure_and_save()
        if state == STATE_READY_TO_MEASURE:
            # Preserve failed geometry instead of retrying until a favorable
            # answer appears. Manual recovery controls are immediately usable.
            full_experiment_active = False
            next_step_label.text = "TRIAL INVALID — EXPERIMENT STOPPED TO AVOID RETRY BIAS"
            next_step_label.color = color.red
            return

    if state == STATE_COMPLETE:
        full_experiment_active = False
        build_research_report()
        build_validation_report()
        next_step_label.text = "ALL 18 TRIALS COMPLETE — REPORTS READY"
        next_step_label.color = color.green
        return

    if state == STATE_SAVED:
        update_instruction_labels()
        trial_info_label.text = (
            f"TRIAL {current_overall_trial_number()} OF {TOTAL_EXPERIMENT_TRIALS} COMPLETE — "
            "REVIEW IT BELOW OR CLICK NEXT TRIAL"
        )


def run_full_experiment(evt=None):
    """Run the current trial automatically from release through result."""
    global full_experiment_active

    if not model_prevalidation_passed:
        if state == STATE_PREVALIDATING:
            show_diagnostics_still_running()
        else:
            next_step_label.text = "SIMULATOR CHECK FOUND A PROBLEM — EXPERIMENT PAUSED"
            next_step_label.color = color.red
        return
    if state != STATE_READY:
        return

    full_experiment_active = True
    _run_and_save_current_full_trial()


def continue_full_experiment(evt=None):
    """Prepare the next trial without starting it."""
    if not full_experiment_active or state != STATE_SAVED:
        return

    next_trial()
    if state == STATE_READY:
        next_step_label.text = "NEXT TRIAL READY — CLICK RUN CURRENT TRIAL"
        next_step_label.color = color.yellow


def exit_full_experiment(evt=None):
    """Return to manual control without discarding a saved result."""
    global full_experiment_active
    global abort_current_run

    full_experiment_active = False
    if state in [STATE_RUNNING, STATE_SETTLING]:
        abort_current_run = True
        next_step_label.text = "STOPPING FULL EXPERIMENT — MANUAL CONTROLS WILL RETURN"
        return

    update_instruction_labels()

# ==========================================================
# BUTTONS
# ==========================================================

scene.append_to_caption(
    "<div style='width:1100px;text-align:center'>"
    "<b>EXPERIMENT</b><br>"
    "Run one complete trial, review it, then prepare the next."
    "</div>"
    "<span style='display:inline-block;width:365px'></span>"
)
full_run_button = button(text="RUN CURRENT TRIAL", bind=run_full_experiment)
scene.append_to_caption("     ")
continue_full_run_button = button(text="NEXT TRIAL", bind=continue_full_experiment)

scene.append_to_caption(
    "<br><br><div style='width:1100px;text-align:center'>"
    "<b>REPLAYS</b><br>Review any completed trial."
    "</div>"
    "<span style='display:inline-block;width:150px'></span>"
)
previous_replay_button = button(text="⏮ PREVIOUS", bind=previous_saved_replay)
scene.append_to_caption("     ")
replay_button = button(text="▶ PLAY", bind=replay_saved_trial)
scene.append_to_caption("     ")
pause_replay_button = button(text="⏸ PAUSE", bind=pause_replay)
scene.append_to_caption("     ")
restart_replay_button = button(text="↻ REPLAY", bind=restart_replay)
scene.append_to_caption("     ")
next_replay_button = button(text="NEXT ⏭", bind=next_saved_replay)
scene.append_to_caption("     ")
close_replay_button = button(text="✕ CLOSE", bind=close_replay_player)
scene.append_to_caption(
    "<br><span style='display:inline-block;width:190px'></span>Timeline: "
)
replay_timeline = slider(min=0, max=1000, value=0, step=1, length=620, bind=scrub_replay)
scene.append_to_caption(
    "<br><span style='display:inline-block;width:190px'></span>"
)
replay_status_text = wtext(text="No replays yet. Complete the first trial to create one.")

scene.append_to_caption(
    "<br><br><div style='width:1100px;text-align:center'>"
    "<b>UNIVERSAL SPEED</b><br>Controls both experiments and replays."
    "</div>"
    "<span style='display:inline-block;width:465px'></span>"
)
universal_speed_menu = menu(
    choices=["1× Normal", "2× Fast", "4× Maximum"],
    selected="1× Normal",
    bind=change_universal_speed,
)

scene.append_to_caption(
    "<br><br><div style='width:1100px;text-align:center'>"
    "Results, reports, data, and replays are recorded automatically.<br>"
    "<span style='color:#aeb6c1'>Physics: PyBullet 3D | Display: VPython</span>"
    "</div>"
)

# ==========================================================
# INITIALIZE
# ==========================================================

initialize_results_file()
update_material_preview()
update_saved_result_label()

debug_pilot = os.environ.get("AOR_DEBUG_PILOT")
if HEADLESS_MODE and debug_pilot is not None:
    debug_index = int(debug_pilot)
    debug_scale = float(os.environ.get("AOR_DEBUG_SCALE", "1.0"))
    debug_seed = int(
        os.environ.get(
            "AOR_DEBUG_SEED",
            str(PRECHECK_SEED + debug_index * 1009),
        )
    )
    debug_condition = CONDITIONS[debug_index]
    debug_record = run_hidden_pilot(
        debug_index,
        debug_seed,
        debug_scale if debug_condition["material_class"] == "quartz" else quartz_friction_scale,
        debug_scale if debug_condition["material_class"] == "glass" else glass_friction_scale,
    )
    print(json.dumps(debug_record, indent=2), flush=True)
    _disconnect_pybullet()
    raise SystemExit(0 if debug_record["angle"] is not None else 2)

state = STATE_PREVALIDATING
update_instruction_labels()
prevalidation_ok = run_scientific_integrity_precheck()

if HEADLESS_MODE:
    print(
        "PREVALIDATION_PASSED"
        if prevalidation_ok
        else "PREVALIDATION_FAILED"
    )
    _disconnect_pybullet()
    raise SystemExit(0 if prevalidation_ok else 2)
else:
    # Keep VPython responsive and advance the non-blocking replay player.
    while True:
        rate(30)
        replay_tick()
