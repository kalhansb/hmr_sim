# hmr_sim — code comment notes

Long comments from files in the `hmr_sim` package, moved out of the code on 2026-09-23 so the sources carry short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word. Files with many moved comments have their own notes doc next to this one.

One section per source file, in file order. Each entry names the function (or section) the comment sat in, the line of code it was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [config/scenarios/cmu_forest_2robot_lidar.yaml](#configscenarioscmu_forest_2robot_lidaryaml) — 1
- [config/scenarios/cmu_forest_3robot_lidar.yaml](#configscenarioscmu_forest_3robot_lidaryaml) — 1
- [config/scenarios/cmu_forest_4robot_lidar.yaml](#configscenarioscmu_forest_4robot_lidaryaml) — 1
- [config/scenarios/cmu_forest_nb_2robot_lidar.yaml](#configscenarioscmu_forest_nb_2robot_lidaryaml) — 1
- [config/scenarios/cmu_forest_nb_3robot_lidar.yaml](#configscenarioscmu_forest_nb_3robot_lidaryaml) — 1
- [config/scenarios/cmu_forest_nb_4robot_lidar.yaml](#configscenarioscmu_forest_nb_4robot_lidaryaml) — 1
- [config/scenarios/flatforest_2robot_lidar.yaml](#configscenariosflatforest_2robot_lidaryaml) — 1
- [config/scenarios/flatforest_3robot_lidar.yaml](#configscenariosflatforest_3robot_lidaryaml) — 1
- [config/scenarios/flatforest_dense2_2robot_lidar.yaml](#configscenariosflatforest_dense2_2robot_lidaryaml) — 1
- [config/scenarios/flatforest_dense_2robot_lidar.yaml](#configscenariosflatforest_dense_2robot_lidaryaml) — 1
- [config/scenarios/flatforest_dense_3robot_lidar.yaml](#configscenariosflatforest_dense_3robot_lidaryaml) — 1
- [config/scenarios/flatforest_dense_4robot_lidar.yaml](#configscenariosflatforest_dense_4robot_lidaryaml) — 1
- [include/hmr_net_sim_system/HMRNetSim.hh](#includehmr_net_sim_systemhmrnetsimhh) — 1
- [include/world_query_system/WorldQuerySystem.hh](#includeworld_query_systemworldquerysystemhh) — 1
- [launch/_robot_registry.py](#launch_robot_registrypy) — 1
- [launch/_world_registry.py](#launch_world_registrypy) — 3
- [launch/comms_sim.launch.py](#launchcomms_simlaunchpy) — 4
- [scripts/comms_smoke_test.sh](#scriptscomms_smoke_testsh) — 2
- [src/hmr_comms_relay_node.cpp](#srchmr_comms_relay_nodecpp) — 1
- [src/hmr_comms_sim_node.cpp](#srchmr_comms_sim_nodecpp) — 12
- [src/ros2_gz_generic_bridge_node.cpp](#srcros2_gz_generic_bridge_nodecpp) — 1
- [worlds/flatforest/densify_forest.py](#worldsflatforestdensify_forestpy) — 1

## config/scenarios/cmu_forest_2robot_lidar.yaml

### scenario-cmu-forest-2robot

**CMU forest two-robot scenario notes** — in `Top level`, attached to `world: cmu_forest` (line 3)

```text
Identical to flatforest_dense_2robot_lidar.yaml in every respect except the
world: same robots, same sensors, same spawn poses, so a comparison against
the ts1b series isolates the stand and nothing else.

Why this world exists as a second stand. flatforest_dense is a procedural
lattice -- 270 identical oak instances on a jittered grid over roughly one
hectare, no structure above the stem scale. cmu_forest is a hand-built stand:
234 oaks and 108 pines placed irregularly over 150 x 150 m, two houses, and a
perimeter fence. Inside the 100 x 100 m ROI it carries 152 stems/ha against
flatforest_dense's ~250, with a mean nearest-neighbour stem spacing of 6.56 m.
It is the SPARSER of the two, and it is clumped rather than uniform.

That matters because the comms link here is occlusion-gated -- no measured
dropout in the campaign record had clear line-of-sight -- so the stand decides
how often the team loses contact. A clumped stand at 0.6x the mean density is
not "the same experiment, easier": it can produce FEWER, LONGER outages than a
uniform stand of the same mean density, because occlusion arrives in runs.
That is the thing worth measuring, and it is why mean density alone does not
predict the result.

Spawn poses are the flatforest line, unchanged and verified in this world:
atlas at the origin, bestla at y=+3 facing away. Nearest obstacle to either
spawn is 5.6 m (an oak at 4.17, 6.74), the initial link is 3.00 m, and no
perimeter wall lies within the ROI -- the fence sits at x = -69/+81 and
y = -77/+73, 19 m clear of the nearest ROI edge. The only non-tree obstacles
inside the ROI are cmu_house_1 at (-27.0, 33.4) and cmu_house_2 at (1.1, -36.0).

BEFORE running anything that TERMINATES on coverage here, re-measure the
unknown-fraction floor. ts1b's done_unknown_fraction=0.64 was calibrated on
flatforest_dense and does not transfer: a different stem count, a clumped
layout, and two solid buildings that permanently shadow their own footprints
all move the floor, and they do not all move it the same way. "No criterion
window exists any more" is a real possible outcome. Measure it on a smoke
cell, do not assume the sparser stand is the easier one.
```

## config/scenarios/cmu_forest_3robot_lidar.yaml

### scenario-cmu-forest-3robot

**CMU forest three-robot scenario notes** — in `Top level`, attached to `world: cmu_forest` (line 3)

```text
Identical to flatforest_dense_3robot_lidar.yaml in every respect except the
world, and identical to cmu_forest_2robot_lidar.yaml in every respect except
that a third robot exists.

atlas and bestla keep the exact spawn poses of the 2-robot scenario, so the
first two robots of a 3-robot run start where a 2-robot run of the same seed
starts them. Those poses are in turn the flatforest line unchanged, so a cell
here and the matching ts1b cell differ in the stand and in nothing else.

Spawn geometry as in flatforest_dense_3robot_lidar.yaml: the three sit on the
y axis at -3 / 0 / +3 facing away from each other, all on
COSTAR_HUSKY_SENSOR_CONFIG_LIDAR so team size is the only thing that changed,
and the team STARTS connected -- longest initial link 6.00 m against the 30 m
horizon -- which the comms warm-up and the planner's peer discovery both
assume. Verified clear in this world: the tightest spawn clearance of the
three is 5.58 m (husky at y=-3 to an oak at 1.66, -8.33).

The stand is 152 stems/ha inside the ROI against flatforest_dense's ~250, and
clumped rather than uniform, with mean nearest-neighbour stem spacing 6.56 m.
Sparser on the mean does not mean better connected: occlusion in a clumped
stand arrives in runs, so the outage COUNT can fall while outage DURATION
rises. Read the per-rung connectivity endpoints, not just the totals.

BEFORE running anything that TERMINATES on coverage here, re-measure the
unknown-fraction floor -- ts1b's 0.64 was calibrated on flatforest_dense and
does not transfer. See the note in cmu_forest_2robot_lidar.yaml.
```

## config/scenarios/cmu_forest_4robot_lidar.yaml

### scenario-cmu-forest-4robot

**CMU forest four-robot scenario notes** — in `Top level`, attached to `world: cmu_forest` (line 3)

```text
Identical to flatforest_dense_4robot_lidar.yaml in every respect except the
world, and identical to cmu_forest_3robot_lidar.yaml in every respect except
that a fourth robot exists.

The first three robots keep the exact spawn poses of the 2- and 3-robot
scenarios, so team size is the only thing that moves across the rungs. skadi
extends the line to y=+6 with the alternating yaw, so no robot starts by
driving into a neighbour.

NOTE the registry default differs. hmr_sim's _world_registry.py lists
cmu_forest's default_spawn_points as (0,0), (0,3), (0,-3), (3,0), all at
yaw 0.0. That default is NOT used here, for two reasons: putting the fourth
robot at (3,0) breaks the line the earlier rungs established, and yaw 0.0 on
every robot points them all the same way, which the flatforest scenarios
deliberately avoid. Explicit poses in this file override the registry.

Spawn clearance verified in this world -- the tightest of the four is skadi at
(0, 6), 4.24 m from an oak at (4.17, 6.74). That is the smallest clearance in
the whole rung set and it is still ~4x the Husky half-width, but it is the
first thing to check if a smoke cell reports a spawn-time collision.

Longest initial link is husky(0,-3) to skadi(0,+6) = 9.00 m, well inside the
30 m horizon, so the team STARTS fully connected. Confirm that empirically on
the first cell by reading all six pairs out of link_states.csv at t=0 rather
than trusting the geometry -- that is how it was established for the ts1b
rungs and the check has caught a bad assumption before.

Record achieved RTF per team size. CPU contention masquerades as comms
behaviour: a slower box means fewer planner cycles per sim-second, which
looks like a planner that reconnects less eagerly. The ts1b baselines are
median wall/t_sim = 1.171 (N=2), 1.792 (N=3), 2.494 (N=4) on flatforest_dense;
this world has 426 models against flatforest_dense's ~2000, so it should be
no slower, and a figure that comes out WORSE is a signal something else is
wrong, not a fact about the stand.

BEFORE running anything that TERMINATES on coverage here, re-measure the
unknown-fraction floor -- ts1b's 0.64 was calibrated on flatforest_dense and
does not transfer. See the note in cmu_forest_2robot_lidar.yaml.
```

## config/scenarios/cmu_forest_nb_2robot_lidar.yaml

### scenario-cmu-forest-nb-2robot

**Buildings-free CMU forest, two robots** — in `Top level`, attached to `world: cmu_forest_nb` (line 4)

```text
Identical to cmu_forest_2robot_lidar.yaml in every respect except the world,
and that world is cmu_forest with its three house models deleted and nothing
else touched: same 234 oaks, same 108 pines, same poses, same perimeter fence.

WHY THE BUILDINGS ARE GONE. The ts1c N=2 pilot lost a cell: a robot sat at
(-6.95, -42.68) for 2558 sim-seconds and ended censored_at_T, at a spot an
earlier plateau probe had pinned on a DIFFERENT seed 0.28 m away. Both sites
are 1.64 m from house_2's west wall. Both were on the `off` arm, which makes
the trap differential across the contrast rather than merely expensive.

WHAT WAS RULED OUT. The natural reading, that the stand pinches into dead
ends, does not survive its control. Modelled the way the global planning map
actually sees the world (obstacle triangles in the z range [0.05, 1.00] body
slab; 1.60 m clearance required, from inflation 1.5 m at 0.40 m/cell against
the local planner's 0.80 m), 19.4% of cmu_forest's free ROI lies behind a
throat too narrow for the global planner to route out of. flatforest_dense
scores 28.0% on the identical measure and produced ZERO stalls over 300 s in
711 robot-runs. Corridor geometry does not predict stalling; cutting trees
would have changed the stand and fixed nothing.

The buildings are what the two worlds actually differ in, and every large
pocket in cmu_forest sits on a house face. Deleting them opens the observed
site from a 1.40 m throat to 1.79 m and moves total pocket area 19.4% to
18.0%, i.e. the stand is left as it was.

The stand-vs-flatforest comparison this world exists for is UNCHANGED: 234
oaks and 108 pines over 150 x 150 m, 152 stems/ha inside the ROI against
flatforest_dense's ~250, clumped rather than uniform, mean nearest-neighbour
stem spacing 6.56 m. The perimeter fence stays and is still outside the ROI
(all 80 walls at r >= 68.7 m).

COVERAGE THRESHOLD, MEASURED HERE 2026-09-11, USE 0.58 AND NOT 0.64.
Two probe cells (prefix cfnb_plateau, off arm, seeds 801/802) ran the full
3000 s at done_unknown_fraction=0.01 so they traverse the whole curve and the
crossing is read off the series afterwards. Both therefore end censored_at_T
by design.

The curve is a STAIRCASE, not a smooth decay: long flats punctuated by
map-merge steps, so a threshold is never approached, it is jumped over, and
two thresholds inside one flat are the same experiment under two names. In
seed801 every threshold from 0.68 down to 0.52 latches within 41 s of itself,
at t = 1565 to 1606. seed802 has two cliffs instead of one, at t ~ 1010 and
t ~ 1755.

That second cliff is what rules out the inherited 0.64. It latches seed802 at
t=1010 with u=0.6371 while the team still goes on to reach 0.4917, i.e. it
stops the cell with 0.145 of the map still recoverable and BEFORE the larger
of the two merge events. It is not measuring a plateau, it is measuring which
side of a cliff the run happened to be on.

0.58 latches at t=1595 and t=1750, both at the knee: the per-300 s rate is
flat to three decimals from t=1820 onward in seed801 and near-flat in seed802.
It clears the slowest robot's 3000 s value by +0.067 and +0.069, which is the
same margin flatforest_dense2 was accepted on (+0.0698).

KNIFE EDGE, STATED RATHER THAN HIDDEN: because the curve is a staircase,
0.60 lands seed802 on the early cliff and 0.58 on the late one, a 730 s
difference for a 0.02 change in threshold. Every threshold sits on some
cliff for some seed; this is a property of merge-driven coverage, not of this
number. n=2 probes.

Floor, for the record: 0.4917 and 0.4939 at 3000 s, against cmu_forest's
0.5139 -- and cmu_forest's was measured with atlas stuck in the house_2 trap
for 1297 s, so it was never the achievable floor to begin with. Anything at
or below 0.51 never latches: the SLOWER robot of each pair ends at 0.5134 and
0.5111, and the criterion is per-robot.
```

## config/scenarios/cmu_forest_nb_3robot_lidar.yaml

### scenario-cmu-forest-nb-3robot

**Buildings-free CMU forest, three robots** — in `Top level`, attached to `world: cmu_forest_nb` (line 4)

```text
Identical to cmu_forest_3robot_lidar.yaml in every respect except the world,
and that world is cmu_forest with its three house models deleted and nothing
else touched: same 234 oaks, same 108 pines, same poses, same perimeter fence.

WHY THE BUILDINGS ARE GONE. The ts1c N=2 pilot lost a cell: a robot sat at
(-6.95, -42.68) for 2558 sim-seconds and ended censored_at_T, at a spot an
earlier plateau probe had pinned on a DIFFERENT seed 0.28 m away. Both sites
are 1.64 m from house_2's west wall. Both were on the `off` arm, which makes
the trap differential across the contrast rather than merely expensive.

WHAT WAS RULED OUT. The natural reading, that the stand pinches into dead
ends, does not survive its control. Modelled the way the global planning map
actually sees the world (obstacle triangles in the z range [0.05, 1.00] body
slab; 1.60 m clearance required, from inflation 1.5 m at 0.40 m/cell against
the local planner's 0.80 m), 19.4% of cmu_forest's free ROI lies behind a
throat too narrow for the global planner to route out of. flatforest_dense
scores 28.0% on the identical measure and produced ZERO stalls over 300 s in
711 robot-runs. Corridor geometry does not predict stalling; cutting trees
would have changed the stand and fixed nothing.

The buildings are what the two worlds actually differ in, and every large
pocket in cmu_forest sits on a house face. Deleting them opens the observed
site from a 1.40 m throat to 1.79 m and moves total pocket area 19.4% to
18.0%, i.e. the stand is left as it was.

The stand-vs-flatforest comparison this world exists for is UNCHANGED: 234
oaks and 108 pines over 150 x 150 m, 152 stems/ha inside the ROI against
flatforest_dense's ~250, clumped rather than uniform, mean nearest-neighbour
stem spacing 6.56 m. The perimeter fence stays and is still outside the ROI
(all 80 walls at r >= 68.7 m).

COVERAGE THRESHOLD, MEASURED HERE 2026-09-11, USE 0.58 AND NOT 0.64.
Two probe cells (prefix cfnb_plateau, off arm, seeds 801/802) ran the full
3000 s at done_unknown_fraction=0.01 so they traverse the whole curve and the
crossing is read off the series afterwards. Both therefore end censored_at_T
by design.

The curve is a STAIRCASE, not a smooth decay: long flats punctuated by
map-merge steps, so a threshold is never approached, it is jumped over, and
two thresholds inside one flat are the same experiment under two names. In
seed801 every threshold from 0.68 down to 0.52 latches within 41 s of itself,
at t = 1565 to 1606. seed802 has two cliffs instead of one, at t ~ 1010 and
t ~ 1755.

That second cliff is what rules out the inherited 0.64. It latches seed802 at
t=1010 with u=0.6371 while the team still goes on to reach 0.4917, i.e. it
stops the cell with 0.145 of the map still recoverable and BEFORE the larger
of the two merge events. It is not measuring a plateau, it is measuring which
side of a cliff the run happened to be on.

0.58 latches at t=1595 and t=1750, both at the knee: the per-300 s rate is
flat to three decimals from t=1820 onward in seed801 and near-flat in seed802.
It clears the slowest robot's 3000 s value by +0.067 and +0.069, which is the
same margin flatforest_dense2 was accepted on (+0.0698).

KNIFE EDGE, STATED RATHER THAN HIDDEN: because the curve is a staircase,
0.60 lands seed802 on the early cliff and 0.58 on the late one, a 730 s
difference for a 0.02 change in threshold. Every threshold sits on some
cliff for some seed; this is a property of merge-driven coverage, not of this
number. n=2 probes.

Floor, for the record: 0.4917 and 0.4939 at 3000 s, against cmu_forest's
0.5139 -- and cmu_forest's was measured with atlas stuck in the house_2 trap
for 1297 s, so it was never the achievable floor to begin with. Anything at
or below 0.51 never latches: the SLOWER robot of each pair ends at 0.5134 and
0.5111, and the criterion is per-robot.
```

## config/scenarios/cmu_forest_nb_4robot_lidar.yaml

### scenario-cmu-forest-nb-4robot

**Buildings-free CMU forest, four robots** — in `Top level`, attached to `world: cmu_forest_nb` (line 4)

```text
Identical to cmu_forest_4robot_lidar.yaml in every respect except the world,
and that world is cmu_forest with its three house models deleted and nothing
else touched: same 234 oaks, same 108 pines, same poses, same perimeter fence.

WHY THE BUILDINGS ARE GONE. The ts1c N=2 pilot lost a cell: a robot sat at
(-6.95, -42.68) for 2558 sim-seconds and ended censored_at_T, at a spot an
earlier plateau probe had pinned on a DIFFERENT seed 0.28 m away. Both sites
are 1.64 m from house_2's west wall. Both were on the `off` arm, which makes
the trap differential across the contrast rather than merely expensive.

WHAT WAS RULED OUT. The natural reading, that the stand pinches into dead
ends, does not survive its control. Modelled the way the global planning map
actually sees the world (obstacle triangles in the z range [0.05, 1.00] body
slab; 1.60 m clearance required, from inflation 1.5 m at 0.40 m/cell against
the local planner's 0.80 m), 19.4% of cmu_forest's free ROI lies behind a
throat too narrow for the global planner to route out of. flatforest_dense
scores 28.0% on the identical measure and produced ZERO stalls over 300 s in
711 robot-runs. Corridor geometry does not predict stalling; cutting trees
would have changed the stand and fixed nothing.

The buildings are what the two worlds actually differ in, and every large
pocket in cmu_forest sits on a house face. Deleting them opens the observed
site from a 1.40 m throat to 1.79 m and moves total pocket area 19.4% to
18.0%, i.e. the stand is left as it was.

The stand-vs-flatforest comparison this world exists for is UNCHANGED: 234
oaks and 108 pines over 150 x 150 m, 152 stems/ha inside the ROI against
flatforest_dense's ~250, clumped rather than uniform, mean nearest-neighbour
stem spacing 6.56 m. The perimeter fence stays and is still outside the ROI
(all 80 walls at r >= 68.7 m).

COVERAGE THRESHOLD, MEASURED HERE 2026-09-11, USE 0.58 AND NOT 0.64.
Two probe cells (prefix cfnb_plateau, off arm, seeds 801/802) ran the full
3000 s at done_unknown_fraction=0.01 so they traverse the whole curve and the
crossing is read off the series afterwards. Both therefore end censored_at_T
by design.

The curve is a STAIRCASE, not a smooth decay: long flats punctuated by
map-merge steps, so a threshold is never approached, it is jumped over, and
two thresholds inside one flat are the same experiment under two names. In
seed801 every threshold from 0.68 down to 0.52 latches within 41 s of itself,
at t = 1565 to 1606. seed802 has two cliffs instead of one, at t ~ 1010 and
t ~ 1755.

That second cliff is what rules out the inherited 0.64. It latches seed802 at
t=1010 with u=0.6371 while the team still goes on to reach 0.4917, i.e. it
stops the cell with 0.145 of the map still recoverable and BEFORE the larger
of the two merge events. It is not measuring a plateau, it is measuring which
side of a cliff the run happened to be on.

0.58 latches at t=1595 and t=1750, both at the knee: the per-300 s rate is
flat to three decimals from t=1820 onward in seed801 and near-flat in seed802.
It clears the slowest robot's 3000 s value by +0.067 and +0.069, which is the
same margin flatforest_dense2 was accepted on (+0.0698).

KNIFE EDGE, STATED RATHER THAN HIDDEN: because the curve is a staircase,
0.60 lands seed802 on the early cliff and 0.58 on the late one, a 730 s
difference for a 0.02 change in threshold. Every threshold sits on some
cliff for some seed; this is a property of merge-driven coverage, not of this
number. n=2 probes.

Floor, for the record: 0.4917 and 0.4939 at 3000 s, against cmu_forest's
0.5139 -- and cmu_forest's was measured with atlas stuck in the house_2 trap
for 1297 s, so it was never the achievable floor to begin with. Anything at
or below 0.51 never latches: the SLOWER robot of each pair ends at 0.5134 and
0.5111, and the criterion is per-robot.
```

## config/scenarios/flatforest_2robot_lidar.yaml

### scenario-flatforest-2robot

**Flatforest two-robot sensing and bridge** — in `Top level`, attached to `world: flatforest` (line 3)

```text
Both robots use COSTAR_HUSKY_SENSOR_CONFIG_LIDAR (VLP-16-style gpu_lidar
1800x16 @ 10 Hz + IMU, no rendering cameras), overriding the registry's
default rgbd+segmentation models for atlas/bestla. Features list drives the
ros_gz bridge: each robot gets /<name>/velodyne_points (frame <name>/velodyne,
mounted on base_link at x=0.0012 z=0.716), /<name>/imu/data,
/<name>/odom_ground_truth and /<name>/cmd_vel.
```

## config/scenarios/flatforest_3robot_lidar.yaml

### scenario-flatforest-3robot

**Flatforest three-robot scenario notes** — in `Top level`, attached to `world: flatforest` (line 3)

```text
The N=3 counterpart of flatforest_2robot_lidar.yaml, and identical to it in
every respect except that a third robot exists: same world, same model, same
feature set, and atlas/bestla keep their exact spawn poses so a 2-robot and a
3-robot run of the same seed start the first two robots in the same place.

All three use COSTAR_HUSKY_SENSOR_CONFIG_LIDAR (VLP-16-style gpu_lidar
1800x16 @ 10 Hz + IMU, no rendering cameras), overriding the registry's
defaults — rgbd+segmentation for atlas/bestla, and husky's own
SENSOR_CONFIG_1 — so the three robots carry identical sensing and team size
is the only thing that changed. Features drive the ros_gz bridge: each robot
gets /<name>/velodyne_points (frame <name>/velodyne, mounted on base_link at
x=0.0012 z=0.716), /<name>/imu/data, /<name>/odom_ground_truth and
/<name>/cmd_vel.

husky is the third name because it is already a registered UGV; the registry
entry supplies nothing here (every field is overridden) but an unregistered
name would depend on resolve_robot_spec's tolerance for one, which is a
thinner guarantee than an entry that exists.

Spawn geometry: the three sit on the y axis at -3 / 0 / +3, facing away from
each other. The pair scenario separates atlas and bestla by 3 m and turns
bestla around; this keeps that spacing and gives the third the mirrored pose,
so no robot starts inside another's footprint and none of them starts by
driving into a neighbour. Note this makes the team START connected, which is
what the comms emulator's warm-up and the planner's peer discovery both
assume — a scenario that spawns a robot out of range would begin every run in
an outage and confound bring-up with the treatment.
```

## config/scenarios/flatforest_dense2_2robot_lidar.yaml

### scenario-flatforest-dense2-2robot

**Very dense flatforest two-robot notes** — in `Top level`, attached to `world: flatforest_dense2` (line 3)

```text
Identical to flatforest_dense_2robot_lidar.yaml in every respect except the
world: same robots, same sensors, same spawn poses, same 30 dBm radio. A
comparison against flatforest_dense isolates stem density and nothing else,
which is the whole point — see §27 of comms_reconnection_experiment.md, where
this pair is pre-registered as a two-level dose–response on environment
severity.

WHY 400 STEMS/HA. flatforest_dense (250/ha) puts about 3.9 trunks in the
Fresnel corridor of a 50 m link against the ~3.8 needed to reach the 2 dB
cutoff — it sits exactly ON the cliff edge, so whether the radio is up is
decided by the AR(1) fade and by one trunk of geometric luck. Measured over
29 untreated pb3g2 cells: disconnected 55.5 % of run time, median 16 outages
per run, and 0.0 % of disconnected samples with clear line of sight. This
world carries 6.21 trunks on the same link — about 2.3 trunks × 11.98 dB
≈ 28 dB past cutoff — so connection becomes the exception that needs
favourable geometry rather than a coin flip.

Transmit power stays at the shipped 30 dBm and is NOT a variable here.
run_campaign.sh:36 rules that out on physical grounds, and a calibrated
replay of the real NextBandwidth state machine (100.000 % agreement at 0 dB
offset) showed a 20 dB cut — a hundredfold power reduction — moves
triggerable outages only +35 %, because outage duration is set by how fast a
robot walks out of a tree shadow, not by link margin. Severity belongs to the
environment.

SPAWN CLEARANCE IS VALIDATED, NOT ASSUMED. densify_forest.py places stems by
rejection sampling against other stems; nothing in it knows where robots
spawn, and at 5.4× flatforest's density that silence stops being evidence.
Measured: the two spawn poses used below have 2.75 m and 2.13 m to the
nearest trunk — byte-identical to flatforest_dense and flatforestv2, because
the nearest stem is an original that the densifier never moves. flatforest_
dense ran 116 cells at exactly those clearances with zero failures, so this
geometry is empirically sufficient rather than merely plausible.

BEFORE RUNNING ANYTHING THAT TERMINATES ON COVERAGE, re-measure the
unknown-fraction floor. This is a blocking gate, not a courtesy: unknown
fraction decays to a floor set by permanently-shadowed voxels, and that floor
RISES with occlusion. flatforest_dense's 0.60 criterion cleared its floor by
+0.0989 (lowest reached 0.5011 over 30 off cells, planner-CSV source); 1.6×
the trunks can plausibly consume all of that. If the floor lands above the
criterion no run terminates and completion time measures the cap; if it lands
just below, completion time measures the stopping rule instead of the
exploration policy. Neither announces itself in a results table.

THE MARGIN MOVED, AND THE OLD NUMBER IS WHY THIS IS WORTH READING TWICE. This
said +0.0699 / 0.5301 until 2026-08-23. Both readings are correct. The floor
is a MINIMUM over samples; the events log samples 6-8× sparser than the
planner CSV; a minimum over a sparse grid can only read HIGH. Same world,
same criterion, +0.0290 of margin — the size of floor2.py's own fail
threshold — decided purely by which file was read. See §29.57 and §29.62.

HOW IT IS ENFORCED, because "check first" was an instruction to a human and
nothing ran it. The fd2s smoke is the throwaway prefix this paragraph asks
for; `floor2.py fd2s 0.60` is registered in fd2s_readouts.sh and now exits 1
rather than 0 when it fails; and gate_and_launch.sh's gate2 C1 independently
requires every smoke cell to actually REACH the criterion, which is strictly
stronger than "a floor exists below it" — but blind to the margin, which is
the number only floor2.py prints. Both, not either.
```

## config/scenarios/flatforest_dense_2robot_lidar.yaml

### scenario-flatforest-dense-2robot

**Dense flatforest two-robot notes** — in `Top level`, attached to `world: flatforest_dense` (line 3)

```text
Identical to flatforest_2robot_lidar.yaml in every respect except the world:
same robots, same sensors, same spawn poses, so a comparison against the
sparse world isolates stem density and nothing else.

The world is flatforest_dense (250 stems/ha against flatforest's 74), which
exists because the sparse stand cannot break the shipped 30 dBm radio —
measured over a 300 s run, the pair stayed connected for 100 % of 1580 link
samples with a minimum SNR of 18.6 dB against a 2 dB cutoff. That is why
every earlier "comms severity" level in the reconnection experiment was
produced by detuning transmit power, which is not a variable a field
experiment has. Here the link fails on geometry: about 3.9 trunks land in the
Fresnel corridor of a 50 m link, which is the ~3.8 needed to reach cutoff.

Before running anything that TERMINATES on coverage in this world, re-measure
the unknown-fraction floor. flatforest's 0.4922 floor and 0.55 criterion were
calibrated against 74 stems/ha and do not transfer: three times the trunks
means more permanently-shadowed voxels and a higher floor. "No criterion
window exists any more" is a real possible outcome here.
```

## config/scenarios/flatforest_dense_3robot_lidar.yaml

### scenario-flatforest-dense-3robot

**Dense flatforest three-robot notes** — in `Top level`, attached to `world: flatforest_dense` (line 3)

```text
Identical to flatforest_3robot_lidar.yaml in every respect except the world,
and identical to flatforest_dense_2robot_lidar.yaml in every respect except
that a third robot exists. It is the cell this pair of files was missing: the
existing 3-robot scenario sits in the SPARSE world (74 stems/ha) while every
campaign in this tree since the 30 m radio regime has run the DENSE one
(250 stems/ha).

That matters more here than anywhere else. The comms link in this project is
occlusion-gated -- no measured dropout in the campaign record had clear
line-of-sight -- so stem density, not transmit power, is what decides how
often the team loses contact. Running the N=3 step on flatforest would drop
density by 3.4x at the same moment team size changed, and a reconnection
experiment would be comparing arms in a world that barely disconnects. Team
size has to be the only thing that moved.

atlas and bestla keep the exact spawn poses of both 2-robot scenarios, so the
first two robots of a 3-robot run start where a 2-robot run of the same seed
starts them.

Spawn geometry, sensing, features and the husky registry note are all as in
flatforest_3robot_lidar.yaml: the three sit on the y axis at -3 / 0 / +3
facing away from each other, all on COSTAR_HUSKY_SENSOR_CONFIG_LIDAR so team
size is the only thing that changed, and the team STARTS connected, which the
comms warm-up and the planner's peer discovery both assume.

Before running anything that TERMINATES on coverage in this world, re-measure
the coverage ceiling: the dense world's reachable fraction is not the sparse
world's, and a done_unknown threshold tuned on one is not valid on the other.
```

## config/scenarios/flatforest_dense_4robot_lidar.yaml

### scenario-flatforest-dense-4robot

**Dense flatforest four-robot notes** — in `Top level`, attached to `world: flatforest_dense` (line 3)

```text
The N=4 rung of the team-size series. Identical to
flatforest_dense_3robot_lidar.yaml in every respect except that a fourth robot
exists: same world, same model, same feature set for every robot, and atlas,
bestla and husky keep their exact spawn poses, so the first three robots of a
4-robot run of a given seed start where a 3-robot run of that seed starts
them, and the first two where a 2-robot run starts them. Team size is the only
thing that moves along the series.

The world is flatforest_dense (250 stems/ha) for the reason the 3-robot file
gives: the comms link here is occlusion-gated, so stem density and not
transmit power decides how often contact is lost. Changing density at the same
moment team size changed would make the series uninterpretable.

skadi is the fourth name. It is registered in _robot_registry (Norse, beside
atlas and bestla) rather than left unregistered: resolve_robot_spec does
tolerate an unknown name as long as the scenario supplies sdf_file, but that
tolerance is a weaker guarantee than an entry that exists, and the registry is
the file a future reader checks. Every field is overridden here anyway.

Spawn geometry: the four sit on the y axis at -3 / 0 / +3 / +6, alternating
yaw so no robot starts by driving into a neighbour. The line is extended
rather than rearranged into a square precisely so the earlier poses survive
unchanged. Longest initial link is husky(-3) to skadi(+6) = 9 m, inside the
30 m radio horizon, so the team STARTS fully connected -- which the comms
warm-up and the planner's peer discovery both assume. At 70 dB per trunk a
single stem in the Fresnel corridor would break a link even at 9 m, so this
assumption is checked empirically on the smoke cell by reading all 6 pairs out
of link_states.csv at t=0, not asserted from the world SDF.

All four use COSTAR_HUSKY_SENSOR_CONFIG_LIDAR (VLP-16-style gpu_lidar
1800x16 @ 10 Hz + IMU, no rendering cameras), so the robots carry identical
sensing and team size is the only thing that changed.

Load: four lidars render on the gz sensors thread and four scovox mappers cost
~1 core-second per sim-second each on a 20-core box whose gz main loop is
already at the one-core wall. Expect a lower achieved RTF than N=2. That is a
wall-time cost, but it is ALSO a confound -- CPU contention shows up as
apparent comms behaviour -- so the achieved RTF and the mapper keep-up must be
recorded per team size and reported alongside the result.
```

## include/hmr_net_sim_system/HMRNetSim.hh

### hmr-net-sim-plugin-class

**The HMRNetSim plugin class** — in `HMRNetSim — declarations`, attached to `class HMRNetSim:` (line 157)

```text
This is the main plugin's class. It must inherit from System and at least
one other interface.
Here we use `ISystemPostUpdate`, which is used to get results after
physics runs. The opposite of that, `ISystemPreUpdate`, would be used by
plugins that want to send commands.
```

## include/world_query_system/WorldQuerySystem.hh

### world-query-occupancy-method

**How WorldQuerySystem decides occupancy** — in `WorldQuerySystem — declarations`, attached to `class WorldQuerySystem :` (line 13)

```text
\brief Gazebo system plugin that queries occupancy and semantic labels
for a grid of 3D points using multi-directional raycasting.

Casts 6 rays (±X, ±Y, ±Z) from each query point. A point is "occupied"
if any ray hits within ray_length. Semantic label comes from the nearest
hit visual's Label user data.
```

## launch/_robot_registry.py

### robot-registry-skadi

**Why skadi defaults to lidar-only** — in `Module scope`, attached to `'type': 'ugv',` (line 41)

```text
Fourth UGV, added for the N=4 rung of the team-size series. The
lidar-only sensor config is the default here rather than an
rgbd/segmentation one because every scenario that names skadi wants
the same sensing as the other three, and a default that disagrees
with every call site is a trap for the next reader.
```

## launch/_world_registry.py

### world-flatforest-dense

**Why flatforest_dense exists** — in `Module scope`, attached to `'flatforest_dense': {` (line 25)

```text
Same walled 104x104 m stand as flatforest, densified from 74 to 250
stems/ha by densify_forest.py. Exists because the sparse world cannot
break the shipped 30 dBm radio (measured: 100 % connected, min SNR
18.6 dB against a 2 dB cutoff), which is why every earlier "comms
severity" level was faked by detuning transmit power. Here the link
drops on geometry alone once the robots are ~50 m apart.
Its coverage floor is NOT flatforest's: more trunks mean more
permanently-shadowed voxels, so re-calibrate done_unknown_fraction
against this world before running anything that terminates on it.
```

### world-flatforest-dense2

**flatforest_dense2 density and coverage floor** — in `Module scope`, attached to `'flatforest_dense2': {` (line 44)

```text
Second dose point on the density axis: same walled 104x104 m stand,
433 stems (400/ha) against flatforest_dense's 270 (250/ha). Registered
2026-08-23 for the pb4d campaign pre-registered in §27 of
comms_reconnection_experiment.md.
WHY 400 AND NOT MORE POWER: the 250/ha world puts ~3.9 trunks in the
Fresnel corridor of a 50 m link against the ~3.8 needed to reach the
2 dB cutoff -- i.e. it sits exactly on the cliff edge, where a link is
decided by which side of one trunk the robot passes. 400/ha gives 6.21
trunks, ~28 dB past cutoff, so outages are set by geometry rather than
by a knife-edge. Transmit power was measured and rejected as the lever:
a calibrated replay of NextBandwidth shows a 20 dB cut buys only +35 %
triggerable outages.
Spawn clearances at the two poses actually used are 2.75 m and 2.13 m,
IDENTICAL to flatforest_dense, which ran 120 cells without a spawn
failure. Only the unused (0, -3) tightened, 5.35 -> 3.70 m.
SAME FLOOR WARNING AS ABOVE, AND IT BITES HARDER: pb3g2 cleared its
done_unknown_fraction=0.60 criterion by only +0.0989 (lowest reached
0.5011 over 30 off cells, planner-CSV source). 1.6x the trunks shadow
more voxels permanently and can eat that margin outright, at which
point completion time stops measuring exploration and starts measuring
the stopping rule.
MEASURED HERE 2026-08-23, and it clears: 3 smoke cells (prefix fd2s)
reach 0.5302 / 0.5355 / 0.5504, so the floor is at or below 0.5302 and
0.60 has +0.0698 of open water. All three crossed 0.60 -- at 1838 s,
3165 s and 3565 s. The smoke ran at done_unknown_fraction=0.30 on
purpose so cells traverse the whole curve and the crossing is read off
the series afterwards; every cell therefore ends censored_at_T, which
is the design and not a failure. Re-measure with a throwaway prefix and
scratchpad/floor2.py if this world is ever regenerated.
```

### world-cmu-forest-nb

**cmu_forest_nb: houses removed, floor remeasured** — in `Module scope`, attached to `'cmu_forest_nb': {` (line 93)

```text
cmu_forest with the three houses deleted and NOTHING else changed.
Registered 2026-09-11 after the ts1c N=2 pilot lost a cell to a robot
that sat 1.64 m from cmu_house_2's west wall for 2558 sim-seconds, at a
spot an earlier probe had pinned on a different seed 0.28 m away. Both
events were on the `off` arm, so the trap was not merely expensive, it
was differential across the contrast.
The stand is NOT the cause and was left alone: modelled as the global
planning map sees it, 19.4% of cmu_forest's free ROI sits behind a
throat below the planner's 1.60 m requirement, but flatforest_dense
scores 28.0% on the same measure with ZERO stalls over 300 s in 711
robot-runs. Corridor width does not predict stalling. Deleting the
houses opens the observed site from a 1.40 m to a 1.79 m throat and
moves total pocket area only 19.4% -> 18.0%.
FLOOR MEASURED HERE 2026-09-11: use done_unknown_fraction=0.58, NOT the
inherited 0.64. Two probe cells (cfnb_plateau, off arm, seeds 801/802,
3000 s at done_unknown=0.01) reach 0.4917 and 0.4939, against
cmu_forest's 0.5139 -- and cmu_forest's was measured with atlas stuck in
the house_2 trap for 1297 s, so it was never the achievable floor.
The curve is a staircase driven by map merges, and 0.64 lands seed802 on
the EARLY cliff: it latches at t=1010 with 0.145 of the map still
recoverable. 0.58 latches both seeds at the knee (t=1595 and t=1750) and
clears the slower robot's 3000 s value by +0.067. See the long note in
config/scenarios/cmu_forest_nb_2robot_lidar.yaml for the full table and
for the knife-edge caveat that comes with any threshold on a staircase.
```

## launch/comms_sim.launch.py

### comms-launch-seed-override

**Per-run fading seed override** — in `generate_launch_description`, attached to `overrides = {` (line 114)

```text
Fading is a pure function of (seed, tick), so the seed IS the run's link
realisation. Paired-seed designs need to vary it per run while holding
everything else fixed, and editing the shared params file per run makes
the arm and its pairing impossible to reconstruct afterwards. Absent, the
params file's value stands.
```

### comms-launch-tx-power-dial

**tx_power_dbm as the severity dial** — in `generate_launch_description`, attached to `if 'tx_power_dbm' in cli:` (line 131)

```text
tx_power_dbm is THE severity dial for the link model: everything
downstream (SNR -> BER -> bandwidth tier -> connected) is monotone in it,
so it is what a calibration sweep varies to place the outage rate where
the experiment needs it. Exposed here so a run can record and reproduce
its severity from the command line instead of by editing the installed
params yaml, which leaves no trace in the run directory and silently
re-scopes every run that follows.
```

### comms-launch-tree-attenuation-dial

**tree_attenuation_db as second severity dial** — in `generate_launch_description`, attached to `if 'tree_attenuation_db' in cli:` (line 146)

```text
tree_attenuation_db is the OTHER severity dial, and the two are not
interchangeable: tx_power_dbm shifts every link by the same number of dB,
while this one shifts a link in proportion to trees_on_link. Raising it
therefore selects for occlusion-driven outages -- outages that happen
because a trunk moved into the Fresnel zone, not because the robots drove
apart -- which is the failure mode a forest reconnection experiment is
supposed to be about. Exposed here for the same reason as tx_power_dbm:
editing the installed params yaml leaves no trace in the run directory and
silently re-scopes every run that follows.
```

### comms-launch-reliable-queue-cap

**Reliable backlog cap and overflow loss** — in `generate_launch_description`, attached to `if 'reliable_queue_max_bytes' in cli:` (line 175)

```text
Reliable-relay backlog cap. On overflow the node drops the OLDEST queued
delta and never retransmits it, so the receiver's merged map is missing
those voxels permanently. That is fatal to any experiment whose premise is
"the backlog drains at contact" — the robot-known/team-observed gap can
then never snap shut, and the receiver's unknown fraction is biased
upwards for the rest of the run. It has to be settable per run because the
required depth is the offered map-delta load times the longest outage, and
both move with voxel resolution and tx_power_dbm.
```

## scripts/comms_smoke_test.sh

### smoke-t5-reconnect-burst

**Capturing the reconnect backlog burst** — in `bring beta back`, attached to `( timeout 14 ros2 topic echo --qos-depth 200 /beta/rx/alpha/chatter_rel 2>/dev/null | grep -c hello_rel > "$OU` (line 76)

```text
Counter subscribes while the link is still down so the reconnect burst is
fully captured, then beta returns. 8 consecutive high samples at 5 Hz to
reconnect (~1.6 s), then the queue drains. Steady-state is ~2 msg/s; the
~20 s of downtime backlog (~40 msgs) on top proves queueing.
--qos-depth 200: the burst arrives faster than the python echo drains its
subscriber queue; the default depth-10 reader history would shed most of it.
```

### smoke-t8-tree-parse-truth

**T8 ground truth from the world file** — in `T8: SDF tree parsing on the real flatforest world`, attached to `T8_OAKS=$(grep -c '<model name="Oak tree' "$WORLD")` (line 112)

```text
Ground truth is derived from the world file, not hardcoded, so this asserts
the PARSER rather than a number someone has to remember to update: 80 oaks as
top-level <model>s plus 8 pines carried as <include>s of model://cmu_pine_tree.
The pines are the regression guard — they were silently uncounted until
2026-08-15 because the matcher tested only the include's instance name
("pine_N"), which contains no configured substring.
```

## src/hmr_comms_relay_node.cpp

### relay-node-overview

**Legacy comms relay node overview** — in `File scope`, attached to `#include <rclcpp/rclcpp.hpp>` (line 2)

```text
A ROS2 node that simulates a communications pipeline between robots.
It subscribes to each robot's topics, reads network metrics from the
HMRNetSim Gazebo plugin (bridged to ROS2), and relays or drops messages
based on PER, PDR, and bandwidth.

Topic convention:
  Robot publishes on:  /<robot>/<comms_topic>        (existing topic)
  Receiver gets on:    /<receiver>/rx/<sender>/<comms_topic>

Parameters:
  robot_names:   list of robot name strings, e.g. ["atlas", "bestla"]
  comms_topics:  list of topic name strings, e.g. ["rgbd_camera/points", "velodyne_points"]

The node auto-discovers message types from the actual topic publishers,
so no message type configuration is needed.
```

## src/hmr_comms_sim_node.cpp

### comms-sim-origin

**Successor to the plugin and relay pair** — in `File scope`, attached to `#include <rclcpp/rclcpp.hpp>` (line 3)

```text
Successor to the HMRNetSim gz plugin + hmr_comms_relay_node pair: both roles
(link-physics "oracle" and message "cable") live in this one ROS2 node, so the
same emulator runs against the live sim, a bag replay, or any pose source —
Gazebo is not involved. Tree positions come from parsing the world SDF once at
startup (trees are static; the plugin only ever read them once anyway).
```

### comms-sim-awgn-ber-tier

**BER at the tier actually in use** — in `AwgnQam64Ber`, attached to `double AwgnQam64Ber(double power_w, double noise_w, double spectral_efficiency)` (line 123)

```text
64-QAM BER over AWGN (line of sight).

spectral_efficiency is bits/s/Hz AT THE TIER THE LINK IS ACTUALLY IN, passed
in by the caller, not the top tier: Eb/N0 = (S/N) / (R/B), so a link that has
downshifted spreads the same received power over fewer bits per second and
each bit gets more energy. Hardcoding the 72 Mbps value here charged a
downshifted link the error rate of a gear it was not in — see the
"Deliberately removed" note below for what that cost.

The constellation stays 64-QAM at every tier, which the 28.9 and 7.2 Mbps
tiers are not (802.11n MCS3 is 16-QAM, MCS0 is BPSK). At equal Eb/N0 a
lower-order constellation has a LOWER error rate, so this is a pessimistic
bound on the BER of the real tier, not an estimate of it. That is deliberate:
ber is a published diagnostic that gates nothing, and a bound written down as
a bound beats a second pair of closed forms nobody has checked.
```

### comms-sim-no-ber-gating

**Why BER never gates delivery** — in `ToLower`, attached to `std::string ToLower(std::string s)` (line 161)

```text
Deliberately removed: MessageSuccessProbability(ber, bits).

It applied the BER above to a WHOLE serialized message, and its result was
used to drop best-effort messages and to inflate reliable airtime. Both were
wrong. The BER functions hardcode spectral_efficiency = 72e6/20e6 — 64-QAM at
the TOP rate — regardless of the tier NextBandwidth actually selected, so a
link that had correctly downshifted to 7.2 Mbps was charged the error rate of
a gear it was not in: slow AND lossy for one weak signal, which is the
opposite of what rate adaptation is for. Measured at tx_power_dbm=-14, median
BER on CONNECTED samples was 0.116, so a 200-byte beacon survived with
probability ~1e-70: robots exchanged 3544 intent beacons and 7 arrived, peer
records never formed, and the pursuit manoeuvre could never arm.
HMRNetSim.cc, which this node was ported from, never did this — it gates on
the SNR>=2 dB boundary (PDR 1.0 below, 1e-8 above) and publishes ber/per as
diagnostics only.

Generation 9 made BER a function of the selected tier, which is the second
half of the condition this note used to set for reintroducing BER gating. The
first half still stands and is the binding one: DO NOT gate delivery on BER.
The bandwidth state machine is the only authority on whether a message gets
through, and a second, independent loss mechanism on top of it is what
produced the 7-of-3544 beacon result above.
```

### comms-sim-tree-name-substrings

**Tree name substrings and empty lists** — in `HmrCommsSimNode`, attached to `tree_name_substrings_ = declare_parameter<std::vector<std::string>>(` (line 210)

```text
A world may name its trees by species rather than by the word "tree"
(flatforestv2 has 80 "Oak tree" models plus 8 "pine_*" includes), so the
match is a set of substrings, ANY of which qualifies. A species missing
from this list is transparent to the radio, which inflates the link budget
rather than erroring — so LoadTreePositions() logs both the matched count
and the names it rejected, and adding a world means reading that line.
Defaults cover every tree species in the shipped worlds: "pine" alone does
NOT match `pinus_pinaster` (20 per forest world), hence "pinus".
To match nothing, write [""] — NOT []. An empty yaml/CLI list has no
inferable element type and arrives NOT_SET, which aborts the node during
base-class construction, before any code here runs. That is generic rclcpp
behaviour for every list parameter (`reliable_topics` included), not a
property of this one; main() turns the abort into a readable message.
```

### comms-sim-legacy-tree-substring

**Deprecated scalar tree_name_substring** — in `HmrCommsSimNode`, attached to `const auto legacy_substring = declare_parameter<std::string>("tree_name_substring", "");` (line 232)

```text
Superseded scalar. Replaces (not extends) the list, so a world config that
still sets it keeps its own species set rather than silently gaining the
new defaults. Not bit-for-bit the old behaviour even so: include URIs are
now tested alongside instance names, so legacy "tree" picks up flatforest's
pines anyway (their URI is model://cmu_pine_tree) — 88, where it used to
find 80. That widening is the point of this change, not a regression.
(Declared after the list so a NOT_SET list above cannot skip it.)
```

### comms-sim-tree-attenuation-70db

**Why 70 dB per trunk** — in `HmrCommsSimNode`, attached to `tree_attenuation_db_ = declare_parameter<double>("tree_attenuation_db", 70.0);` (line 256)

```text
70 dB/trunk makes ONE tree in the Fresnel zone fatal: the observed
one-tree link geometries needed 51-66 dB of extra loss to cross the
SNR floor, which the paper's fitted 11.98 (under which links survived
~4 trunks) never supplied. Runs before 2026-09 used 11.98 and are a
different radio regime — never pool across the change.
```

### comms-sim-pose-timeout

**Stale poses invalidate links** — in `HmrCommsSimNode`, attached to `pose_timeout_s_ = declare_parameter<double>("pose_timeout_s", 2.0);` (line 275)

```text
A pose older than this (in the node's own clock, so sim time when
use_sim_time is set) does not describe where the robot is now, and a link
model run on it reports a link that may not exist. Links touching a stale
endpoint go invalid: no delivery, and a zeroed diagnostic row. Set <= 0
to restore the pre-generation-9 behaviour of trusting a pose forever.
```

### comms-sim-tier-before-ber

**Tier chosen before BER** — in `ComputePair`, attached to `ps.bandwidth_mbps = NextBandwidth(ps, snr_db);` (line 663)

```text
Tier first, then BER at that tier. NextBandwidth reads only snr_history
and ps.bandwidth_mbps and writes only snr_history, so this reordering
leaves the selected tier — and therefore every delivery decision —
bit-identical to generation 8. Only the ber column moves.
```

### comms-sim-ber-when-no-gear

**BER published when no gear** — in `ComputePair`, attached to `ber = 1.0;` (line 672)

```text
No gear engaged: there is no rate to define a bit error rate at, and
nothing is getting through. 1.0, the same value an invalid row
publishes, rather than a number computed at a rate the link is not
using. Every consumer of this column should already be masking on
connected; this makes an unmasked read wrong in the safe direction.
```

### comms-sim-best-effort-delivery

**Best-effort delivery follows the tier** — in `OnTxMessage`, attached to `if (uniform_(drop_rng_) < residual_pdr_) {` (line 842)

```text
Delivery is the bandwidth state machine's call, as in HMRNetSim.cc:
above the SNR>=2 dB boundary (i.e. ps.connected, checked above) the
reference plugin delivers with PDR=1e-8, and its ber/per exist only to
be published. Link quality reaches the experiment through the TIER —
a weak link is slow, which surfaces as airtime pressure and backlog in
cost_s below — not as vanished messages.
```

### comms-sim-reliable-airtime-cost

**Reliable airtime cost is bits per tier** — in `DrainReliable`, attached to `const double cost_s = bits / (ps.bandwidth_mbps * 1e6);` (line 879)

```text
The tier is the whole quality model, so a byte costs bits/tier and
nothing more. The BER-derived retransmission multiplier that used to
scale this was never physical: it was capped at retx_cap while the
true expected number of transmissions at the BER this node computes is
astronomically larger, and that cap is the only reason 60 kB map
deltas flowed at all on a link where 200-byte beacons were dying.
```

### comms-sim-empty-list-abort

**Catching the empty-list parameter abort** — in `main`, attached to `RCLCPP_FATAL(rclcpp::get_logger("hmr_comms_sim"),` (line 1068)

```text
Raised from the Node base constructor while ingesting overrides, so no
catch inside the node body can see it. Overwhelmingly this is an empty
list written as `[]`: yaml and `--ros-args -p` cannot infer an element
type, the value arrives NOT_SET, and the default abort is an opaque
std::terminate that names neither the cause nor the fix.
```

## src/ros2_gz_generic_bridge_node.cpp

### gz-bridge-overview

**Generic ROS2 Gazebo bridge overview** — in `File scope`, attached to `#include <rclcpp/rclcpp.hpp>` (line 3)

```text
A bidirectional bridge between ROS2 and Gazebo transport that serializes
ROS2 messages to binary (CDR format) and transports them as
gz::msgs::StringMsg payloads over Gazebo transport, and vice versa.

This allows bridging ANY ROS2 message type without needing corresponding
Gazebo protobuf message definitions or custom type conversion code.

ROS2 → GZ:
  Subscribes to ROS2 topics using generic subscriptions (CDR serialized),
  wraps the binary payload in a gz::msgs::StringMsg, and publishes to
  Gazebo transport topics.

GZ → ROS2:
  Subscribes to Gazebo transport topics for gz::msgs::StringMsg payloads,
  unwraps the binary CDR data, and publishes to ROS2 topics using generic
  publishers.

Example: bridging octomap_msgs/msg/Octomap between ROS2 and Gazebo

Parameters:
  ros2_to_gz_ros_topics:  ROS2 source topic names
  ros2_to_gz_gz_topics:   Gazebo destination topic names
  ros2_to_gz_ros_types:   ROS2 message type strings

  gz_to_ros2_gz_topics:   Gazebo source topic names
  gz_to_ros2_ros_topics:  ROS2 destination topic names
  gz_to_ros2_ros_types:   ROS2 message type strings
```

## worlds/flatforest/densify_forest.py

### densify-world-name-rewrite

**Rewriting the generated world name** — in `main`, attached to `if args.world_name:` (line 223)

```text
The launcher addresses /world/<name>/create and /world/<name>/set_pose off
the registry short-name, and _world_registry.py's contract is that the
SDF's internal <world name> matches it. Leaving this as "flatforest" makes
a dense-world run spawn robots into the service namespace of the sparse
one — which fails late and looks like a sim bug, not a naming bug.
```
