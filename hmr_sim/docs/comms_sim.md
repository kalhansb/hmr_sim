# hmr_comms_sim — message-level wireless-link emulator

`hmr_comms_sim_node` emulates the inter-robot radio at the ROS message level.
It replaces the `HMRNetSim` Gazebo plugin + `hmr_comms_relay_node` pair with a
single ROS 2 node that both **models the links** (the "oracle") and **applies
their effects to relayed topics** (the "cable"). Gazebo is not involved: robot
poses come from any `nav_msgs/Odometry` source and tree positions are parsed
from the world SDF once at startup, so the same emulator runs against the live
sim, a rosbag replay, or synthetic pose publishers.

```text
                     ┌───────────── hmr_comms_sim ─────────────┐
/<r>/odom_ground_truth ─► LinkModel (link_rate_hz):            │
world SDF (trees) ─────►   distance + trees → SNR → {connected,│──► ~/link_states
                     │     bandwidth, BER} per pair            │    ~/robot_index
                     │                                         │    ~/stats
/<r>/<topic> ──────────► Relay: per-topic policy               │
                     │     reliable    → queue, deliver late   │──► /<rx>/rx/<r>/<topic>
                     │     best_effort → drop under loss/load  │
                     └─────────────────────────────────────────┘
```

## Link model (per robot pair, at `link_rate_hz`, default 5 Hz)

1. **Distance** between the two poses (3D).
2. **Trees on the link**: world models whose name — or, for an `<include>`, whose
   instance name or model URI — contains any substring in `tree_name_substrings`
   (default `["tree", "pine"]`, case-insensitive), and whose trunk lies within
   `tree_radius_m + FresnelZoneRadius(d, frequency_hz)` of the 2D line segment.
   A species absent from that list is transparent to the radio and inflates the
   link budget, so the node logs both `Loaded N tree positions` and
   `Not counted as trees: …` at startup — check the second line when adding a
   world. In `flatforestv2` the count is 80 oak `<model>`s + 8 `pine_*`
   `<include>`s = 88. Note `"pine"` does **not** match `pinus_pinaster` (20 per
   forest world), which is why `"pinus"` is a separate default.

   Perimeter walls (`cmu_grey_wall`) are deliberately not counted, and this costs
   nothing: they form a closed square at ±52 m while the sim ROI is ±50 m, so by
   convexity no link between two in-ROI points can cross one. Shrubs are excluded
   on the different ground that `tree_attenuation_db` is a trunk figure.
3. **Path loss** (forest model, [IEEE 9260568](https://ieeexplore.ieee.org/document/9260568)):
   `P0 + 20·log10(d) + N_trees·Lv + fade`, where `fade` is an AR(1) shadow-fading
   process (stationary std `fade_sigma_db`, memory `fade_alpha`) — it evolves
   smoothly over seconds instead of being white noise per sample, and it is a
   pure function of `(seed, tick)`, so link traces are exactly repeatable and
   independent of the relayed traffic.

   `Lv` (`tree_attenuation_db`) defaults to **70 dB per trunk** as of 2026-09:
   a single tree in the Fresnel zone kills the link (the observed one-tree
   geometries needed 51–66 dB of extra loss to cross the SNR floor). The
   paper's fitted 11.98 dB — under which links survived ~4 trunks — was the
   default for every earlier campaign; runs on the two values are different
   radio regimes and must never be pooled.

   Past `max_range_m` (default **30 m**, 3D distance; `<= 0` disables) a
   further **200 dB** is added — a hard radio horizon, without which a
   tree-free 30 dBm link stays decodable for kilometres on free-space loss
   alone. It lands as path loss rather than a `connected=false` override, so
   the published SNR/BER/tier columns stay mutually consistent and the tier
   hysteresis (step 5) still smooths the boundary crossing.
4. **SNR → BER**: 64-QAM over AWGN when the link is tree-free, Rayleigh otherwise.
5. **Bandwidth tier** `{72, 28.9, 7.2, 0}` Mbps with the original 3-of-8 /
   8-of-8 SNR hysteresis — now genuinely spanning `8 / link_rate_hz` seconds
   (the plugin sampled it per sim step, so its "history" was ~8 ms).
6. `connected = bandwidth > 0`.

## The shared channel

All links contend for **one airtime budget**: `airtime_capacity` seconds of
channel per second (burst-capped at `airtime_burst_s`). A message of `B` bytes
on a link currently at `R` Mbps consumes `8B/R` seconds of it. This models what
independent per-pair token buckets cannot: with N robots on one channel, one
pair's traffic really does eat the others' throughput. Admission requires a
positive balance; the balance may then go negative (bounded debt), so a message
larger than the burst allowance is still sendable at the long-run rate.

## Relay policies

Topics are relayed from `/<sender>/<topic>` to `/<receiver>/rx/<sender>/<topic>`
(message type and publisher QoS discovered at runtime). Two policies:

- **`reliable_topics`** — never dropped. Each directional link keeps a FIFO
  queue drained only while the link is up and airtime is available; downtime
  grows a backlog that is delivered **late, never lost** (like TCP/Zenoh
  reliable). Airtime cost is scaled by expected retransmissions
  `min(1/(1−PER_msg), retx_cap)` — bad links pay more per byte. The queue is
  bounded by `reliable_queue_max_bytes`; overflow drops oldest **with a warning**,
  because for the scovox delta stream a lost delta permanently holes the
  receiver's merged map. Use this policy for `scovox_node/scovox_bin`: map
  quality under comms stress then degrades to *staleness*, which is the clean,
  defensible metric — not silent voxel loss confounding the merge experiments.
- **`best_effort_topics`** — dropped when the link is down, when the shared
  channel is out of airtime, or by a coin flip with probability
  `1 − (1−BER)^bits` computed from the **actual serialized size** (no assumed
  packet size). For streams where the newest sample supersedes older ones
  (pointclouds, odometry).

Every delivered message is deferred by its transmission time plus `delay_ms`.

A listed topic is looked for once a second until it appears. One that the
running stack never publishes stays pending for the whole run: the poll never
stops, a "Waiting for N relay topics to appear: …" line naming them prints
every 10 s, and "All relay topics discovered" never prints. `pending_warn_sec`
(default 120) seconds after the last topic that was discovered, or after the
first poll if none was, the emulator warns once, naming what is still pending.
The poll continues after the warning, so a late publisher is still relayed.
List only the topics the running node publishes.

## Diagnostics

- `~/link_states` (`Float64MultiArray`, at `link_rate_hz`) — one row per
  unordered pair: `[i, j, distance_m, trees_on_link, path_loss_db, snr_db,
  ber, bandwidth_mbps, connected]`.
- `~/robot_index` (latched `String` JSON) — maps `i/j` to robot names and
  documents the columns; rosbag-friendly.
- `~/stats` (`String` JSON, every `stats_period_s`) — per-directional-link
  `relayed / bytes / drop_ber / drop_airtime / drop_disconnected /
  drop_overflow / backlog_bytes`.

## Usage

```bash
# with the sim, described by the same scenario file the sim was launched with:
ros2 launch hmr_sim comms_sim.launch.py scenario:=flatforest_2robot_lidar.yaml

# CLI shortcut (registered world):
ros2 launch hmr_sim comms_sim.launch.py world:=flatforest robots:=atlas,bestla

# bag replay / arbitrary poses (world_sdf optional -> pure distance model):
ros2 launch hmr_sim comms_sim.launch.py robots:=bunker,curt \
    world_sdf:=/path/to/world.sdf use_sim_time:=false \
    params_file:=/path/to/overrides.yaml
```

Radio, policy and channel parameters live in
[`config/comms_sim_params.yaml`](../config/comms_sim_params.yaml); the launch
file injects `robot_names` and `world_sdf` on top.

## Caveats — read before trusting an experiment

- **Topic leakage.** DDS is one broadcast domain: nothing stops a consumer from
  subscribing to `/robotA/scovox_node/scovox_bin` directly and getting perfect
  comms by accident. Every cross-robot consumer must be remapped to the
  `/rx/<sender>/...` topic — audit this per experiment; one wrong remap
  silently gives infinite bandwidth.
- **Clock basis.** Timers and token buckets run on the node clock. With
  `use_sim_time:=true` (the default in the launch file) emulated bandwidth
  scales correctly with RTF; without a `/clock` publisher the node would idle.
- **Consumer QoS depth.** A reconnect drains the reliable backlog as a burst.
  The rx publishers keep `rx_qos_depth` (default 100) samples of history, but a
  subscriber with a shallow queue can still shed messages at the DDS layer —
  give rx-topic consumers a comparable depth.
- **What is NOT modeled.** MAC retries beyond the airtime cost factor, packet
  fragmentation (a lost fragment killing a large UDP message), DDS discovery
  traffic, and transport congestion dynamics. If a reviewer needs the wire to
  be real, validate a subset of runs with `tc netem` between per-robot
  containers and show the curves match.

## Smoke test

[`scripts/comms_smoke_test.sh`](../scripts/comms_smoke_test.sh) is a scripted
end-to-end check: relay of both policies, hard link gate at extreme range,
backlog delivered on reconnect, and tree parsing against `flatforestv2.sdf`
(asserts 88 = 80 oak `<model>`s + 8 pine `<include>`s, both counts derived from
the world file rather than hardcoded; T8 exits non-zero on mismatch). It runs
two fake robots on wall clock with `ros2 topic pub`
poses on a random DDS domain — no Gazebo needed.
