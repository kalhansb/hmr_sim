#!/usr/bin/env python3
"""Drive hmr_radio_node (v2_phy) live: 3 robots, reliable bulk from a and c,
best-effort beacons from all. Phase 1: a-b-c line, a and c hidden (40 m).
Phase 2: c moved to (10, 10), all in carrier sense. Prints delivered Mbps per
stream per phase and link_mac means."""
import json, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float64MultiArray

PHASES = [("hidden line", {"a": (0, 0), "b": (20, 0), "c": (40, 0)}),
          ("all hear", {"a": (0, 0), "b": (20, 0), "c": (10, 10)})]
PHASE_S = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
BULK = "x" * 100000


class Driver(Node):
    def __init__(self):
        super().__init__("radio_driver")
        self.pos = dict(PHASES[0][1])
        rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.odom = {r: self.create_publisher(Odometry, f"/{r}/odom_ground_truth", 10) for r in "abc"}
        self.bulk = {r: self.create_publisher(String, f"/{r}/chatter_rel", rel) for r in "ac"}
        self.beacon = {r: self.create_publisher(String, f"/{r}/beacon", be) for r in "abc"}
        self.rx = {}
        for dst in "abc":
            for src in "abc":
                if src == dst:
                    continue
                for t, q in (("chatter_rel", QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)),
                             ("beacon", QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT))):
                    key = (src, dst, t)
                    self.rx[key] = [0, 0]
                    self.create_subscription(String, f"/{dst}/rx/{src}/{t}",
                                             lambda m, k=key: self._rx(k, m), q)
        self.mac = []
        self.create_subscription(Float64MultiArray, "/hmr_comms_sim/link_mac", self._mac, 50)
        self.stats = None
        self.create_subscription(String, "/hmr_comms_sim/stats", lambda m: setattr(self, "stats", m.data), 10)
        self.regime = None
        self.create_subscription(String, "/hmr_comms_sim/radio_regime", lambda m: setattr(self, "regime", m.data),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(0.05, self._tick_odom)
        self.create_timer(0.05, self._tick_bulk)
        self.create_timer(0.1, self._tick_beacon)
        self.sent_beacons = 0

    def _rx(self, key, m):
        self.rx[key][0] += 1
        self.rx[key][1] += len(m.data)

    def _mac(self, m):
        self.mac.append(list(m.data))

    def _tick_odom(self):
        for r, (x, y) in self.pos.items():
            o = Odometry()
            o.pose.pose.position.x = float(x)
            o.pose.pose.position.y = float(y)
            self.odom[r].publish(o)

    def _tick_bulk(self):
        for r in "ac":
            self.bulk[r].publish(String(data=BULK))

    def _tick_beacon(self):
        for r in "abc":
            self.beacon[r].publish(String(data="b" * 200))
        self.sent_beacons += 1


def main():
    rclpy.init()
    d = Driver()
    t0 = time.time()
    while d.regime is None and time.time() - t0 < 20:
        rclpy.spin_once(d, timeout_sec=0.1)
    reg = json.loads(d.regime) if d.regime else {}
    print("regime", reg.get("radio_regime"), reg.get("radio_stack"), "mac_slot_us", reg.get("mac_slot_us"),
          "link_mac_columns" in reg, flush=True)
    for name, pos in PHASES:
        d.pos = dict(pos)
        # settle: hysteresis needs 8 link ticks (1.6 s) plus queue drain
        t_end = time.time() + 4.0
        while time.time() < t_end:
            rclpy.spin_once(d, timeout_sec=0.01)
        for k in d.rx:
            d.rx[k] = [0, 0]
        d.mac.clear()
        b0 = d.sent_beacons
        t_start = time.time()
        while time.time() - t_start < PHASE_S:
            rclpy.spin_once(d, timeout_sec=0.01)
        dt = time.time() - t_start
        print(f"== {name} ({dt:.1f} s)")
        for (src, dst, t), (n, b) in sorted(d.rx.items()):
            if n:
                extra = f" beacons {n}/{d.sent_beacons - b0}" if t == "beacon" else ""
                print(f"  {src}->{dst} {t:12s} {8 * b / dt / 1e6:7.2f} Mbps{extra}")
        cols = ["i", "j", "mcs", "phy", "per", "busy", "gp", "hid", "coll", "backlog", "conn", "valid"]
        rows = {}
        for msg in d.mac:
            for k in range(0, len(msg), 12):
                r = msg[k:k + 12]
                rows.setdefault((int(r[0]), int(r[1])), []).append(r)
        for (i, j), rs in sorted(rows.items()):
            mean = [sum(r[c] for r in rs) / len(rs) for c in range(12)]
            if mean[5] > 0.01 or mean[10] > 0.5:
                print(f"  mac {'abc'[i]}->{'abc'[j]} " + " ".join(f"{c}={v:.3g}" for c, v in zip(cols[2:], mean[2:])))
    if d.stats:
        s = json.loads(d.stats)
        for l in s["links"]:
            print("  stats", l["from"], "->", l["to"], {k: l[k] for k in ("relayed", "drop_ber", "drop_disconnected",
                  "drop_overflow", "busy_s", "txops", "hidden_s") if k in l})
    d.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
