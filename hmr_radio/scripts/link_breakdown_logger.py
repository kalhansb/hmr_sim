#!/usr/bin/env python3
"""Log hmr_radio's ~/link_breakdown to CSV and ~/radio_regime to a JSON sidecar.

link_breakdown says where each link's loss comes from, one row per pair per
tick; its column names travel in the latched radio_regime message and are
checked here, so a CSV is never written at a width its publisher did not
declare. Invalid rows (valid == 0, a pair without two fresh poses) are counted
and skipped, as link_logger.py does for link_states.

  link_breakdown_logger.py --out <cell>/link_breakdown.csv
    -> link_breakdown.csv, and radio_regime.json next to it
"""

import argparse
import csv
import json
import os
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float64MultiArray, String

COLUMNS = ["i", "j", "distance_m", "pl_ground_db", "l_background_db",
           "l_trunks_db", "l_bushes_db", "l_veg_db", "l_ridges_db",
           "l_range_db", "fade_db", "path_loss_db", "snr_db",
           "trunks_on_link", "valid"]


class BreakdownLogger(Node):
    def __init__(self, ns, out_path):
        super().__init__("link_breakdown_logger")
        self.out = open(out_path, "w", newline="")
        self.w = csv.writer(self.out)
        self.w.writerow(["t_sim"] + COLUMNS)
        self.regime_path = os.path.join(os.path.dirname(out_path) or ".",
                                        "radio_regime.json")
        self.rows = 0
        self.masked = 0
        self.refused = 0
        self.since_flush = 0
        self.create_subscription(Float64MultiArray, f"{ns}/link_breakdown",
                                 self.cb, 50)
        self.create_subscription(
            String, f"{ns}/radio_regime", self.on_regime,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.get_logger().info(f"logging {ns}/link_breakdown -> {out_path}")

    def on_regime(self, msg):
        try:
            regime = json.loads(msg.data)
        except ValueError as e:
            self.get_logger().error(f"radio_regime is not JSON: {e}")
            return
        cols = regime.get("link_breakdown_columns")
        if cols != COLUMNS:
            self.get_logger().error(
                f"radio_regime declares link_breakdown columns {cols}, this "
                f"script writes {COLUMNS}. Update the script before trusting "
                "the CSV.")
        with open(self.regime_path, "w") as fh:
            json.dump(regime, fh, indent=1, sort_keys=True)
            fh.write("\n")
        self.get_logger().info(
            f"radio_regime {regime.get('radio_regime')} -> {self.regime_path}")

    def cb(self, msg):
        t = self.get_clock().now().nanoseconds * 1e-9
        stride = len(COLUMNS)
        declared = int(msg.layout.dim[1].size) if len(msg.layout.dim) >= 2 else None
        if declared != stride or len(msg.data) % stride:
            self.refused += 1
            self.get_logger().error(
                f"link_breakdown declares {declared} columns for {len(msg.data)} "
                f"values; this script writes {stride}. Message skipped.",
                throttle_duration_sec=10.0)
            return
        d = list(msg.data)
        for k in range(0, len(d), stride):
            row = d[k:k + stride]
            if row[14] <= 0.0:
                self.masked += 1
                continue
            self.w.writerow([f"{t:.3f}"] + [f"{v:g}" for v in row])
            self.rows += 1
            self.since_flush += 1
        if self.since_flush >= 500:
            self.since_flush = 0
            self.out.flush()

    def destroy_node(self):
        self.out.flush()
        self.out.close()
        print(f"[link_breakdown_logger] rows={self.rows} masked={self.masked} "
              f"refused_msgs={self.refused}", file=sys.stderr, flush=True)
        return super().destroy_node()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ns", default="/hmr_comms_sim")
    ap.add_argument("--out", required=True)
    args, _ = ap.parse_known_args()
    rclpy.init(args=sys.argv)
    node = BreakdownLogger(args.ns, args.out)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
