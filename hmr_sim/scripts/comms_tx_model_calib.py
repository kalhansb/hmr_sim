#!/usr/bin/env python3
"""Known-answer cases for hmr_comms_sim's transmission models and the latest
policy (DESIGN_gen34 §12.7, §12.12).

Runs the real node as a child process on a private ROS domain, drives two
robots' poses, publishes std_msgs/String payloads of chosen sizes and watches
what arrives on the rx topics and in ~/stats. No world file: pure distance
model, so 5 m apart is 72 Mbps and 2000 km apart is down.

  C1  admission delivers a small message ahead of a large one queued before
      it (the pre-§12.12 behaviour, kept as the default).
  C2  progressive delivers the same two in order.
  C3  progressive: a 9 MB message takes its air time at 72 Mbps under the
      0.6 airtime capacity, not the admission model's bits/tier.
  C4  progressive, latest: a link drop mid-frame aborts the frame, frames
      queued while down are superseded, and only the newest arrives.
  C5  progressive, reliable: a link drop mid-message resumes it on reconnect;
      it arrives once, nothing counted lost.

Needs a sourced workspace with hmr_sim built. Prints PASS/FAIL per case and
exits non-zero on any failure.
"""

import json
import os
import random
import subprocess
import sys
import time

os.environ['ROS_DOMAIN_ID'] = str(100 + random.randrange(100))

import rclpy  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy  # noqa: E402
from std_msgs.msg import String  # noqa: E402

NEAR, FAR = 5.0, 2.0e6
MB = 1_000_000


class Harness:
    def __init__(self, model):
        self.proc = subprocess.Popen(
            ['ros2', 'run', 'hmr_sim', 'hmr_comms_sim_node', '--ros-args',
             '-p', 'robot_names:=[a,b]',
             '-p', 'reliable_topics:=[rel]',
             '-p', 'latest_topics:=[lat]',
             '-p', 'best_effort_topics:=[""]',
             '-p', 'use_sim_time:=false',
             '-p', 'stats_period_s:=1.0',
             '-p', 'airtime_capacity:=0.6', '-p', 'airtime_burst_s:=0.25',
             '-p', 'rx_qos_depth:=100',
             '-p', f'transmission_model:={model}'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True)
        self.node = rclpy.create_node(f'tx_calib_{model}')
        self.b_x = NEAR
        self.got = []            # (topic, tag, size, t)
        self.stats = None
        rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.pub = {t: self.node.create_publisher(String, f'/a/{t}', rel)
                    for t in ('rel', 'lat')}
        for t in ('rel', 'lat'):
            self.node.create_subscription(
                String, f'/b/rx/a/{t}',
                lambda m, t=t: self.got.append(
                    (t, m.data.split(':', 1)[0], len(m.data), time.monotonic())),
                rel)
        self.node.create_subscription(
            String, '/hmr_comms_sim/stats',
            lambda m: setattr(self, 'stats', json.loads(m.data)), 10)
        self.pa = self.node.create_publisher(Odometry, '/a/odom_ground_truth', 10)
        self.pb = self.node.create_publisher(Odometry, '/b/odom_ground_truth', 10)
        self.node.create_timer(0.1, self._poses)

    def _poses(self):
        a = Odometry()
        b = Odometry()
        b.pose.pose.position.x = self.b_x
        self.pa.publish(a)
        self.pb.publish(b)

    def spin(self, sec, until=None):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            if until and until():
                return True
        return bool(until and until())

    def ready(self):
        # Both relays formed and matched, then a few link ticks at 72 Mbps.
        ok = self.spin(30, lambda: all(
            self.node.count_publishers(f'/b/rx/a/{t}') > 0 and
            self.pub[t].get_subscription_count() > 0 for t in ('rel', 'lat')))
        self.spin(2.0)
        return ok

    def send(self, topic, tag, size):
        self.pub[topic].publish(String(data=f'{tag}:' + 'x' * (size - len(tag) - 1)))
        return time.monotonic()

    def link(self):
        s = (self.stats or {}).get('links', [])
        return next((x for x in s if x['from'] == 'a' and x['to'] == 'b'), {})

    def close(self):
        self.node.destroy_node()
        try:
            os.killpg(self.proc.pid, 2)
            self.proc.wait(5)
        except Exception:
            os.killpg(self.proc.pid, 9)


fails = 0


def check(label, ok, detail=''):
    global fails
    print(f"  {'PASS' if ok else 'FAIL'}  {label}{'' if ok else ': ' + detail}")
    fails += 0 if ok else 1


def order_case(model):
    h = Harness(model)
    try:
        if not h.ready():
            check(f'{model}: relay formed', False, 'no relay in 30 s')
            return None
        h.send('rel', 'big', 2 * MB)
        h.send('rel', 'small', 1000)
        h.spin(5, lambda: len(h.got) >= 2)
        return [g[1] for g in h.got]
    finally:
        h.close()


def main():
    rclpy.init()

    got = order_case('admission')
    check('C1 admission: small overtakes the large one queued before it',
          got == ['small', 'big'], f'arrival order {got}')
    got = order_case('progressive')
    check('C2 progressive: delivered in queue order',
          got == ['big', 'small'], f'arrival order {got}')

    h = Harness('progressive')
    try:
        check('C3 relay formed', h.ready())
        t0 = h.send('lat', 'f', 9 * MB)
        h.spin(10, lambda: h.got)
        lat = h.got[0][3] - t0 if h.got else None
        # 72 Mbit: 0.625 s draining the 0.25 s burst at 72 Mbps, then the
        # rest at 0.6 x 72 Mbps -> ~1.25 s. Admission would say 1.0 s.
        check('C3 progressive: 9 MB at 72 Mbps under capacity 0.6 takes ~1.25 s',
              lat is not None and 1.1 <= lat <= 2.5, f'latency {lat}')
    finally:
        h.close()

    h = Harness('progressive')
    try:
        check('C4 relay formed', h.ready())
        h.send('lat', 'cut', 9 * MB)
        h.spin(0.1)
        # Down takes ~1 s (72 -> 28.9 -> 7.2 -> 0, 3-of-8 hysteresis at 5 Hz)
        # while the frame needs ~1.25 s at 72 alone, far longer once slowed.
        h.b_x = FAR
        h.spin(3.0)
        check('C4 nothing delivered from the cut frame', not h.got, f'{h.got}')
        for tag in ('s1', 's2', 's3'):
            h.send('lat', tag, 100_000)
            h.spin(0.3)
        h.b_x = NEAR                    # 8 of 8 high: ~1.6 s, then 7.2 Mbps
        h.spin(8.0, lambda: h.got)
        h.spin(2.0)
        tags = [g[1] for g in h.got]
        check('C4 only the newest frame arrives after reconnect',
              tags == ['s3'], f'arrived {tags}')
        h.spin(2.0)
        lk = h.link()
        check('C4 counters: 1 aborted, 2 superseded, nothing lost',
              lk.get('drop_aborted') == 1 and lk.get('drop_superseded') == 2 and
              lk.get('latest_relayed') == 1 and lk.get('drop_overflow') == 0,
              f'{lk}')
    finally:
        h.close()

    h = Harness('progressive')
    try:
        check('C5 relay formed', h.ready())
        h.send('rel', 'long', 9 * MB)
        h.spin(0.1)
        h.b_x = FAR
        h.spin(3.0)
        check('C5 nothing delivered while down', not h.got, f'{h.got}')
        h.b_x = NEAR
        h.spin(15.0, lambda: h.got)
        h.spin(1.0)
        got = [(g[1], g[2]) for g in h.got]
        check('C5 the reliable message resumes and arrives once, whole',
              got == [('long', 9 * MB)], f'{got}')
        h.spin(2.0)
        lk = h.link()
        check('C5 counters: nothing aborted or lost',
              lk.get('drop_aborted') == 0 and lk.get('drop_overflow') == 0 and
              lk.get('relayed') == 1, f'{lk}')
    finally:
        h.close()

    rclpy.shutdown()
    print('ALL PASS' if fails == 0 else f'{fails} FAILURE(S)')
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
