// PHY and MAC tests for hmr_radio's radio_mac (DESIGN_radio_v2.md §13). The
// constants are ns-3 3.44's, recorded by the oracle programs in test/oracle/
// (README.md there): NistErrorRateModel, WifiPhy::CalculateTxDuration, and
// saturated UDP over AdhocWifiMac with no explicit BAR and the 20 µs slot.
// Nothing from ns-3 is linked. A changed constant means re-running the oracle,
// never editing a number to make a test pass.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <utility>
#include <vector>

#include "hmr_radio/radio_mac.hpp"

using hmr_radio::MacParams;
using hmr_radio::ShareInput;
using hmr_radio::ShareOutput;

namespace
{

// SNR (dB) at which MpduPer(mcs, ., bytes) crosses target; PER falls with SNR.
double SnrAtPer(int mcs, double target, int bytes)
{
  double lo = -10.0, hi = 50.0;
  for (int k = 0; k < 100; ++k) {
    const double mid = 0.5 * (lo + hi);
    (hmr_radio::MpduPer(mcs, mid, bytes) > target ? lo : hi) = mid;
  }
  return 0.5 * (lo + hi);
}

// The oracle's scenarios (test/oracle/ns3_sat.cc) as ShareInput: 20 dBm, noise
// figure 7 dB, losses in dB, detection at the MAC's threshold.
struct Scene
{
  int n;
  ShareInput in;
  explicit Scene(int nodes)
  : n(nodes)
  {
    in.n = n;
    in.noise_dbm = -174.0 + 10.0 * std::log10(20e6) + 7.0;
    in.hear.assign(n * n, 0);
    in.active.assign(n * n, 0);
    in.rx_dbm.assign(n * n, -300.0);
    in.snr_db.assign(n * n, -300.0);
    in.mcs.assign(n * n, 0);
  }
  void Link(int a, int b, double loss_db)
  {
    for (const auto & [x, y] : {std::pair{a, b}, std::pair{b, a}}) {
      in.rx_dbm[x * n + y] = 20.0 - loss_db;
      in.snr_db[x * n + y] = in.rx_dbm[x * n + y] - in.noise_dbm;
      in.hear[x * n + y] = in.snr_db[x * n + y] >= MacParams().detect_snr_db;
    }
  }
  void Flow(int s, int d, int mcs)
  {
    in.active[s * n + d] = 1;
    in.mcs[s * n + d] = mcs;
  }
  double Mbps(const ShareOutput & o, int s, int d) const {return o.goodput_bps[s * n + d] / 1e6;}
};

// k senders to one receiver, every node hearing every other at 60 dB.
Scene Saturated(const std::vector<int> & mcs)
{
  const int k = static_cast<int>(mcs.size());
  Scene sc(k + 1);
  for (int a = 0; a <= k; ++a) {
    for (int b = a + 1; b <= k; ++b) {
      sc.Link(a, b, 60.0);
    }
  }
  for (int s = 0; s < k; ++s) {
    sc.Flow(s, k, mcs[s]);
  }
  return sc;
}

// 0 -> 1 and 2 -> 1; 0 and 2 cannot hear each other.
Scene Hidden(int mcs, double strong_db, double hidden_db)
{
  Scene sc(3);
  sc.Link(0, 1, strong_db);
  sc.Link(2, 1, hidden_db);
  sc.Link(0, 2, 200.0);
  sc.Flow(0, 1, mcs);
  sc.Flow(2, 1, mcs);
  return sc;
}

}  // namespace

// P1: the NIST error model against ns-3's NistErrorRateModel, one 1542 B MPDU
// (test/oracle/ns3_per.cc): SNR at PER 0.9 / 0.5 / 0.1 per MCS.
TEST(P1, NistPerMatchesNs3)
{
  const double oracle[8][3] = {
    {3.079, 3.430, 3.968}, {6.089, 6.441, 6.979}, {8.935, 9.303, 9.873},
    {12.530, 12.921, 13.514}, {15.627, 16.018, 16.620}, {20.366, 20.762, 21.365},
    {21.583, 21.997, 22.630}, {22.747, 23.161, 23.795}};
  const double targets[3] = {0.9, 0.5, 0.1};
  for (int m = 0; m < hmr_radio::kNumMcs; ++m) {
    for (int t = 0; t < 3; ++t) {
      EXPECT_NEAR(SnrAtPer(m, targets[t], 1542), oracle[m][t], 0.01)
        << "MCS" << m << " PER " << targets[t];
    }
  }
  EXPECT_NEAR(hmr_radio::MpduPer(2, 10.0, 1542), 0.0659963, 1e-4);
  EXPECT_NEAR(hmr_radio::MpduPer(1, 10.0, 1542), 0.0, 1e-6);
  const double rates[8] = {7.2, 14.4, 21.7, 28.9, 43.3, 57.8, 65.0, 72.2};
  for (int m = 0; m < hmr_radio::kNumMcs; ++m) {
    EXPECT_NEAR(hmr_radio::McsRateBps(m) / 1e6, rates[m], 0.05) << "MCS" << m;
  }
}

// P2: frame durations against WifiPhy::CalculateTxDuration (ns3_timing.cc).
TEST(P2, DurationsMatchNs3)
{
  const struct {int mcs, bytes; double us;} ppdu[] = {
    {0, 1544, 1755.6}, {0, 4632, 5179.2}, {0, 64848, 71880.0},
    {3, 1544, 470.4}, {3, 4632, 1327.2}, {3, 64848, 18002.4},
    {7, 1544, 214.8}, {7, 4632, 556.8}, {7, 64848, 7227.6}};
  for (const auto & c : ppdu) {
    EXPECT_NEAR(hmr_radio::HtPpduDurationS(c.mcs, c.bytes) * 1e6, c.us, 0.05)
      << "MCS" << c.mcs << " " << c.bytes << " B";
  }
  // Compressed block ack at 6 / 12 / 24 Mbps.
  EXPECT_NEAR(hmr_radio::BlockAckDurationS(0) * 1e6, 74.0, 0.05);
  EXPECT_NEAR(hmr_radio::BlockAckDurationS(1) * 1e6, 50.0, 0.05);
  EXPECT_NEAR(hmr_radio::BlockAckDurationS(2) * 1e6, 50.0, 0.05);
  for (int m = 3; m < hmr_radio::kNumMcs; ++m) {
    EXPECT_NEAR(hmr_radio::BlockAckDurationS(m) * 1e6, 38.0, 0.05);
  }
  EXPECT_NEAR(hmr_radio::AifsS(MacParams()) * 1e6, 70.0, 1e-9);
}

// P3: the aggregates ns-3 built under saturation (PSDU trace in ns3_sat.cc).
TEST(P3, FullAggregates)
{
  const MacParams mp;
  EXPECT_EQ(hmr_radio::MpduBytes(mp), 1538);
  EXPECT_EQ(hmr_radio::AmpduBytes(mp, 1), 1542);
  EXPECT_EQ(hmr_radio::AmpduBytes(mp, 3), 4630);
  const auto a0 = hmr_radio::FullAggregate(mp, 0);
  EXPECT_EQ(a0.mpdus, 3);
  EXPECT_EQ(a0.bytes, 4630);
  EXPECT_EQ(hmr_radio::FullAggregate(mp, 3).mpdus, 12);
  const auto a7 = hmr_radio::FullAggregate(mp, 7);
  EXPECT_EQ(a7.mpdus, 31);
  EXPECT_EQ(a7.bytes, 47862);
  EXPECT_NEAR(a7.ppdu_s * 1e6, 5344.8, 0.05);
  for (int m = 0; m < hmr_radio::kNumMcs; ++m) {
    const auto a = hmr_radio::FullAggregate(mp, m);
    EXPECT_LE(a.ppdu_s, mp.ppdu_max_s);
    EXPECT_LE(a.bytes, mp.max_ampdu_bytes);
    EXPECT_LE(a.mpdus, mp.max_ampdu_mpdus);
    // A backlog cap shortens the aggregate and never lengthens it.
    EXPECT_EQ(hmr_radio::FullAggregate(mp, m, 1).mpdus, 1);
    EXPECT_EQ(hmr_radio::FullAggregate(mp, m, 1000).mpdus, a.mpdus);
  }
}

// R1: the rate control settles on the best expected throughput and follows a
// fade with the EWMA's memory, not at once.
TEST(R1, RateControlSettlesAndLags)
{
  const MacParams mp;
  const double dt = 0.2;   // the node's 5 Hz link tick
  auto best = [&](double snr) {
      int b = 0;
      double tp = -1.0;
      for (int m = 0; m < hmr_radio::kNumMcs; ++m) {
        const double s = 1.0 - hmr_radio::MpduPer(m, snr, hmr_radio::MpduBytes(mp));
        const auto a = hmr_radio::FullAggregate(mp, m);
        if (s >= mp.minstrel_min_success && s * a.payload_bits / a.txop_s > tp) {
          tp = s * a.payload_bits / a.txop_s;
          b = m;
        }
      }
      return b;
    };
  for (const double snr : {0.0, 5.0, 10.0, 15.0, 20.0, 30.0}) {
    hmr_radio::RateControl rc;
    for (int k = 0; k < 20; ++k) {
      rc.Update(mp, snr, dt);
    }
    EXPECT_EQ(rc.mcs(), best(snr)) << snr << " dB";
  }
  EXPECT_EQ(best(30.0), 7);
  EXPECT_EQ(best(10.0), 2);
  hmr_radio::RateControl rc;
  for (int k = 0; k < 20; ++k) {
    rc.Update(mp, 30.0, dt);
  }
  int ticks = 0;
  while (rc.Update(mp, 10.0, dt) != 2 && ticks < 50) {
    ++ticks;
  }
  EXPECT_GE(ticks, 1);   // one bad sample does not move it
  EXPECT_LE(ticks, 3);   // and it has moved within 0.8 s
}

// M1: Bianchi's retry-limited chain against a slot-level Monte Carlo of the
// same backoff, n stations that all hear each other. tau and the fraction of
// slots carrying a success (what sets throughput) agree within 1.5 %. p is the
// decoupling approximation's known weak spot: two stations that collide climb
// to the same stage together and re-collide more often than independence
// says, so at n = 2 the model's p is 5.5 % low (Monte Carlo over 2e7 slots:
// 0.1046 vs 0.1108), 3 % at n = 3, within 1.5 % from n = 4.
TEST(M1, BianchiMatchesSlotMonteCarlo)
{
  const MacParams mp;
  EXPECT_NEAR(hmr_radio::BackoffTau(0.0, mp.cw_min, mp.cw_max, mp.retry_limit),
    2.0 / (mp.cw_min + 2), 1e-12);
  for (const int n : {2, 4, 8}) {
    std::mt19937_64 rng(20260929 + n);
    std::vector<int> stage(n, 0), counter(n);
    auto draw = [&](int j) {
        const int w = std::min((mp.cw_min + 1) << std::min(j, 30), mp.cw_max + 1);
        return std::uniform_int_distribution<int>(0, w - 1)(rng);
      };
    for (int i = 0; i < n; ++i) {
      counter[i] = draw(0);
    }
    const long slots = 3000000;
    long attempts = 0, collided = 0, successes = 0;
    std::vector<int> tx;
    for (long t = 0; t < slots; ++t) {
      tx.clear();
      for (int i = 0; i < n; ++i) {
        if (counter[i] == 0) {
          tx.push_back(i);
        } else {
          --counter[i];
        }
      }
      attempts += static_cast<long>(tx.size());
      successes += tx.size() == 1;
      for (const int i : tx) {
        if (tx.size() > 1) {
          ++collided;
          stage[i] = stage[i] < mp.retry_limit ? stage[i] + 1 : 0;
        } else {
          stage[i] = 0;
        }
        counter[i] = draw(stage[i]);
      }
    }
    const auto bp = hmr_radio::BianchiFixedPoint(n, mp.cw_min, mp.cw_max, mp.retry_limit);
    const double tau_mc = double(attempts) / (double(slots) * n);
    const double p_mc = double(collided) / double(attempts);
    const double succ = n * bp.tau * std::pow(1.0 - bp.tau, n - 1);
    EXPECT_NEAR(bp.tau / tau_mc, 1.0, 0.01) << "n=" << n;
    EXPECT_NEAR(succ / (double(successes) / double(slots)), 1.0, 0.015) << "n=" << n;
    EXPECT_NEAR(bp.p / p_mc, 1.0, n == 2 ? 0.07 : n == 4 ? 0.02 : 0.015) << "n=" << n;
    EXPECT_LE(bp.p, p_mc * (n == 2 ? 0.97 : 1.02)) << "n=" << n;
  }
}

// M2: saturated contention against ns-3 (ns3_sat.cc scenario sat, mean of seeds
// 1-3, 5 s each). Totals within 3 %, each flow within 6 % or 0.25 Mbps: the
// performance anomaly (a slow sender dragging the fast ones) included.
TEST(M2, ContentionMatchesNs3)
{
  const MacParams mp;
  const struct {std::vector<int> mcs; std::vector<double> flows;} cases[] = {
    {{0}, {6.449}},
    {{3}, {25.959}},
    {{7}, {65.051}},
    {{7, 7}, {29.468, 32.142}},
    {{7, 7, 7, 7}, {11.000, 15.181, 15.876, 15.821}},
    {{7, 3, 0}, {20.227, 7.959, 1.894}},
    {{7, 0}, {30.819, 3.126}}};
  for (const auto & c : cases) {
    Scene sc = Saturated(c.mcs);
    const ShareOutput o = hmr_radio::ShareChannel(mp, sc.in);
    const int k = static_cast<int>(c.mcs.size());
    double total = 0.0, total_ns3 = 0.0;
    for (int s = 0; s < k; ++s) {
      total += sc.Mbps(o, s, k);
      total_ns3 += c.flows[s];
    }
    EXPECT_NEAR(total / total_ns3, 1.0, k == 1 ? 0.005 : 0.03) << "senders " << k;
    // ns-3's split among equal senders is seed noise; the model's is equal.
    const bool equal = std::all_of(c.mcs.begin(), c.mcs.end(),
        [&](int m) {return m == c.mcs[0];});
    for (int s = 0; s < k && !equal; ++s) {
      EXPECT_NEAR(sc.Mbps(o, s, k), c.flows[s], std::max(0.25, 0.06 * c.flows[s]))
        << "flow " << s << " of " << k;
    }
  }
}

// M3: hidden terminals against ns-3 (scenario hidden: 0 -> 1 and 2 -> 1 at the
// given losses, 0 and 2 deaf to each other).
TEST(M3, HiddenSenderWithoutAggregation)
{
  // The hidden sender sends single MPDUs (ns3_sat --hiddenNoAgg=1, seeds 1-2):
  // the case a sender without a block-ack agreement is in, and the one that
  // isolates the model's two mechanisms, a receiver locked on the hidden frame
  // and the MPDUs a hidden start overlaps. Strong link within 7 %.
  const MacParams mp;
  const struct {int mcs; double hidden_db, strong_ns3;} cases[] = {
    {7, 60.0, 55.628}, {7, 75.0, 57.029}, {3, 60.0, 19.340}, {3, 75.0, 25.019}};
  for (const auto & c : cases) {
    Scene sc = Hidden(c.mcs, 60.0, c.hidden_db);
    sc.in.backlog_bytes.assign(9, 1e9);
    sc.in.backlog_bytes[2 * 3 + 1] = mp.mpdu_payload_bytes;
    const ShareOutput o = hmr_radio::ShareChannel(mp, sc.in);
    EXPECT_NEAR(sc.Mbps(o, 0, 1) / c.strong_ns3, 1.0, 0.07)
      << "MCS" << c.mcs << " hidden at " << c.hidden_db << " dB";
    EXPECT_LT(sc.Mbps(o, 2, 1), 0.3);   // ns-3: 0.00-0.04 Mbps
    EXPECT_EQ(o.domain_size[0], 1);
    EXPECT_EQ(o.domain_size[2], 1);
  }
}

TEST(M3, HiddenPairBothAggregatingIsConservative)
{
  // Both senders saturated with full aggregates at equal power. ns-3 settles
  // into one winner with a block-ack agreement and a loser without one, and
  // which is which changes with the seed (seed 2 split 3.9 / 35.5 Mbps); the
  // fluid model has no such state and splits evenly at a lower total. Known
  // limit (DESIGN_radio_v2.md §13): the model's total sits between 0.3 and 1.0
  // of ns-3's. Seeds 1-3 means: MCS7 39.650, MCS3 17.341 Mbps.
  const MacParams mp;
  const struct {int mcs; double total_ns3;} cases[] = {{7, 39.650}, {3, 17.341}};
  for (const auto & c : cases) {
    Scene sc = Hidden(c.mcs, 60.0, 60.0);
    const ShareOutput o = hmr_radio::ShareChannel(mp, sc.in);
    const double total = sc.Mbps(o, 0, 1) + sc.Mbps(o, 2, 1);
    EXPECT_GT(total, 0.3 * c.total_ns3) << "MCS" << c.mcs;
    EXPECT_LT(total, c.total_ns3) << "MCS" << c.mcs;
    EXPECT_NEAR(sc.Mbps(o, 0, 1), sc.Mbps(o, 2, 1), 1e-6);
    EXPECT_GT(o.hidden_loss[0 * 3 + 1], 0.5);
  }
  // A hidden robot too weak at the receiver (30 dB down) to break MCS0 costs
  // the strong link almost nothing: ns-3 6.319 / 6.227 Mbps (seeds 1-2), the
  // weak sender 0.000. The model does not count a receiver locked on the weak
  // frame (no breaker, no q_pre), hence its 3 % excess.
  Scene sc = Hidden(0, 60.0, 90.0);
  const ShareOutput o = hmr_radio::ShareChannel(mp, sc.in);
  EXPECT_NEAR(sc.Mbps(o, 0, 1) / 6.273, 1.0, 0.05);
  EXPECT_LT(sc.Mbps(o, 2, 1), 0.3);
}

// M4: spatial reuse. Two pairs out of each other's range (ns3_sat scenario
// reuse: 65.051 and 65.008 Mbps) each get a lone sender's goodput.
TEST(M4, SpatialReuse)
{
  const MacParams mp;
  Scene sc(4);
  for (int a = 0; a < 4; ++a) {
    for (int b = a + 1; b < 4; ++b) {
      sc.Link(a, b, 200.0);
    }
  }
  sc.Link(0, 1, 60.0);
  sc.Link(2, 3, 60.0);
  sc.Flow(0, 1, 7);
  sc.Flow(2, 3, 7);
  const ShareOutput o = hmr_radio::ShareChannel(mp, sc.in);
  EXPECT_NEAR(sc.Mbps(o, 0, 1) / 65.051, 1.0, 0.005);
  EXPECT_NEAR(sc.Mbps(o, 2, 3) / 65.008, 1.0, 0.005);
  EXPECT_DOUBLE_EQ(o.hidden_loss[0 * 4 + 1], 0.0);
  EXPECT_EQ(o.domain_size[0], 1);
  // An idle link is not a contender: the same scene with one flow is a lone
  // sender to the bit.
  Scene one = sc;
  one.in.active[2 * 4 + 3] = 0;
  const ShareOutput o1 = hmr_radio::ShareChannel(mp, one.in);
  EXPECT_DOUBLE_EQ(one.Mbps(o1, 0, 1), sc.Mbps(o, 0, 1));
}
