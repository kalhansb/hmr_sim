// Radio emulator v2, PHY and MAC. See radio_mac.hpp and DESIGN_radio_v2.md §13.
#include "hmr_radio/radio_mac.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace hmr_radio
{

namespace
{

constexpr double kSymbolS = 3.6e-6;       // short guard interval
constexpr double kHtMfPreambleS = 36e-6;  // L-STF, L-LTF, L-SIG, HT-SIG, HT-STF, 1 HT-LTF
constexpr double kSignalExtS = 6e-6;      // 2.4 GHz OFDM signal extension

struct McsDef
{
  int bits;   // coded bits per subcarrier: 1 BPSK, 2 QPSK, 4 16-QAM, 6 64-QAM
  int code;   // 1: 1/2, 2: 2/3, 3: 3/4, 5: 5/6
  int ndbps;
};
constexpr McsDef kMcs[kNumMcs] = {
  {1, 1, 26}, {2, 1, 52}, {2, 3, 78}, {4, 1, 104},
  {4, 3, 156}, {6, 2, 208}, {6, 3, 234}, {6, 5, 260}};

void CheckMcs(int mcs)
{
  if (mcs < 0 || mcs >= kNumMcs) {
    throw std::out_of_range("HT MCS out of range");
  }
}

}  // namespace

int McsNdbps(int mcs)
{
  CheckMcs(mcs);
  return kMcs[mcs].ndbps;
}

double McsRateBps(int mcs)
{
  return McsNdbps(mcs) / kSymbolS;
}

double UncodedBer(int mcs, double snr)
{
  CheckMcs(mcs);
  switch (kMcs[mcs].bits) {
    case 1: return 0.5 * std::erfc(std::sqrt(snr));
    case 2: return 0.5 * std::erfc(std::sqrt(snr / 2.0));
    case 4: return 0.75 * 0.5 * std::erfc(std::sqrt(snr / 10.0));
    default: return 7.0 / 12.0 * 0.5 * std::erfc(std::sqrt(snr / 42.0));
  }
}

double CodedBitErrorProb(int mcs, double snr)
{
  const double p = UncodedBer(mcs, snr);
  const double d = std::sqrt(4.0 * p * (1.0 - p));
  // Information-weight spectra of the 802.11 K=7 (133, 171) code and its
  // punctured rates, from the free distance on.
  static const double c12[] = {36, 211, 1404, 11633, 77433, 502690, 3322763, 21292910,
    134365911};
  static const double c23[] = {3, 70, 285, 1276, 6160, 27128, 117019, 498860, 2103891,
    8784123};
  static const double c34[] = {42, 201, 1492, 10469, 62935, 379644, 2253373, 13073811,
    75152755, 428005675};
  static const double c56[] = {92, 528, 8694, 79453, 792114, 7375573, 67884974, 610875423,
    5427275376.0, 47664215639.0};
  double pe = 0.0;
  switch (kMcs[mcs].code) {
    case 1:
      for (int k = 0; k < 9; ++k) {pe += c12[k] * std::pow(d, 10 + 2 * k);}
      pe *= 0.5;
      break;
    case 2:
      for (int k = 0; k < 10; ++k) {pe += c23[k] * std::pow(d, 6 + k);}
      pe /= 4.0;
      break;
    case 3:
      for (int k = 0; k < 10; ++k) {pe += c34[k] * std::pow(d, 5 + k);}
      pe /= 6.0;
      break;
    default:
      for (int k = 0; k < 10; ++k) {pe += c56[k] * std::pow(d, 4 + k);}
      pe /= 10.0;
      break;
  }
  return std::min(pe, 1.0);
}

double ChunkSuccess(int mcs, double snr, uint64_t nbits)
{
  if (UncodedBer(mcs, snr) == 0.0) {
    return 1.0;
  }
  return std::pow(1.0 - CodedBitErrorProb(mcs, snr), static_cast<double>(nbits));
}

double MpduPer(int mcs, double snr_db, int mpdu_bytes)
{
  return 1.0 - ChunkSuccess(mcs, std::pow(10.0, snr_db / 10.0),
    static_cast<uint64_t>(mpdu_bytes) * 8);
}

int MpduBytes(const MacParams & mp)
{
  return mp.mpdu_payload_bytes + mp.mpdu_overhead_bytes;
}

int AmpduBytes(const MacParams & mp, int k)
{
  if (k <= 0) {
    return 0;
  }
  const int sub = 4 + MpduBytes(mp);
  const int padded = (sub + 3) / 4 * 4;
  return (k - 1) * padded + sub;
}

double HtPpduDurationS(int mcs, int psdu_bytes)
{
  const double bits = 16.0 + 8.0 * psdu_bytes + 6.0;
  return kHtMfPreambleS + kSignalExtS + std::ceil(bits / McsNdbps(mcs)) * kSymbolS;
}

double BlockAckDurationS(int mcs)
{
  // Non-HT OFDM at the highest basic rate not above the data rate:
  // 6 Mbps for MCS0, 12 for MCS1-2, 24 above.
  CheckMcs(mcs);
  const int ndbps = mcs == 0 ? 24 : mcs <= 2 ? 48 : 96;
  return 20e-6 + std::ceil((16.0 + 8.0 * 32 + 6.0) / ndbps) * 4e-6 + kSignalExtS;
}

double AifsS(const MacParams & mp)
{
  return mp.sifs_s + mp.aifsn * mp.slot_s;
}

Aggregate FullAggregate(const MacParams & mp, int mcs, int mpdu_cap)
{
  Aggregate a;
  int k = 1;
  while (k < mp.max_ampdu_mpdus && (mpdu_cap <= 0 || k < mpdu_cap) && AmpduBytes(mp, k + 1) <= mp.max_ampdu_bytes &&
    HtPpduDurationS(mcs, AmpduBytes(mp, k + 1)) <= mp.ppdu_max_s)
  {
    ++k;
  }
  a.mpdus = k;
  a.bytes = AmpduBytes(mp, k);
  a.ppdu_s = HtPpduDurationS(mcs, a.bytes);
  a.payload_bits = 8.0 * mp.mpdu_payload_bytes * k;
  a.txop_s = AifsS(mp) + a.ppdu_s + mp.sifs_s + BlockAckDurationS(mcs);
  a.collision_s = AifsS(mp) + a.ppdu_s + mp.sifs_s + BlockAckDurationS(0);
  return a;
}

double BackoffTau(double p, int cw_min, int cw_max, int retry_limit)
{
  const double w = cw_min + 1.0;
  const int m = static_cast<int>(std::lround(std::log2((cw_max + 1.0) / w)));
  double tx = 0.0, slots = 0.0, pj = 1.0;
  for (int j = 0; j <= retry_limit; ++j) {
    const double wj = w * std::pow(2.0, std::min(j, m));
    tx += pj;
    slots += pj * (wj + 1.0) / 2.0;
    pj *= p;
  }
  return tx / slots;
}

BianchiPoint BianchiFixedPoint(int n, int cw_min, int cw_max, int retry_limit)
{
  BianchiPoint b;
  if (n <= 1) {
    b.tau = BackoffTau(0.0, cw_min, cw_max, retry_limit);
    return b;
  }
  // p - (1 - (1 - tau(p))^(n-1)) is increasing in p on [0, 1): bisect.
  double lo = 0.0, hi = 1.0;
  for (int k = 0; k < 100; ++k) {
    const double mid = 0.5 * (lo + hi);
    const double f = mid - (1.0 - std::pow(1.0 - BackoffTau(mid, cw_min, cw_max, retry_limit), n - 1));
    (f < 0.0 ? lo : hi) = mid;
  }
  b.p = 0.5 * (lo + hi);
  b.tau = BackoffTau(b.p, cw_min, cw_max, retry_limit);
  return b;
}

int RateControl::Update(const MacParams & mp, double snr_db, double dt_s)
{
  const double w = init_ ? 1.0 - std::pow(mp.minstrel_keep_per_100ms, dt_s / 0.1) : 1.0;
  int best = 0;
  double best_tp = -1.0;
  for (int m = 0; m < kNumMcs; ++m) {
    const double s = 1.0 - MpduPer(m, snr_db, MpduBytes(mp));
    p_[m] = (1.0 - w) * p_[m] + w * s;
    if (p_[m] < mp.minstrel_min_success) {
      continue;
    }
    const Aggregate a = FullAggregate(mp, m);
    const double tp = p_[m] * a.payload_bits / a.txop_s;
    if (tp > best_tp) {
      best_tp = tp;
      best = m;
    }
  }
  init_ = true;
  mcs_ = best;
  return mcs_;
}

ShareOutput ShareChannel(const MacParams & mp, const ShareInput & in)
{
  const int n = in.n;
  ShareOutput out;
  out.goodput_bps.assign(n * n, 0.0);
  out.hidden_loss.assign(n * n, 0.0);
  out.mpdu_success.assign(n * n, 0.0);
  out.txop_rate.assign(n * n, 0.0);
  out.duty.assign(n, 0.0);
  out.attempt_rate.assign(n, 0.0);
  out.collision_p.assign(n, 0.0);
  out.fail_p.assign(n, 0.0);
  out.domain_size.assign(n, 0);

  // Per link its aggregate; per robot its active destinations and mean
  // exchange times.
  std::vector<Aggregate> agg(n * n);
  std::vector<std::vector<int>> dests(n), dom(n);
  std::vector<double> ts(n, 0.0), tc(n, 0.0), tppdu(n, 0.0);
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      const int l = i * n + j;
      if (i != j && in.active[l]) {
        dests[i].push_back(j);
        const int cap = in.backlog_bytes.empty() ? 0 :
          std::max(1, static_cast<int>(std::ceil(in.backlog_bytes[l] / mp.mpdu_payload_bytes)));
        agg[l] = FullAggregate(mp, in.mcs[l], cap);
        const Aggregate & a = agg[l];
        ts[i] += a.txop_s;
        tc[i] += a.collision_s;
        tppdu[i] += a.ppdu_s;
      }
    }
    if (!dests[i].empty()) {
      ts[i] /= dests[i].size();
      tc[i] /= dests[i].size();
      tppdu[i] /= dests[i].size();
    }
  }
  for (int i = 0; i < n; ++i) {
    if (dests[i].empty()) {
      continue;
    }
    dom[i].push_back(i);
    for (int k = 0; k < n; ++k) {
      if (k != i && !dests[k].empty() && in.hear[i * n + k]) {
        dom[i].push_back(k);
      }
    }
    out.domain_size[i] = static_cast<int>(dom[i].size());
  }

  // Which robots break which links: h hidden from the sender i (or d itself,
  // which cannot receive while it sends) with enough power at d to fail i's MCS.
  const double noise_mw = std::pow(10.0, in.noise_dbm / 10.0);
  std::vector<std::vector<int>> breakers(n * n);
  for (int i = 0; i < n; ++i) {
    for (const int d : dests[i]) {
      const int l = i * n + d;
      for (int h = 0; h < n; ++h) {
        if (h == i || dests[h].empty() || in.hear[i * n + h]) {
          continue;   // idle, or in i's domain: carrier sense keeps them apart
        }
        bool breaks = h == d;
        if (!breaks) {
          const double s_mw = std::pow(10.0, in.rx_dbm[l] / 10.0);
          const double i_mw = std::pow(10.0, in.rx_dbm[h * n + d] / 10.0);
          breaks = MpduPer(in.mcs[l], 10.0 * std::log10(s_mw / (noise_mw + i_mw)),
            MpduBytes(mp)) > 0.5;
        }
        if (breaks) {
          breakers[l].push_back(h);
        }
      }
    }
  }

  // Fixed point over tau: attempts and duty follow from tau; hidden robots'
  // duty sets each sender's failure probability, which sets its tau.
  std::vector<double> tau(n, 0.0), q_pre(n * n, 0.0);
  for (int i = 0; i < n; ++i) {
    if (!dests[i].empty()) {
      tau[i] = BackoffTau(0.0, mp.cw_min, mp.cw_max, mp.retry_limit);
    }
  }
  for (int it = 0; it < 200; ++it) {
    for (int i = 0; i < n; ++i) {
      if (dests[i].empty()) {
        continue;
      }
      double idle = 1.0, others_idle = 1.0, succ_ts = 0.0, max_tc = 0.0, succ_sum = 0.0;
      for (const int k : dom[i]) {
        idle *= 1.0 - tau[k];
        if (k != i) {
          others_idle *= 1.0 - tau[k];
        }
        max_tc = std::max(max_tc, tc[k]);
      }
      for (const int k : dom[i]) {
        const double succ_k = tau[k] * idle / (1.0 - tau[k]);
        succ_sum += succ_k;
        succ_ts += succ_k * ts[k];
      }
      const double e_slot = idle * mp.slot_s + succ_ts + (1.0 - idle - succ_sum) * max_tc;
      out.attempt_rate[i] = tau[i] / e_slot;
      out.duty[i] = out.attempt_rate[i] * tppdu[i];
      out.collision_p[i] = 1.0 - others_idle;
    }
    double change = 0.0;
    for (int i = 0; i < n; ++i) {
      if (dests[i].empty()) {
        continue;
      }
      double q_mean = 0.0;
      for (const int d : dests[i]) {
        const int l = i * n + d;
        double clear = 1.0;
        for (const int h : breakers[l]) {
          clear *= 1.0 - std::min(1.0, out.duty[h]);
        }
        q_pre[l] = 1.0 - clear;
        q_mean += q_pre[l];
      }
      q_mean /= dests[i].size();
      out.fail_p[i] = 1.0 - (1.0 - out.collision_p[i]) * (1.0 - q_mean);
      const double t = BackoffTau(out.fail_p[i], mp.cw_min, mp.cw_max, mp.retry_limit);
      change = std::max(change, std::fabs(t - tau[i]));
      tau[i] = 0.5 * tau[i] + 0.5 * t;
    }
    if (change < 1e-9) {
      break;
    }
  }

  // Hidden starts during the frame arrive as Poisson processes (rate
  // attempt_rate[h], length tppdu[h]); an MPDU starting t into the frame, m
  // long, is lost when some start fell in the last min(t, tppdu[h]) + m.
  // Midpoint rule over t.
  const int kSteps = 64;
  std::vector<double> f_mid(n * n, 1.0);
  for (int i = 0; i < n; ++i) {
    for (const int d : dests[i]) {
      const int l = i * n + d;
      if (breakers[l].empty()) {
        continue;
      }
      const double t_frame = agg[l].ppdu_s;
      const double m = t_frame / agg[l].mpdus;
      double sum = 0.0;
      for (int s = 0; s < kSteps; ++s) {
        const double t = (s + 0.5) * t_frame / kSteps;
        double hits = 0.0;
        for (const int h : breakers[l]) {
          hits += out.attempt_rate[h] * (std::min(t, tppdu[h]) + m);
        }
        sum += std::exp(-hits);
      }
      f_mid[l] = sum / kSteps;
    }
  }

  for (int i = 0; i < n; ++i) {
    for (const int d : dests[i]) {
      const int l = i * n + d;
      const Aggregate & a = agg[l];
      const double per = MpduPer(in.mcs[l], in.snr_db[l], MpduBytes(mp));
      out.hidden_loss[l] = 1.0 - (1.0 - q_pre[l]) * f_mid[l];
      out.mpdu_success[l] = (1.0 - per) * f_mid[l];
      out.txop_rate[l] = out.attempt_rate[i] * (1.0 - out.collision_p[i]) * (1.0 - q_pre[l]) /
        dests[i].size();
      out.goodput_bps[l] = out.txop_rate[l] * a.payload_bits * out.mpdu_success[l];
    }
  }
  return out;
}

}  // namespace hmr_radio
