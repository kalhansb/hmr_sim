// Radio emulator v2, PHY and MAC (DESIGN_radio_v2.md §13): 802.11n HT rates with
// the NIST error-rate model, Minstrel-HT's rate choice, and an analytic EDCA
// channel share (Bianchi) with carrier-sense domains, hidden terminals and
// spatial reuse. No ROS dependency: the node, radio_probe and the tests call this
// one implementation. ns-3 3.44 is the external oracle (test/oracle/); none of
// its code is used here.
#pragma once

#include <array>
#include <cstdint>
#include <vector>

namespace hmr_radio
{

// ---- PHY: HT, 20 MHz, 1 spatial stream, short guard interval (L5, L6) --------

constexpr int kNumMcs = 8;

// Data bits per OFDM symbol and PHY rate (bit/s) of HT MCS m (3.6 µs symbols).
int McsNdbps(int mcs);
double McsRateBps(int mcs);

// NIST OFDM error-rate model (Pei & Henderson 2010): uncoded BER of the MCS's
// modulation, the Viterbi union bound for its code rate (weight spectra from
// Proakis / Frenger / Haccoun & Begin), and the chunk success (1 - pe)^nbits.
double UncodedBer(int mcs, double snr_linear);
double CodedBitErrorProb(int mcs, double snr_linear);
double ChunkSuccess(int mcs, double snr_linear, uint64_t nbits);
// Frame error rate of one MPDU of mpdu_bytes at snr_db.
double MpduPer(int mcs, double snr_db, int mpdu_bytes);

// ---- MAC (L8, L9): EDCA AC_BE, 2.4 GHz, HT-mixed preamble -------------------

struct MacParams
{
  // 2.4 GHz IBSS (no base station, peer to peer): the long ERP slot. An IBSS
  // never uses the short slot; ns-3 3.44 defaults to 20 us here too.
  double slot_s = 20e-6;
  double sifs_s = 10e-6;
  int aifsn = 3;                // AC_BE: AIFS = SIFS + 3 slots
  int cw_min = 15;
  int cw_max = 1023;
  int retry_limit = 7;          // MPDU transmissions = retry_limit + 1
  int mpdu_payload_bytes = 1472;   // UDP payload of a 1500 B IP packet
  int mpdu_overhead_bytes = 66;    // QoS header 26 + LLC/SNAP 8 + IP/UDP 28 + FCS 4
  int max_ampdu_bytes = 65535;
  int max_ampdu_mpdus = 64;     // block-ack window
  double ppdu_max_s = 5.484e-3;    // aPPDUMaxTime, HT-mixed
  // Preamble detection: a frame is detected, holds carrier sense busy and can be
  // received only at SNR >= detect_snr_db (ns-3 ThresholdPreambleDetectionModel).
  double detect_snr_db = 4.0;
  // Minstrel-HT EWMA: weight kept per 100 ms update interval.
  double minstrel_keep_per_100ms = 0.75;
  double minstrel_min_success = 0.1;
};

// Bytes of one MPDU (payload + overhead) and of an A-MPDU of k MPDUs (each with
// a 4 B delimiter, padded to 4 B except the last).
int MpduBytes(const MacParams & mp);
int AmpduBytes(const MacParams & mp, int k);
// HT-mixed PPDU duration for a PSDU of psdu_bytes (ns-3 CalculateTxDuration,
// 2.4 GHz: 36 µs preamble + 6 µs signal extension + ceil((22 + 8 L) / Ndbps) symbols).
double HtPpduDurationS(int mcs, int psdu_bytes);
// Compressed block ack (32 B) at the control-response rate for data at mcs.
double BlockAckDurationS(int mcs);
double AifsS(const MacParams & mp);

struct Aggregate
{
  int mpdus = 0;
  int bytes = 0;            // PSDU bytes
  double ppdu_s = 0.0;
  double payload_bits = 0.0;   // application bits carried if every MPDU succeeds
  double txop_s = 0.0;      // AIFS + PPDU + SIFS + block ack (a successful exchange)
  double collision_s = 0.0;    // AIFS + PPDU + SIFS + 6 Mbps block-ack timeout
};
// The largest A-MPDU within the byte, MPDU-count and duration caps.
Aggregate FullAggregate(const MacParams & mp, int mcs, int mpdu_cap = 0);

// Per-slot transmit probability of a saturated station whose exchanges fail with
// probability p (Bianchi 2000's chain with a retry limit: stage j draws from
// W_j = (cw_min + 1) 2^min(j, m), j = 0 .. retry_limit).
double BackoffTau(double p, int cw_min, int cw_max, int retry_limit);
// Bianchi fixed point for n identical saturated stations that all hear each
// other: tau per slot, p the conditional collision probability.
struct BianchiPoint
{
  double tau = 0.0;
  double p = 0.0;
};
BianchiPoint BianchiFixedPoint(int n, int cw_min, int cw_max, int retry_limit);

// Minstrel-HT's choice on analytic success: an EWMA of the MPDU success per MCS,
// updated every link tick; the MCS maximising success x payload / TXOP time
// among those at or above minstrel_min_success (MCS0 otherwise).
class RateControl
{
public:
  int Update(const MacParams & mp, double snr_db, double dt_s);
  int mcs() const { return mcs_; }
  double success(int m) const { return p_[m]; }

private:
  std::array<double, kNumMcs> p_{};
  bool init_ = false;
  int mcs_ = 0;
};

// One channel-share evaluation over n robots (a fluid model of EDCA on the air):
//   hear[i*n+j]    robot i detects robot j's preamble (symmetric)
//   rx_dbm[i*n+j]  power robot j receives from robot i (dBm)
//   mcs/snr_db[i*n+j]  the i->j link's current MCS and SNR
//   active[i*n+j]  i has traffic for j on a connected link
//   backlog_bytes[i*n+j]  optional: caps the link's aggregate to its backlog
//                  (empty = every active link saturated)
// Every active robot contends with the active robots it hears (its domain) and
// splits its TXOPs round-robin over its active destinations. A hidden robot
// (not in the sender's domain, or the destination itself) whose signal breaks
// the frame at the receiver costs it two ways: already on the air when the
// frame starts, the receiver is locked on it and the whole exchange fails (the
// sender's contention window grows, as for a collision); starting during the
// frame, it breaks the MPDUs it overlaps and the block ack still returns.
// Each robot's tau solves BackoffTau at its own failure probability, iterated
// to a fixed point, so a sender that keeps failing backs off exponentially.
// Per MPDU a link also loses the PER at its SNR.
struct ShareInput
{
  int n = 0;
  double noise_dbm = -94.0;
  std::vector<char> hear, active;
  std::vector<double> rx_dbm, snr_db, backlog_bytes;
  std::vector<int> mcs;
};
struct ShareOutput
{
  std::vector<double> goodput_bps;   // per link i*n+j, application bits/s
  // per link, 1 - (1 - q_pre) f_mid: the share of MPDUs hidden robots cost,
  // whole exchanges lost to a locked receiver (q_pre) and MPDUs overlapped by
  // a hidden start (1 - f_mid)
  std::vector<double> hidden_loss;
  std::vector<double> mpdu_success;  // per link, in an exchange that got through: (1 - PER) f_mid
  std::vector<double> txop_rate;     // per link, successful TXOPs/s
  std::vector<double> duty;          // per robot, fraction of time on the air
  std::vector<double> attempt_rate;  // per robot, TXOP attempts/s
  std::vector<double> collision_p;   // per robot, collision probability in its domain
  std::vector<double> fail_p;        // per robot, exchange failure incl. hidden robots
  std::vector<int> domain_size;      // per robot
};
ShareOutput ShareChannel(const MacParams & mp, const ShareInput & in);

}  // namespace hmr_radio
