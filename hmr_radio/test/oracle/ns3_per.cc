// Oracle: ns-3 NistErrorRateModel PER for HT MCS0-7 (20 MHz, 1 SS, SGI) per MPDU size.
#include "ns3/core-module.h"
#include "ns3/wifi-module.h"
#include <cstdio>
using namespace ns3;
int main()
{
  Ptr<NistErrorRateModel> e = CreateObject<NistErrorRateModel>();
  const int bytes = 1542;
  for (int m = 0; m < 8; ++m) {
    WifiMode mode = HtPhy::GetHtMcs(m);
    WifiTxVector tx(mode, 0, WIFI_PREAMBLE_HT_MF, NanoSeconds(400), 1, 1, 0, MHz_u{20}, true);
    auto per = [&](double db) {
      return 1.0 - e->GetChunkSuccessRate(mode, tx, std::pow(10.0, db / 10.0), bytes * 8);
    };
    auto at = [&](double target) {
      double lo = -10, hi = 50;
      for (int k = 0; k < 60; ++k) { double mid = (lo + hi) / 2; if (per(mid) > target) lo = mid; else hi = mid; }
      return hi;
    };
    std::printf("MCS%d rate %.1f Mbps PER0.9 %.3f PER0.5 %.3f PER0.1 %.3f  per@10dB %.6g\n", m,
      mode.GetDataRate(MHz_u{20}, NanoSeconds(400), 1) / 1e6, at(0.9), at(0.5), at(0.1), per(10));
  }
}
