// Oracle: ns-3 HT timing (2.4 GHz, 20 MHz, 1 SS, SGI): PPDU max time, max PSDU, TX durations.
#include "ns3/core-module.h"
#include "ns3/wifi-module.h"
#include <cstdio>
using namespace ns3;
int main()
{
  std::printf("PpduMaxTime HT_MF %.3f us\n", GetPpduMaxTime(WIFI_PREAMBLE_HT_MF).GetMicroSeconds() * 1.0);
  for (int m : {0, 3, 7}) {
    WifiTxVector tx(HtPhy::GetHtMcs(m), 0, WIFI_PREAMBLE_HT_MF, NanoSeconds(400), 1, 1, 0, MHz_u{20}, true);
    for (uint32_t b : {1544u, 4632u, 64848u}) {
      Time t = WifiPhy::CalculateTxDuration(b, tx, WIFI_PHY_BAND_2_4GHZ);
      std::printf("MCS%d %6u B -> %.1f us\n", m, b, t.GetNanoSeconds() / 1000.0);
    }
  }
  for (auto mode : {OfdmPhy::GetOfdmRate6Mbps(), OfdmPhy::GetOfdmRate12Mbps(), OfdmPhy::GetOfdmRate24Mbps()}) {
    WifiTxVector tx(mode, 0, WIFI_PREAMBLE_LONG, NanoSeconds(800), 1, 1, 0, MHz_u{20}, false);
    Time t = WifiPhy::CalculateTxDuration(32, tx, WIFI_PHY_BAND_2_4GHZ);
    std::printf("%s 32 B (compressed BA) -> %.1f us\n", mode.GetUniqueName().c_str(), t.GetNanoSeconds() / 1000.0);
  }
}
