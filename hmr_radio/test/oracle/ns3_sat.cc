// Oracle: ns-3 802.11n (2.4 GHz, 20 MHz, 1 SS, SGI, A-MPDU 65535, ad hoc, EDCA BE)
// saturated UDP goodput. Losses are set per node pair (MatrixPropagationLossModel),
// so the scenarios isolate the MAC.
//   sat:     n senders -> node n (sink), all in range, sender k at MCS mcs[k]
//   hidden:  0 -> 1 and 2 -> 1; 0 and 2 cannot detect each other
//   reuse:   0 -> 1 and 2 -> 3; the two pairs cannot detect each other at all
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/propagation-module.h"
#include "ns3/wifi-module.h"
#include <cstdio>
#include <map>
#include <sstream>
using namespace ns3;

int main(int argc, char ** argv)
{
  std::string scenario = "sat", mcsList = "7";
  int n = 1;
  double duration = 10.0, strong = 60.0, far = 200.0, hiddenIntf = 60.0;
  uint32_t seed = 1;
  int slotUs = 0;
  bool bar = true, hiddenNoAgg = false;
  int traceNode = 0;
  CommandLine cmd;
  cmd.AddValue("scenario", "sat | hidden | reuse", scenario);
  cmd.AddValue("n", "senders (sat)", n);
  cmd.AddValue("mcs", "comma list, one per sender (last repeats)", mcsList);
  cmd.AddValue("duration", "measured seconds", duration);
  cmd.AddValue("strong", "loss dB between linked nodes", strong);
  cmd.AddValue("hiddenIntf", "loss dB from the hidden sender to the receiver", hiddenIntf);
  cmd.AddValue("seed", "run number", seed);
  cmd.AddValue("slot", "slot us (0: ns-3 default)", slotUs);
  cmd.AddValue("traceNode", "node whose PSDUs are histogrammed", traceNode);
  cmd.AddValue("hiddenNoAgg", "hidden sender (node 2) sends single MPDUs", hiddenNoAgg);
  cmd.AddValue("bar", "explicit BAR after a missed block ack (ns-3 default true)", bar);
  cmd.Parse(argc, argv);
  Config::SetDefault("ns3::QosTxop::UseExplicitBarAfterMissedBlockAck", BooleanValue(bar));
  RngSeedManager::SetSeed(12345);
  RngSeedManager::SetRun(seed);
  std::vector<int> mcs;
  {
    std::stringstream ss(mcsList);
    std::string t;
    while (std::getline(ss, t, ',')) mcs.push_back(std::stoi(t));
  }
  int nodesN = scenario == "sat" ? n + 1 : scenario == "hidden" ? 3 : 4;
  std::vector<std::pair<int, int>> flows;
  if (scenario == "sat") for (int k = 0; k < n; ++k) flows.push_back({k, n});
  else if (scenario == "hidden") flows = {{0, 1}, {2, 1}};
  else flows = {{0, 1}, {2, 3}};

  NodeContainer nodes;
  nodes.Create(nodesN);
  MobilityHelper mob;
  mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mob.Install(nodes);
  auto loss = CreateObject<MatrixPropagationLossModel>();
  loss->SetDefaultLoss(strong);
  if (scenario == "hidden") {
    loss->SetLoss(nodes.Get(0)->GetObject<MobilityModel>(), nodes.Get(2)->GetObject<MobilityModel>(), far, true);
    loss->SetLoss(nodes.Get(2)->GetObject<MobilityModel>(), nodes.Get(1)->GetObject<MobilityModel>(), hiddenIntf, true);
  } else if (scenario == "reuse") {
    loss->SetDefaultLoss(far);
    loss->SetLoss(nodes.Get(0)->GetObject<MobilityModel>(), nodes.Get(1)->GetObject<MobilityModel>(), strong, true);
    loss->SetLoss(nodes.Get(2)->GetObject<MobilityModel>(), nodes.Get(3)->GetObject<MobilityModel>(), strong, true);
  }
  auto chan = CreateObject<YansWifiChannel>();
  chan->SetPropagationLossModel(loss);
  chan->SetPropagationDelayModel(CreateObject<ConstantSpeedPropagationDelayModel>());

  YansWifiPhyHelper phy;
  phy.SetChannel(chan);
  phy.Set("ChannelSettings", StringValue("{1, 20, BAND_2_4GHZ, 0}"));
  phy.Set("TxPowerStart", DoubleValue(20.0));
  phy.Set("TxPowerEnd", DoubleValue(20.0));
  phy.Set("RxNoiseFigure", DoubleValue(7.0));
  phy.SetErrorRateModel("ns3::NistErrorRateModel");
  phy.SetPreambleDetectionModel("ns3::ThresholdPreambleDetectionModel",
    "Threshold", DoubleValue(4.0), "MinimumRssi", DoubleValue(-100.0));

  NetDeviceContainer devs;
  for (int k = 0; k < nodesN; ++k) {
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211n);
    int m = mcs[std::min<size_t>(k, mcs.size() - 1)];
    if (scenario == "hidden" && k == 2) m = mcs[std::min<size_t>(1, mcs.size() - 1)];
    if (scenario == "reuse" && k == 2) m = mcs[std::min<size_t>(1, mcs.size() - 1)];
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
      "DataMode", StringValue("HtMcs" + std::to_string(m)),
      "ControlMode", StringValue("HtMcs0"));
    wifi.ConfigHtOptions("ShortGuardIntervalSupported", BooleanValue(true));
    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac", "QosSupported", BooleanValue(true));
    devs.Add(wifi.Install(phy, mac, nodes.Get(k)));
  }

  if (hiddenNoAgg) Config::Set("/NodeList/2/DeviceList/0/$ns3::WifiNetDevice/Mac/BE_MaxAmpduSize", UintegerValue(0));
  if (slotUs > 0) Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/Slot", TimeValue(MicroSeconds(slotUs)));
  {
    auto ph = DynamicCast<WifiNetDevice>(devs.Get(0))->GetPhy();
    std::printf("ns-3 slot %.0f us sifs %.0f us\n", ph->GetSlot().GetMicroSeconds() * 1.0, ph->GetSifs().GetMicroSeconds() * 1.0);
  }
  InternetStackHelper stack;
  stack.Install(nodes);
  Ipv4AddressHelper addr;
  addr.SetBase("10.1.0.0", "255.255.255.0");
  Ipv4InterfaceContainer ifs = addr.Assign(devs);
  Ipv4StaticRoutingHelper sr;
  // Static ARP so no broadcast exchange precedes the flows.
  for (int a = 0; a < nodesN; ++a) {
    auto arp = nodes.Get(a)->GetObject<Ipv4L3Protocol>()->GetInterface(1)->GetArpCache();
    for (int b = 0; b < nodesN; ++b) {
      if (a == b) continue;
      auto e = arp->Add(ifs.GetAddress(b));
      e->SetMacAddress(devs.Get(b)->GetAddress());
      e->MarkPermanent();
    }
  }
  std::vector<Ptr<PacketSink>> sinks;
  uint16_t port = 9000;
  for (auto [s, d] : flows) {
    PacketSinkHelper sink("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), port));
    auto app = sink.Install(nodes.Get(d));
    app.Start(Seconds(0.0));
    sinks.push_back(DynamicCast<PacketSink>(app.Get(0)));
    UdpClientHelper cli(ifs.GetAddress(d), port);
    cli.SetAttribute("MaxPackets", UintegerValue(0));
    cli.SetAttribute("Interval", TimeValue(MicroSeconds(20)));
    cli.SetAttribute("PacketSize", UintegerValue(1472));   // UDP payload (includes the 12 B SeqTs header)
    auto c = cli.Install(nodes.Get(s));
    c.Start(Seconds(0.5));
    ++port;
  }
  static std::map<std::string, uint64_t> hist;
  static uint64_t ppdus = 0;
  static double airS = 0;
  Config::ConnectWithoutContext("/NodeList/" + std::to_string(traceNode) + "/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxPsduBegin",
    MakeCallback(+[](WifiConstPsduMap psdus, WifiTxVector txv, Watt_u) {
      if (Simulator::Now() < Seconds(1.5)) return;
      auto p = psdus.begin()->second;
      ++ppdus;
      airS += WifiPhy::CalculateTxDuration(p->GetSize(), txv, WIFI_PHY_BAND_2_4GHZ).GetSeconds();
      ++hist[std::to_string(p->GetNMpdus()) + "x" + std::to_string(p->GetSize()) + "B " + txv.GetMode().GetUniqueName() + " " + (*p->begin())->GetHeader().GetTypeString()];
    }));
  static std::map<std::string, uint64_t> rhist;
  Config::ConnectWithoutContext("/NodeList/" + std::to_string(nodesN - 1) + "/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxPsduBegin",
    MakeCallback(+[](WifiConstPsduMap psdus, WifiTxVector txv, Watt_u) {
      if (Simulator::Now() < Seconds(1.5)) return;
      auto p = psdus.begin()->second;
      char b[160];
      std::snprintf(b, sizeof b, "%uB %s %s %.1fus", p->GetSize(), txv.GetMode().GetUniqueName().c_str(),
        (*p->begin())->GetHeader().GetTypeString(),
        WifiPhy::CalculateTxDuration(p->GetSize(), txv, WIFI_PHY_BAND_2_4GHZ).GetNanoSeconds() / 1000.0);
      ++rhist[b];
    }));
  std::vector<uint64_t> at1(sinks.size());
  Simulator::Schedule(Seconds(1.5), [&]() { for (size_t k = 0; k < sinks.size(); ++k) at1[k] = sinks[k]->GetTotalRx(); });
  Simulator::Stop(Seconds(1.5 + duration));
  Simulator::Run();
  double tot = 0;
  for (size_t k = 0; k < sinks.size(); ++k) {
    double mbps = (sinks[k]->GetTotalRx() - at1[k]) * 8.0 / duration / 1e6;
    tot += mbps;
    std::printf("flow %d->%d goodput %.3f Mbps\n", flows[k].first, flows[k].second, mbps);
  }
  std::printf("total %.3f Mbps\n", tot);
  std::printf("traced node PPDUs/s %.1f air %.4f\n", ppdus / duration, airS / duration);
  for (auto & [k, v] : rhist) std::printf("  rx-side tx %s: %lu\n", k.c_str(), (unsigned long)v);
  for (auto & [k, v] : hist) std::printf("  %s: %lu\n", k.c_str(), (unsigned long)v);
  Simulator::Destroy();
}
