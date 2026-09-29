# ns-3 oracle for radio_mac (DESIGN_radio_v2.md §13)

`test_radio_mac` checks hmr_radio's PHY and MAC against constants recorded from
ns-3 3.44. These programs recorded them. colcon never builds them, and no ns-3
code is linked into hmr_radio: ns-3 is GPL-2.0 and serves only as the
reference we compare against.

| file | records | used by |
|---|---|---|
| `ns3_per.cc` | NistErrorRateModel: SNR at PER 0.9 / 0.5 / 0.1, HT MCS0-7, one 1542 B MPDU | P1 |
| `ns3_timing.cc` | WifiPhy::CalculateTxDuration for PSDUs and the compressed block ack | P2 |
| `ns3_sat.cc` | saturated UDP goodput, scenarios `sat`, `hidden`, `reuse` (per-pair losses via MatrixPropagationLossModel) | P3, M2-M4 |
| `run_ref.sh` | the `ns3_sat` runs behind M2-M4 | |
| `ref_ns3.txt`, `ref_ns3_phy.txt` | the recorded outputs | |
| `model_sat.cpp` | the same scenarios through `ShareChannel` (no ns-3) | side-by-side checks |
| `node_live_test.py` | drives a running `hmr_radio_node` (v2_phy): hidden line, then all in range | manual live check |

## Build (no root)

```
pip install --target ns3lib ns3==3.44.post0   # the wheel ships headers and libs
N=ns3lib/ns3
for p in ns3_per ns3_timing ns3_sat; do
  g++ -std=c++20 -O1 -I$N/include $p.cc -o $p -L$N/lib64 -Wl,-rpath,$PWD/$N/lib64 \
    $(ls $N/lib64/libns3.44-*.so | sed 's#.*/lib\(ns3.44-[a-z0-9-]*\)\.so#-l\1#')
done
g++ -std=c++17 -O2 -I../../include model_sat.cpp ../../src/radio_mac.cpp -o model_sat
./run_ref.sh > ref_ns3.txt
```

Strip conda from `PATH` first if it shadows the system compiler or python.

## Settings that matter

- `--bar=0`: no explicit block-ack request after a missed block ack (Linux
  mac80211 behaviour). ns-3's default sends BARs, which leaves it 1-5 % below
  the model under contention.
- Slot 20 µs: the 2.4 GHz long slot an IBSS uses. It is also ns-3's default
  there. The model defaults to the same value (`MacParams::slot_s`).
- Preamble detection at 4 dB (`ThresholdPreambleDetectionModel`, MinimumRssi
  -100 dBm). This is the model's `detect_snr_db`.
- UDP payload 1472 B: `PacketSize` includes the 12 B SeqTs header, so each
  MPDU is 1538 B, as in the model.

## Known gap

Two hidden senders that both aggregate at equal power: ns-3 ends with one
winner holding a block-ack agreement and one loser without, and which sender
is which depends on the seed (3.9 / 35.5 Mbps on seed 2, 18.5 / 19.0 on seed
1). The fluid model splits evenly and gives about 0.45 of ns-3's total. M3
asserts that bound. With the loser forced to single MPDUs
(`--hiddenNoAgg=1`), the model matches within 6 %.

## Live check

With the scratch-built node on its own DDS domain:

```
ROS_DOMAIN_ID=77 hmr_radio_node --ros-args --params-file config/profiles/neutral.yaml \
  -p 'robot_names:=[a,b,c]' -p 'reliable_topics:=[chatter_rel]' \
  -p 'best_effort_topics:=[beacon]' -p transmission_model:=progressive \
  -p seed:=1 -p use_sim_time:=false &
ROS_DOMAIN_ID=77 python3 node_live_test.py 30
```
