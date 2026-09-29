#!/bin/bash
# The ns-3 3.44 reference set behind test_radio_mac M2-M4: no explicit BAR, the
# default (20 us) slot, 5 s per run. Run from a directory holding the built
# ns3_sat (README.md); the output is ref_ns3.txt.
run() { for s in $SEEDS; do printf '%s seed%s ' "$*" $s; nice -n 19 ./ns3_sat "$@" --bar=0 --duration=5 --seed=$s 2>&1 | grep -E "^total|^flow" | tr '\n' ' '; echo; done; }
SEEDS="1"
run --scenario=sat --n=1 --mcs=0
run --scenario=sat --n=1 --mcs=3
run --scenario=sat --n=1 --mcs=7
SEEDS="1 2 3"
run --scenario=sat --n=2 --mcs=7
run --scenario=sat --n=4 --mcs=7
run --scenario=sat --n=3 --mcs=7,3,0
run --scenario=sat --n=2 --mcs=7,0
SEEDS="1"
run --scenario=reuse --mcs=7
SEEDS="1 2 3"
run --scenario=hidden --mcs=7 --strong=60 --hiddenIntf=60
run --scenario=hidden --mcs=3 --strong=60 --hiddenIntf=60
SEEDS="1 2"
run --scenario=hidden --mcs=0 --strong=60 --hiddenIntf=90
for m in 7 3; do for h in 60 75; do
  run --scenario=hidden --mcs=$m --strong=60 --hiddenIntf=$h --hiddenNoAgg=1
done; done
