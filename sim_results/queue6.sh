#!/usr/bin/env bash
# Pinned comparison: car stack on CAR_CORES, simulator tooling on cores 6-7.
R=/d/GW_UNIVERSITY/ARL/AUTOBOT_V2/UOG_ARL_CAR_CALIB/repo; B=/d/GW_UNIVERSITY/ARL/AUTOBOT_V2/UOG_ARL_CAR_CALIB/repo_base
{
for cc in "0-5:6" "0-1:2" "0:1"; do
  cores=${cc%%:*}; n=${cc##*:}
  (export RESULTS_DIR=$R/sim_results; cd $B; PROFILE=1 CAR_CORES=$cores $B/sim_results/run_batch.sh 2${n}_pin_orig_car${n}core 3 1)
  (cd $R; PROFILE=1 CAR_CORES=$cores sim_results/run_batch.sh 3${n}_pin_numpy_car${n}core 3 1)
done
echo QUEUE6 DONE
} >> $R/sim_results/queue6.console.txt 2>&1
