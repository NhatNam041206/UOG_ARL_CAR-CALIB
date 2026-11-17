#!/usr/bin/env bash
# P1+P2 re-measure, then A1 variants one at a time. Resumable: re-run this script
# after a sleep / restart; finished runs are skipped (run_batch.sh).
R=/d/GW_UNIVERSITY/ARL/AUTOBOT_V2/UOG_ARL_CAR_CALIB/repo; cd $R
{
PROFILE=1 CAR_CORES=0-1 sim_results/run_batch.sh 40_p1p2_car2core 5 1
PROFILE=1 CAR_CORES=0-5 sim_results/run_batch.sh 41_p1p2_car6core 3 1
for v in a1_beams240 a1_particles1000 a1_beams180; do
  PROFILE=1 CAR_CORES=0-1 SIM_NAV2_PARAMS=sim_results/variants/$v.yaml sim_results/run_batch.sh 5x_${v}_car2core 5 1
  PROFILE=1 CAR_CORES=0-5 SIM_NAV2_PARAMS=sim_results/variants/$v.yaml sim_results/run_batch.sh 5x_${v}_car6core 3 1
done
echo QUEUE7 DONE
} >> $R/sim_results/queue7.console.txt 2>&1
