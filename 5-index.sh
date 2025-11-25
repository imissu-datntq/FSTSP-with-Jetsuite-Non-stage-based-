#!/bin/bash

# setup directories to save output
current_dir=$(pwd)
mkdir -p $current_dir/log/5-index/
mkdir -p $current_dir/result/5-index/loop_true_revisit_true/Niels/
mkdir -p $current_dir/result/5-index/loop_false_revisit_true/Niels/
mkdir -p $current_dir/result/5-index/loop_true_revisit_false/Niels/
mkdir -p $current_dir/result/5-index/loop_false_revisit_false/Niels/

echo "Initializing 5-index scripts"
# Run 5-index
nohup ./run_controller.sh 5 "true" "true" > $current_dir/log/5-index/loop_true_revisit_true.txt 2>&1 &
# Wait the above to run a little first to avoid Race condition
sleep 1
nohup ./run_controller.sh 5 "true" "false" > $current_dir/log/5-index/loop_true_revisit_false.txt 2>&1 &
sleep 1
nohup ./run_controller.sh 5 "false" "true" > $current_dir/log/5-index/loop_false_revisit_true.txt 2>&1 &
sleep 1
nohup ./run_controller.sh 5 "false" "false" > $current_dir/log/5-index/loop_false_revisit_false.txt 2>&1 &
sleep 1