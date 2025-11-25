#!/bin/bash

# Limit number of call to solver script. This only requires a small delay between calls
called=$(pgrep -afc "^.*run_solver\.sh.*$")
while [ $called -ge 4 ]; do
  echo "$called instances of this script are running. Waiting other instances done..."
  sleep 90
  called=$(pgrep -afc "^.*run_solver\.sh.*$")
done

current_dir=$(pwd)
nohup ./run_solver.sh $1 $2 $3 > $current_dir/log/$1-index/loop_$2_revisit_$3.txt 2>&1 &
