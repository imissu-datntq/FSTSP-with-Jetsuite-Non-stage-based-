#!/bin/bash
echo "Running Non-revisit..."
nohup ./run_nonrv.sh > nonrv.log 2>&1 &
sleep 1
echo "Running revisit..."
nohup ./run_rv.sh > rv.log 2>&1 &
