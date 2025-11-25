#!/bin/bash
echo "<<<<<<<<<<<<< Revisit >>>>>>>>>>>"

# Required parameters
input_dir="Murray_instances/FSTSP/FSTSP_10_customer_problems/"
output_dir="result/revisit/"
driver="RV-FSTSP"

# Solver parameters
t=-1
screen_mode=1
dtl=20
sl=1
sr=1
thread=16
check="true"
allow_revisit="true"

instances=$(ls $input_dir)
total_instances=$(ls $input_dir | wc -l)
echo "Total instances: $total_instances"

i=0
project_dir=$(pwd)
for instance in $instances; do
    echo "Solving instance: $instance"
    echo "-----------------------------------"
    ./$driver -i "$project_dir"/$input_dir"$instance" -o $output_dir -s $screen_mode -t $t --thread=$thread --dtl=$dtl --sl=$sl --sr=$sr --check=$check --revisit=$allow_revisit
    echo "-----------------------------------"
    i=$((i+1))
    echo "Solved $i/$total_instances instances"
    echo
done
