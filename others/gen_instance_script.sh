#!/bin/bash
binary_file="/home/cuong/CLionProjects/RV-FSTSP/cmake-build-release/RV-FSTSP"
n_customer=10
side_of_area=10
depot_position=(c e r)
request_distribution=(c r rc)
truck_speed=0.3
drone_speed=1.2
# Nested for loops to iterate over elements in both arrays
for dp in "${depot_position[@]}"; do
    for rd in "${request_distribution[@]}"; do
      for v in {1..10}; do
        "$binary_file" "$n_customer" "$side_of_area" "$dp" "$rd" "$v" "$truck_speed" "$drone_speed"
        done
    done
done