#!/bin/bash

add_record() {
  instance=$1
  type=$2
  is_loop=$3
  is_revisit=$4

  if [ -f "result/${type}-index/loop_${is_loop}_revisit_${is_revisit}/Niels/RV-FSTSP_Niels_instances_restricted_maxradius_${instance}.csv" ]; then
    mapfile -t content < <(sed -E 's/^[^0-9]+\,([0-9]+\.?[0-9]*%?)$/\1/' result/${type}-index/loop_${is_loop}_revisit_${is_revisit}/Niels/RV-FSTSP_Niels_instances_restricted_maxradius_${instance}.csv | awk '/^[0-9]+\.?[0-9]*%?$/{print}')
  elif [ -f "result/${type}-index/loop_${is_loop}_revisit_${is_revisit}/Niels/RV-FSTSP_Niels_instances_restricted_novisit_${instance}.csv" ]; then
    mapfile -t content < <(sed -E 's/^[^0-9]+\,([0-9]+\.?[0-9]*%?)$/\1/' result/${type}-index/loop_${is_loop}_revisit_${is_revisit}/Niels/RV-FSTSP_Niels_instances_restricted_novisit_${instance}.csv | awk '/^[0-9]+\.?[0-9]*%?$/{print}')
  else
    echo "Warning: File not found for instance ${instance} with ${type}-index, loop: ${is_loop}, revisit: ${is_revisit}"
    return
  fi

  # Counter stupid scientific notation of Gap field that regex could not catch
  if [ ${#content[@]} -eq 11 ]; then
    content=("${content[@]:0:2}" "0%" "${content[@]:2}")
  fi

  obj=${content[0]}
  lb=${content[1]}
  gap=${content[2]}
  solve_time=${content[3]}
  truck_served=${content[4]}
  drone_served=${content[5]}
  thread=${content[6]}
  dtl=${content[7]}
  sl=${content[8]}
  sr=${content[9]}

  echo "$instance,$type,$is_loop,$is_revisit,$solve_time,$obj,$lb,$gap,$truck_served,$drone_served,$thread,$dtl,$sl,$sr" >> summary.csv
}

echo "instance,type,loop,revisit,time (s),obj,lb,gap,T.serve,D.serve,thread,dtl,sl,sr" > summary.csv
instances=$(ls Niels_instances/restricted/maxradius Niels_instances/restricted/novisit | grep -E "^(.+\-n(1?[0-9]|20)\-.+)\..+$")
for instance in $instances; do
  for mode in 3 5; do
    add_record $instance $mode true true
    add_record $instance $mode true false
    add_record $instance $mode false true
    add_record $instance $mode false false
  done
done

echo "Summary is saved to $(pwd)/summary.csv"
