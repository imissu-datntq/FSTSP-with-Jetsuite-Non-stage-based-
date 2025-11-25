#!/bin/bash
folder_path="/home/orlab/CLionProjects/RV-FSTSP/Murray_instances/FSTSP/FSTSP_10_customer_problems"
binary_file="/home/orlab/CLionProjects/RV-FSTSP/cmake-build-release/RV-FSTSP-goc-revisit"

dtl_values=(20)
for val in "${dtl_values[@]}"; do

    # Iterate through files in the directory
    find "$folder_path" -maxdepth 1 -type d | while IFS= read -r folder; do
        # Extract the folder name from the full path
        folder_name=$(basename "$folder")
        output="/home/orlab/CLionProjects/RV-FSTSP/goc_revisit_murray/${val}_${folder_name}.out"
        touch "$output"
        # Call the binary file with the folder name as an argument
        "$binary_file" "$folder_name" "$val" 2>&1 | tee "$output"
    done
done