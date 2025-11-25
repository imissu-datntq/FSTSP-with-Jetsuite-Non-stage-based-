#!/bin/bash
folder_path="/home/cuong/CLionProjects/RV-FSTSP/generated_instances/c_c"
binary_file="/home/cuong/CLionProjects/RV-FSTSP/cmake-build-release/RV-FSTSP"

dtl_values=(20)
for val in "${dtl_values[@]}"; do

    # Iterate through files in the directory
    find "$folder_path" -maxdepth 1 -type d | while IFS= read -r folder; do
        # Extract the folder name from the full path
        folder_name=$(basename "$folder")
        output="/home/cuong/CLionProjects/RV-FSTSP/og_c_c/${val}_${folder_name}.out"
        touch "$output"
        # Call the binary file with the folder name as an argument
        "$binary_file" "$folder_name" "$val" 2>&1 | tee "$output"
    done
done